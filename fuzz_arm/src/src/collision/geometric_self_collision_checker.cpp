#include "collision/geometric_self_collision_checker.hpp"
#include <iostream>
#include <unordered_set>

namespace simulation {

GeometricSelfCollisionChecker::GeometricSelfCollisionChecker(
    const RobotModel &model, const kinematics::KinematicChain &chain)
    : chain_(chain) {

  // RobotModelから全リンク情報を取得
  const auto &all_joints = model.getJoints();

  // リンク名とインデックスのマッピングを作成
  std::map<std::string, int> link_name_to_index;
  std::vector<std::string> link_names;

  // ルートリンクを追加
  std::string root_name = model.getRootLinkName();
  link_names.push_back(root_name);
  link_name_to_index[root_name] = 0;

  // 全ジョイントを走査し、子リンクを順次追加
  // （URDFのツリー構造に従って順序付け）
  std::function<void(const std::string &)> add_children;
  add_children = [&](const std::string &parent_name) {
    for (const auto &[joint_name, joint] : all_joints) {
      if (joint.parent_link == parent_name) {
        const std::string &child_name = joint.child_link;
        if (link_name_to_index.find(child_name) == link_name_to_index.end()) {
          link_names.push_back(child_name);
          link_name_to_index[child_name] = (int)link_names.size() - 1;
          add_children(child_name); // 再帰的に子を追加
        }
      }
    }
  };
  add_children(root_name);

  // リンク親子関係マップを構築（固定リンク位置計算用）
  for (const auto &[joint_name, joint] : all_joints) {
    fixed_link_info_[joint.child_link] =
        std::make_pair(joint.parent_link, joint.origin);

    if (joint.type == kinematics::JointType::Fixed) {
      fixed_link_connectivity_[joint.child_link] = joint.parent_link;
    }
  }

  // ベースリンクから固定ジョイントのみで繋がっているリンクを特定
  std::unordered_set<std::string> base_fixed_links;
  base_fixed_links.insert(root_name);
  bool base_set_changed = true;
  while (base_set_changed) {
    base_set_changed = false;
    for (const auto &[child, parent] : fixed_link_connectivity_) {
      if (base_fixed_links.count(parent) && !base_fixed_links.count(child)) {
        base_fixed_links.insert(child);
        base_set_changed = true;
      }
    }
  }

  // デバッグ出力：登録されたリンク一覧
  std::cout << "[GeometricChecker] Registered " << link_names.size()
            << " links for collision checking:" << std::endl;
  for (size_t i = 0; i < link_names.size(); ++i) {
    std::cout << "  [" << i << "] " << link_names[i] << std::endl;
  }

  // 各リンクの衝突形状を登録
  for (size_t link_idx = 0; link_idx < link_names.size(); ++link_idx) {
    const std::string &name = link_names[link_idx];

    // ベースリンクは固定されているため、自己干渉チェック対象から除外
    if (link_idx == 0) {
      std::cout << "[GeometricChecker] Skipping base_link (fixed to world)"
                << std::endl;
      continue;
    }

    const auto *props = model.getLink(name);
    if (!props)
      continue;

    for (const auto &col : props->collisions) {
      collision::SelfCollisionChecker::CollisionObject obj;
      obj.id = (int)collision_objects_.size();
      obj.name = name + "_" + col.name;
      obj.is_fixed_to_base = (base_fixed_links.count(name) > 0);

      // Geometry Conversion
      bool valid = false;
      if (col.geometry.type == GeometryType::SPHERE) {
        obj.type = collision::SelfCollisionChecker::ShapeType::SPHERE;
        obj.sphere.radius = col.geometry.size.x();
        obj.sphere.center = Eigen::Vector3d::Zero();
        valid = true;
      } else if (col.geometry.type == GeometryType::CYLINDER) {
        obj.type = collision::SelfCollisionChecker::ShapeType::CAPSULE;
        double r = col.geometry.size.x();
        double l = col.geometry.size.y();
        obj.capsule.radius = r;
        obj.capsule.a = Eigen::Vector3d(0, 0, -l / 2);
        obj.capsule.b = Eigen::Vector3d(0, 0, l / 2);
        valid = true;
      } else if (col.geometry.type == GeometryType::BOX) {
        obj.type = collision::SelfCollisionChecker::ShapeType::BOX;
        // Box collision with reduced safety margin (2mm)
        obj.box.extents =
            col.geometry.size * 0.5 + Eigen::Vector3d(0.002, 0.002, 0.002);
        obj.box.center = Eigen::Vector3d::Zero();
        obj.box.rotation = Eigen::Matrix3d::Identity();
        valid = true;
      } else if (col.geometry.type == GeometryType::MESH) {
        std::cout << "[GeometricChecker] Warning: Mesh collision geometry for "
                  << obj.name << " approximated as Sphere(r=0.05)."
                  << std::endl;
        obj.type = collision::SelfCollisionChecker::ShapeType::SPHERE;
        obj.sphere.radius = 0.05;
        obj.sphere.center = Eigen::Vector3d::Zero();
        valid = true;
      }

      if (valid) {
        collision_objects_.push_back(obj);
        ObjectLinkMap map;
        map.link_index = (int)link_idx;
        map.link_name = name;
        map.local_tf = col.origin;
        object_map_.push_back(map);
      }
    }
  }

  // --- 衝突除外の初期設定 ---

  // 1. URDFのジョイント構成に基づいて隣接リンクを特定
  for (const auto &[joint_name, joint] : all_joints) {
    addCollisionExclusion(joint.parent_link, joint.child_link);
  }

  // 2. 仮想リンクを考慮した除外ペアの伝播
  // Bが衝突形状を持たない場合、Aと「Bの子」のペアも除外
  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto &[joint_name, joint] : all_joints) {
      const std::string &A = joint.parent_link;
      const std::string &B = joint.child_link;

      const auto *link_B = model.getLink(B);
      if (link_B && link_B->collisions.empty()) {
        for (const auto &[j2_name, j2] : all_joints) {
          if (j2.parent_link == B) {
            const std::string &C = j2.child_link;
            if (collision_exclusion_pairs_.find({A, C}) ==
                collision_exclusion_pairs_.end()) {
              addCollisionExclusion(A, C);
              changed = true;
            }
          }
        }
      }
    }
  }

  // 4. 隣従リンク（Parent-of-Parent）の除外設定
  // 5cm幅のロボットで10cm以下のリンクを介している場合、関節部で必ず干渉するため
  for (const auto &[joint_name, joint] : all_joints) {
    const std::string &B = joint.parent_link; // 中間リンク
    const std::string &C = joint.child_link;  // 子リンク

    for (const auto &[j_prev_name, j_prev] : all_joints) {
      if (j_prev.child_link == B) {
        const std::string &A = j_prev.parent_link; // 親の親リンク
        // A <-> C の直接干渉を除外
        addCollisionExclusion(A, C);
      }
    }
  }

  // 5. 兄弟リンク（指など）の除外設定
  for (const auto &[j1_name, j1] : all_joints) {
    for (const auto &[j2_name, j2] : all_joints) {
      if (j1_name != j2_name && j1.parent_link == j2.parent_link) {
        addCollisionExclusion(j1.child_link, j2.child_link);
      }
    }
  }

  // 3. 内部チェッカーに反映（同一リンク内、または除外ペア）
  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    for (size_t j = i + 1; j < collision_objects_.size(); ++j) {
      const std::string &n1 = object_map_[i].link_name;
      const std::string &n2 = object_map_[j].link_name;

      if (n1 == n2 || shouldSkipCollision(n1, n2)) {
        checker_.setIgnorePair(collision_objects_[i].id,
                               collision_objects_[j].id);
      }
    }
  }
}

void GeometricSelfCollisionChecker::updateBodyPoses(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
    const std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
        &orientations) {

  // KinematicChainのbuildAllLinkTransformsを使用して全リンクの姿勢を取得
  std::map<std::string, Eigen::Isometry3d> link_transforms;
  chain_.buildAllLinkTransforms(positions, orientations, fixed_link_info_,
                                link_transforms);

  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    auto &obj = collision_objects_[i];
    const auto &map = object_map_[i];

    // リンクの姿勢を取得（固定リンクも対応）
    auto it = link_transforms.find(map.link_name);
    if (it == link_transforms.end()) {
      std::cerr << "[GeometricChecker] Warning: Link '" << map.link_name
                << "' not found in link_transforms" << std::endl;
      continue;
    }
    Eigen::Isometry3d link_tf = it->second;

    // Object Transform (World) = Link_World * Local_Offset
    Eigen::Isometry3d obj_tf = link_tf * map.local_tf;

    // Update Primitive Geometry
    if (obj.type == collision::SelfCollisionChecker::ShapeType::SPHERE) {
      // Sphere center is (0,0,0) in local, so just translation of obj_tf
      obj.sphere.center = obj_tf.translation();
    } else if (obj.type ==
               collision::SelfCollisionChecker::ShapeType::CAPSULE) {
      // Transform endpoints a, b
      // Original local a, b were (0,0,-l/2), (0,0,l/2) usually
      // But we store current world a,b in struct.
      // We need to store observable original local shape?
      // Actually, CollisionObject stores the *currrent* shape state usually?
      // Wait, CollisionObject struct has `Capsule shape`.
      // If I modify `obj.capsule.a`, I lose the local original.
      // I need to reconstruct from dimensions.
      // Re-read Constructor: I calculated a/b based on size.
      // I should probably store 'local/original' geometry or re-calculate.
      // Optimization: Pre-compute local a/b.
      // Since I only have `obj.capsule` (which is modified), I can't easily
      // reset. Hack: I know cylinder was created Z-aligned with length L.
      // We can get L from current a-b norm? Yes.
      double len = (obj.capsule.b - obj.capsule.a).norm();
      Eigen::Vector3d local_a(0, 0, -len / 2.0);
      Eigen::Vector3d local_b(0, 0, len / 2.0);

      obj.capsule.a = obj_tf * local_a;
      obj.capsule.b = obj_tf * local_b;
    } else if (obj.type == collision::SelfCollisionChecker::ShapeType::BOX) {
      obj.box.center = obj_tf.translation();
      obj.box.rotation = obj_tf.rotation();
      // extents remains constant
    }
  }
}

bool GeometricSelfCollisionChecker::checkCollision() {
  return checker_.checkSelfCollision(collision_objects_);
}

void GeometricSelfCollisionChecker::addCollisionExclusion(
    const std::string &link1, const std::string &link2) {
  collision_exclusion_pairs_.insert({link1, link2});
  collision_exclusion_pairs_.insert({link2, link1}); // 双方向

  // 内部チェッカーにも反映（既にオブジェクトが登録されている場合）
  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    if (object_map_[i].link_name == link1) {
      for (size_t j = 0; j < collision_objects_.size(); ++j) {
        if (object_map_[j].link_name == link2) {
          checker_.setIgnorePair(collision_objects_[i].id,
                                 collision_objects_[j].id);
        }
      }
    }
  }
}

bool GeometricSelfCollisionChecker::shouldSkipCollision(
    const std::string &link1, const std::string &link2) const {
  return collision_exclusion_pairs_.count({link1, link2}) > 0;
}

} // namespace simulation
