#include "collision/geometric_self_collision_checker.hpp"
#include "common/resource_utils.hpp"
#include "robot_model/stl_loader.hpp"
#include <iostream>
#include <unordered_set>

#ifdef USE_FCL
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#endif

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
  std::function<void(const std::string &)> add_children;
  add_children = [&](const std::string &parent_name) {
    for (const auto &[joint_name, joint] : all_joints) {
      if (joint.parent_link == parent_name) {
        const std::string &child_name = joint.child_link;
        if (link_name_to_index.find(child_name) == link_name_to_index.end()) {
          link_names.push_back(child_name);
          link_name_to_index[child_name] = (int)link_names.size() - 1;
          add_children(child_name);
        }
      }
    }
  };
  add_children(root_name);

  for (const auto &[joint_name, joint] : all_joints) {
    fixed_link_info_[joint.child_link] =
        std::make_pair(joint.parent_link, joint.origin);

    if (joint.type == kinematics::JointType::Fixed) {
      fixed_link_connectivity_[joint.child_link] = joint.parent_link;
    }
  }

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

  std::cout << "[GeometricChecker] Registered " << link_names.size()
            << " links for collision checking:" << std::endl;
  for (size_t i = 0; i < link_names.size(); ++i) {
    std::cout << "  [" << i << "] " << link_names[i] << std::endl;
  }

  for (size_t link_idx = 0; link_idx < link_names.size(); ++link_idx) {
    const std::string &name = link_names[link_idx];

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
        obj.box.extents =
            col.geometry.size * 0.5 + Eigen::Vector3d(0.002, 0.002, 0.002);
        obj.box.center = Eigen::Vector3d::Zero();
        obj.box.rotation = Eigen::Matrix3d::Identity();
        valid = true;
      } else if (col.geometry.type == GeometryType::MESH) {
        std::string resolved_mesh =
            robot_sim::common::resolvePath(col.geometry.mesh_filename);
        std::cout << "[GeometricChecker] Loading STL: " << resolved_mesh
                  << std::endl;

        try {
          MeshData mesh_data =
              StlLoader::loadBinaryStl(resolved_mesh, col.geometry.size);
          obj.type = collision::SelfCollisionChecker::ShapeType::MESH;

          for (size_t i = 0; i < mesh_data.indices.size(); i += 3) {
            collision::Triangle tri;
            tri.v0 = Eigen::Vector3d(mesh_data.vertices[mesh_data.indices[i] * 3],
                                     mesh_data.vertices[mesh_data.indices[i] * 3 + 1],
                                     mesh_data.vertices[mesh_data.indices[i] * 3 + 2]);
            tri.v1 = Eigen::Vector3d(mesh_data.vertices[mesh_data.indices[i + 1] * 3],
                                     mesh_data.vertices[mesh_data.indices[i + 1] * 3 + 1],
                                     mesh_data.vertices[mesh_data.indices[i + 1] * 3 + 2]);
            tri.v2 = Eigen::Vector3d(mesh_data.vertices[mesh_data.indices[i + 2] * 3],
                                     mesh_data.vertices[mesh_data.indices[i + 2] * 3 + 1],
                                     mesh_data.vertices[mesh_data.indices[i + 2] * 3 + 2]);
            obj.mesh.triangles.push_back(tri);
            obj.mesh.bounds.expand(tri.v0);
            obj.mesh.bounds.expand(tri.v1);
            obj.mesh.bounds.expand(tri.v2);
          }
          valid = true;

#ifdef USE_FCL
          int fcl_id = fcl_detector_.addRobotMeshLink(col.geometry.mesh_filename, col.geometry.size);
          object_fcl_ids_.push_back(fcl_id);
#endif
        } catch (const std::exception &e) {
          std::cerr << "[GeometricChecker] Error loading mesh " << resolved_mesh
                    << ": " << e.what() << ". Approximating as Sphere."
                    << std::endl;
          obj.type = collision::SelfCollisionChecker::ShapeType::SPHERE;
          obj.sphere.radius = 0.05;
          obj.sphere.center = Eigen::Vector3d::Zero();
          valid = true;
#ifdef USE_FCL
          object_fcl_ids_.push_back(-1);
#endif
        }
      }

      if (valid) {
        collision_objects_.push_back(obj);
        ObjectLinkMap map;
        map.link_index = (int)link_idx;
        map.link_name = name;
        map.local_tf = col.origin;
        object_map_.push_back(map);

#ifdef USE_FCL
        if (obj.type != collision::SelfCollisionChecker::ShapeType::MESH) {
          int fcl_id = fcl_detector_.addRobotLink(obj.capsule);
          object_fcl_ids_.push_back(fcl_id);
        }
#endif
      }
    }
  }

  for (const auto &[joint_name, joint] : all_joints) {
    addCollisionExclusion(joint.parent_link, joint.child_link);
  }

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

  for (const auto &[joint_name, joint] : all_joints) {
    const std::string &B = joint.parent_link;
    const std::string &C = joint.child_link;
    for (const auto &[j_prev_name, j_prev] : all_joints) {
      if (j_prev.child_link == B) {
        const std::string &A = j_prev.parent_link;
        addCollisionExclusion(A, C);
      }
    }
  }

  for (const auto &[j1_name, j1] : all_joints) {
    for (const auto &[j2_name, j2] : all_joints) {
      if (j1_name != j2_name && j1.parent_link == j2.parent_link) {
        addCollisionExclusion(j1.child_link, j2.child_link);
      }
    }
  }

  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    for (size_t j = i + 1; j < collision_objects_.size(); ++j) {
      const std::string &n1 = object_map_[i].link_name;
      const std::string &n2 = object_map_[j].link_name;
      if (n1 == n2 || shouldSkipCollision(n1, n2)) {
        checker_.setIgnorePair(collision_objects_[i].id,
                               collision_objects_[j].id);
#ifdef USE_FCL
        if (object_fcl_ids_[i] != -1 && object_fcl_ids_[j] != -1) {
          fcl_ignore_pairs_.push_back({object_fcl_ids_[i], object_fcl_ids_[j]});
        }
#endif
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

  std::map<std::string, Eigen::Isometry3d> link_transforms;
  chain_.buildAllLinkTransforms(positions, orientations, fixed_link_info_,
                                link_transforms);

  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    auto &obj = collision_objects_[i];
    const auto &map = object_map_[i];

    auto it = link_transforms.find(map.link_name);
    if (it == link_transforms.end()) continue;
    
    Eigen::Isometry3d obj_tf = it->second * map.local_tf;

    if (obj.type == collision::SelfCollisionChecker::ShapeType::SPHERE) {
      obj.sphere.center = obj_tf.translation();
    } else if (obj.type == collision::SelfCollisionChecker::ShapeType::CAPSULE) {
      double len = (obj.capsule.b - obj.capsule.a).norm();
      obj.capsule.a = obj_tf * Eigen::Vector3d(0, 0, -len / 2.0);
      obj.capsule.b = obj_tf * Eigen::Vector3d(0, 0, len / 2.0);
    } else if (obj.type == collision::SelfCollisionChecker::ShapeType::BOX) {
      obj.box.center = obj_tf.translation();
      obj.box.rotation = obj_tf.rotation();
    }

#ifdef USE_FCL
    int fcl_id = object_fcl_ids_[i];
    if (fcl_id != -1) {
      if (obj.type == collision::SelfCollisionChecker::ShapeType::CAPSULE) {
        fcl_detector_.updateRobotLinkCapsule(fcl_id, obj.capsule.a, obj.capsule.b);
      } else {
        fcl_detector_.updateRobotLinkPose(fcl_id, obj_tf);
      }
    }
#endif
  }
}

bool GeometricSelfCollisionChecker::checkCollision() {
#ifdef USE_FCL
  if (strict_mode_) {
    return fcl_detector_.checkSelfCollision(fcl_ignore_pairs_);
  }
#endif
  return checker_.checkSelfCollision(collision_objects_);
}

void GeometricSelfCollisionChecker::addCollisionExclusion(
    const std::string &link1, const std::string &link2) {
  collision_exclusion_pairs_.insert({link1, link2});
  collision_exclusion_pairs_.insert({link2, link1});

  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    if (object_map_[i].link_name == link1) {
      for (size_t j = 0; j < collision_objects_.size(); ++j) {
        if (object_map_[j].link_name == link2) {
          checker_.setIgnorePair(collision_objects_[i].id,
                                 collision_objects_[j].id);
#ifdef USE_FCL
          if (object_fcl_ids_[i] != -1 && object_fcl_ids_[j] != -1) {
            fcl_ignore_pairs_.push_back({object_fcl_ids_[i], object_fcl_ids_[j]});
          }
#endif
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
