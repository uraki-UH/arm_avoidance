#include "collision/geometric_self_collision_checker.hpp"
#include "collision/self_collision_policy.hpp"
#include <cmath>
#include <fstream>
#include <cstdint>
#include <stdexcept>
#include <tuple>
#include "common/resource_utils.hpp"
#include "robot_model/stl_loader.hpp"
#include <algorithm>
#include <iostream>
#include <unordered_set>

#ifdef USE_FCL
#include "collision/fcl/solid_voxel_geometry.hpp"
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#endif

namespace simulation {

GeometricSelfCollisionChecker::GeometricSelfCollisionChecker(
    const RobotModel &model, const kinematics::KinematicChain &chain,
    bool enable_fcl_backend, double voxel_size)
    : chain_(chain)
#ifdef USE_FCL
      , use_fcl_backend_(enable_fcl_backend)
#endif
{

  if (!std::isfinite(voxel_size) || voxel_size < 0.0) {
    throw std::invalid_argument("Invalid self-collision voxel size");
  }
#ifndef USE_FCL
  if (voxel_size > 0.0) throw std::runtime_error("Solid voxel collision requires FCL support");
#else
  if (voxel_size > 0.0 && !enable_fcl_backend) {
    throw std::invalid_argument("Solid voxel collision requires the complete geometry backend");
  }
#endif

#ifdef USE_FCL
  enable_solid_containment_ = voxel_size > 0.0;
  using solid_cache_key = std::tuple<std::string, double, double, double, double>;
  std::map<solid_cache_key, std::shared_ptr<const collision::solid_voxel_geometry>> solid_geometry_cache;
#endif

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

    const auto *props = model.getLink(name);
    if (!props)
      continue;

    for (const auto &col : props->collisions) {
      collision::SelfCollisionChecker::CollisionObject obj;
      obj.id = (int)collision_objects_.size();
      obj.name = name + "_" + col.name;
      obj.is_fixed_to_base = (base_fixed_links.count(name) > 0);

#ifdef USE_FCL
      std::shared_ptr<const collision::solid_voxel_geometry> solid_geometry;
#endif
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
          // 既存STLローダーの空返却・短い読込みを、検査不能のまま通過させない事前確認。
          std::ifstream mesh_input(resolved_mesh, std::ios::binary | std::ios::ate);
          if (!mesh_input || mesh_input.tellg() < 84 || !col.geometry.size.allFinite() ||
              (col.geometry.size.array() == 0.0).any()) {
            throw std::runtime_error("Invalid collision STL file or scale");
          }
          const auto num_mesh_bytes = static_cast<std::uint64_t>(mesh_input.tellg());
          mesh_input.seekg(80);
          std::uint32_t num_mesh_triangles = 0;
          mesh_input.read(reinterpret_cast<char *>(&num_mesh_triangles), sizeof(num_mesh_triangles));
          if (!mesh_input || num_mesh_triangles == 0 || num_mesh_triangles > 5000000 ||
              num_mesh_bytes != 84ULL + 50ULL * num_mesh_triangles) {
            throw std::runtime_error("Truncated or invalid collision STL");
          }
          MeshData mesh_data;
#ifdef USE_FCL
          mesh_data = collision::load_solid_voxel_stl(resolved_mesh, col.geometry.size);
          has_mesh_geometry_ = true;
#else
          throw std::runtime_error("Mesh collision requires FCL support");
#endif
          if (mesh_data.vertices.empty() || mesh_data.vertices.size() % 3 != 0 ||
              mesh_data.indices.size() != 3ULL * num_mesh_triangles ||
              !std::all_of(mesh_data.vertices.begin(), mesh_data.vertices.end(),
                           [](float value) { return std::isfinite(value); }) ||
              !std::all_of(mesh_data.indices.begin(), mesh_data.indices.end(),
                           [&mesh_data](std::uint32_t idx) { return idx < mesh_data.vertices.size() / 3; })) {
            throw std::runtime_error("Invalid collision mesh arrays");
          }
#ifdef USE_FCL
          if (enable_solid_containment_) {
            const solid_cache_key key{resolved_mesh, col.geometry.size.x(),
                col.geometry.size.y(), col.geometry.size.z(), voxel_size};
            const auto found = solid_geometry_cache.find(key);
            if (found != solid_geometry_cache.end()) {
              solid_geometry = found->second;
            } else {
              solid_geometry = std::make_shared<const collision::solid_voxel_geometry>(
                  collision::build_solid_voxel_geometry(mesh_data, voxel_size));
              if (solid_geometry->surface_component_points.empty()) {
                throw std::runtime_error("Missing collision surface component seeds");
              }
              solid_geometry_cache.emplace(key, solid_geometry);
            }
          }
#endif
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

        } catch (const std::exception &e) {
          throw std::runtime_error("Cannot load collision mesh " + resolved_mesh + ": " + e.what());
        }
#ifdef USE_FCL
        if (!use_fcl_backend_) {
          throw std::runtime_error("Mesh collision requires the complete geometry backend");
        }
#else
        throw std::runtime_error("Mesh collision requires FCL support");
#endif
      }

      if (valid) {
        collision_objects_.push_back(obj);
        ObjectLinkMap map;
        map.link_index = (int)link_idx;
        map.link_name = name;
        map.local_tf = col.origin;
        object_map_.push_back(map);

#ifdef USE_FCL
        if (use_fcl_backend_) {
          int fcl_id = -1;
          if (obj.type == collision::SelfCollisionChecker::ShapeType::SPHERE) {
            fcl_id = fcl_detector_.addRobotLink(obj.sphere);
          } else if (obj.type == collision::SelfCollisionChecker::ShapeType::BOX) {
            fcl_id = fcl_detector_.addRobotLink(obj.box);
          } else if (obj.type == collision::SelfCollisionChecker::ShapeType::CAPSULE) {
            fcl_id = fcl_detector_.addRobotLink(obj.capsule);
          } else if (obj.type == collision::SelfCollisionChecker::ShapeType::MESH) {
            // 狭い隙間の細密セル比較を避ける元メッシュ表面の交差判定
            fcl_id = fcl_detector_.addRobotMeshLink(col.geometry.mesh_filename,
                                                    col.geometry.size);
          }
          if (fcl_id < 0 || !fcl_detector_.getRobotLink(fcl_id)) {
            throw std::runtime_error("Cannot create collision object for " + name);
          }
          object_fcl_ids_.push_back(fcl_id);
          solid_geometries_.push_back(std::move(solid_geometry));
        }
#endif
      }
    }
  }

  // 固定部品を同一剛体として扱い、祖父母・兄弟の一律除外を禁止。
  for (const auto &pair : collect_self_collision_exclusions(model)) {
    addCollisionExclusion(pair.first, pair.second);
  }

  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    for (size_t j = i + 1; j < collision_objects_.size(); ++j) {
      const std::string &n1 = object_map_[i].link_name;
      const std::string &n2 = object_map_[j].link_name;
      if (n1 == n2 || shouldSkipCollision(n1, n2)) {
        checker_.setIgnorePair(collision_objects_[i].id,
                               collision_objects_[j].id);
#ifdef USE_FCL
          if (use_fcl_backend_ && i < object_fcl_ids_.size() &&
              j < object_fcl_ids_.size() && object_fcl_ids_[i] != -1 &&
              object_fcl_ids_[j] != -1) {
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
    if (it == link_transforms.end()) {
      throw std::runtime_error("Missing collision link transform: " + map.link_name);
    }
    
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
    if (use_fcl_backend_) {
      int fcl_id = object_fcl_ids_[i];
      if (fcl_id != -1) {
        if (obj.type == collision::SelfCollisionChecker::ShapeType::CAPSULE) {
          fcl_detector_.updateRobotLinkCapsule(fcl_id, obj.capsule.a, obj.capsule.b);
        } else {
          fcl_detector_.updateRobotLinkPose(fcl_id, obj_tf);
        }
      }
    }
#endif
  }
}

#ifdef USE_FCL
bool GeometricSelfCollisionChecker::has_point_inside(
    std::size_t obj_idx, const Eigen::Vector3d &point) const {
  if (!point.allFinite()) throw std::runtime_error("Nonfinite collision component point");
  const auto &geometry = solid_geometries_.at(obj_idx);
  if (geometry) {
    const auto object = getFCLObject(static_cast<int>(obj_idx));
    const Eigen::Vector3d local = object->getTransform().inverse() * point;
    const auto *cell = geometry->tree->search(local.x(), local.y(), local.z());
    return cell && geometry->tree->isNodeOccupied(cell);
  }
  const auto &object = collision_objects_.at(obj_idx);
  using shape_type = collision::SelfCollisionChecker::ShapeType;
  if (object.type == shape_type::SPHERE) {
    return (point - object.sphere.center).squaredNorm() <=
        object.sphere.radius * object.sphere.radius;
  }
  if (object.type == shape_type::BOX) {
    const Eigen::Vector3d local = object.box.rotation.transpose() * (point - object.box.center);
    return (local.cwiseAbs().array() <= object.box.extents.array()).all();
  }
  if (object.type == shape_type::CAPSULE) {
    const Eigen::Vector3d axis = object.capsule.b - object.capsule.a;
    const double length_squared = axis.squaredNorm();
    const double ratio = length_squared > 0.0
        ? std::clamp((point - object.capsule.a).dot(axis) / length_squared, 0.0, 1.0) : 0.0;
    return (point - object.capsule.a - ratio * axis).squaredNorm() <=
        object.capsule.radius * object.capsule.radius;
  }
  throw std::runtime_error("Missing solid collision containment geometry");
}

bool GeometricSelfCollisionChecker::has_solid_containment(
    std::size_t first_idx, std::size_t second_idx) const {
  if (!solid_geometries_.at(first_idx) && !solid_geometries_.at(second_idx)) return false;
  const auto first = getFCLObject(static_cast<int>(first_idx));
  const auto second = getFCLObject(static_cast<int>(second_idx));
  if (!first || !second) throw std::runtime_error("Missing solid collision object");
  if (!collision::has_collision_bounds_overlap(*first, *second)) return false;
  const auto has_component_inside = [this](std::size_t source_idx, std::size_t target_idx) {
    const auto source = getFCLObject(static_cast<int>(source_idx));
    const auto &geometry = solid_geometries_.at(source_idx);
    if (geometry) {
      // 非退化な各表面成分の元頂点。別成分だけの完全内包も検査対象
      for (const auto &point : geometry->surface_component_points) {
        if (has_point_inside(target_idx, source->getTransform() * point)) return true;
      }
      return false;
    }
    // 凸primitiveの代表内部点
    return has_point_inside(target_idx, source->getTranslation());
  };
  return has_component_inside(first_idx, second_idx) ||
      has_component_inside(second_idx, first_idx);
}
#endif

bool GeometricSelfCollisionChecker::checkCollision() {
#ifdef USE_FCL
  if (use_fcl_backend_ && strict_mode_) {
    if (fcl_detector_.checkSelfCollision(fcl_ignore_pairs_, true)) return true;
    if (enable_solid_containment_) {
      for (std::size_t first_idx = 0; first_idx < collision_objects_.size(); ++first_idx) {
        for (std::size_t second_idx = first_idx + 1; second_idx < collision_objects_.size(); ++second_idx) {
          const auto &first_name = object_map_[first_idx].link_name;
          const auto &second_name = object_map_[second_idx].link_name;
          if (first_name == second_name || shouldSkipCollision(first_name, second_name)) continue;
          if (has_solid_containment(first_idx, second_idx)) return true;
        }
      }
    }
    return false;
  }
#endif
  return checker_.checkSelfCollision(collision_objects_);
}

std::vector<std::pair<std::string, std::string>>
GeometricSelfCollisionChecker::collectSelfCollisionPairs() const {
  std::vector<std::pair<std::string, std::string>> pairs;

  for (size_t i = 0; i < collision_objects_.size(); ++i) {
    for (size_t j = i + 1; j < collision_objects_.size(); ++j) {
      const auto &obj_i = collision_objects_[i];
      const auto &obj_j = collision_objects_[j];
      const auto &link_i = object_map_[i].link_name;
      const auto &link_j = object_map_[j].link_name;

      if (link_i == link_j || shouldSkipCollision(link_i, link_j)) {
        continue;
      }

      bool is_colliding = checker_.checkPair(obj_i, obj_j);
#ifdef USE_FCL
      if (use_fcl_backend_ && strict_mode_) {
        const auto first = getFCLObject(static_cast<int>(i));
        const auto second = getFCLObject(static_cast<int>(j));
        if (first && second) {
          if (!collision::has_collision_bounds_overlap(*first, *second)) continue;
          fcl::CollisionRequest<double> request;
          fcl::CollisionResult<double> result;
          fcl::collide(first.get(), second.get(), request, result);
          is_colliding = result.isCollision() ||
              (enable_solid_containment_ && has_solid_containment(i, j));
        }
      }
#endif
      if (is_colliding) {
        std::string a = link_i;
        std::string b = link_j;
        if (a > b)
          std::swap(a, b);
        pairs.emplace_back(a, b);
      }
    }
  }

  std::sort(pairs.begin(), pairs.end());
  pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
  return pairs;
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
          if (use_fcl_backend_ && i < object_fcl_ids_.size() &&
              j < object_fcl_ids_.size() && object_fcl_ids_[i] != -1 &&
              object_fcl_ids_[j] != -1) {
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
