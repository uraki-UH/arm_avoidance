#include "simulation/robot/ode/ode_robot_builder.hpp"
#include "simulation/robot/geometry_management.hpp"

#include "simulation/robot/ode/ode_collision_manager.hpp"

#include <map>
#include <ode/ode.h>
#include <stdexcept>
#include <vector>
#include <utility>

namespace simulation {

OdeRobotBuilder::OdeRobotBuilder(dWorldID world, dSpaceID space,
                                 CollisionManager *collision_manager,
                                 MeshCache *mesh_cache)
    : world_(world), space_(space), collision_manager_(collision_manager),
      mesh_cache_(mesh_cache) {
  if (!world_ || !space_) {
    throw std::runtime_error(
        "OdeRobotBuilder: Invalid ODE world or space ID provided.");
  }
}

OdeRobotBuilder::~OdeRobotBuilder() {
  tri_mesh_data_cache_.clear();
}

// ヘルパー関数: Eigen::Isometry3dをdVector3（位置）とdMatrix3（回転）に変換
void eigenIsometryToOde(const Eigen::Isometry3d &iso, dVector3 pos,
                        dMatrix3 R) {
  pos[0] = iso.translation().x();
  pos[1] = iso.translation().y();
  pos[2] = iso.translation().z();
  pos[3] = 0; // dVector3 は4要素配列（末尾は使われない）

  // ODE の dMatrix3 は各行に本体の X, Y, Z 軸ベクトルを格納する。
  // Eigen の回転行列では、各列が X, Y, Z 軸ベクトルに対応するため、
  // Eigen の「列」を ODE の「行」にコピーする（＝転置）。
  Eigen::Matrix3d rot_mat = iso.rotation();
  // ODE Row 0 (X-axis)
  R[0] = rot_mat(0, 0);
  R[1] = rot_mat(1, 0);
  R[2] = rot_mat(2, 0);
  R[3] = 0;
  // ODE Row 1 (Y-axis)
  R[4] = rot_mat(0, 1);
  R[5] = rot_mat(1, 1);
  R[6] = rot_mat(2, 1);
  R[7] = 0;
  // ODE Row 2 (Z-axis)
  R[8] = rot_mat(0, 2);
  R[9] = rot_mat(1, 2);
  R[10] = rot_mat(2, 2);
  R[11] = 0;
}

// 慣性の設定（簡易的に慣性テンソルをそのまま渡す）
void OdeRobotBuilder::setOdeInertia(dBodyID body, const Inertial &inertial,
                                    const LinkProperties * /*link_props*/) {
  dMass m;
  dMassSetZero(&m);

  if (inertial.mass > 0) {
    const auto &I = inertial.inertia_tensor;
    // dMassSetParameters: m, cgx,cgy,cgz, Ixx,Iyy,Izz, Ixy, Ixz, Iyz
    dMassSetParameters(&m, inertial.mass, 0, 0, 0, I(0, 0), I(1, 1), I(2, 2),
                       I(0, 1), I(0, 2), I(1, 2));
  }
  dBodySetMass(body, &m);
}

dTriMeshDataID OdeRobotBuilder::createTriMeshData(
    robot_sim::simulation::MeshEntry &mesh_entry) {
  auto it = tri_mesh_data_cache_.find(&mesh_entry);
  if (it != tri_mesh_data_cache_.end()) {
    return it->second;
  }

  auto cache_entry = std::make_shared<TriMeshCacheEntry>();
  cache_entry->data_id = dGeomTriMeshDataCreate();
  if (!cache_entry->data_id) {
    return nullptr;
  }

  cache_entry->vertices.reserve(mesh_entry.original_vertices.size());
  for (float v : mesh_entry.original_vertices) {
    cache_entry->vertices.push_back(static_cast<dReal>(v));
  }
  cache_entry->indices.reserve(mesh_entry.indices.size());
  for (uint32_t idx : mesh_entry.indices) {
    cache_entry->indices.push_back(static_cast<dTriIndex>(idx));
  }

  const int vertex_count =
      static_cast<int>(cache_entry->vertices.size() / 3);
  const int index_count = static_cast<int>(cache_entry->indices.size());

#ifdef dDOUBLE
  dGeomTriMeshDataBuildDouble(cache_entry->data_id, cache_entry->vertices.data(),
                              sizeof(dReal) * 3, vertex_count,
                              cache_entry->indices.data(), index_count,
                              sizeof(dTriIndex) * 3);
#else
  dGeomTriMeshDataBuildSingle(cache_entry->data_id, cache_entry->vertices.data(),
                              sizeof(dReal) * 3, vertex_count,
                              cache_entry->indices.data(), index_count,
                              sizeof(dTriIndex) * 3);
#endif

  mesh_entry.backend_data = cache_entry;
  tri_mesh_data_cache_[&mesh_entry] = cache_entry->data_id;
  return cache_entry->data_id;
}

dGeomID OdeRobotBuilder::createOdeGeometry(const std::string &link_name,
                                           int geom_index, const Geometry &geom,
                                           dBodyID body,
                                           const Eigen::Isometry3d &origin,
                                           uint32_t categoryBits,
                                           uint32_t collideBits) {
  dGeomID ode_geom = nullptr;
  dVector3 geom_pos;
  dMatrix3 geom_R;
  eigenIsometryToOde(origin, geom_pos, geom_R);

  std::shared_ptr<robot_sim::simulation::MeshEntry> visual_mesh_override;
  Eigen::Vector3d visual_center_override = Eigen::Vector3d::Zero();

  switch (geom.type) {
  case GeometryType::BOX:
    ode_geom = dCreateBox(space_, geom.size.x(), geom.size.y(), geom.size.z());
    break;
  case GeometryType::CYLINDER:
    ode_geom = dCreateCylinder(space_, geom.size.x(), geom.size.y());
    break;
  case GeometryType::SPHERE:
    ode_geom = dCreateSphere(space_, geom.size.x());
    break;
  case GeometryType::MESH:
    if (mesh_cache_) {
      auto mesh_entry = mesh_cache_->getMesh(geom.mesh_filename, geom.size);
      if (mesh_entry) {
        visual_mesh_override = mesh_entry; // Store for visual fallback
        if (use_mesh_) {
          dTriMeshDataID mesh_data_id = createTriMeshData(*mesh_entry);
          if (mesh_data_id) {
            ode_geom =
                dCreateTriMesh(space_, mesh_data_id, nullptr, nullptr, nullptr);
          }
        } else {
          // Simplify to Box
          if (!mesh_entry->original_vertices.empty()) {
            ::simulation::MeshData mesh_data;
            mesh_data.vertices = mesh_entry->original_vertices;
            mesh_data.indices = mesh_entry->indices;
            Eigen::Vector3d min_corner, max_corner;
            robot_sim::simulation::GeometrySimplifier::calculateAABB(
                mesh_data, min_corner, max_corner);
            Eigen::Vector3d size = robot_sim::simulation::GeometrySimplifier::getSize(min_corner, max_corner);
            Eigen::Vector3d center = robot_sim::simulation::GeometrySimplifier::getCenter(min_corner, max_corner);
            visual_center_override = center;

            ode_geom = dCreateBox(space_, size.x(), size.y(), size.z());
            
            // Adjust geometry origin by mesh center offset
            Eigen::Isometry3d adjusted_origin = origin;
            adjusted_origin.translate(center);
            eigenIsometryToOde(adjusted_origin, geom_pos, geom_R);
          }
        }
      } else {
        std::cerr << "WARNING: Failed to load mesh '" << geom.mesh_filename
                  << "'" << std::endl;
      }
    } else {
      std::cerr << "WARNING: MeshCache is not provided. Skipping mesh '"
                << geom.mesh_filename << "'" << std::endl;
    }
    break;
  default:
    std::cerr << "WARNING: Unsupported geometry type. Skipping." << std::endl;
    break;
  }

  if (ode_geom) {
    if (body) {
      dGeomSetBody(ode_geom, body);
      dGeomSetOffsetPosition(ode_geom, geom_pos[0], geom_pos[1], geom_pos[2]);
      dGeomSetOffsetRotation(ode_geom, geom_R);
    } else {
      // ボディがない場合、リンクのワールド座標に基づいて配置する
      // (origin_in_body はこの場合、リンクのワールド座標 *
      // collision_prop.origin になっている必要がある)
      dGeomSetPosition(ode_geom, geom_pos[0], geom_pos[1], geom_pos[2]);
      dGeomSetRotation(ode_geom, geom_R);
    }
    dGeomSetCategoryBits(ode_geom, categoryBits);
    dGeomSetCollideBits(ode_geom, collideBits);

    if (collision_manager_) {
      std::string geom_name = link_name + "_geom_" + std::to_string(geom_index);
      collision_manager_->registerGeom(ode_geom, geom_name, categoryBits,
                                       collideBits, 1.0, 0.1, visual_mesh_override,
                                       visual_center_override);
    }
  }

  return ode_geom;
}

// cpp
void OdeRobotBuilder::buildRecursive(
    const RobotModel &model, const std::string &current_link_name,
    [[maybe_unused]] dBodyID parent_body_id,
    const Eigen::Isometry3d
        &current_link_origin_world, // 現在リンク原点（link frame / joint
                                    // frame）のワールド変換
    std::map<std::string, OdeRobotComponent> &components, uint32_t categoryBits,
    uint32_t collideBits) {
  const LinkProperties *link_props = model.getLink(current_link_name);
  if (!link_props) {
    throw std::runtime_error("Link not found in model: " + current_link_name);
  }

  // current_link_origin_world は「このリンクのリンク原点（link frame / joint
  // frame）」のワールド変換。
  Eigen::Isometry3d link_origin_world = current_link_origin_world;

  // ボディ（慣性中心）を作成し、COMワールド座標を設定
  dBodyID current_body_id = nullptr;
  OdeRobotComponent component;
  component.link_name = current_link_name;

  dVector3 pos;
  dMatrix3 R;

  if (link_props->inertial.mass > 0) {
    current_body_id = dBodyCreate(world_);
    component.body_id = current_body_id;

    setOdeInertia(current_body_id, link_props->inertial, link_props);

    // body のワールド変換 = link_origin_world * inertial.origin
    Eigen::Isometry3d body_com_world =
        link_origin_world * link_props->inertial.origin;

    eigenIsometryToOde(body_com_world, pos, R);
    dBodySetPosition(current_body_id, pos[0], pos[1], pos[2]);
    dBodySetRotation(current_body_id, R);
  } else {
    // 質量ゼロの場合、リンク原点の位置をデバッグ参考用に計算
    eigenIsometryToOde(link_origin_world, pos, R);
  }

  // ジオメトリ作成
  for (size_t i = 0; i < link_props->collisions.size(); ++i) {
    const auto &collision_prop = link_props->collisions[i];
    // ジオメトリの原点
    Eigen::Isometry3d origin_in_body;
    if (current_body_id) {
      // ボディ（COM）基準
      origin_in_body =
          link_props->inertial.origin.inverse() * collision_prop.origin;
    } else {
      // ボディがない場合：リンクのワールド座標基準
      origin_in_body = link_origin_world * collision_prop.origin;
    }

    dGeomID geom = createOdeGeometry(current_link_name, static_cast<int>(i),
                                     collision_prop.geometry, current_body_id,
                                     origin_in_body, categoryBits, collideBits);
    if (geom) {
      component.geom_ids.push_back(geom);
    }
  }

  components[current_link_name] = component;

  // 子ジョイント（このリンクを親とするジョイント）を列挙して再帰的に処理
  for (const auto &joint_pair : model.getJoints()) {
    const JointProperties &joint_props = joint_pair.second;
    if (joint_props.parent_link == current_link_name) {
      // joint frame のワールド変換（parent link origin -> joint frame）
      Eigen::Isometry3d joint_frame_world =
          link_origin_world * joint_props.origin;
      // 子リンク原点ワールドは joint_frame_world（URDF の規約）
      Eigen::Isometry3d child_link_origin_world = joint_frame_world;

      // 先に子リンクを再帰的に作成（これが子の body を生成する）
      buildRecursive(model, joint_props.child_link, current_body_id,
                     child_link_origin_world, components, categoryBits,
                     collideBits);

      // 子のコンポーネントを取得してジョイントを作成・接続
      auto it = components.find(joint_props.child_link);
      if (it == components.end()) {
        throw std::runtime_error("Failed to create child link component: " +
                                 joint_props.child_link);
      }
      dBodyID child_body = it->second.body_id;

      // ジョイント作成 (変数はここで宣言し、子のボディがない場合はスキップ)
      if (!child_body) {
        // std::cout << "[DEBUG Builder] Skip joint '" << joint_props.name
        //           << "' because child link '" << joint_props.child_link
        //           << "' has no ODE body." << std::endl;
        continue;
      }
      dJointID ode_joint = nullptr;

      // 計算用デバッグ情報
      Eigen::Vector3d anchor_world = joint_frame_world.translation();
      Eigen::Vector3d axis_world =
          -(joint_frame_world.rotation() * joint_props.axis);
      // 正規化（ゼロベクトル回避）
      double axis_norm = axis_world.norm();
      if (axis_norm > 1e-12)
        axis_world /= axis_norm;

      // std::cout << "[DEBUG Builder] Joint '" << joint_props.name << "'
      // parent='"
      //           << joint_props.parent_link << "' child='"
      //           << joint_props.child_link
      //           << "' type=" << static_cast<int>(joint_props.type) <<
      //           std::endl;
      // std::cout << "[DEBUG Builder]  joint_frame_world pos="
      //           << anchor_world.transpose() << " rot=\n"
      //           << joint_frame_world.rotation() << std::endl;
      // std::cout << "[DEBUG Builder]  anchor_world=" <<
      // anchor_world.transpose()
      //           << " axis_world=" << axis_world.transpose() << std::endl;

      dBodyID parent_for_attach = current_body_id;
      if (joint_props.type == kinematics::JointType::Revolute) {
        ode_joint = dJointCreateHinge(world_, nullptr);
        dJointAttach(ode_joint, parent_for_attach, child_body);
        dJointSetHingeAnchor(ode_joint, anchor_world.x(), anchor_world.y(),
                             anchor_world.z());
        dJointSetHingeAxis(ode_joint, axis_world.x(), axis_world.y(),
                           axis_world.z());
        // Set default motor parameters for Hinge joints
        dJointSetHingeParam(ode_joint, dParamVel,
                            0.0); // Default target velocity
        double fMax = (joint_props.limits.effort > 0) ? joint_props.limits.effort : 100.0;
        dJointSetHingeParam(ode_joint, dParamFMax, fMax);
        // Check for joint limits in URDF and apply if they exist
        if (joint_props.limits.lower <
            joint_props.limits.upper) { // Check for valid limits
          dJointSetHingeParam(ode_joint, dParamLoStop,
                              joint_props.limits.lower);
          dJointSetHingeParam(ode_joint, dParamHiStop,
                              joint_props.limits.upper);
        }
        // std::cout << "[DEBUG Builder] Creating Hinge joint for '"
        //           << joint_props.name << "'. ID: " << (void *)ode_joint
        //           << std::endl; // Debug output for joint ID
        // std::cout << "[DEBUG Builder] Joint " << ode_joint
        //           << " attached to parent " << parent_for_attach
        //           << " and child " << child_body << std::endl;
      } else if (joint_props.type == kinematics::JointType::Fixed) {
        ode_joint = dJointCreateFixed(world_, nullptr);
        dJointAttach(ode_joint, parent_for_attach, child_body);
        dJointSetFixed(ode_joint);
        // std::cout << "[DEBUG Builder] Creating Fixed joint for '"
        //           << joint_props.name << "'. ID: " << (void *)ode_joint
        //           << std::endl; // Debug output for joint ID
      } else if (joint_props.type == kinematics::JointType::Prismatic) {
        ode_joint = dJointCreateSlider(world_, nullptr);
        dJointAttach(ode_joint, parent_for_attach, child_body);
        dJointSetSliderAxis(ode_joint, axis_world.x(), axis_world.y(),
                            axis_world.z());
        // Set default motor parameters for Slider joints
        dJointSetSliderParam(ode_joint, dParamVel,
                             0.0); // Default target velocity
        double fMax = (joint_props.limits.effort > 0) ? joint_props.limits.effort : 100.0;
        dJointSetSliderParam(ode_joint, dParamFMax, fMax);
        dJointSetSliderParam(ode_joint, dParamLoStop, joint_props.limits.lower);
        dJointSetSliderParam(ode_joint, dParamHiStop, joint_props.limits.upper);
      }

      // 保存（子のコンポーネントにジョイントIDを保持）
      if (ode_joint) {
        components[joint_props.child_link].joint_id = ode_joint;
        components[joint_props.child_link].joint_name = joint_props.name;
      }
    }
  }
}

std::map<std::string, OdeRobotComponent> OdeRobotBuilder::build(
    const RobotModel &model, uint32_t categoryBits, uint32_t collideBits,
    const Eigen::Vector3d &base_position,
    const Eigen::Quaterniond &base_orientation, bool fix_root_link) {
  std::map<std::string, OdeRobotComponent> components;

  if (model.getLinks().empty()) {
    throw std::runtime_error("Robot model has no links.");
  }

  Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
  base_transform.translate(base_position);
  base_transform.rotate(base_orientation);

  std::string root_link_name = model.getRootLinkName();
  if (root_link_name.empty()) {
    throw std::runtime_error(
        "Could not determine root link of the robot model.");
  }

  // 最初の呼び出しでは root のリンク原点ワールド変換を base_transform
  // として渡す
  buildRecursive(model, root_link_name, nullptr, base_transform, components,
                 categoryBits, collideBits);

  if (fix_root_link) {
    auto it = components.find(root_link_name);
    if (it != components.end() && it->second.body_id) {
      dJointID fixed_joint = dJointCreateFixed(world_, 0);
      dJointAttach(fixed_joint, it->second.body_id, 0);
      dJointSetFixed(fixed_joint);
    }
  }

  return components;
}

} // namespace simulation
