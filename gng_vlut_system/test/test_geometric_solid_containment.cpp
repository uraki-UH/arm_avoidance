#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "collision/geometric_self_collision_checker.hpp"
#include "common/parallel_queries.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/stl_loader.hpp"

namespace {

simulation::MeshData make_box_mesh(const Eigen::Vector3d &min_point,
                                   const Eigen::Vector3d &max_point) {
  simulation::MeshData mesh;
  for (int idx = 0; idx < 8; ++idx) {
    for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
      mesh.vertices.push_back(static_cast<float>(
          (idx & (1 << axis_idx)) ? max_point[axis_idx] : min_point[axis_idx]));
    }
  }
  mesh.indices = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6,
                  0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7,
                  0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  return mesh;
}

void append_mesh(simulation::MeshData &target,
                 const simulation::MeshData &source) {
  const auto offset = static_cast<std::uint32_t>(target.vertices.size() / 3);
  target.vertices.insert(target.vertices.end(), source.vertices.begin(),
                         source.vertices.end());
  for (const auto idx : source.indices) target.indices.push_back(idx + offset);
}

// 各試験だけで使用する小さな閉面STLと、例外経路を含む後始末
class temporary_binary_stl {
 public:
  explicit temporary_binary_stl(const simulation::MeshData &mesh) {
    const std::string pattern =
        (std::filesystem::temp_directory_path() / "solid_containment_XXXXXX").string();
    std::vector<char> path_chars(pattern.begin(), pattern.end());
    path_chars.push_back('\0');
    const int descriptor = mkstemp(path_chars.data());
    if (descriptor < 0) throw std::runtime_error("一時STLの作成失敗");
    close(descriptor);
    path = path_chars.data();
    try {
      std::ofstream stream(path, std::ios::binary);
      const std::array<char, 80> header{};
      stream.write(header.data(), header.size());
      const auto num_triangles = static_cast<std::uint32_t>(mesh.indices.size() / 3);
      stream.write(reinterpret_cast<const char *>(&num_triangles), sizeof(num_triangles));
      for (std::size_t idx = 0; idx < mesh.indices.size(); idx += 3) {
        const std::array<float, 3> normal{};
        stream.write(reinterpret_cast<const char *>(normal.data()), sizeof(normal));
        for (std::size_t corner_idx = 0; corner_idx < 3; ++corner_idx) {
          const auto vertex_idx = mesh.indices[idx + corner_idx];
          stream.write(reinterpret_cast<const char *>(mesh.vertices.data() +
                           static_cast<std::size_t>(vertex_idx) * 3), sizeof(float) * 3);
        }
        const std::uint16_t attribute = 0;
        stream.write(reinterpret_cast<const char *>(&attribute), sizeof(attribute));
      }
      stream.close();
      if (!stream) throw std::runtime_error("一時STLの書込み失敗");
    } catch (...) {
      std::error_code error;
      std::filesystem::remove(path, error);
      throw;
    }
  }

  temporary_binary_stl(const temporary_binary_stl &) = delete;
  temporary_binary_stl &operator=(const temporary_binary_stl &) = delete;

  ~temporary_binary_stl() {
    std::error_code error;
    std::filesystem::remove(path, error);
  }

  std::string path;
};

simulation::Collision make_mesh_shape(const temporary_binary_stl &file) {
  simulation::Collision shape;
  shape.name = "mesh";
  shape.geometry.type = simulation::GeometryType::MESH;
  shape.geometry.size = Eigen::Vector3d::Ones();
  shape.geometry.mesh_filename = file.path;
  return shape;
}

simulation::Collision make_primitive_shape(simulation::GeometryType type,
                                           const Eigen::Vector3d &size) {
  simulation::Collision shape;
  shape.name = "primitive";
  shape.geometry.type = type;
  shape.geometry.size = size;
  return shape;
}

// 二つの可動関節による、試験対象二形状の構造的隣接除外の回避
simulation::RobotModel make_model(const simulation::Collision &body_shape,
                                  const simulation::Collision &tool_shape) {
  simulation::RobotModel model;
  model.setRootLinkName("body");
  for (const auto *name : {"body", "middle", "tool"}) {
    simulation::LinkProperties link;
    link.name = name;
    if (link.name == "body") link.collisions.push_back(body_shape);
    if (link.name == "tool") link.collisions.push_back(tool_shape);
    model.addLink(link);
  }
  for (const auto &pair : {std::make_pair("body", "middle"),
                           std::make_pair("middle", "tool")}) {
    simulation::JointProperties joint;
    joint.name = std::string(pair.first) + "_to_" + pair.second;
    joint.parent_link = pair.first;
    joint.child_link = pair.second;
    joint.type = kinematics::JointType::Revolute;
    joint.axis = Eigen::Vector3d::UnitZ();
    joint.has_limits = true;
    joint.limits.lower = -1.0;
    joint.limits.upper = 1.0;
    model.addJoint(joint);
  }
  return model;
}

void expect_collision(const simulation::Collision &body_shape,
                       const simulation::Collision &tool_shape,
                       double voxel_size, bool is_expected_colliding,
                       const std::vector<double> &q = {0.0, 0.0}) {
  const auto model = make_model(body_shape, tool_shape);
  auto chain = simulation::createMultiArmKinematicChain(model, {{"body", "tool", ""}});
  ASSERT_EQ(chain->getTotalDOF(), 2U);
  simulation::GeometricSelfCollisionChecker checker(model, *chain, true, voxel_size);
  ASSERT_EQ(checker.getCollisionObjects().size(), 2U);
  ASSERT_FALSE(checker.shouldSkipCollision("body", "tool"));
  chain->updateKinematics(q);
  checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
  for (std::size_t idx = 0; idx < checker.getCollisionObjects().size(); ++idx) {
    Eigen::Isometry3d expected = body_shape.origin;
    if (checker.getLinkNameForObject(idx) == "tool") {
      Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
      motion.rotate(Eigen::AngleAxisd(q[0] + q[1], Eigen::Vector3d::UnitZ()));
      expected = motion * tool_shape.origin;
    }
    ASSERT_NE(checker.getFCLObject(static_cast<int>(idx)), nullptr);
    EXPECT_TRUE(checker.getFCLObject(static_cast<int>(idx))->getTransform().matrix()
                    .isApprox(expected.matrix(), 1e-12));
  }
  const bool is_colliding = checker.checkCollision();
  const auto pairs = checker.collectSelfCollisionPairs();
  EXPECT_EQ(is_colliding, is_expected_colliding);
  EXPECT_EQ(!pairs.empty(), is_expected_colliding);
  EXPECT_EQ(is_colliding, !pairs.empty());
  if (is_expected_colliding) {
    const std::vector<std::pair<std::string, std::string>> expected{{"body", "tool"}};
    EXPECT_EQ(pairs, expected);
  } else {
    EXPECT_TRUE(pairs.empty());
  }
}

void expect_contained_primitive(simulation::GeometryType type,
                                 const Eigen::Vector3d &size) {
  const temporary_binary_stl outer_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.05), Eigen::Vector3d::Constant(0.05)));
  const auto mesh = make_mesh_shape(outer_file);
  const auto primitive = make_primitive_shape(type, size);
  // 表面交差のない完全内包と、形状登録順を反転した同一結果
  for (const bool is_reversed : {false, true}) {
    SCOPED_TRACE(is_reversed);
    const auto &body = is_reversed ? primitive : mesh;
    const auto &tool = is_reversed ? mesh : primitive;
    expect_collision(body, tool, 0.0, false);
    expect_collision(body, tool, 0.005, true);
  }
}

// 内部球の中心による完全内包の検出
TEST(geometric_solid_containment, detects_sphere_fully_inside_mesh) {
  expect_contained_primitive(simulation::GeometryType::SPHERE, {0.008, 0.0, 0.0});
}

// 各面2 mm膨張後も表面と離れたBOXの完全内包
TEST(geometric_solid_containment, detects_box_fully_inside_mesh) {
  expect_contained_primitive(simulation::GeometryType::BOX, {0.014, 0.012, 0.016});
}

// CYLINDERから生成したカプセルの完全内包
TEST(geometric_solid_containment, detects_capsule_fully_inside_mesh) {
  expect_contained_primitive(simulation::GeometryType::CYLINDER, {0.006, 0.020, 0.0});
}

// 第一成分が外側、第二成分だけが内側となる分離メッシュの双方向検査
TEST(geometric_solid_containment, detects_second_disconnected_mesh_component_both_orders) {
  const temporary_binary_stl outer_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.03), Eigen::Vector3d::Constant(0.03)));
  auto components = make_box_mesh({-0.10, -0.01, -0.01}, {-0.08, 0.01, 0.01});
  append_mesh(components, make_box_mesh(Eigen::Vector3d::Constant(-0.01),
                                       Eigen::Vector3d::Constant(0.01)));
  const temporary_binary_stl components_file(components);
  const auto outer = make_mesh_shape(outer_file);
  const auto disconnected = make_mesh_shape(components_file);
  for (const bool is_reversed : {false, true}) {
    SCOPED_TRACE(is_reversed);
    const auto &body = is_reversed ? disconnected : outer;
    const auto &tool = is_reversed ? outer : disconnected;
    expect_collision(body, tool, 0.0, false);
    expect_collision(body, tool, 0.005, true);
  }
}

// 球・BOX・カプセルの内部にある第二メッシュ成分の保持
TEST(geometric_solid_containment, detects_second_mesh_component_inside_primitives) {
  auto components = make_box_mesh({-0.10, -0.006, -0.006}, {-0.08, 0.006, 0.006});
  append_mesh(components, make_box_mesh(Eigen::Vector3d::Constant(-0.006),
                                       Eigen::Vector3d::Constant(0.006)));
  const temporary_binary_stl components_file(components);
  const auto mesh = make_mesh_shape(components_file);
  const std::vector<simulation::Collision> primitives{
      make_primitive_shape(simulation::GeometryType::SPHERE, {0.025, 0.0, 0.0}),
      make_primitive_shape(simulation::GeometryType::BOX, {0.045, 0.045, 0.045}),
      make_primitive_shape(simulation::GeometryType::CYLINDER, {0.022, 0.020, 0.0})};
  for (std::size_t idx = 0; idx < primitives.size(); ++idx) {
    SCOPED_TRACE(idx);
    expect_collision(primitives[idx], mesh, 0.005, true);
    expect_collision(mesh, primitives[idx], 0.005, true);
  }
}

// 二つの分離壁およびU字壁の外部とつながる空間の非占有
TEST(geometric_solid_containment, preserves_open_channel_and_u_shaped_cavity) {
  const temporary_binary_stl inner_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.009), Eigen::Vector3d::Constant(0.009)));
  const auto inner_mesh = make_mesh_shape(inner_file);
  const auto sphere = make_primitive_shape(simulation::GeometryType::SPHERE,
                                           {0.012, 0.0, 0.0});
  for (const bool has_bottom : {false, true}) {
    SCOPED_TRACE(has_bottom);
    auto walls = make_box_mesh({-0.07, -0.07, -0.03}, {-0.04, 0.07, 0.03});
    append_mesh(walls, make_box_mesh({0.04, -0.07, -0.03}, {0.07, 0.07, 0.03}));
    if (has_bottom) {
      append_mesh(walls, make_box_mesh({-0.04, -0.07, -0.03}, {0.04, -0.04, 0.03}));
    }
    const temporary_binary_stl walls_file(walls);
    const auto outer = make_mesh_shape(walls_file);
    for (const auto &inner : {sphere, inner_mesh}) {
      expect_collision(outer, inner, 0.0, false);
      expect_collision(outer, inner, 0.005, false);
      expect_collision(inner, outer, 0.005, false);
    }
  }
}

// 可動リンクとcollision原点を合成した座標での内包・表面交差・分離
TEST(geometric_solid_containment, transforms_seeds_and_surfaces_with_collision_origins) {
  const temporary_binary_stl outer_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.05), Eigen::Vector3d::Constant(0.05)));
  const temporary_binary_stl inner_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.005), Eigen::Vector3d::Constant(0.005)));
  auto outer = make_mesh_shape(outer_file);
  outer.origin.translation() = Eigen::Vector3d(0.23, -0.11, 0.34);
  outer.origin.rotate(Eigen::AngleAxisd(0.73, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
  const std::vector<double> q{0.4, -0.15};
  Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
  motion.rotate(Eigen::AngleAxisd(q[0] + q[1], Eigen::Vector3d::UnitZ()));
  for (const double offset : {0.015, 0.048, 0.075}) {
    SCOPED_TRACE(offset);
    auto inner = make_mesh_shape(inner_file);
    Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
    local.translation() = Eigen::Vector3d(offset, 0.002, -0.003);
    inner.origin = motion.inverse() * outer.origin * local;
    const bool has_surface_intersection = offset == 0.048;
    const bool is_contained_or_intersecting = offset != 0.075;
    expect_collision(outer, inner, 0.0, has_surface_intersection, q);
    expect_collision(outer, inner, 0.005, is_contained_or_intersecting, q);
  }
}


// 原点移動・任意回転・大きな姿勢変化・衝突後の復帰を含む通常FCLとの照合。
TEST(mesh_collision_front, matches_uncached_fcl_through_pose_changes) {
  auto mesh = make_box_mesh({-0.09, -0.07, -0.04}, {-0.055, 0.07, 0.04});
  append_mesh(mesh, make_box_mesh({0.055, -0.07, -0.04}, {0.09, 0.07, 0.04}));
  append_mesh(mesh, make_box_mesh({-0.055, -0.07, -0.04}, {0.055, -0.045, 0.04}));
  const temporary_binary_stl body_file(mesh);
  const temporary_binary_stl tool_file(make_box_mesh(
      {-0.018, -0.022, -0.026}, {0.018, 0.022, 0.026}));
  collision::FCLCollisionDetector detector;
  const int body = detector.addRobotMeshLink(body_file.path, Eigen::Vector3d::Ones());
  const int tool = detector.addRobotMeshLink(tool_file.path, Eigen::Vector3d::Ones());
  ASSERT_GE(body, 0);
  ASSERT_GE(tool, 0);
  std::mt19937 generator(137);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int num_collisions = 0;
  int num_clear = 0;
  for (int iter = 0; iter < 192; ++iter) {
    SCOPED_TRACE(iter);
    Eigen::Isometry3d common_pose = Eigen::Isometry3d::Identity();
    common_pose.translation() = Eigen::Vector3d(unit(generator), unit(generator), unit(generator));
    common_pose.rotate(Eigen::AngleAxisd(unit(generator), Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
    Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
    relative_pose.translation() = Eigen::Vector3d(0.15 * unit(generator), 0.09 * unit(generator), 0.02 * unit(generator));
    relative_pose.rotate(Eigen::AngleAxisd(2.0 * unit(generator), Eigen::Vector3d(2.0, 1.0, 3.0).normalized()));
    detector.updateRobotLinkPose(body, common_pose);
    detector.updateRobotLinkPose(tool, common_pose * relative_pose);
    const bool is_expected = detector.checkSelfCollision({}, false);
    EXPECT_EQ(detector.checkSelfCollision({}, true), is_expected);
    EXPECT_EQ(detector.checkSelfCollision({}, true), is_expected);
    if (is_expected) ++num_collisions;
    else ++num_clear;
  }
  EXPECT_GT(num_collisions, 0);
  EXPECT_GT(num_clear, 0);
}

// 細分化の継続と周期的再構築をまたぐ、狭い隙間から接触・交差・離脱までの照合。
TEST(mesh_collision_front, preserves_contacts_after_long_noncolliding_motion) {
  const temporary_binary_stl file(make_box_mesh(
      {-0.03, -0.02, -0.01}, {0.03, 0.02, 0.01}));
  collision::FCLCollisionDetector detector;
  const int first = detector.addRobotMeshLink(file.path, Eigen::Vector3d::Ones());
  const int second = detector.addRobotMeshLink(file.path, Eigen::Vector3d::Ones());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  detector.updateRobotLinkPose(first, pose);
  for (int iter = 0; iter < 160; ++iter) {
    SCOPED_TRACE(iter);
    pose.translation() = Eigen::Vector3d(0.0001 * iter, 0.0, 0.02001);
    detector.updateRobotLinkPose(second, pose);
    ASSERT_FALSE(detector.checkSelfCollision({}, false));
    EXPECT_FALSE(detector.checkSelfCollision({}, true));
  }
  for (const double offset : {0.02, 0.019, -0.019, -0.02, -0.02001, 0.02001}) {
    SCOPED_TRACE(offset);
    pose.translation() = Eigen::Vector3d(0.0, 0.0, offset);
    detector.updateRobotLinkPose(second, pose);
    EXPECT_EQ(detector.checkSelfCollision({}, true), detector.checkSelfCollision({}, false));
  }
}

// 除外ペアの変更と同一IDへの別形状再登録後の判定。
TEST(mesh_collision_front, respects_exclusions_and_geometry_reregistration) {
  const temporary_binary_stl small_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.01), Eigen::Vector3d::Constant(0.01)));
  const temporary_binary_stl large_file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.04), Eigen::Vector3d::Constant(0.04)));
  collision::FCLCollisionDetector detector;
  for (const auto &path : {small_file.path, large_file.path}) {
    detector.clearObstacles();
    const int first = detector.addRobotMeshLink(path, Eigen::Vector3d::Ones());
    const int second = detector.addRobotMeshLink(path, Eigen::Vector3d::Ones());
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    detector.updateRobotLinkPose(first, pose);
    pose.translation().x() = 0.06;
    detector.updateRobotLinkPose(second, pose);
    EXPECT_EQ(detector.checkSelfCollision({}, true), path == large_file.path);
    pose.translation().x() = 0.005;
    detector.updateRobotLinkPose(second, pose);
    EXPECT_FALSE(detector.checkSelfCollision({{second, first}}, true));
    EXPECT_TRUE(detector.checkSelfCollision({}, true));
    EXPECT_TRUE(detector.checkSelfCollision({}, false));
  }
}

// メッシュ対primitiveの通常FCL経路と、別チェッカー間のキャッシュ独立性。
TEST(mesh_collision_front, preserves_primitive_queries_and_detector_independence) {
  const temporary_binary_stl file(make_box_mesh(
      Eigen::Vector3d::Constant(-0.04), Eigen::Vector3d::Constant(0.04)));
  collision::FCLCollisionDetector first_detector;
  collision::FCLCollisionDetector second_detector;
  for (auto *detector : {&first_detector, &second_detector}) {
    const int mesh = detector->addRobotMeshLink(file.path, Eigen::Vector3d::Ones());
    collision::Sphere sphere;
    sphere.radius = detector == &first_detector ? 0.01 : 0.07;
    sphere.center = Eigen::Vector3d::Zero();
    const int primitive = detector->addRobotLink(sphere);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    detector->updateRobotLinkPose(mesh, pose);
    for (int iter = 0; iter < 80; ++iter) {
      pose.translation().x() = (iter - 40) * 0.003;
      detector->updateRobotLinkPose(primitive, pose);
      EXPECT_EQ(detector->checkSelfCollision({}, true), detector->checkSelfCollision({}, false));
    }
  }
}


// 多数の非交差部分を保持した後の遠端衝突。探索境界の上限到達時にも検査を維持。
TEST(mesh_collision_front, preserves_late_contacts_after_large_noncolliding_mesh) {
  simulation::MeshData outer;
  simulation::MeshData inner;
  for (int idx = 0; idx < 1024; ++idx) {
    const Eigen::Vector3d center(0.04 * idx, 0.0, 0.0);
    append_mesh(outer, make_box_mesh(center - Eigen::Vector3d::Constant(0.01),
                                      center + Eigen::Vector3d::Constant(0.01)));
    append_mesh(inner, make_box_mesh(center - Eigen::Vector3d::Constant(0.005),
                                      center + Eigen::Vector3d::Constant(0.005)));
  }
  const temporary_binary_stl outer_file(outer);
  const temporary_binary_stl inner_file(inner);
  collision::FCLCollisionDetector detector;
  const int outer_id = detector.addRobotMeshLink(outer_file.path, Eigen::Vector3d::Ones());
  const int inner_id = detector.addRobotMeshLink(inner_file.path, Eigen::Vector3d::Ones());
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  detector.updateRobotLinkPose(outer_id, identity);
  for (int iter = 0; iter < 5; ++iter) {
    Eigen::Isometry3d pose = identity;
    pose.translation().y() = 0.00001 * iter;
    detector.updateRobotLinkPose(inner_id, pose);
    ASSERT_FALSE(detector.checkSelfCollision({}, false));
    EXPECT_FALSE(detector.checkSelfCollision({}, true));
  }
  Eigen::Isometry3d pose = identity;
  pose.rotate(Eigen::AngleAxisd(0.0002, Eigen::Vector3d::UnitY()));
  detector.updateRobotLinkPose(inner_id, pose);
  ASSERT_TRUE(detector.checkSelfCollision({}, false));
  EXPECT_TRUE(detector.checkSelfCollision({}, true));
}

}  // 無名名前空間の終端


TEST(geometric_solid_containment, query_clones_share_geometry_and_keep_independent_poses) {
  const temporary_binary_stl outer_file(make_box_mesh({.08, -.025, -.025}, {.12, .025, .025}));
  auto tool = make_primitive_shape(simulation::GeometryType::SPHERE, {.008, 0, 0});
  tool.origin.translation() = Eigen::Vector3d(.1, 0, 0);
  const auto model = make_model(make_mesh_shape(outer_file), tool);
  auto chain = simulation::createMultiArmKinematicChain(model, {{"body", "tool", ""}});
  simulation::GeometricSelfCollisionChecker original(model, *chain, true, .001);
  std::vector<std::unique_ptr<simulation::GeometricSelfCollisionChecker>> workers;
  for (int idx = 0; idx < 4; ++idx) {
    workers.push_back(original.clone_for_queries());
    ASSERT_NE(workers.back()->getFCLObject(0), original.getFCLObject(0));
    EXPECT_EQ(workers.back()->getFCLObject(0)->collisionGeometry(), original.getFCLObject(0)->collisionGeometry());
  }
  const auto query = [&](simulation::GeometricSelfCollisionChecker &checker, double angle) {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
    chain->forwardKinematicsAt(std::vector<double>{angle, 0}, positions, orientations);
    checker.updateBodyPoses(positions, orientations);
    return checker.checkCollision();
  };
  std::vector<unsigned char> expected(128), actual(128);
  for (std::size_t idx = 0; idx < expected.size(); ++idx) expected[idx] = query(original, idx%2 ? 0.0 : 1.2);
  ASSERT_NE(expected[0], expected[1]);
  robot_sim::common::parallel_queries(actual.size(), workers.size(), [&](std::size_t worker_idx, std::size_t idx) {
    actual[idx] = query(*workers[worker_idx], idx%2 ? 0.0 : 1.2);
  });
  EXPECT_EQ(actual, expected);
  EXPECT_TRUE(original.checkCollision());
}
