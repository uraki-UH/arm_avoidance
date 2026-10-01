#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <fcl/narrowphase/collision.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "collision/fcl/fcl_collision_detector.hpp"
#include "collision/fcl/solid_voxel_geometry.hpp"

namespace {

simulation::MeshData make_box_mesh(const Eigen::Vector3d& min_point,
                                   const Eigen::Vector3d& max_point) {
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

class temporary_binary_stl {
 public:
  explicit temporary_binary_stl(const simulation::MeshData& mesh) {
    std::string path_pattern =
        (std::filesystem::temp_directory_path() / "solid_voxel_XXXXXX").string();
    std::vector<char> path_chars(path_pattern.begin(), path_pattern.end());
    path_chars.push_back('\0');
    const int descriptor = mkstemp(path_chars.data());
    if (descriptor < 0) {
      throw std::runtime_error("Failed to create STL test fixture");
    }
    close(descriptor);
    path = path_chars.data();
    std::ofstream stream(path, std::ios::binary);
    const std::array<char, 80> header{};
    stream.write(header.data(), header.size());
    const auto num_triangles = static_cast<std::uint32_t>(mesh.indices.size() / 3);
    stream.write(reinterpret_cast<const char*>(&num_triangles), sizeof(num_triangles));
    for (std::size_t idx = 0; idx < mesh.indices.size(); idx += 3) {
      const std::array<float, 3> normal{};
      stream.write(reinterpret_cast<const char*>(normal.data()), sizeof(normal));
      for (std::size_t corner_idx = 0; corner_idx < 3; ++corner_idx) {
        const auto vertex_idx = mesh.indices[idx + corner_idx];
        stream.write(reinterpret_cast<const char*>(mesh.vertices.data() +
                         static_cast<std::size_t>(vertex_idx) * 3),
                     sizeof(float) * 3);
      }
      const std::uint16_t attribute = 0;
      stream.write(reinterpret_cast<const char*>(&attribute), sizeof(attribute));
    }
    if (!stream) {
      throw std::runtime_error("Failed to write STL test fixture");
    }
  }

  ~temporary_binary_stl() {
    std::error_code error;
    std::filesystem::remove(path, error);
  }

  std::string path;
};

// 閉じた箱の表面と内部の占有
TEST(solid_voxel_geometry_test, fills_closed_mesh_surface_and_interior) {
  const auto mesh = make_box_mesh(Eigen::Vector3d(-0.051, -0.041, -0.031),
                                  Eigen::Vector3d(0.051, 0.041, 0.031));
  const auto geometry = collision::build_solid_voxel_geometry(mesh, 0.01);
  EXPECT_GT(geometry.num_surface_cells, 0U);
  EXPECT_GT(geometry.num_interior_cells, 0U);
  const auto* center = geometry.tree->search(0.0, 0.0, 0.0);
  ASSERT_NE(center, nullptr);
  EXPECT_TRUE(geometry.tree->isNodeOccupied(center));
  EXPECT_EQ(geometry.tree->search(0.2, 0.0, 0.0), nullptr);
  for (std::size_t idx = 0; idx < mesh.vertices.size(); idx += 3) {
    const auto* vertex = geometry.tree->search(
        mesh.vertices[idx], mesh.vertices[idx + 1], mesh.vertices[idx + 2]);
    ASSERT_NE(vertex, nullptr);
    EXPECT_TRUE(geometry.tree->isNodeOccupied(vertex));
  }
}

// 回転並進した体積内部の球衝突
TEST(solid_voxel_geometry_test, detects_sphere_inside_transformed_solid) {
  const auto mesh = make_box_mesh(Eigen::Vector3d(-0.04, -0.03, -0.02),
                                  Eigen::Vector3d(0.04, 0.03, 0.02));
  const auto geometry = collision::build_solid_voxel_geometry(mesh, 0.005);
  auto shape = std::make_shared<fcl::OcTree<double>>(geometry.tree);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.23, -0.11, 0.34);
  pose.rotate(Eigen::AngleAxisd(0.73, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
  fcl::CollisionObject<double> body(shape, pose);
  auto sphere_shape = std::make_shared<fcl::Sphere<double>>(0.002);
  Eigen::Isometry3d sphere_pose = Eigen::Isometry3d::Identity();
  sphere_pose.translation() = pose * Eigen::Vector3d(0.01, -0.01, 0.005);
  fcl::CollisionObject<double> sphere(sphere_shape, sphere_pose);
  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;
  fcl::collide(&body, &sphere, request, result);
  EXPECT_TRUE(result.isCollision());
  sphere_pose.translation() = pose * Eigen::Vector3d(0.15, 0.0, 0.0);
  sphere.setTransform(sphere_pose);
  sphere.computeAABB();
  result.clear();
  fcl::collide(&body, &sphere, request, result);
  EXPECT_FALSE(result.isCollision());
}

// 接触辺を共有する閉じた複数固体
TEST(solid_voxel_geometry_test, accepts_closed_solids_with_shared_edge) {
  auto mesh = make_box_mesh(Eigen::Vector3d(-0.04, -0.04, -0.02),
                            Eigen::Vector3d(0.0, 0.0, 0.02));
  const auto second = make_box_mesh(Eigen::Vector3d(0.0, 0.0, -0.02),
                                    Eigen::Vector3d(0.04, 0.04, 0.02));
  const auto offset = static_cast<std::uint32_t>(mesh.vertices.size() / 3);
  mesh.vertices.insert(mesh.vertices.end(), second.vertices.begin(), second.vertices.end());
  for (const auto idx : second.indices) mesh.indices.push_back(idx + offset);
  const auto geometry = collision::build_solid_voxel_geometry(mesh, 0.005);
  EXPECT_NE(geometry.tree->search(-0.02, -0.02, 0.0), nullptr);
  EXPECT_NE(geometry.tree->search(0.02, 0.02, 0.0), nullptr);
}

// 分離した固体ごとの元頂点による、第二成分の完全内包の検出根拠
TEST(solid_voxel_geometry_test, preserves_original_vertices_for_disconnected_components) {
  auto mesh = make_box_mesh(Eigen::Vector3d(-0.06, -0.02, -0.02),
                            Eigen::Vector3d(-0.04, 0.02, 0.02));
  const auto second = make_box_mesh(Eigen::Vector3d(0.04, -0.02, -0.02),
                                    Eigen::Vector3d(0.06, 0.02, 0.02));
  const auto offset = static_cast<std::uint32_t>(mesh.vertices.size() / 3);
  mesh.vertices.insert(mesh.vertices.end(), second.vertices.begin(), second.vertices.end());
  for (const auto idx : second.indices) mesh.indices.push_back(idx + offset);
  const auto geometry = collision::build_solid_voxel_geometry(mesh, 0.005);
  ASSERT_EQ(geometry.surface_component_points.size(), 2U);
  for (const auto& point : geometry.surface_component_points) {
    bool is_original_vertex = false;
    for (std::size_t idx = 0; idx < mesh.vertices.size(); idx += 3) {
      const Eigen::Vector3d source(mesh.vertices[idx], mesh.vertices[idx + 1],
                                    mesh.vertices[idx + 2]);
      is_original_vertex = is_original_vertex || (point.array() == source.array()).all();
    }
    EXPECT_TRUE(is_original_vertex);
  }
  const auto outer_mesh = make_box_mesh(Eigen::Vector3d(0.03, -0.03, -0.03),
                                        Eigen::Vector3d(0.07, 0.03, 0.03));
  const auto outer = collision::build_solid_voxel_geometry(outer_mesh, 0.005);
  std::size_t num_contained_points = 0;
  for (const auto& point : geometry.surface_component_points) {
    const auto* node = outer.tree->search(point.x(), point.y(), point.z());
    if (node && outer.tree->isNodeOccupied(node)) ++num_contained_points;
  }
  EXPECT_EQ(num_contained_points, 1U);
  EXPECT_LT(geometry.surface_component_points.front().x(), 0.0);
  EXPECT_GT(geometry.surface_component_points.back().x(), 0.0);
}

// 面積ゼロの接続面による、分離した固体の成分統合の防止
TEST(solid_voxel_geometry_test, keeps_components_separate_across_degenerate_connectors) {
  auto mesh = make_box_mesh(Eigen::Vector3d(-0.06, -0.02, -0.02),
                            Eigen::Vector3d(-0.04, 0.02, 0.02));
  const auto second = make_box_mesh(Eigen::Vector3d(0.04, -0.02, -0.02),
                                    Eigen::Vector3d(0.06, 0.02, 0.02));
  const auto offset = static_cast<std::uint32_t>(mesh.vertices.size() / 3);
  mesh.vertices.insert(mesh.vertices.end(), second.vertices.begin(), second.vertices.end());
  for (const auto idx : second.indices) mesh.indices.push_back(idx + offset);
  const auto original = collision::build_solid_voxel_geometry(mesh, 0.005);
  mesh.indices.insert(mesh.indices.end(), {0, 1, offset, offset, 1, 0});
  const auto connected = collision::build_solid_voxel_geometry(mesh, 0.005);
  ASSERT_EQ(original.surface_component_points.size(), 2U);
  ASSERT_EQ(connected.surface_component_points.size(), 2U);
  EXPECT_EQ(original.num_surface_cells, connected.num_surface_cells);
  EXPECT_EQ(original.num_interior_cells, connected.num_interior_cells);
  for (std::size_t idx = 0; idx < original.surface_component_points.size(); ++idx) {
    EXPECT_TRUE((original.surface_component_points[idx].array() ==
                 connected.surface_component_points[idx].array()).all());
  }
}

// 開いた面と不正入力の拒否
TEST(solid_voxel_geometry_test, rejects_open_and_invalid_mesh_data) {
  const auto mesh = make_box_mesh(Eigen::Vector3d::Constant(-0.04),
                                  Eigen::Vector3d::Constant(0.04));
  auto opened = mesh;
  opened.indices.resize(opened.indices.size() - 3);
  EXPECT_THROW(collision::build_solid_voxel_geometry(opened, 0.005), std::invalid_argument);
  const auto opened_indices = opened.indices;
  opened.indices.insert(opened.indices.end(), opened_indices.begin(), opened_indices.end());
  EXPECT_THROW(collision::build_solid_voxel_geometry(opened, 0.005), std::invalid_argument);
  auto invalid = mesh;
  invalid.vertices[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(collision::build_solid_voxel_geometry(invalid, 0.005), std::invalid_argument);
  invalid = mesh;
  invalid.indices[0] = 999999;
  EXPECT_THROW(collision::build_solid_voxel_geometry(invalid, 0.005), std::invalid_argument);
  EXPECT_THROW(collision::build_solid_voxel_geometry(mesh, 0.0), std::invalid_argument);
  EXPECT_THROW(collision::build_solid_voxel_geometry(mesh, 0.005, 16), std::length_error);
  EXPECT_THROW(collision::build_solid_voxel_geometry(mesh, 1e-8), std::invalid_argument);
  auto degenerate = mesh;
  degenerate.indices = {0, 0, 0};
  EXPECT_THROW(collision::build_solid_voxel_geometry(degenerate, 0.005), std::invalid_argument);
}

// 面積ゼロの付加面による占有不変
TEST(solid_voxel_geometry_test, preserves_occupancy_with_degenerate_face) {
  auto mesh = make_box_mesh(Eigen::Vector3d::Constant(-0.04),
                            Eigen::Vector3d::Constant(0.04));
  const auto original = collision::build_solid_voxel_geometry(mesh, 0.01);
  mesh.indices.insert(mesh.indices.end(), {0, 0, 0});
  const auto actual = collision::build_solid_voxel_geometry(mesh, 0.01);
  EXPECT_EQ(actual.num_surface_cells, original.num_surface_cells);
  EXPECT_EQ(actual.num_interior_cells, original.num_interior_cells);
}

// 同一直線上の辺分割を接続する面積ゼロ三角形の閉鎖維持
TEST(solid_voxel_geometry_test, preserves_boundary_edges_of_collinear_connector_faces) {
  const auto original = make_box_mesh(Eigen::Vector3d::Constant(-0.04),
                                      Eigen::Vector3d::Constant(0.04));
  auto mesh = original;
  mesh.vertices.insert(mesh.vertices.end(), {0.0F, -0.04F, -0.04F});
  mesh.indices.erase(mesh.indices.begin(), mesh.indices.begin() + 3);
  mesh.indices.insert(mesh.indices.end(), {0, 2, 8, 8, 2, 1, 0, 8, 1});
  const auto expected = collision::build_solid_voxel_geometry(original, 0.01);
  const auto actual = collision::build_solid_voxel_geometry(mesh, 0.01);
  EXPECT_EQ(actual.num_surface_cells, expected.num_surface_cells);
  EXPECT_EQ(actual.num_interior_cells, expected.num_interior_cells);
  ASSERT_NE(actual.tree->search(0.0, 0.0, 0.0), nullptr);
  EXPECT_TRUE(actual.tree->isNodeOccupied(actual.tree->search(0.0, 0.0, 0.0)));
}

// 剛体占有形状の登録と除外ペアの維持
TEST(solid_voxel_geometry_test, registers_rigid_voxels_and_preserves_ignore_pairs) {
  const auto mesh = make_box_mesh(Eigen::Vector3d::Constant(-0.04),
                                  Eigen::Vector3d::Constant(0.04));
  const temporary_binary_stl file(mesh);
  collision::FCLCollisionDetector detector;
  const int first_idx = detector.addRobotVoxelMeshLink(
      file.path, Eigen::Vector3d::Ones(), 0.01);
  const int second_idx = detector.addRobotVoxelMeshLink(
      file.path, Eigen::Vector3d::Ones(), 0.01);
  ASSERT_EQ(first_idx, 0);
  ASSERT_EQ(second_idx, 1);
  ASSERT_NE(detector.getRobotLink(first_idx), nullptr);
  EXPECT_EQ(detector.getRobotLink(first_idx)->collisionGeometry(),
            detector.getRobotLink(second_idx)->collisionGeometry());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.rotate(Eigen::AngleAxisd(0.47, Eigen::Vector3d::UnitZ()));
  pose.translation() = Eigen::Vector3d(0.01, 0.01, 0.01);
  detector.updateRobotLinkPose(second_idx, pose);
  EXPECT_TRUE(detector.checkSelfCollision());
  EXPECT_FALSE(detector.checkSelfCollision({{first_idx, second_idx}}));
  pose.translation().x() = 0.2;
  detector.updateRobotLinkPose(second_idx, pose);
  EXPECT_FALSE(detector.checkSelfCollision());
}

// 木の内部座標を維持した占有包絡と、回転時の内包衝突
TEST(solid_voxel_geometry_test, bounds_occupied_cells_without_changing_octree_collision) {
  const auto mesh = make_box_mesh(Eigen::Vector3d(0.021, -0.038, 0.011),
                                  Eigen::Vector3d(0.079, 0.018, 0.043));
  const temporary_binary_stl file(mesh);
  collision::FCLCollisionDetector detector;
  const int body_idx = detector.addRobotVoxelMeshLink(
      file.path, Eigen::Vector3d::Ones(), 0.005);
  const auto body = detector.getRobotLink(body_idx);
  ASSERT_NE(body, nullptr);
  EXPECT_EQ(body->getNodeType(), fcl::GEOM_OCTREE);
  const auto* octree = dynamic_cast<const fcl::OcTree<double>*>(
      body->collisionGeometry().get());
  ASSERT_NE(octree, nullptr);
  EXPECT_GT(octree->getRootBV().max_.x(), 100.0);
  EXPECT_LT((body->getAABB().max_ - body->getAABB().min_).maxCoeff(), 0.071);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-0.12, 0.18, -0.23);
  pose.rotate(Eigen::AngleAxisd(0.83, Eigen::Vector3d(2.0, 3.0, 1.0).normalized()));
  detector.updateRobotLinkPose(body_idx, pose);
  const auto& bounds = body->getAABB();
  EXPECT_LT((bounds.max_ - bounds.min_).maxCoeff(), 0.14);
  for (std::size_t idx = 0; idx < mesh.vertices.size(); idx += 3) {
    const Eigen::Vector3d vertex = pose * Eigen::Vector3d(
        mesh.vertices[idx], mesh.vertices[idx + 1], mesh.vertices[idx + 2]);
    EXPECT_TRUE((vertex.array() >= bounds.min_.array() - 1e-10).all());
    EXPECT_TRUE((vertex.array() <= bounds.max_.array() + 1e-10).all());
  }
  collision::Sphere sphere;
  sphere.radius = 0.001;
  sphere.center = pose * Eigen::Vector3d(0.05, -0.01, 0.026);
  const int sphere_idx = detector.addRobotLink(sphere);
  EXPECT_TRUE(detector.checkSelfCollision());
  Eigen::Isometry3d sphere_pose = Eigen::Isometry3d::Identity();
  sphere_pose.translation() = pose * Eigen::Vector3d(0.4, 0.0, 0.0);
  detector.updateRobotLinkPose(sphere_idx, sphere_pose);
  EXPECT_FALSE(detector.checkSelfCollision());
}

// ランダム回転・並進後の全頂点を含む最小の軸整列包絡
TEST(solid_voxel_geometry_test, tight_world_bounds_cover_all_transformed_corners) {
  const auto mesh = make_box_mesh(Eigen::Vector3d(0.021, -0.038, 0.011),
                                  Eigen::Vector3d(0.079, 0.018, 0.043));
  const temporary_binary_stl file(mesh);
  collision::FCLCollisionDetector detector;
  const int body_idx = detector.addRobotVoxelMeshLink(
      file.path, Eigen::Vector3d::Ones(), 0.005);
  const auto body = detector.getRobotLink(body_idx);
  const auto local_bounds = body->collisionGeometry()->aabb_local;
  collision::Sphere sphere;
  sphere.radius = 0.001;
  sphere.center.setZero();
  const int sphere_idx = detector.addRobotLink(sphere);
  std::mt19937 random(20261001);
  std::uniform_real_distribution<double> sample(-1.0, 1.0);
  for (int sample_idx = 0; sample_idx < 32; ++sample_idx) {
    Eigen::Vector3d axis(sample(random), sample(random), sample(random));
    if (axis.norm() < 1e-6) axis = Eigen::Vector3d::UnitX();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(sample(random), sample(random), sample(random));
    pose.rotate(Eigen::AngleAxisd(sample(random) * std::acos(-1.0), axis.normalized()));
    detector.updateRobotLinkPose(body_idx, pose);
    const auto bounds = collision::get_tight_world_aabb(*body);
    Eigen::Vector3d min_point = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_point = -min_point;
    for (int corner_idx = 0; corner_idx < 8; ++corner_idx) {
      Eigen::Vector3d corner;
      for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
        corner[axis_idx] = (corner_idx & (1 << axis_idx))
            ? local_bounds.max_[axis_idx] : local_bounds.min_[axis_idx];
      }
      const Eigen::Vector3d point = pose * corner;
      min_point = min_point.cwiseMin(point);
      max_point = max_point.cwiseMax(point);
      EXPECT_TRUE((point.array() >= bounds.min_.array()).all());
      EXPECT_TRUE((point.array() <= bounds.max_.array()).all());
    }
    EXPECT_LT((bounds.min_ - min_point).norm(), 1e-8);
    EXPECT_LT((bounds.max_ - max_point).norm(), 1e-8);
    Eigen::Isometry3d sphere_pose = Eigen::Isometry3d::Identity();
    sphere_pose.translation() = pose * Eigen::Vector3d(0.05, -0.01, 0.026);
    detector.updateRobotLinkPose(sphere_idx, sphere_pose);
    EXPECT_TRUE(detector.checkSelfCollision());
    sphere_pose.translation() = pose * Eigen::Vector3d(0.4, 0.0, 0.0);
    detector.updateRobotLinkPose(sphere_idx, sphere_pose);
    EXPECT_FALSE(detector.checkSelfCollision());
  }
}

// ワールド AABB の重なる非交差 OBB の棄却と、接触・交差の維持
TEST(solid_voxel_geometry_test, rejects_disjoint_oriented_bounds_and_keeps_contacts) {
  auto shape = std::make_shared<fcl::Box<double>>(0.2, 0.02, 0.02);
  Eigen::Isometry3d first_pose = Eigen::Isometry3d::Identity();
  first_pose.rotate(Eigen::AngleAxisd(std::acos(-1.0) / 4.0, Eigen::Vector3d::UnitZ()));
  fcl::CollisionObject<double> first(shape, first_pose);
  Eigen::Isometry3d second_pose = first_pose;
  second_pose.translation() = first_pose.linear() * Eigen::Vector3d(0.0, 0.025, 0.0);
  fcl::CollisionObject<double> second(shape, second_pose);
  EXPECT_TRUE(collision::get_tight_world_aabb(first).overlap(
      collision::get_tight_world_aabb(second)));
  EXPECT_FALSE(collision::has_collision_bounds_overlap(first, second));
  second_pose.translation() = first_pose.linear() * Eigen::Vector3d(0.0, 0.02, 0.0);
  second.setTransform(second_pose);
  EXPECT_TRUE(collision::has_collision_bounds_overlap(first, second));
  second_pose.translation() = first_pose.linear() * Eigen::Vector3d(0.0, 0.015, 0.0);
  second.setTransform(second_pose);
  EXPECT_TRUE(collision::has_collision_bounds_overlap(first, second));
  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;
  fcl::collide(&first, &second, request, result);
  EXPECT_TRUE(result.isCollision());
}

// 小数六桁では区別できないスケール値の、別形状としての登録
TEST(solid_voxel_geometry_test, preserves_distinct_mesh_scales_in_geometry_cache) {
  const auto mesh = make_box_mesh(Eigen::Vector3d::Constant(-0.04),
                                  Eigen::Vector3d::Constant(0.04));
  const temporary_binary_stl file(mesh);
  collision::FCLCollisionDetector detector;
  const int first_idx = detector.addRobotMeshLink(file.path, Eigen::Vector3d::Ones());
  const Eigen::Vector3d second_scale(1.0000004, 1.0, 1.0);
  const int second_idx = detector.addRobotMeshLink(file.path, second_scale);
  const int repeat_idx = detector.addRobotMeshLink(file.path, second_scale);
  const auto first = detector.getRobotLink(first_idx)->collisionGeometry();
  const auto second = detector.getRobotLink(second_idx)->collisionGeometry();
  const auto repeated = detector.getRobotLink(repeat_idx)->collisionGeometry();
  EXPECT_NE(first, second);
  EXPECT_EQ(second, repeated);
  EXPECT_GT(second->aabb_local.max_.x(), first->aabb_local.max_.x());
  EXPECT_LT(second->aabb_local.min_.x(), first->aabb_local.min_.x());
}

// 姿勢・除外・形状追加・ID再利用に対する、キャッシュ有無の判定一致
TEST(solid_voxel_geometry_test, preserves_self_collision_results_with_exact_pose_cache) {
  collision::FCLCollisionDetector detector;
  collision::Sphere first_sphere;
  first_sphere.radius = 0.1;
  first_sphere.center.setZero();
  collision::Sphere second_sphere = first_sphere;
  second_sphere.center = Eigen::Vector3d(0.15, 0.15, 0.0);
  const int first_idx = detector.addRobotLink(first_sphere);
  const int second_idx = detector.addRobotLink(second_sphere);
  const auto expect_result = [&](bool is_expected,
      const std::vector<std::pair<int, int>>& ignore_pairs = {}) {
    EXPECT_EQ(detector.checkSelfCollision(ignore_pairs, true), is_expected);
    EXPECT_EQ(detector.checkSelfCollision(ignore_pairs), is_expected);
    EXPECT_EQ(detector.checkSelfCollision(ignore_pairs, true), is_expected);
  };
  // 包絡の重なる非交差形状での再判定
  expect_result(false);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.14, 0.14, 0.0);
  detector.updateRobotLinkPose(second_idx, pose);
  expect_result(true);
  expect_result(false, {{first_idx, second_idx}});
  expect_result(false, {{second_idx, first_idx}});
  expect_result(true);
  pose.translation() = Eigen::Vector3d(0.15, 0.15, 0.0);
  detector.updateRobotLinkPose(second_idx, pose);
  expect_result(false);
  pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  detector.updateRobotLinkPose(second_idx, pose);
  expect_result(false);
  pose.translation() = Eigen::Vector3d(0.15, 0.15, 0.0);
  detector.updateRobotLinkPose(second_idx, pose);
  expect_result(false);
  const int third_idx = detector.addRobotLink(first_sphere);
  expect_result(true);
  expect_result(false, {{first_idx, third_idx}});
  expect_result(true);
  detector.clearObstacles();
  first_sphere.radius = 0.02;
  second_sphere.radius = 0.02;
  second_sphere.center = Eigen::Vector3d(0.015, 0.015, 0.0);
  EXPECT_EQ(detector.addRobotLink(first_sphere), first_idx);
  EXPECT_EQ(detector.addRobotLink(second_sphere), second_idx);
  expect_result(true);
}

// 中心不変の回転による衝突変化と、包絡通過後の再判定
TEST(solid_voxel_geometry_test, invalidates_exact_pose_cache_after_rotation) {
  collision::FCLCollisionDetector detector;
  collision::Box box;
  box.center.setZero();
  box.rotation.setIdentity();
  box.extents = Eigen::Vector3d(0.1, 0.01, 0.01);
  const int box_idx = detector.addRobotLink(box);
  collision::Sphere sphere;
  sphere.radius = 0.022;
  sphere.center = Eigen::Vector3d(0.115, 0.025, 0.0);
  const int sphere_idx = detector.addRobotLink(sphere);
  const auto expect_result = [&](bool is_expected) {
    ASSERT_TRUE(collision::has_collision_bounds_overlap(
        *detector.getRobotLink(box_idx), *detector.getRobotLink(sphere_idx)));
    EXPECT_EQ(detector.checkSelfCollision({}, true), is_expected);
    EXPECT_EQ(detector.checkSelfCollision(), is_expected);
    EXPECT_EQ(detector.checkSelfCollision({}, true), is_expected);
  };
  expect_result(true);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.rotate(Eigen::AngleAxisd(-3.0 * std::acos(-1.0) / 180.0,
                               Eigen::Vector3d::UnitZ()));
  detector.updateRobotLinkPose(box_idx, pose);
  expect_result(false);
  detector.updateRobotLinkPose(box_idx, Eigen::Isometry3d::Identity());
  expect_result(true);
}

// STL欠落と破損による形状登録の拒否
TEST(solid_voxel_geometry_test, rejects_missing_and_truncated_stl) {
  collision::FCLCollisionDetector detector;
  EXPECT_THROW(detector.addRobotVoxelMeshLink(
      "/missing_solid_voxel_fixture.stl", Eigen::Vector3d::Ones(), 0.01),
      std::runtime_error);
  const auto mesh = make_box_mesh(Eigen::Vector3d::Constant(-0.04),
                                  Eigen::Vector3d::Constant(0.04));
  const temporary_binary_stl file(mesh);
  std::filesystem::resize_file(file.path, 84 + 50);
  EXPECT_THROW(detector.addRobotVoxelMeshLink(
      file.path, Eigen::Vector3d::Ones(), 0.01), std::runtime_error);
  // 二番目以降の非有限頂点も、既存頂点への誤った統合前に拒否
  auto invalid = mesh;
  invalid.vertices[3] = std::numeric_limits<float>::quiet_NaN();
  const temporary_binary_stl invalid_file(invalid);
  EXPECT_THROW(collision::load_solid_voxel_stl(
      invalid_file.path, Eigen::Vector3d::Ones()), std::runtime_error);
  EXPECT_EQ(detector.addRobotMeshLink(invalid_file.path, Eigen::Vector3d::Ones()), -1);
  EXPECT_TRUE(detector.getRobotLinks().empty());
}

}  // 無名名前空間
