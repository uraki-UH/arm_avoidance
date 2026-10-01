#include <gtest/gtest.h>

#include "gng/GrowingNeuralGas.hpp"
#include "safety_engine/vlut/iself_collision_checker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using gng_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
using edge_type = std::pair<std::int32_t, std::int32_t>;

class temporary_files {
public:
  temporary_files() {
    char pattern[] = "/tmp/gng_collision_filter_XXXXXX";
    const char *directory = ::mkdtemp(pattern);
    if (!directory) throw std::runtime_error("Temporary directory creation failed");
    directory_ = directory;
  }

  ~temporary_files() {
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
  }

  std::string path(const std::string &name) const {
    return (directory_ / name).string();
  }

private:
  std::filesystem::path directory_;
};

template <typename value_type>
void write_scalar(std::ofstream &output, const value_type &value) {
  output.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

void write_vector(std::ofstream &output, std::initializer_list<float> values) {
  const Eigen::Index rows = static_cast<Eigen::Index>(values.size());
  const Eigen::Index columns = 1;
  write_scalar(output, rows);
  write_scalar(output, columns);
  for (float value : values) write_scalar(output, value);
}

void write_edges(std::ofstream &output, const std::vector<edge_type> &edges) {
  write_scalar(output, static_cast<std::int32_t>(edges.size()));
  for (const auto &edge : edges) {
    write_scalar(output, edge.first);
    write_scalar(output, edge.second);
    write_scalar(output, std::int32_t{1});
    write_scalar(output, true);
  }
}

// 正規 load 経路用の3ノード・角度1層・座標2層の GNG v9 フィクスチャ。
void write_fixture(const std::string &path, bool has_angle_collision_edge = true) {
  static_assert(sizeof(bool) == 1 && sizeof(int) == sizeof(std::int32_t));
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Fixture file creation failed");
  write_scalar(output, std::uint32_t{9});
  write_scalar(output, std::int32_t{2});
  write_scalar(output, std::int32_t{3});
  const std::array<float, 3> angles{0.0f, 0.16f, 0.24f};
  for (std::int32_t idx = 0; idx < 3; ++idx) {
    write_scalar(output, idx);
    write_scalar(output, 0.0f);
    write_scalar(output, 0.0f);
    write_vector(output, {angles[idx]});
    write_vector(output, {angles[idx], 0.0f, 0.0f});
    write_scalar(output, std::int32_t{2});
    write_vector(output, {angles[idx], 0.0f, 0.0f});
    write_vector(output, {-angles[idx], 0.0f, 0.0f});
    write_scalar(output, std::int32_t{0});
    for (bool flag : {false, false, true, true, false}) write_scalar(output, flag);
    write_vector(output, {1.0f, 0.0f, 0.0f});
    write_scalar(output, 0.0f);
    write_scalar(output, 0.0f);
    write_scalar(output, 1.0f);
    write_scalar(output, false);
    write_scalar(output, 0.0f);
    write_scalar(output, 0.0f);
    write_scalar(output, false);
  }
  const std::vector<edge_type> coordinate_edges{{0, 1}, {1, 2}};
  write_edges(output, has_angle_collision_edge ? coordinate_edges
                                               : std::vector<edge_type>{{1, 2}});
  write_edges(output, coordinate_edges);
  write_edges(output, coordinate_edges);
  if (!output) throw std::runtime_error("Fixture write failed");
}

kinematics::KinematicChain make_chain() {
  kinematics::KinematicChain chain;
  chain.setBase(Eigen::Vector3d::Zero());
  kinematics::Link link;
  link.name = "test_link";
  kinematics::Joint joint;
  joint.name = "test_joint";
  joint.type = kinematics::JointType::Revolute;
  joint.axis1 = Eigen::Vector3d::UnitZ();
  joint.values = {0.0};
  joint.min_limits = {-1.0};
  joint.max_limits = {1.0};
  chain.addSegment(link, joint);
  return chain;
}

class fake_collision_checker : public simulation::ISelfCollisionChecker {
public:
  explicit fake_collision_checker(std::function<bool(double)> is_colliding)
      : is_colliding_(std::move(is_colliding)) {}

  void updateBodyPoses(
      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>> &orientations) override {
    if (orientations.empty()) throw std::runtime_error("Missing FK orientation");
    const auto &orientation = orientations.back();
    angle_ = 2.0 * std::atan2(orientation.z(), orientation.w());
  }

  bool checkCollision() override {
    checked_angles.push_back(angle_);
    return is_colliding_(angle_);
  }

  std::vector<double> checked_angles;

private:
  std::function<bool(double)> is_colliding_;
  double angle_ = 0.0;
};

bool is_in_narrow_collision_band(double angle) {
  return angle > 0.067 && angle < 0.071;
}

void expect_only_safe_edge(const gng_type &graph) {
  ASSERT_EQ(graph.getCoordLayerCount(), 2);
  EXPECT_TRUE(graph.getNeighborsAngle(0).empty());
  EXPECT_EQ(graph.getNeighborsAngle(1), std::vector<int>({2}));
  EXPECT_EQ(graph.getNeighborsAngle(2), std::vector<int>({1}));
  for (int layer_idx = 0; layer_idx < 2; ++layer_idx) {
    EXPECT_TRUE(graph.getNeighborsCoord(0, layer_idx).empty());
    EXPECT_EQ(graph.getNeighborsCoord(1, layer_idx), std::vector<int>({2}));
    EXPECT_EQ(graph.getNeighborsCoord(2, layer_idx), std::vector<int>({1}));
  }
}

}  // 無名名前空間の終端

TEST(gng_collision_filter, interior_collision_removes_edges_from_all_layers) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker checker(is_in_narrow_collision_band);
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  ASSERT_EQ(graph.getActiveIndices().size(), 3U);
  ASSERT_EQ(graph.getNeighborsAngle(0), std::vector<int>({1}));
  for (int layer_idx = 0; layer_idx < 2; ++layer_idx) {
    ASSERT_EQ(graph.getNeighborsCoord(0, layer_idx), std::vector<int>({1}));
  }
  graph.setSelfCollisionChecker(&checker);
  graph.setCollisionAware(true);
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices().size(), 3U);
  expect_only_safe_edge(graph);
  EXPECT_TRUE(std::any_of(checker.checked_angles.begin(), checker.checked_angles.end(),
                          is_in_narrow_collision_band));
  ASSERT_TRUE(graph.save(files.path("filtered.bin")));
  gng_type reloaded(1, 3, &chain);
  ASSERT_TRUE(reloaded.load(files.path("filtered.bin")));
  expect_only_safe_edge(reloaded);
}

TEST(gng_collision_filter, coordinate_only_edges_receive_collision_checks) {
  temporary_files files;
  write_fixture(files.path("input.bin"), false);
  auto chain = make_chain();
  fake_collision_checker checker(is_in_narrow_collision_band);
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  ASSERT_TRUE(graph.getNeighborsAngle(0).empty());
  for (int layer_idx = 0; layer_idx < 2; ++layer_idx) {
    ASSERT_EQ(graph.getNeighborsCoord(0, layer_idx), std::vector<int>({1}));
  }
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices().size(), 3U);
  expect_only_safe_edge(graph);
}

TEST(gng_collision_filter, colliding_node_removes_incident_edges_in_all_layers) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker checker([](double angle) { return std::abs(angle) < 0.005; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices(), std::vector<int>({1, 2}));
  EXPECT_EQ(graph.nodeAt(0).id, -1);
  expect_only_safe_edge(graph);
  ASSERT_TRUE(graph.save(files.path("filtered.bin")));
  gng_type reloaded(1, 3, &chain);
  ASSERT_TRUE(reloaded.load(files.path("filtered.bin")));
  EXPECT_EQ(reloaded.getActiveIndices(), std::vector<int>({1, 2}));
  expect_only_safe_edge(reloaded);
}
