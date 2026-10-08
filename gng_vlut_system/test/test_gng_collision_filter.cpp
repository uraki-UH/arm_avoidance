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
#include <iterator>
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

// 正規load経路用の角度1層・座標2層のGNG v9。中間ノード欠落時の不正参照も検証対象。
void write_fixture(const std::string &path, bool has_angle_collision_edge = true,
                   bool has_middle_node = true, bool enable_complete_graph = false) {
  static_assert(sizeof(bool) == 1 && sizeof(int) == sizeof(std::int32_t));
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Fixture file creation failed");
  write_scalar(output, std::uint32_t{9});
  write_scalar(output, std::int32_t{2});
  write_scalar(output, std::int32_t{has_middle_node ? 3 : 2});
  const std::array<float, 3> angles{0.0f, 0.16f, 0.24f};
  for (std::int32_t idx = 0; idx < 3; ++idx) {
    if (idx == 1 && !has_middle_node) continue;
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
  write_edges(output, enable_complete_graph ? std::vector<edge_type>{{0, 1}, {0, 2}, {1, 2}}
      : has_angle_collision_edge ? coordinate_edges : std::vector<edge_type>{{1, 2}});
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

TEST(gng_collision_filter, removing_node_erases_reverse_angle_edges) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  graph.setNodeActive(1, false);
  graph.removeInactiveElements();
  ASSERT_EQ(graph.getActiveIndices().size(), 2U);
  for (int id : {0, 2}) {
    EXPECT_TRUE(graph.getNeighborsAngle(id).empty());
    EXPECT_FALSE(graph.isEdgeActive(id, 1));
    for (int layer_idx = 0; layer_idx < 2; ++layer_idx)
      EXPECT_TRUE(graph.getNeighborsCoord(id, layer_idx).empty());
  }
  ASSERT_TRUE(graph.save(files.path("filtered.bin")));
  gng_type reloaded(1, 3, &chain);
  ASSERT_TRUE(reloaded.load(files.path("filtered.bin")));
  EXPECT_FALSE(reloaded.isEdgeActive(0, 1));
  EXPECT_FALSE(reloaded.isEdgeActive(2, 1));
}

TEST(gng_collision_filter, missing_endpoint_does_not_create_hidden_edges_on_load) {
  temporary_files files;
  write_fixture(files.path("input.bin"), true, false);
  auto chain = make_chain();
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  ASSERT_EQ(graph.getActiveIndices().size(), 2U);
  for (int id : {0, 2}) {
    EXPECT_TRUE(graph.getNeighborsAngle(id).empty());
    EXPECT_FALSE(graph.isEdgeActive(id, 1, 0));
    EXPECT_FALSE(graph.isEdgeActive(id, 1, 1));
  }
}

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

namespace {
void enable_static_cache(gng_type &graph) {
  auto params = graph.getParams();
  params.enable_static_collision_cache = true;
  graph.setParams(params);
}
std::vector<char> read_bytes(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
}  // 無名名前空間の終端

TEST(gng_collision_filter, static_cache_preserves_all_layers_and_saved_graph) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker cached_checker(is_in_narrow_collision_band);
  fake_collision_checker reference_checker(is_in_narrow_collision_band);
  gng_type cached(1, 3, &chain), reference(1, 3, &chain);
  ASSERT_TRUE(cached.load(files.path("input.bin")));
  ASSERT_TRUE(reference.load(files.path("input.bin")));
  enable_static_cache(cached);
  cached.setSelfCollisionChecker(&cached_checker);
  reference.setSelfCollisionChecker(&reference_checker);
  cached.strictFilter();
  reference.strictFilter();
  expect_only_safe_edge(cached);
  expect_only_safe_edge(reference);
  cached_checker.checked_angles.clear();
  cached.strictFilter();
  reference.strictFilter();
  EXPECT_TRUE(cached_checker.checked_angles.empty());
  ASSERT_TRUE(cached.save(files.path("cached.bin")));
  ASSERT_TRUE(reference.save(files.path("reference.bin")));
  EXPECT_EQ(read_bytes(files.path("cached.bin")), read_bytes(files.path("reference.bin")));
}

TEST(gng_collision_filter, static_cache_rechecks_retained_reference_edits_and_new_interior_collision) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  const auto is_colliding = [](double angle) { return angle > -0.095 && angle < -0.075; };
  fake_collision_checker cached_checker(is_colliding), reference_checker(is_colliding);
  gng_type cached(1, 3, &chain), reference(1, 3, &chain);
  ASSERT_TRUE(cached.load(files.path("input.bin")));
  ASSERT_TRUE(reference.load(files.path("input.bin")));
  enable_static_cache(cached);
  cached.setSelfCollisionChecker(&cached_checker);
  reference.setSelfCollisionChecker(&reference_checker);
  auto &retained_nodes = cached.getNodes();
  cached.strictFilter();
  reference.strictFilter();
  cached_checker.checked_angles.clear();
  retained_nodes[0].weight_angle[0] = -0.16f;
  reference.nodeAt(0).weight_angle[0] = -0.16f;
  cached.strictFilter();
  reference.strictFilter();
  EXPECT_TRUE(std::any_of(cached_checker.checked_angles.begin(),
                          cached_checker.checked_angles.end(), is_colliding));
  expect_only_safe_edge(cached);
  ASSERT_TRUE(cached.save(files.path("cached.bin")));
  ASSERT_TRUE(reference.save(files.path("reference.bin")));
  EXPECT_EQ(read_bytes(files.path("cached.bin")), read_bytes(files.path("reference.bin")));
}

TEST(gng_collision_filter, static_cache_rechecks_moved_endpoint) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker checker([](double angle) { return angle < -0.01; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  enable_static_cache(graph);
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  graph.nodeAt(1).weight_angle[0] = -0.1f;
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices(), std::vector<int>({0, 2}));
  for (int id : {0, 2}) {
    EXPECT_TRUE(graph.getNeighborsAngle(id).empty());
    for (int layer = 0; layer < 2; ++layer) EXPECT_TRUE(graph.getNeighborsCoord(id, layer).empty());
  }
}

TEST(gng_collision_filter, static_cache_avoids_duplicate_endpoint_checks) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker checker([](double) { return false; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  enable_static_cache(graph);
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  for (double endpoint : {0.0, 0.16, 0.24}) {
    EXPECT_EQ(std::count_if(checker.checked_angles.begin(), checker.checked_angles.end(),
                           [endpoint](double value) { return std::abs(value - endpoint) < 1e-7; }), 1);
  }
  EXPECT_GT(checker.checked_angles.size(), 3U);
}

TEST(gng_collision_filter, static_cache_is_invalidated_by_checker_condition_changes) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  bool has_obstacle = false;
  fake_collision_checker checker([&](double angle) { return has_obstacle && std::abs(angle) < 0.005; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  enable_static_cache(graph);
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  has_obstacle = true;
  graph.invalidate_collision_cache();
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices(), std::vector<int>({1, 2}));
  fake_collision_checker replacement([](double angle) { return angle > 0.15 && angle < 0.17; });
  graph.setSelfCollisionChecker(&replacement);
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices(), std::vector<int>({2}));
}

TEST(gng_collision_filter, cache_disabled_by_default_observes_changed_conditions) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  bool has_obstacle = false;
  fake_collision_checker checker([&](double angle) { return has_obstacle && std::abs(angle) < 0.005; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  EXPECT_FALSE(graph.getParams().enable_static_collision_cache);
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  has_obstacle = true;
  graph.strictFilter();
  EXPECT_EQ(graph.getActiveIndices(), std::vector<int>({1, 2}));
}

TEST(gng_collision_filter, static_cache_is_invalidated_by_load_and_params) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker checker([](double) { return false; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  enable_static_cache(graph);
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  checker.checked_angles.clear();
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  graph.strictFilter();
  EXPECT_FALSE(checker.checked_angles.empty());
  checker.checked_angles.clear();
  graph.setParams(graph.getParams());
  graph.strictFilter();
  EXPECT_FALSE(checker.checked_angles.empty());
}

TEST(gng_collision_filter, static_cache_rechecks_reused_node_id) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  fake_collision_checker checker([](double) { return false; });
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  auto params = graph.getParams();
  params.enable_static_collision_cache = true;
  params.ais_threshold = 0.001f;
  graph.setParams(params);
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  graph.setNodeActive(1, false);
  graph.removeInactiveElements();
  Eigen::VectorXf sample(1);
  sample[0] = 0.16f;
  graph.gngTrain({sample}, 1);
  ASSERT_EQ(graph.nodeAt(1).id, 1);
  ASSERT_FLOAT_EQ(graph.nodeAt(1).weight_angle[0], 0.16f);
  checker.checked_angles.clear();
  graph.strictFilter();
  EXPECT_TRUE(std::any_of(checker.checked_angles.begin(), checker.checked_angles.end(),
                          [](double angle) { return std::abs(angle - 0.16) < 1e-7; }));
}

TEST(gng_collision_filter, static_cache_checks_coordinate_edges_added_after_filter) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto cached_chain = make_chain();
  auto reference_chain = make_chain();
  fake_collision_checker cached_checker(is_in_narrow_collision_band);
  fake_collision_checker reference_checker(is_in_narrow_collision_band);
  gng_type cached(1, 3, &cached_chain);
  gng_type reference(1, 3, &reference_chain);
  ASSERT_TRUE(cached.load(files.path("input.bin")));
  ASSERT_TRUE(reference.load(files.path("input.bin")));
  enable_static_cache(cached);
  cached.setSelfCollisionChecker(&cached_checker);
  reference.setSelfCollisionChecker(&reference_checker);
  cached.strictFilter();
  reference.strictFilter();
  cached_checker.checked_angles.clear();
  Eigen::VectorXf sample(1);
  sample[0] = 0.0f;
  // 同位置TCPのID順で追加される、途中に干渉区間のある0--1辺。
  cached.trainCoordEdges({sample}, 1);
  reference.trainCoordEdges({sample}, 1);
  ASSERT_EQ(cached.getNeighborsCoord(0), std::vector<int>({1}));
  cached.strictFilter();
  reference.strictFilter();
  EXPECT_TRUE(std::any_of(cached_checker.checked_angles.begin(),
                          cached_checker.checked_angles.end(), is_in_narrow_collision_band));
  expect_only_safe_edge(cached);
  ASSERT_TRUE(cached.save(files.path("cached.bin")));
  ASSERT_TRUE(reference.save(files.path("reference.bin")));
  EXPECT_EQ(read_bytes(files.path("cached.bin")), read_bytes(files.path("reference.bin")));
}


TEST(gng_collision_filter, sparse_batch_checks_interiors_and_projects_only_safe_edges) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  std::vector<char> reference;
  for (int num_workers : {1, 4}) {
    gng_type graph(1, 3, &chain);
    ASSERT_TRUE(graph.load(files.path("input.bin")));
    std::vector<std::function<bool(const Eigen::VectorXf &)>> queries(num_workers,
        [](const Eigen::VectorXf &angles) { return is_in_narrow_collision_band(angles[0]); });
    graph.build_sparse_safe_graph(1, queries);
    expect_only_safe_edge(graph);
    ASSERT_TRUE(graph.save(files.path("sparse.bin")));
    const auto bytes = read_bytes(files.path("sparse.bin"));
    if (reference.empty()) reference.assign(bytes.begin(), bytes.end());
    else EXPECT_EQ(std::vector<char>(bytes.begin(), bytes.end()), reference);
  }
}

TEST(gng_collision_filter, sparse_batch_removes_colliding_nodes_before_edge_queries) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  graph.build_sparse_safe_graph(1, {[](const Eigen::VectorXf &angles) {
    return std::abs(angles[0]-.16f) < .001f;
  }});
  EXPECT_EQ(graph.getActiveIndices(), std::vector<int>({0, 2}));
  for (int idx : graph.getActiveIndices()) {
    EXPECT_TRUE(graph.getNeighborsAngle(idx).empty());
    EXPECT_TRUE(graph.getNeighborsCoord(idx, 0).empty());
    EXPECT_TRUE(graph.getNeighborsCoord(idx, 1).empty());
  }
}

TEST(gng_collision_filter, sparse_batch_propagates_worker_failure) {
  temporary_files files;
  write_fixture(files.path("input.bin"));
  auto chain = make_chain();
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  EXPECT_THROW(graph.build_sparse_safe_graph(1, {[](const Eigen::VectorXf &) -> bool {
    throw std::runtime_error("collision worker failure");
  }}), std::runtime_error);
  EXPECT_THROW(graph.build_sparse_safe_graph(1, {}), std::invalid_argument);
  EXPECT_THROW(graph.build_sparse_safe_graph(0, {[](const Eigen::VectorXf &) { return false; }}), std::invalid_argument);
}


TEST(gng_collision_filter, sparse_batch_removes_redundant_edges_and_preserves_connection) {
  temporary_files files;
  write_fixture(files.path("input.bin"), true, true, true);
  auto chain = make_chain();
  gng_type graph(1, 3, &chain);
  ASSERT_TRUE(graph.load(files.path("input.bin")));
  graph.build_sparse_safe_graph(1, {[](const Eigen::VectorXf &) { return false; }});
  EXPECT_EQ(graph.getNeighborsAngle(0), std::vector<int>({1}));
  EXPECT_EQ(graph.getNeighborsAngle(2), std::vector<int>({1}));
  EXPECT_EQ(graph.getNeighborsAngle(1).size(), 2U);
  for (int idx : graph.getActiveIndices()) {
    for (int layer = 0; layer < 2; ++layer)
      EXPECT_EQ(graph.getNeighborsAngle(idx), graph.getNeighborsCoord(idx, layer));
  }
}
