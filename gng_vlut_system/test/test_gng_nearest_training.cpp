#include <gtest/gtest.h>

#include "gng/GrowingNeuralGas.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace {

using gng_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
using edge_type = std::tuple<std::int32_t, std::int32_t, std::int32_t>;

class temporary_files {
public:
  temporary_files() {
    char pattern[] = "/tmp/gng_nearest_training_XXXXXX";
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

struct fixture_node {
  int id;
  Eigen::VectorXf angle;
  bool is_active = true;
};

template <typename value_type>
void write_scalar(std::ofstream &output, const value_type &value) {
  output.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

template <typename value_type>
void write_vector(std::ofstream &output, const value_type &value) {
  const Eigen::Index rows = value.rows();
  const Eigen::Index columns = 1;
  write_scalar(output, rows);
  write_scalar(output, columns);
  output.write(reinterpret_cast<const char *>(value.data()), rows * sizeof(float));
}

void write_edges(std::ofstream &output, const std::vector<edge_type> &edges) {
  write_scalar(output, static_cast<std::int32_t>(edges.size()));
  for (const auto &[first, second, age] : edges) {
    write_scalar(output, first);
    write_scalar(output, second);
    write_scalar(output, age);
    write_scalar(output, true);
  }
}

// 正規 load 経路による疎 ID・多次元・左右座標層の初期状態。
void write_fixture(const std::string &path,
                   const std::vector<fixture_node> &fixture_nodes,
                   const std::vector<edge_type> &angle_edges = {}) {
  static_assert(sizeof(bool) == 1 && sizeof(int) == sizeof(std::int32_t));
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("Fixture file creation failed");
  write_scalar(output, std::uint32_t{9});
  write_scalar(output, std::int32_t{2});
  write_scalar(output, static_cast<std::int32_t>(fixture_nodes.size()));
  for (const auto &node : fixture_nodes) {
    const Eigen::Vector3f left(node.angle[0], node.angle[1], node.angle[2]);
    const Eigen::Vector3f right(-node.angle[2], node.angle[0], -node.angle[1]);
    write_scalar(output, static_cast<std::int32_t>(node.id));
    write_scalar(output, static_cast<float>(node.id + 1) * 0.125f);
    write_scalar(output, 0.0f);
    write_vector(output, node.angle);
    write_vector(output, left);
    write_scalar(output, std::int32_t{2});
    write_vector(output, left);
    write_vector(output, right);
    write_scalar(output, std::int32_t{0});
    for (bool flag : {false, false, true, node.is_active, false}) {
      write_scalar(output, flag);
    }
    const Eigen::Vector3f direction = Eigen::Vector3f::UnitX();
    write_vector(output, direction);
    write_scalar(output, 0.0f);
    write_scalar(output, 0.0f);
    write_scalar(output, 1.0f);
    write_scalar(output, false);
    write_scalar(output, 0.0f);
    write_scalar(output, 0.0f);
    write_scalar(output, false);
  }
  write_edges(output, angle_edges);
  write_edges(output, {});
  write_edges(output, {});
  if (!output) throw std::runtime_error("Fixture write failed");
}

Eigen::VectorXf axis_point(int num_dims, float value) {
  Eigen::VectorXf point = Eigen::VectorXf::Zero(num_dims);
  point[0] = value;
  return point;
}

std::vector<Eigen::VectorXf> make_samples(int num_dims, int num_samples,
                                        std::uint32_t seed) {
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> distribution(-1.0f, 1.0f);
  std::vector<Eigen::VectorXf> result;
  for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
    Eigen::VectorXf point(num_dims);
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx) {
      point[dim_idx] = distribution(generator);
    }
    result.push_back(std::move(point));
  }
  return result;
}

GNG::GngParameters make_params(bool enable_nearest_index, int max_node_num = 128) {
  GNG::GngParameters params;
  params.enable_nearest_index = enable_nearest_index;
  params.max_node_num = max_node_num;
  params.start_node_num = 0;
  params.n_best_candidates = 4;
  params.ais_threshold = 100.0f;
  params.lambda = 17;
  params.max_edge_age = 8;
  params.learn_rate_s1 = 0.12f;
  params.learn_rate_s2 = 0.035f;
  return params;
}

void expect_same_graph(const gng_type &indexed, const gng_type &scanned) {
  ASSERT_EQ(indexed.getMaxNodeNum(), scanned.getMaxNodeNum());
  ASSERT_EQ(indexed.getCoordLayerCount(), scanned.getCoordLayerCount());
  EXPECT_EQ(indexed.getActiveIndices(), scanned.getActiveIndices());
  for (int node_idx = 0; node_idx < static_cast<int>(indexed.getMaxNodeNum());
       ++node_idx) {
    SCOPED_TRACE(node_idx);
    const auto &actual = indexed.nodeAt(node_idx);
    const auto &expected = scanned.nodeAt(node_idx);
    ASSERT_EQ(actual.id, expected.id);
    if (actual.id == -1) continue;
    EXPECT_EQ(actual.error_angle, expected.error_angle);
    EXPECT_EQ(actual.task_density_ema, expected.task_density_ema);
    EXPECT_EQ(actual.status.active, expected.status.active);
    EXPECT_EQ(actual.status.self_collision_free, expected.status.self_collision_free);
    EXPECT_EQ(actual.status.joint_limit_score, expected.status.joint_limit_score);
    ASSERT_EQ(actual.weight_angle.size(), expected.weight_angle.size());
    EXPECT_TRUE((actual.weight_angle.array() == expected.weight_angle.array()).all());
    EXPECT_TRUE((actual.weight_coord.array() == expected.weight_coord.array()).all());
    ASSERT_EQ(actual.weight_coords.size(), expected.weight_coords.size());
    for (std::size_t layer_idx = 0; layer_idx < actual.weight_coords.size(); ++layer_idx) {
      EXPECT_TRUE((actual.weight_coords[layer_idx].array() ==
                   expected.weight_coords[layer_idx].array()).all());
    }
    EXPECT_EQ(indexed.getNeighborsAngle(node_idx), scanned.getNeighborsAngle(node_idx));
    for (int layer_idx = 0; layer_idx < indexed.getCoordLayerCount(); ++layer_idx) {
      EXPECT_EQ(indexed.getNeighborsCoord(node_idx, layer_idx),
                scanned.getNeighborsCoord(node_idx, layer_idx));
    }
  }
}

void train_one(gng_type &indexed, gng_type &scanned, const Eigen::VectorXf &sample) {
  const std::vector<Eigen::VectorXf> samples{sample};
  indexed.gngTrain(samples, 1);
  scanned.gngTrain(samples, 1);
}

std::vector<char> read_bytes(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  return std::vector<char>(std::istreambuf_iterator<char>(input),
                           std::istreambuf_iterator<char>());
}

// 左右で異なる TCP と再現可能なサンプル列を持つ実 FK チェーン。
class deterministic_chain : public kinematics::KinematicChain {
public:
  explicit deterministic_chain(int num_dims) : num_dims_(num_dims) {
    setBase(Eigen::Vector3d::Zero());
    for (int joint_idx = 0; joint_idx < num_dims_; ++joint_idx) {
      kinematics::Link link;
      link.name = "test_link_" + std::to_string(joint_idx);
      link.vector = Eigen::Vector3d(0.04, 0.01, 0.03);
      kinematics::Joint joint;
      joint.name = "test_joint_" + std::to_string(joint_idx);
      joint.type = kinematics::JointType::Revolute;
      joint.axis1 = Eigen::Vector3d::Unit(joint_idx % 3);
      joint.values = {0.0};
      joint.min_limits = {-1.0};
      joint.max_limits = {1.0};
      addSegment(link, joint);
    }
  }

  void sampleRandomJointValues(std::vector<double> &values) const override {
    values.resize(num_dims_);
    for (double &value : values) value = distribution_(generator_);
  }

  std::size_t getArmCount() const override { return 2; }

  Eigen::Vector3d getEEFPosition(std::size_t arm_idx) const override {
    return arm_idx == 0 ? kinematics::KinematicChain::getEEFPosition()
                        : getJointPosition(num_dims_ / 2);
  }

  Eigen::Quaterniond getEEFOrientation(std::size_t arm_idx) const override {
    return arm_idx == 0 ? kinematics::KinematicChain::getEEFOrientation()
                        : getJointOrientation(num_dims_ / 2);
  }

private:
  int num_dims_;
  mutable std::mt19937 generator_{7021};
  mutable std::uniform_real_distribution<double> distribution_{-0.8, 0.8};
};

}  // 無名名前空間の終端

TEST(gng_nearest_training, moving_and_inserted_nodes_match_full_scan) {
  temporary_files files;
  class status_only_provider : public gng_type::IStatusProvider {
  public:
    bool can_modify_node_positions() const override { return false; }

    std::vector<GNG::UpdateTrigger> getTriggers() const override {
      return {GNG::UpdateTrigger::NODE_ADDED};
    }

    void update(gng_type::NodeType &node, GNG::UpdateTrigger) override {
      ++num_updates;
      node.status.joint_limit_score = static_cast<float>(num_updates);
    }

    int num_updates = 0;
  };
  for (int num_dims : {3, 7, 14}) {
    SCOPED_TRACE(num_dims);
    const auto samples = make_samples(num_dims, 720, 195 + num_dims);
    write_fixture(files.path("input.bin"),
                  {{0, samples[0]}, {3, samples[1]}, {17, samples[2]}, {41, samples[3]}});
    gng_type indexed(num_dims, 3, nullptr);
    gng_type scanned(num_dims, 3, nullptr);
    indexed.setParams(make_params(true));
    scanned.setParams(make_params(false));
    indexed.setStatsLogPath(files.path("indexed.dat"));
    scanned.setStatsLogPath(files.path("scanned.dat"));
    ASSERT_TRUE(indexed.load(files.path("input.bin")));
    ASSERT_TRUE(scanned.load(files.path("input.bin")));
    auto indexed_provider = std::make_shared<status_only_provider>();
    auto scanned_provider = std::make_shared<status_only_provider>();
    indexed.registerStatusProvider(indexed_provider);
    scanned.registerStatusProvider(scanned_provider);
    // 一つの学習呼出し内の移動・挿入・辺老化と、周期誤差減衰の一致。
    for (int batch_idx = 0; batch_idx < 24; ++batch_idx) {
      std::srand(9301 + batch_idx);
      indexed.gngTrain(samples, 30);
      std::srand(9301 + batch_idx);
      scanned.gngTrain(samples, 30);
      expect_same_graph(indexed, scanned);
    }
    expect_same_graph(indexed, scanned);
    EXPECT_GT(indexed.getActiveIndices().size(), 4U);
    EXPECT_GT(indexed_provider->num_updates, 0);
    EXPECT_EQ(indexed_provider->num_updates, scanned_provider->num_updates);
    EXPECT_FALSE(indexed.get_nodes()[0].weight_angle.isApprox(samples[0]));
    ASSERT_TRUE(indexed.save(files.path("indexed.bin")));
    ASSERT_TRUE(scanned.save(files.path("scanned.bin")));
    EXPECT_EQ(read_bytes(files.path("indexed.bin")), read_bytes(files.path("scanned.bin")));
  }
}

TEST(gng_nearest_training, aged_node_removal_and_reused_id_match_full_scan) {
  temporary_files files;
  write_fixture(files.path("input.bin"),
                {{0, axis_point(7, 0.0f)}, {1, axis_point(7, 0.1f)},
                 {2, axis_point(7, 0.12f)}}, {{0, 2, 9}});
  gng_type indexed(7, 3, nullptr);
  gng_type scanned(7, 3, nullptr);
  auto indexed_params = make_params(true, 3);
  auto scanned_params = make_params(false, 3);
  indexed_params.max_edge_age = scanned_params.max_edge_age = 2;
  indexed.setParams(indexed_params);
  scanned.setParams(scanned_params);
  indexed.setStatsLogPath(files.path("indexed.dat"));
  scanned.setStatsLogPath(files.path("scanned.dat"));
  ASSERT_TRUE(indexed.load(files.path("input.bin")));
  ASSERT_TRUE(scanned.load(files.path("input.bin")));
  // 同一呼出しの後半で削除済みノード近傍を照会するサンプル順。
  int sample_seed = 0;
  for (;; ++sample_seed) {
    std::srand(sample_seed);
    const int first_sample_idx = std::rand() % 2;
    const int second_sample_idx = std::rand() % 2;
    if (first_sample_idx == 0 && second_sample_idx == 1) break;
    ASSERT_LT(sample_seed, 1000);
  }
  const std::vector<Eigen::VectorXf> samples{
      axis_point(7, 0.01f), axis_point(7, 0.12f)};
  std::srand(sample_seed);
  indexed.gngTrain(samples, 2);
  std::srand(sample_seed);
  scanned.gngTrain(samples, 2);
  expect_same_graph(indexed, scanned);
  ASSERT_EQ(indexed.get_nodes()[2].id, -1);
  indexed_params.ais_threshold = scanned_params.ais_threshold = 0.01f;
  indexed.setParams(indexed_params);
  scanned.setParams(scanned_params);
  train_one(indexed, scanned, axis_point(7, 5.0f));
  expect_same_graph(indexed, scanned);
  ASSERT_EQ(indexed.get_nodes()[2].id, 2);
  EXPECT_EQ(indexed.get_nodes()[2].weight_angle[0], 5.0f);
  train_one(indexed, scanned, axis_point(7, 4.9f));
  expect_same_graph(indexed, scanned);
}

TEST(gng_nearest_training, inactive_existing_node_remains_a_candidate) {
  temporary_files files;
  write_fixture(files.path("input.bin"),
                {{0, axis_point(3, 0.0f), false}, {1, axis_point(3, 1.0f)},
                 {2, axis_point(3, -1.0f)}});
  gng_type indexed(3, 3, nullptr);
  gng_type scanned(3, 3, nullptr);
  indexed.setParams(make_params(true, 3));
  scanned.setParams(make_params(false, 3));
  indexed.setStatsLogPath(files.path("indexed.dat"));
  scanned.setStatsLogPath(files.path("scanned.dat"));
  ASSERT_TRUE(indexed.load(files.path("input.bin")));
  ASSERT_TRUE(scanned.load(files.path("input.bin")));
  train_one(indexed, scanned, axis_point(3, 0.1f));
  expect_same_graph(indexed, scanned);
  EXPECT_GT(indexed.get_nodes()[0].weight_angle[0], 0.0f);
  EXPECT_FALSE(indexed.get_nodes()[0].status.active);
  EXPECT_EQ(indexed.getNeighborsAngle(0), std::vector<int>({1}));
}

TEST(gng_nearest_training, reload_resize_and_enable_switch_discard_stale_points) {
  temporary_files files;
  write_fixture(files.path("first.bin"),
                {{0, axis_point(14, -0.1f)}, {9, axis_point(14, 0.1f)},
                 {27, axis_point(14, 0.4f)}});
  write_fixture(files.path("second.bin"),
                {{2, axis_point(14, 2.0f)}, {4, axis_point(14, 2.1f)},
                 {8, axis_point(14, 2.4f)}});
  gng_type indexed(14, 3, nullptr);
  gng_type scanned(14, 3, nullptr);
  indexed.setParams(make_params(true, 32));
  scanned.setParams(make_params(false, 32));
  indexed.setStatsLogPath(files.path("indexed.dat"));
  scanned.setStatsLogPath(files.path("scanned.dat"));
  ASSERT_TRUE(indexed.load(files.path("first.bin")));
  ASSERT_TRUE(scanned.load(files.path("first.bin")));
  train_one(indexed, scanned, axis_point(14, 0.02f));
  indexed.setParams(make_params(false, 32));
  train_one(indexed, scanned, axis_point(14, 0.07f));
  indexed.setParams(make_params(true, 32));
  train_one(indexed, scanned, axis_point(14, 0.09f));
  expect_same_graph(indexed, scanned);
  ASSERT_TRUE(indexed.load(files.path("second.bin")));
  ASSERT_TRUE(scanned.load(files.path("second.bin")));
  train_one(indexed, scanned, axis_point(14, 2.07f));
  expect_same_graph(indexed, scanned);
  indexed.setParams(make_params(true, 16));
  scanned.setParams(make_params(false, 16));
  ASSERT_TRUE(indexed.getActiveIndices().empty());
  ASSERT_TRUE(indexed.load(files.path("second.bin")));
  ASSERT_TRUE(scanned.load(files.path("second.bin")));
  train_one(indexed, scanned, axis_point(14, 2.31f));
  expect_same_graph(indexed, scanned);
}

TEST(gng_nearest_training, retained_mutable_references_match_full_scan) {
  temporary_files files;
  write_fixture(files.path("input.bin"),
                {{0, axis_point(7, 0.0f)}, {1, axis_point(7, 1.0f)},
                 {2, axis_point(7, 2.0f)}, {3, axis_point(7, 3.0f)}});
  for (int accessor_idx = 0; accessor_idx < 4; ++accessor_idx) {
    SCOPED_TRACE(accessor_idx);
    gng_type indexed(7, 3, nullptr);
    gng_type scanned(7, 3, nullptr);
    indexed.setParams(make_params(true, 4));
    scanned.setParams(make_params(false, 4));
    indexed.setStatsLogPath(files.path("indexed.dat"));
    scanned.setStatsLogPath(files.path("scanned.dat"));
    ASSERT_TRUE(indexed.load(files.path("input.bin")));
    ASSERT_TRUE(scanned.load(files.path("input.bin")));
    train_one(indexed, scanned, axis_point(7, 0.1f));
    auto get_mutable_node = [accessor_idx](gng_type &graph) -> gng_type::NodeType * {
      if (accessor_idx == 0) return &graph.nodeAt(3);
      if (accessor_idx == 1) return &graph.getNodes()[3];
      gng_type::NodeType *result = nullptr;
      const auto capture = [&](int node_idx, auto &node) {
        if (node_idx == 3) result = &node;
      };
      if (accessor_idx == 2) graph.forEachActive(capture);
      else graph.forEachActiveValid(capture);
      return result;
    };
    auto *indexed_node = get_mutable_node(indexed);
    auto *scanned_node = get_mutable_node(scanned);
    ASSERT_NE(indexed_node, nullptr);
    ASSERT_NE(scanned_node, nullptr);
    // 参照取得後の学習を挟んだ、同じ参照からの複数回編集。
    for (int iter = 0; iter < 5; ++iter) {
      train_one(indexed, scanned, axis_point(7, 0.15f));
      indexed_node->weight_angle = scanned_node->weight_angle = axis_point(7, -5.0f - iter);
      train_one(indexed, scanned, axis_point(7, -5.05f - iter));
      expect_same_graph(indexed, scanned);
      EXPECT_LT(indexed.get_nodes()[3].weight_angle[0], -5.0f - iter);
    }
  }
}

TEST(gng_nearest_training, both_tcp_layers_match_after_training_and_refresh) {
  temporary_files files;
  const auto samples = make_samples(14, 64, 4012);
  std::vector<fixture_node> fixture_nodes;
  for (int node_idx = 0; node_idx < 48; ++node_idx) {
    fixture_nodes.push_back({node_idx, samples[node_idx]});
  }
  write_fixture(files.path("input.bin"), fixture_nodes);
  deterministic_chain indexed_chain(14);
  deterministic_chain scanned_chain(14);
  gng_type indexed(14, 3, &indexed_chain);
  gng_type scanned(14, 3, &scanned_chain);
  indexed.setParams(make_params(true, 48));
  scanned.setParams(make_params(false, 48));
  indexed.setStatsLogPath(files.path("indexed.dat"));
  scanned.setStatsLogPath(files.path("scanned.dat"));
  ASSERT_TRUE(indexed.load(files.path("input.bin")));
  ASSERT_TRUE(scanned.load(files.path("input.bin")));
  indexed.gngTrainOnTheFly(64);
  scanned.gngTrainOnTheFly(64);
  indexed.refresh_coord_weights();
  scanned.refresh_coord_weights();
  expect_same_graph(indexed, scanned);
  EXPECT_FALSE(indexed.get_nodes()[0].weight_coords[0].isApprox(
      indexed.get_nodes()[0].weight_coords[1]));
  std::srand(1903);
  indexed.trainCoordEdges(samples, 120);
  std::srand(1903);
  scanned.trainCoordEdges(samples, 120);
  expect_same_graph(indexed, scanned);
  for (int layer_idx = 0; layer_idx < 2; ++layer_idx) {
    indexed.trainCoordEdgesOnTheFly(160, layer_idx);
    scanned.trainCoordEdgesOnTheFly(160, layer_idx);
    expect_same_graph(indexed, scanned);
    std::size_t num_neighbors = 0;
    for (int node_idx : indexed.getActiveIndices()) {
      num_neighbors += indexed.getNeighborsCoord(node_idx, layer_idx).size();
    }
    EXPECT_GT(num_neighbors, 0U);
  }
  // 索引構築後の角度学習、片側更新、一括更新による座標再同期。
  for (int iter = 0; iter < 40; ++iter) train_one(indexed, scanned, samples[iter]);
  indexed.refresh_coord_weights(1);
  scanned.refresh_coord_weights(1);
  indexed.trainCoordEdgesOnTheFly(40, 1);
  scanned.trainCoordEdgesOnTheFly(40, 1);
  expect_same_graph(indexed, scanned);
  indexed.triggerBatchUpdates();
  scanned.triggerBatchUpdates();
  indexed.trainCoordEdgesOnTheFly(40, 0);
  scanned.trainCoordEdgesOnTheFly(40, 0);
  expect_same_graph(indexed, scanned);
  ASSERT_TRUE(indexed.save(files.path("indexed.bin")));
  ASSERT_TRUE(scanned.save(files.path("scanned.bin")));
  EXPECT_EQ(read_bytes(files.path("indexed.bin")), read_bytes(files.path("scanned.bin")));
}

TEST(gng_nearest_training, provider_mutation_through_retained_other_node_reference_matches_scan) {
  temporary_files files;
  std::vector<fixture_node> fixture_nodes;
  for (int node_idx = 0; node_idx < 16; ++node_idx) {
    // load 後にも残る ID 15・16 の追加枠と、容量を保持する末尾 ID 17。
    const int node_id = node_idx < 15 ? node_idx : 17;
    fixture_nodes.push_back({node_id, axis_point(7, static_cast<float>(node_idx))});
  }
  write_fixture(files.path("input.bin"), fixture_nodes);
  gng_type indexed(7, 3, nullptr);
  gng_type scanned(7, 3, nullptr);
  auto indexed_params = make_params(true, 18);
  auto scanned_params = make_params(false, 18);
  indexed_params.ais_threshold = scanned_params.ais_threshold = 0.1f;
  indexed.setParams(indexed_params);
  scanned.setParams(scanned_params);
  indexed.setStatsLogPath(files.path("indexed.dat"));
  scanned.setStatsLogPath(files.path("scanned.dat"));
  ASSERT_TRUE(indexed.load(files.path("input.bin")));
  ASSERT_TRUE(scanned.load(files.path("input.bin")));
  ASSERT_EQ(indexed.getMaxNodeNum(), 18U);
  ASSERT_EQ(indexed.get_nodes()[15].id, -1);
  ASSERT_EQ(indexed.get_nodes()[16].id, -1);

  // provider 登録前の参照を保持し、追加対象以外のノードを変更する経路。
  class retained_node_provider : public gng_type::IStatusProvider {
  public:
    explicit retained_node_provider(gng_type::NodeType &node) : node_(node) {}

    std::vector<GNG::UpdateTrigger> getTriggers() const override {
      return {GNG::UpdateTrigger::NODE_ADDED};
    }

    void update(gng_type::NodeType &added_node, GNG::UpdateTrigger) override {
      if (added_node.id == 15) node_.weight_angle = axis_point(7, 200.0f);
    }

  private:
    gng_type::NodeType &node_;
  };
  indexed.registerStatusProvider(std::make_shared<retained_node_provider>(indexed.nodeAt(0)));
  scanned.registerStatusProvider(std::make_shared<retained_node_provider>(scanned.nodeAt(0)));

  int sample_seed = 0;
  for (;; ++sample_seed) {
    std::srand(sample_seed);
    const int first_sample_idx = std::rand() % 2;
    const int second_sample_idx = std::rand() % 2;
    if (first_sample_idx == 0 && second_sample_idx == 1) break;
    ASSERT_LT(sample_seed, 1000);
  }
  const std::vector<Eigen::VectorXf> samples{
      axis_point(7, 100.0f), axis_point(7, 200.1f)};
  // 一回目の AiS 挿入による外部編集と、同じ学習呼出し内の次回近傍探索。
  std::srand(sample_seed);
  indexed.gngTrain(samples, 2);
  std::srand(sample_seed);
  scanned.gngTrain(samples, 2);
  expect_same_graph(indexed, scanned);
  ASSERT_EQ(indexed.getActiveIndices().size(), 18U);
  EXPECT_EQ(indexed.get_nodes()[0].weight_angle[0], 200.0f);
  ASSERT_EQ(indexed.get_nodes()[15].id, 15);
  ASSERT_EQ(indexed.get_nodes()[16].id, 16);
  EXPECT_EQ(indexed.get_nodes()[15].weight_angle[0], 100.0f);
  EXPECT_EQ(indexed.get_nodes()[16].weight_angle[0], 200.1f);
  EXPECT_EQ(indexed.getNeighborsAngle(15), std::vector<int>({17}));
  EXPECT_EQ(indexed.getNeighborsAngle(16), std::vector<int>({0}));
}
