#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <unistd.h>
#include "visualization/visualization_gng.hpp"
#include "core/common/topological_map_message_builder.hpp"

namespace rv = robot_sim::visualization;

namespace {
std::vector<rv::VisualizationGngSourcePoint> make_source() {
  std::vector<rv::VisualizationGngSourcePoint> points;
  for (int idx = 0; idx < 64; ++idx) {
    rv::VisualizationGngSourcePoint point;
    point.source_node_id = 100 + 3 * idx;
    point.position = {0.03f * (idx % 8), 0.03f * (idx / 8), 0.002f * (idx % 3)};
    point.direction = Eigen::Vector3f::UnitZ();
    point.weight_angle = Eigen::Vector2f(0.2f * (idx % 5), 0.1f * (idx % 7));
    for (int other : {idx - 8, idx + 8, idx - 1, idx + 1}) {
      if (other < 0 || other >= 64 || (std::abs(other - idx) == 1 && other / 8 != idx / 8)) continue;
      point.coord_neighbor_source_node_ids.push_back(100 + 3 * other);
    }
    point.angle_neighbor_source_node_ids = {100 + 3 * ((idx + 9) % 64)};
    points.push_back(std::move(point));
  }
  return points;
}

rv::VisualizationGngTrainingParams params() {
  rv::VisualizationGngTrainingParams out;
  out.target_nodes = 8;
  out.iterations = 2000;
  out.insertion_interval = 40;
  return out;
}
}

TEST(visualization_gng, spatial_membership_is_independent_of_joint_angles) {
  auto source = make_source();
  const auto before = rv::trainVisualizationGng(source, 0, params());
  for (auto &point : source) point.weight_angle *= static_cast<float>(point.source_node_id);
  const auto after = rv::trainVisualizationGng(source, 0, params());
  ASSERT_EQ(before.nodes.size(), after.nodes.size());
  ASSERT_GT(after.nodes.size(), 1U);
  EXPECT_EQ(before.edges, after.edges);
  std::map<int, const rv::VisualizationGngSourcePoint *> by_id;
  for (const auto &point : source) by_id.emplace(point.source_node_id, &point);
  std::set<int> members;
  bool has_summary_position = false;
  for (std::size_t idx = 0; idx < after.nodes.size(); ++idx) {
    const auto &node = after.nodes[idx];
    EXPECT_EQ(before.nodes[idx].source_node_ids, node.source_node_ids);
    EXPECT_EQ(before.nodes[idx].position, node.position);
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    ASSERT_FALSE(node.source_node_ids.empty());
    for (int id : node.source_node_ids) {
      EXPECT_TRUE(members.insert(id).second);
      center += by_id.at(id)->position.cast<double>();
    }
    center /= node.source_node_ids.size();
    EXPECT_LT((center - node.position.cast<double>()).norm(), 1e-7);
    const auto *representative = by_id.at(node.representative_source_node_id);
    EXPECT_EQ(node.representative_joint_angle, representative->weight_angle);
    has_summary_position = has_summary_position || (node.position - representative->position).norm() > 1e-5f;
  }
  EXPECT_EQ(members.size(), source.size());
  EXPECT_TRUE(has_summary_position);
}

TEST(visualization_gng, edges_are_exactly_the_original_graph_quotient) {
  const auto source = make_source();
  const auto model = rv::trainVisualizationGng(source, 0, params());
  std::map<int, std::uint32_t> groups;
  for (std::uint32_t idx = 0; idx < model.nodes.size(); ++idx)
    for (int id : model.nodes[idx].source_node_ids) groups.emplace(id, idx);
  std::set<std::pair<std::uint32_t, std::uint32_t>> expected;
  for (const auto &point : source) {
    for (int id : point.coord_neighbor_source_node_ids) {
      const auto a = groups.at(point.source_node_id), b = groups.at(id);
      if (a != b) expected.emplace(std::min(a, b), std::max(a, b));
    }
  }
  EXPECT_EQ(model.edges, (std::vector<std::pair<std::uint32_t, std::uint32_t>>(expected.begin(), expected.end())));
}

TEST(visualization_gng, fk_metadata_does_not_rewrite_graph) {
  auto source = make_source();
  auto model = rv::trainVisualizationGng(source, 0, params());
  const auto edges = model.edges;
  const auto nodes = model.nodes;
  rv::precomputeVisualizationGngTransitionPaths(source, model,
      [](const Eigen::VectorXf &angle, std::uint32_t) { return Eigen::Vector3f(angle[0], angle[1], 0); });
  EXPECT_EQ(model.edges, edges);
  EXPECT_FALSE(model.transition_paths.empty());
  for (std::size_t idx = 0; idx < nodes.size(); ++idx) EXPECT_EQ(model.nodes[idx].position, nodes[idx].position);
  for (auto &point : source) point.angle_neighbor_source_node_ids.clear();
  rv::precomputeVisualizationGngTransitionPaths(source, model,
      [](const Eigen::VectorXf &, std::uint32_t) { return Eigen::Vector3f::Zero().eval(); });
  EXPECT_EQ(model.edges, edges);
  EXPECT_TRUE(model.transition_paths.empty());
}

TEST(visualization_gng, summary_and_joint_metadata_round_trip) {
  auto source = make_source();
  for (std::size_t idx = 0; idx < source.size(); ++idx) source[idx].label = 1 + idx % 3;
  const auto model = rv::trainVisualizationGng(source, 0, params());
  const auto file = std::filesystem::temp_directory_path() /
      ("visualization_gng_test_" + std::to_string(getpid()) + ".bin");
  struct cleanup {
    std::filesystem::path path;
    ~cleanup() { std::filesystem::remove(path); }
  } cleanup_file{file};
  std::string error;
  ASSERT_TRUE(model.save(file, &error)) << error;
  rv::VisualizationGngModel loaded;
  ASSERT_TRUE(loaded.load(file, &error)) << error;
  ASSERT_EQ(model.nodes.size(), loaded.nodes.size());
  EXPECT_EQ(model.edges, loaded.edges);
  for (std::size_t idx = 0; idx < model.nodes.size(); ++idx) {
    EXPECT_EQ(model.nodes[idx].position, loaded.nodes[idx].position);
    EXPECT_EQ(model.nodes[idx].source_node_ids, loaded.nodes[idx].source_node_ids);
    EXPECT_EQ(model.nodes[idx].representative_joint_angle, loaded.nodes[idx].representative_joint_angle);
    EXPECT_EQ(model.nodes[idx].num_safe_states, loaded.nodes[idx].num_safe_states);
    EXPECT_EQ(model.nodes[idx].num_danger_states, loaded.nodes[idx].num_danger_states);
    EXPECT_EQ(model.nodes[idx].num_collision_states, loaded.nodes[idx].num_collision_states);
  }
  const auto static_model = rv::makeVisualizationGngStaticModel(loaded);
  ASSERT_TRUE(static_model.save(file, &error)) << error;
  rv::VisualizationGngStaticModel loaded_static;
  ASSERT_TRUE(loaded_static.load(file, &error)) << error;
  for (std::size_t idx = 0; idx < model.nodes.size(); ++idx) {
    EXPECT_EQ(model.nodes[idx].num_safe_states, loaded_static.nodes[idx].num_safe_states);
    EXPECT_EQ(model.nodes[idx].num_danger_states, loaded_static.nodes[idx].num_danger_states);
    EXPECT_EQ(model.nodes[idx].num_collision_states, loaded_static.nodes[idx].num_collision_states);
  }
}

TEST(visualization_gng, source_state_counts_preserve_best_label) {
  auto source = make_source();
  for (std::size_t idx = 0; idx < source.size(); ++idx) source[idx].label = 1 + idx % 3;
  const auto model = rv::trainVisualizationGng(source, 0, params());
  std::map<int, std::uint8_t> labels;
  for (const auto &point : source) labels[point.source_node_id] = point.label;
  for (const auto &node : model.nodes) {
    std::uint32_t counts[4]{};
    for (int id : node.source_node_ids) ++counts[labels.at(id)];
    EXPECT_EQ(node.num_safe_states, counts[1]);
    EXPECT_EQ(node.num_danger_states, counts[3]);
    EXPECT_EQ(node.num_collision_states, counts[2]);
    EXPECT_EQ(node.label, counts[1] ? 1 : (counts[3] ? 3 : 2));
  }
}

TEST(visualization_gng, legacy_files_have_no_invented_state_counts) {
  rv::VisualizationGngModel model;
  model.joint_angle_dimension = 1;
  rv::VisualizationGngNode node;
  node.representative_joint_angle = Eigen::VectorXf::Zero(1);
  node.representative_source_node_id = 7;
  node.source_node_ids = {7};
  node.num_safe_states = 1;
  model.nodes.push_back(node);
  const auto file = std::filesystem::temp_directory_path() /
      ("visualization_gng_legacy_" + std::to_string(getpid()) + ".bin");
  struct cleanup {
    std::filesystem::path path;
    ~cleanup() { std::filesystem::remove(path); }
  } cleanup_file{file};
  std::string error;
  for (bool is_static : {false, true}) {
    ASSERT_TRUE(is_static ? rv::makeVisualizationGngStaticModel(model).save(file, &error)
                          : model.save(file, &error)) << error;
    std::ifstream input(file, std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)), {});
    input.close();
    const std::size_t counts_offset = (is_static ? 28 : 40) + 25;
    bytes.erase(bytes.begin() + counts_offset, bytes.begin() + counts_offset + 12);
    bytes[6] = is_static ? '1' : '5';
    bytes[8] = is_static ? 1 : 5;
    std::ofstream output(file, std::ios::binary | std::ios::trunc);
    output.write(bytes.data(), bytes.size());
    output.close();
    if (is_static) {
      rv::VisualizationGngStaticModel loaded;
      ASSERT_TRUE(loaded.load(file, &error)) << error;
      EXPECT_EQ(loaded.nodes[0].num_safe_states, 0U);
    } else {
      rv::VisualizationGngModel loaded;
      ASSERT_TRUE(loaded.load(file, &error)) << error;
      EXPECT_EQ(loaded.nodes[0].num_safe_states, 0U);
    }
  }
}

TEST(visualization_gng, live_counts_update_without_geometry_or_label_changes) {
  struct state_node {
    int id = 0;
    struct state {
      bool active = true;
      bool self_collision_free = true;
      bool is_colliding = false;
      bool is_danger = false;
      Eigen::Vector3f ee_direction = Eigen::Vector3f::UnitZ();
    } status;
  };
  struct source_graph {
    std::vector<state_node> nodes;
    const std::vector<state_node> &getNodes() const { return nodes; }
  };
  auto source = std::make_shared<source_graph>();
  source->nodes.resize(100);
  rv::VisualizationGngModel model;
  model.nodes.resize(1);
  for (int idx = 0; idx < 100; ++idx) {
    source->nodes[idx].id = 3 * idx;
    source->nodes[idx].status.is_colliding = idx > 0;
    model.nodes[0].source_node_ids.push_back(3 * idx);
  }
  rclcpp::init(0, nullptr);
  struct shutdown_guard { ~shutdown_guard() { rclcpp::shutdown(); } } guard;
  rclcpp::Node node("l0_state_counts_test");
  const auto build = [&]() {
    return robot_sim::bridge::topofuzzy::buildVisualizationGngMessage(
        node, source, model, nullptr, "base_link", "base_link");
  };
  auto graph = build();
  ASSERT_EQ(graph.nodes.size(), 1U);
  EXPECT_EQ(graph.nodes[0].label, 1U);
  EXPECT_EQ(graph.nodes[0].num_safe_states, 1U);
  EXPECT_EQ(graph.nodes[0].num_collision_states, 99U);
  for (int idx = 1; idx < 60; ++idx) {
    source->nodes[idx].status.is_colliding = false;
    source->nodes[idx].status.is_danger = true;
  }
  const auto updated = build();
  EXPECT_EQ(updated.nodes[0].label, graph.nodes[0].label);
  EXPECT_EQ(updated.nodes[0].pos, graph.nodes[0].pos);
  EXPECT_EQ(updated.nodes[0].num_safe_states, 1U);
  EXPECT_EQ(updated.nodes[0].num_danger_states, 59U);
  EXPECT_EQ(updated.nodes[0].num_collision_states, 40U);
  ais_gng_msgs::msg::TopologicalMap path;
  path.header = updated.header;
  path.nodes.resize(1);
  path.nodes[0].id = 0;
  path.nodes[0].label = 2;
  std::vector<std::int32_t> mapping(300, 0);
  auto visual_path = robot_sim::bridge::topofuzzy::buildVisualizationPathMessage(
      path, updated, mapping, model, {});
  ASSERT_EQ(visual_path.nodes.size(), 1U);
  EXPECT_EQ(visual_path.nodes[0].label, 2U);
  EXPECT_EQ(visual_path.nodes[0].num_safe_states, 0U);
  EXPECT_EQ(visual_path.nodes[0].num_danger_states, 0U);
  EXPECT_EQ(visual_path.nodes[0].num_collision_states, 0U);
}

TEST(visualization_gng, negative_joint_weight_is_rejected) {
  auto config = params();
  config.joint_motion_weight = -1;
  EXPECT_THROW(rv::trainVisualizationGng(make_source(), 0, config), std::invalid_argument);
}
