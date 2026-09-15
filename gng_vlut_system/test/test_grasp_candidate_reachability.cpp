#include <gtest/gtest.h>
#include <limits>
#include <random>

#include "nodes/planning/goal_node_selection.hpp"

namespace {
using namespace robot_sim::planning;
using candidate = gng_control_msgs::msg::GraspCandidate;

ais_gng_msgs::msg::TopologicalMap make_map(const std::vector<float> &points) {
  ais_gng_msgs::msg::TopologicalMap map;
  map.header.frame_id = "base";
  for (const auto point : points) {
    ais_gng_msgs::msg::TopologicalNode node;
    node.id = map.nodes.size();
    node.label = 1;
    node.pos.x = point;
    node.pos.y = node.pos.z = 0.01F;
    node.normal.z = 1.0F;
    map.nodes.push_back(node);
  }
  return map;
}

gng_control_msgs::msg::GraspCandidateArray make_candidates(const std::vector<double> &points) {
  gng_control_msgs::msg::GraspCandidateArray source;
  source.header.frame_id = source.evaluation_header.frame_id = "base";
  source.voxel_size = 0.1;
  for (const auto point : points) {
    candidate item;
    item.id = 42 + source.candidates.size();
    item.state = candidate::INSIDE;
    item.pose.position.x = point;
    item.pose.position.y = item.pose.position.z = 0.01;
    item.pose.orientation.w = 1.0;
    source.candidates.push_back(item);
  }
  return source;
}

const goal_transform_lookup no_tf = [](const auto &, const auto &) { return std::nullopt; };
const goal_selection_options default_options{8, false, 0.0, 0.0};

TEST(grasp_candidate_reachability, only_inside_selects_planning_ids_without_input_mutation) {
  const auto map = make_map({0.01F, 0.21F});
  auto source = make_candidates({0.02, 0.22, 8.0});
  source.candidates[0].state = candidate::OUTSIDE;
  source.candidates[2].state = candidate::UNKNOWN;
  const auto before = source;
  const auto result = select_goal_nodes(&map, source, nullptr, default_options, no_tf);
  EXPECT_EQ(result.ids, (std::vector<int32_t>{1}));
  ASSERT_EQ(result.map.nodes.size(), 1U);
  EXPECT_EQ(result.map.nodes.front(), map.nodes[1]);
  EXPECT_TRUE(result.map.edges.empty());
  EXPECT_TRUE(result.map.clusters.empty());
  EXPECT_EQ(source, before);
}

TEST(grasp_candidate_reachability, unknown_outside_and_empty_clear_old_goals) {
  const auto map = make_map({0.01F});
  auto source = make_candidates({0.02});
  for (auto state : {candidate::UNKNOWN, candidate::OUTSIDE}) {
    source.candidates[0].state = state;
    EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
  }
  source.candidates.clear();
  EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).map.nodes.empty());
}

TEST(grasp_candidate_reachability, missing_map_tf_and_invalid_pose) {
  const auto map = make_map({0.01F});
  auto source = make_candidates({0.02});
  EXPECT_TRUE(select_goal_nodes(nullptr, source, nullptr, default_options, no_tf).ids.empty());
  source.header.frame_id = "missing";
  EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
  source.header.frame_id = "base";
  source.candidates[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
  source = make_candidates({0.02});
  source.candidates[0].pose.orientation.w = 0;
  EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
}

TEST(grasp_candidate_reachability, motion_rechecks_goal_cells_and_shares_frame_lookup) {
  const auto map = make_map({0.01F});
  auto source = make_candidates({1.02});
  source.header.frame_id = "world";
  for (const auto offset : {-1.0, 0.0, -1.0}) {
    int num_lookups = 0;
    const auto result = select_goal_nodes(&map, source, nullptr, default_options,
        [&](const std::string &target, const std::string &frame) {
          EXPECT_EQ(target, "base");
          EXPECT_EQ(frame, "world");
          ++num_lookups;
          return tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(offset, 0, 0));
        });
    EXPECT_EQ(!result.ids.empty(), offset == -1.0);
    EXPECT_EQ(num_lookups, 1);
  }
}

TEST(grasp_candidate_reachability, collision_filter_and_explicit_permission) {
  auto map = make_map({0.01F});
  map.nodes[0].label = 2;
  const auto source = make_candidates({0.02});
  EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
  auto options = default_options;
  options.allow_collision = true;
  EXPECT_EQ(select_goal_nodes(&map, source, nullptr, options, no_tf).ids, (std::vector<int32_t>{0}));
}

TEST(grasp_candidate_reachability, reachability_cells_do_not_supply_goal_ids) {
  auto map = make_map({0.01F});
  map.nodes[0].id = 81;
  const auto source = make_candidates({0.02, 0.22, 0.03});
  EXPECT_EQ(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids, (std::vector<int32_t>{81}));
  map.nodes.clear();
  EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
}

TEST(grasp_candidate_reachability, ranking_preserves_map_order_and_condition_penalty) {
  auto map = make_map({0.01F, 0.01F});
  map.nodes[0].id = 9;
  map.nodes[1].id = 3;
  auto options = default_options;
  options.num_candidates = 1;
  const auto source = make_candidates({0.02});
  EXPECT_EQ(select_goal_nodes(&map, source, nullptr, options, no_tf).ids, (std::vector<int32_t>{9}));
  ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray features;
  for (const auto &node : map.nodes) {
    ais_gng_feature_msgs::msg::TopologicalNodeFeature feature;
    feature.node_id = node.id;
    feature.manip_valid = true;
    feature.manip_condition_number = node.id == 9 ? 10.0F : 2.0F;
    features.features.push_back(feature);
  }
  options.manipulability_weight = 0.25;
  EXPECT_EQ(select_goal_nodes(&map, source, &features, options, no_tf).ids, (std::vector<int32_t>{3}));
  features.features[1].manip_valid = false;
  EXPECT_EQ(select_goal_nodes(&map, source, &features, options, no_tf).ids, (std::vector<int32_t>{9}));
  features.features[0].manip_condition_number = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(select_goal_nodes(&map, source, &features, options, no_tf).ids, (std::vector<int32_t>{9}));
}

TEST(grasp_candidate_reachability, orientation_weight_and_tf_rotation) {
  auto map = make_map({0.01F, 0.01F});
  map.nodes[1].normal.x = 1;
  map.nodes[1].normal.z = 0;
  auto source = make_candidates({0.02});
  source.header.frame_id = "rotated";
  auto options = default_options;
  options.num_candidates = 1;
  options.orientation_weight = 0.25;
  const auto result = select_goal_nodes(&map, source, nullptr, options,
      [](const auto &, const auto &) {
        tf2::Quaternion rotation;
        rotation.setRPY(0, std::acos(-1.0) / 2, 0);
        return tf2::Transform(rotation, tf2::Vector3(0.02, 0, 0.04));
      });
  EXPECT_EQ(result.ids, (std::vector<int32_t>{1}));
}

TEST(grasp_candidate_reachability, invalid_voxel_grid_and_negative_cells) {
  const auto map = make_map({-0.05F, 0.01F});
  auto source = make_candidates({-0.02});
  EXPECT_EQ(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids, (std::vector<int32_t>{0}));
  for (const auto size : {0.0, -0.1, std::numeric_limits<double>::infinity()}) {
    source.voxel_size = size;
    EXPECT_TRUE(select_goal_nodes(&map, source, nullptr, default_options, no_tf).ids.empty());
  }
}

TEST(grasp_candidate_reachability, spatial_cache_refreshes_geometry_but_reuses_metadata_updates) {
  auto map = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(make_map({0.01F, 0.21F}));
  goal_spatial_index cache;
  EXPECT_TRUE(cache.update(map));
  EXPECT_FALSE(cache.update(map));
  auto next = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(*map);
  next->header.frame_id = "moved_base";
  next->frame_number++;
  next->nodes[0].label = ais_gng_msgs::msg::TopologicalMap::WALL;
  next->nodes[1].id = 42;
  EXPECT_FALSE(cache.update(next));
  EXPECT_TRUE(cache.has_map(next.get()));
  EXPECT_FALSE(cache.has_map(map.get()));
  auto source = make_candidates({0.02, 0.22});
  source.header.frame_id = source.evaluation_header.frame_id = "moved_base";
  EXPECT_EQ(select_goal_nodes(next.get(), source, nullptr, default_options, no_tf, &cache).ids,
      (std::vector<int32_t>{42}));
  next = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(*next);
  next->nodes[1].pos.x = 9.0F;
  EXPECT_TRUE(cache.update(next));
  EXPECT_TRUE(select_goal_nodes(next.get(), source, nullptr, default_options, no_tf, &cache).ids.empty());
  next = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(*next);
  next->nodes.clear();
  EXPECT_TRUE(cache.update(next));
  EXPECT_TRUE(select_goal_nodes(next.get(), source, nullptr, default_options, no_tf, &cache).ids.empty());
  EXPECT_TRUE(cache.update(nullptr));
  EXPECT_FALSE(cache.has_map(nullptr));
}

TEST(grasp_candidate_reachability, indexed_selection_matches_full_scan_across_rotating_frames) {
  auto map = std::make_shared<ais_gng_msgs::msg::TopologicalMap>();
  map->header.frame_id = "base";
  std::mt19937 random(8427);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  ais_gng_feature_msgs::msg::TopologicalNodeFeatureArray features;
  for (std::uint16_t idx = 0; idx < 4096; ++idx) {
    auto &node = map->nodes.emplace_back();
    node.id = 4095 - idx;
    node.pos.x = uniform(random); node.pos.y = uniform(random); node.pos.z = uniform(random);
    node.normal.x = uniform(random); node.normal.y = uniform(random); node.normal.z = uniform(random);
    node.label = idx % 7 == 0 ? ais_gng_msgs::msg::TopologicalMap::WALL : 1;
    auto &feature = features.features.emplace_back();
    feature.node_id = node.id;
    feature.manip_valid = idx % 9 != 0;
    feature.manip_condition_number = 1 + idx % 100;
  }
  map->nodes[0].pos.x = std::numeric_limits<float>::quiet_NaN();
  map->nodes[1].pos.y = std::numeric_limits<float>::infinity();
  goal_spatial_index cache;
  cache.update(map);
  goal_selection_options options;
  for (int iter = 0; iter < 80; ++iter) {
    auto source = make_candidates({});
    source.header.frame_id = "source";
    source.evaluation_header.frame_id = "reach";
    source.voxel_size = 0.03 + 0.03 * (iter % 10);
    source.voxel_origin.x = uniform(random);
    source.voxel_origin.y = uniform(random);
    source.voxel_origin.z = uniform(random);
    tf2::Quaternion rotation;
    rotation.setRPY(uniform(random), uniform(random), uniform(random));
    const tf2::Transform map_to_reach(rotation, tf2::Vector3(uniform(random), uniform(random), uniform(random)));
    rotation.setRPY(uniform(random), uniform(random), uniform(random));
    const tf2::Transform source_to_map(rotation, tf2::Vector3(uniform(random), uniform(random), uniform(random)));
    const goal_transform_lookup lookup = [&](const auto &target, const auto &frame) -> std::optional<tf2::Transform> {
      if (iter == 79) return std::nullopt;
      if (target == "reach" && frame == "base") return map_to_reach;
      if (target == "reach" && frame == "source") return map_to_reach * source_to_map;
      if (target == "base" && frame == "source") return source_to_map;
      return std::nullopt;
    };
    for (int idx = 0; idx < 20; ++idx) {
      const auto &node = map->nodes[2 + random() % (map->nodes.size() - 2)];
      const auto point = source_to_map.inverse() * tf2::Vector3(node.pos.x, node.pos.y, node.pos.z);
      auto &item = source.candidates.emplace_back();
      item.pose.position.x = point.x(); item.pose.position.y = point.y(); item.pose.position.z = point.z();
      item.pose.orientation.w = 1;
      item.state = idx % 5 == 0 ? candidate::OUTSIDE : candidate::INSIDE;
    }
    options.allow_collision = iter % 2 == 0;
    options.num_candidates = 1 + iter % 9;
    const auto expected = select_goal_nodes(map.get(), source, &features, options, lookup);
    const auto actual = select_goal_nodes(map.get(), source, &features, options, lookup, &cache);
    ASSERT_EQ(actual.ids, expected.ids) << iter;
    ASSERT_EQ(actual.map, expected.map) << iter;
    EXPECT_FALSE(cache.update(map));
  }
}

TEST(grasp_candidate_reachability, indexed_cell_boundaries_duplicates_and_invalid_nodes) {
  auto map = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(make_map({
      -0.125F, 0.0F, 0.125F, std::nextafter(0.125F, 0.0F), std::nextafter(0.125F, 1.0F), 0.125F}));
  goal_spatial_index cache;
  cache.update(map);
  for (const double shift : {0.0, 0.125, -0.125, 1e8}) {
    auto source = make_candidates({-0.125 + shift, 0.0 + shift, 0.125 + shift});
    source.header.frame_id = source.evaluation_header.frame_id = "world";
    source.voxel_size = 0.125;
    const goal_transform_lookup lookup = [shift](const auto &target, const auto &) {
      return tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(target == "world" ? shift : -shift, 0, 0));
    };
    EXPECT_EQ(select_goal_nodes(map.get(), source, nullptr, default_options, lookup, &cache).map,
        select_goal_nodes(map.get(), source, nullptr, default_options, lookup).map);
  }
  map = std::make_shared<ais_gng_msgs::msg::TopologicalMap>(*map);
  for (auto &node : map->nodes) node.pos.x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(cache.update(map));
  EXPECT_FALSE(cache.update(std::make_shared<ais_gng_msgs::msg::TopologicalMap>(*map)));
}
}  // 回帰テスト用の補助定義
