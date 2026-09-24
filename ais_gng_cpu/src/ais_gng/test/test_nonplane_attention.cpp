#include "ais_gng/nonplane_attention.hpp"
#include <gtest/gtest.h>

using fuzzrobo::nonplane_attention::select_anchors;
namespace attention = fuzzrobo::boundary_attention;
namespace nonplane = fuzzrobo::topological_plane::nonplane;

TEST(nonplane_attention, component_size_gate_keeps_source_components) {
    ais_gng_msgs::msg::TopologicalMap map;
    map.nodes.resize(16);
    for (std::size_t idx = 0; idx < map.nodes.size(); ++idx) {
        map.nodes[idx].pos.x = static_cast<float>(idx);
    }
    // 5点・4点・単独点・平面6点の独立成分。
    map.edges = {0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8,
        10, 11, 11, 12, 12, 13, 13, 14, 14, 15};
    ais_gng_msgs::msg::PlaneClusterArray planes;
    planes.clusters.resize(1);
    planes.clusters[0].node_indices = {10, 11, 12, 13, 14, 15};
    const auto components = nonplane::extract_components(map, planes);
    const auto anchors = select_anchors(map, components, 5);
    ASSERT_EQ(anchors.size(), 5U);
    for (std::size_t idx = 0; idx < anchors.size(); ++idx) {EXPECT_EQ(anchors[idx][0], idx);}
    EXPECT_EQ(components.components.size(), 3U);
    EXPECT_EQ(select_anchors(map, components, 4).size(), 9U);
    EXPECT_TRUE(select_anchors(map, components, 6).empty());
}

TEST(nonplane_attention, invalid_nodes_do_not_satisfy_size_gate) {
    ais_gng_msgs::msg::TopologicalMap map;
    map.nodes.resize(5);
    nonplane::extraction_result components;
    components.components.push_back({0, {0, 1, 2, 3, 4, 999}});
    map.nodes[4].pos.z = NAN;
    EXPECT_TRUE(select_anchors(map, components, 5).empty());
    EXPECT_EQ(select_anchors(map, components, 4).size(), 4U);
    EXPECT_TRUE(select_anchors(map, {}, 5).empty());
}

TEST(nonplane_attention, current_points_only_within_selected_neighborhood) {
    const float points[] = {0, 0, 0, .2f, 0, 0, .4f, 0, 0, 8, 0, 0};
    const auto weights = attention::make_weights(points, 4, {{0, 0, 0}}, .3);
    const auto result = attention::mix({}, 0, {}, 0, weights, .5);
    EXPECT_EQ(result.ids, (std::vector<uint32_t>{0, 1}));
    EXPECT_FLOAT_EQ(result.ratio, .5);
    EXPECT_NEAR(result.weights[0] + result.weights[1], .5, 1e-6);
    EXPECT_TRUE(attention::mix({}, 0, {}, 0, std::vector<float>(4, 0), .5).ids.empty());
}

TEST(nonplane_attention, overlapping_attention_keeps_each_budget) {
    const auto result = attention::mix({0}, .1, {0, 1, 0}, .2, {0, 1, 3}, .4);
    EXPECT_EQ(result.ids, (std::vector<uint32_t>{0, 1, 2}));
    EXPECT_NEAR(result.ratio, .7, 1e-6);
    EXPECT_NEAR(result.weights[0], .1, 1e-6);
    EXPECT_NEAR(result.weights[1], .3, 1e-6);
    EXPECT_NEAR(result.weights[2], .3, 1e-6);
    EXPECT_NEAR(attention::mix({}, .1, {0, 1}, .2, {}, .4).ratio, .2, 1e-6);
}

TEST(nonplane_attention, stale_rewound_or_changed_frame_has_no_reuse) {
    EXPECT_TRUE(attention::can_reuse("world", 1, "world", 1.1, .1, .5));
    EXPECT_FALSE(attention::can_reuse("world", 1, "world", 1.6, .1, .5));
    EXPECT_FALSE(attention::can_reuse("world", 1, "world", 1.1, .6, .5));
    EXPECT_FALSE(attention::can_reuse("world", 1, "world", 1, .1, .5));
    EXPECT_FALSE(attention::can_reuse("world", 1, "world", .9, .1, .5));
    EXPECT_FALSE(attention::can_reuse("world", 1, "sensor", 1.1, .1, .5));
}
