#include <gtest/gtest.h>
#include "topo_fuzzy_viewer/common/nonplane_graph.h"
#include "topo_fuzzy_viewer/protocol/topological_map_protocol.h"
#include "topo_fuzzy_viewer/common/graph_inspection.h"

namespace {
struct fixture {
    ais_gng_msgs::msg::TopologicalMap map;
    ais_gng_msgs::msg::PlaneClusterArray planes;
    std_msgs::msg::UInt32MultiArray components;
    fixture() {
        map.frame_number = 42;
        map.header.frame_id = "sensor";
        map.header.stamp.sec = 12;
        map.nodes.resize(7);
        const uint16_t ids[] = {90, 15, 99, 70, 40, 3, 500};
        for (std::size_t idx = 0; idx < map.nodes.size(); ++idx) {
            auto& node = map.nodes[idx];
            node.id = ids[idx]; node.pos.x = idx;
            node.normal.z = 1;
            node.frame = 40;
            node.nonplane_component_id = UINT32_MAX;
            node.winner_point_count = 17;
            node.winner_point_covariance[4] = 0.25f;
            node.is_boundary_candidate = true;
            node.boundary_evidence = 2;
        }
        map.nodes[0].pos.x = 100;
        map.nodes[6].pos.x = 106;
        map.edges = {1, 3, 3, 0, 0, 6, 5, 6, 1, 5};
        planes.header = map.header;
        planes.frame_number = map.frame_number;
        planes.clusters.resize(1);
        planes.clusters[0].node_indices = {0, 6};
        components.data = {42, 2, 31, 2, 1, 3, 4, 1, 5};
    }
};
}

TEST(nonplane_graph, preserves_ids_attributes_and_only_real_internal_or_attachment_edges) {
    fixture input;
    auto graph = nonplane_graph::build(input.components, input.map, &input.planes);
    ASSERT_TRUE(graph);
    EXPECT_EQ(graph->frame_number, 42U);
    EXPECT_EQ(graph->header, input.map.header);
    ASSERT_EQ(graph->nodes.size(), 5U);
    const std::vector<uint16_t> expected_ids{90, 15, 70, 3, 500};
    for (std::size_t idx = 0; idx < graph->nodes.size(); ++idx) {
        const auto& node = graph->nodes[idx];
        EXPECT_EQ(node.id, expected_ids[idx]);
        EXPECT_FLOAT_EQ(node.normal.z, 1);
        EXPECT_EQ(node.winner_point_count, 17U);
        EXPECT_FLOAT_EQ(node.winner_point_covariance[4], 0.25f);
        EXPECT_EQ(node.boundary_evidence, 2U);
    }
    EXPECT_EQ(graph->edges, std::vector<uint16_t>({1, 2, 2, 0, 3, 4}));
    ASSERT_EQ(graph->clusters.size(), 2U);
    EXPECT_EQ(graph->clusters[0].nodes, std::vector<uint16_t>({15, 70}));
    EXPECT_FLOAT_EQ(graph->clusters[0].pos.x, 2);
    EXPECT_FLOAT_EQ(graph->clusters[0].scale.x, 2);
    EXPECT_EQ(graph->nodes[0].nonplane_component_id, UINT32_MAX);
    EXPECT_EQ(graph->nodes[1].nonplane_component_id, 31U);
    EXPECT_EQ(input.map.nodes[1].nonplane_component_id, UINT32_MAX);
    const auto packet = topological_map_protocol::serialize(*graph, "/nonplane_components");
    topological_map_protocol::Header header;
    std::memcpy(&header, packet.data(), sizeof(header));
    EXPECT_EQ(header.magic, topological_map_protocol::kMagic);
    EXPECT_EQ(header.node_num, 5U);
    EXPECT_EQ(header.cluster_num, 2U);
    topological_map_protocol::NodeRecord record;
    std::memcpy(&record, packet.data() + sizeof(header) + header.tag_size + header.frame_id_size + sizeof(record), sizeof(record));
    EXPECT_EQ(record.id, 15U);
    EXPECT_EQ(record.nonplane_component_id, 31U);
    EXPECT_FLOAT_EQ(record.normal[2], 1);
    EXPECT_FLOAT_EQ(record.winner_point_covariance[4], 0.25f);
}

TEST(nonplane_graph, empty_clear_singletons_and_empty_components) {
    fixture input;
    input.components.data = {42, 0};
    auto empty = nonplane_graph::build(input.components, input.map, nullptr);
    ASSERT_TRUE(empty);
    EXPECT_TRUE(empty->nodes.empty());
    EXPECT_TRUE(empty->edges.empty());
    input.components.data = {42, 2, 8, 0, 4, 1, 5};
    auto single = nonplane_graph::build(input.components, input.map, &input.planes);
    ASSERT_TRUE(single);
    ASSERT_EQ(single->clusters.size(), 1U);
    EXPECT_EQ(single->clusters[0].id, 4U);
    EXPECT_EQ(single->clusters[0].nodes, std::vector<uint16_t>({3}));
}

TEST(nonplane_graph, mismatched_frames_and_invalid_membership_are_rejected) {
    fixture input;
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, nullptr));
    --input.planes.frame_number;
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
    input.planes.frame_number = 42;
    input.planes.header.frame_id = "other";
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
    input.planes.header = input.map.header;
    for (const std::vector<uint32_t>& data : std::vector<std::vector<uint32_t>>{
        {}, {42}, {43, 0}, {42, 0, 9}, {42, 99}, {42, 1, 5}, {42, 1, 5, 2, 1},
        {42, 1, 5, 1, 99}, {42, 1, 5, 2, 1, 1}, {42, 2, 5, 1, 1, 5, 1, 3},
        {42, 1, UINT32_MAX, 1, 1}, {42, 1, 5, 1, 0}}) {
        input.components.data = data;
        EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
    }
}

TEST(nonplane_graph, invalid_edges_duplicate_ids_and_nonfinite_positions_are_rejected) {
    fixture input;
    input.map.edges.push_back(1);
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
    input.map.edges.push_back(99);
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
    input.map.edges.resize(10);
    input.map.nodes[3].id = input.map.nodes[1].id;
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
    input.map.nodes[3].id = 70;
    input.map.nodes[0].pos.x = NAN;
    EXPECT_FALSE(nonplane_graph::build(input.components, input.map, &input.planes));
}

TEST(nonplane_graph, inspection_and_bounds_exclude_plane_anchors_from_component_membership) {
    fixture input;
    auto map = nonplane_graph::build(input.components, input.map, &input.planes);
    ASSERT_TRUE(map);
    using graph_inspection::json;
    json graph = {{"nodes", json::array()}, {"edges", map->edges}, {"clusters", json::array()}, {"frameId", "sensor"}};
    for (const auto& node : map->nodes) graph["nodes"].push_back({{"id", node.id},
        {"x", node.pos.x}, {"y", node.pos.y}, {"z", node.pos.z}, {"nonplaneComponentId", node.nonplane_component_id}});
    for (const auto& cluster : map->clusters) graph["clusters"].push_back({{"id", cluster.id}, {"nodeIds", cluster.nodes}});
    const json request = {{"source_id", "/nonplane_components"}, {"graph", graph}, {"selection", {{"kind", "node"}, {"id", 15}}}};
    const auto detail = graph_inspection::snapshot(request);
    EXPECT_EQ(detail["selection"]["id"], 31);
    EXPECT_EQ(detail["graph"]["nodes"].size(), 2U);
    EXPECT_EQ(detail["graph"]["edges"], json::array({0, 1}));
    const auto bounds = graph_inspection::bounds_list(request)["bounds"];
    ASSERT_EQ(bounds.size(), 2U);
    for (const auto& box : bounds) EXPECT_LT(box["max_position"][0].get<double>(), 100);
}
