#include <gtest/gtest.h>
#include "topo_fuzzy_viewer/common/graph_inspection.h"

namespace {
using graph_inspection::json;
using graph_inspection::snapshot;

json graph_request() {
    return json::parse(R"({"source_id":"/a","selection":{"kind":"node","id":30},"graph":{
        "frameId":"sensor","nodes":[
            {"id":10,"x":9,"y":0,"z":0},
            {"id":30,"x":1,"y":2,"z":3},
            {"id":50,"x":2,"y":4,"z":6}],
        "edges":[0,1,1,2],"clusters":[{"id":7,"nodeIds":[30,50]}]}})");
}

TEST(graph_inspection, membership_uses_ids_and_edges_use_indices) {
    const auto request = graph_request();
    const auto result = snapshot(request);
    EXPECT_EQ(result["source_id"], "/a");
    EXPECT_EQ(result["selection"], json({{"kind", "cluster"}, {"id", 7}}));
    EXPECT_EQ(result["graph"]["nodes"].size(), 2U);
    EXPECT_EQ(result["graph"]["edges"], json::array({0, 1}));
    EXPECT_EQ(result["graph"]["clusters"], request["graph"]["clusters"]);
    EXPECT_EQ(result["min_position"], json::array({1, 2, 3}));
    EXPECT_EQ(result["max_position"], json::array({2, 4, 6}));
    EXPECT_EQ(result["graph"]["frameId"], "sensor");
    EXPECT_EQ(request["graph"]["nodes"].size(), 3U);
}

TEST(graph_inspection, unassigned_node_is_not_guessed_as_an_object) {
    auto request = graph_request();
    request["selection"]["id"] = 10;
    request["graph"]["nodes"][0]["nonplaneComponentId"] = 0xFFFFFFFFU;
    const auto result = snapshot(request);
    EXPECT_EQ(result["graph"]["nodes"].size(), 1U);
    EXPECT_TRUE(result["graph"]["edges"].empty());
    EXPECT_EQ(result["selection"]["kind"], "node");
}

TEST(graph_inspection, hover_bounds_match_inspection_without_returning_graph) {
    auto request = graph_request();
    const auto full = snapshot(request);
    request["enable_bounds_only"] = true;
    const auto bounds = snapshot(request);
    EXPECT_FALSE(bounds.contains("graph"));
    EXPECT_EQ(bounds["selection"], full["selection"]);
    EXPECT_EQ(bounds["min_position"], full["min_position"]);
    EXPECT_EQ(bounds["max_position"], full["max_position"]);
    EXPECT_EQ(bounds["frame_id"], "sensor");
    request["selection"]["id"] = 999;
    EXPECT_THROW(snapshot(request), std::invalid_argument);
}

TEST(graph_inspection, explicit_nonplane_component_membership) {
    auto request = graph_request();
    request["graph"]["clusters"] = json::array();
    request["graph"]["nodes"][1]["nonplaneComponentId"] = 8U;
    request["graph"]["nodes"][2]["nonplaneComponentId"] = 8U;
    const auto result = snapshot(request);
    EXPECT_EQ(result["selection"]["kind"], "component");
    EXPECT_EQ(result["graph"]["nodes"].size(), 2U);
}

TEST(graph_inspection, batch_bounds_use_objects_not_individual_nodes) {
    auto request = graph_request();
    request.erase("selection");
    auto result = graph_inspection::bounds_list(request).at("bounds");
    ASSERT_EQ(result.size(), 1U);
    EXPECT_EQ(result[0]["selection"], json({{"kind", "cluster"}, {"id", 7}}));
    EXPECT_EQ(result[0]["min_position"], json::array({1, 2, 3}));
    EXPECT_EQ(result[0]["max_position"], json::array({2, 4, 6}));
    request["graph"]["nodes"][0]["nonplaneComponentId"] = 8;
    result = graph_inspection::bounds_list(request).at("bounds");
    EXPECT_EQ(result.size(), 2U);
    request["graph"]["nodes"] = json::array();
    EXPECT_TRUE(graph_inspection::bounds_list(request).at("bounds").empty());
}

TEST(graph_inspection, ambiguous_missing_and_invalid_data_fail_explicitly) {
    auto request = graph_request();
    request["graph"]["clusters"].push_back({{"id", 8}, {"nodeIds", {30}}});
    EXPECT_THROW(snapshot(request), std::invalid_argument);
    request = graph_request();
    request["selection"]["id"] = 999;
    EXPECT_THROW(snapshot(request), std::invalid_argument);
    request = graph_request();
    request["graph"]["edges"][0] = 99;
    EXPECT_THROW(snapshot(request), std::invalid_argument);
}

TEST(graph_inspection, grasp_parts_share_candidate_id_and_apply_marker_pose) {
    json request = {{"source_id", "/markers"},
        {"selection", {{"kind", "marker"}, {"id", 42}, {"ns", "grasp_plane"}}}};
    json part = {{"id", 42}, {"ns", "grasp_plane"}, {"type", "sphere_list"}, {"frameId", "sensor"},
        {"pos", {2, 3, 4}}, {"quat", {0, 0, 1, 0}}, {"points", {{1, 0, 0}}}};
    request["marker_array"]["markers"] = json::array({part});
    part["ns"] = "grasp_nonplane";
    part["points"] = {{0, 1, 0}};
    request["marker_array"]["markers"].push_back(part);
    part["ns"] = "unrelated";
    part["points"] = {{100, 100, 100}};
    request["marker_array"]["markers"].push_back(part);
    const auto result = snapshot(request);
    EXPECT_EQ(result["graph"]["nodes"].size(), 2U);
    EXPECT_EQ(result["min_position"], json::array({1, 2, 4}));
    EXPECT_EQ(result["max_position"], json::array({2, 3, 4}));
    request["enable_bounds_only"] = true;
    const auto bounds = snapshot(request);
    EXPECT_FALSE(bounds.contains("graph"));
    EXPECT_EQ(bounds["min_position"], result["min_position"]);
    EXPECT_EQ(bounds["max_position"], result["max_position"]);
    const auto batch = graph_inspection::bounds_list(request).at("bounds");
    ASSERT_EQ(batch.size(), 2U);
    const auto found = std::find_if(batch.begin(), batch.end(), [](const json& item) {
        return item["selection"]["ns"] == "grasp_plane";
    });
    ASSERT_NE(found, batch.end());
    EXPECT_EQ((*found)["min_position"], result["min_position"]);
    EXPECT_EQ((*found)["max_position"], result["max_position"]);
    std::reverse(request["marker_array"]["markers"].begin(), request["marker_array"]["markers"].end());
    EXPECT_EQ(graph_inspection::bounds_list(request).at("bounds"), batch);
    std::reverse(request["marker_array"]["markers"].begin(), request["marker_array"]["markers"].end());
    request["marker_array"]["markers"][1]["frameId"] = "other";
    EXPECT_THROW(snapshot(request), std::invalid_argument);
    EXPECT_THROW(graph_inspection::bounds_list(request), std::invalid_argument);
}
}  // テスト用名前空間
