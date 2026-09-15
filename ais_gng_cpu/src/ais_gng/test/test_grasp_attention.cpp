#include <ais_gng/grasp_attention.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <random>

using map_type = ais_gng_msgs::msg::TopologicalMap;
using positions_type = std::vector<std::array<float, 3>>;

map_type fixture() {
    map_type map;
    for (auto id : {90, 12, 80, 3, 500}) {
        ais_gng_msgs::msg::TopologicalNode node;
        node.id = id;
        map.nodes.push_back(node);
    }
    ais_gng_msgs::msg::TopologicalCluster first, second;
    first.nodes = {90, 12};
    second.nodes = {80, 3};
    map.clusters = {first, second};
    return map;
}

TEST(grasp_attention, per_cluster_bounds_margin_and_unassigned_nodes) {
    fuzzrobo::grasp_attention::regions regions;
    auto map = fixture();
    const positions_type positions = {{{-1, -1, 0}}, {{-0.5, -0.5, 0}}, {{0.5, 0.5, 0}},
        {{1, 1, 0}}, {{100, 100, 100}}};
    const float points[] = {-0.75f, -0.75f, 0, 0, 0, 0, 0.75f, 0.75f, 0,
        1.02f, 1.02f, 0.02f, 1.04f, 1, 0, 100, 100, 100, NAN, 0, 0};
    regions.assign(map, positions, 0.03);
    // ノードから遠い物体内点と余白を採用、候補間の空間と未所属ノードを除外。
    EXPECT_EQ(regions.select(points, 7), (std::vector<uint32_t>{0, 2, 3}));
    regions.assign(map, positions, 0);
    EXPECT_EQ(regions.select(points, 7), (std::vector<uint32_t>{0, 2}));
    map.clusters.push_back(map.clusters.front());
    regions.assign(map, positions, 0.03);
    EXPECT_EQ(regions.select(points, 7), (std::vector<uint32_t>{0, 2, 3}));
}

TEST(grasp_attention, invalid_membership_and_clear) {
    fuzzrobo::grasp_attention::regions regions;
    auto map = fixture();
    positions_type positions = {{{-1, -1, 0}}, {{-0.5, -0.5, 0}}, {{0.5, 0.5, 0}},
        {{1, 1, 0}}, {{100, 100, 100}}};
    const float points[] = {-0.75f, -0.75f, 0, 0.75f, 0.75f, 0};
    map.clusters[0].nodes.push_back(999);
    regions.assign(map, positions, 0.03);
    EXPECT_EQ(regions.select(points, 2), (std::vector<uint32_t>{1}));
    positions[2][0] = NAN;
    regions.assign(map, positions, 0.03);
    EXPECT_TRUE(regions.select(points, 2).empty());
    positions[2][0] = 0.5f;
    map = fixture();
    map.nodes[1].id = map.nodes[0].id;
    regions.assign(map, positions, 0.03);
    EXPECT_TRUE(regions.select(points, 2).empty());
    map = fixture();
    regions.assign(map, positions, -1);
    EXPECT_TRUE(regions.select(points, 2).empty());
    regions.assign(map, positions, NAN);
    EXPECT_TRUE(regions.select(points, 2).empty());
    regions.assign(map, {}, 0.03);
    EXPECT_TRUE(regions.select(points, 2).empty());
    map.clusters.clear();
    regions.assign(map, positions, 0.03);
    EXPECT_TRUE(regions.select(points, 2).empty());
}

TEST(grasp_attention, bounds_build_and_selection_timing) {
    map_type map;
    positions_type positions;
    std::mt19937 random(42);
    std::uniform_real_distribution<float> dist(0, 0.1);
    for (uint32_t group = 0; group < 10; ++group) {
        ais_gng_msgs::msg::TopologicalCluster cluster;
        for (uint32_t idx = 0; idx < 500; ++idx) {
            ais_gng_msgs::msg::TopologicalNode node;
            node.id = map.nodes.size();
            cluster.nodes.push_back(node.id);
            map.nodes.push_back(node);
            positions.push_back({group * 0.2f + dist(random), dist(random), dist(random)});
        }
        map.clusters.push_back(cluster);
    }
    std::uniform_real_distribution<float> input_dist(-0.2, 2.2);
    std::vector<float> points(300000);
    for (auto &value : points) {value = input_dist(random);}
    fuzzrobo::grasp_attention::regions regions;
    const auto start = std::chrono::steady_clock::now();
    regions.assign(map, positions, 0.03);
    const auto ids = regions.select(points.data(), points.size() / 3);
    std::cout << "10候補・5000ノード・10万入力点のAABB構築と抽出[ms]=" <<
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()
        << " 選択点数=" << ids.size() << '\n';
}
