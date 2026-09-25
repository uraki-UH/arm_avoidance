#include <ais_gng/grasp_attention.hpp>
#include <ais_gng/boundary_attention.hpp>
#include <gtest/gtest.h>
#include <random>

using namespace fuzzrobo;

TEST(spatial_sampling, grasp_cell_gate_keeps_small_regions) {
    ais_gng_msgs::msg::TopologicalMap map;
    map.nodes.resize(2); map.nodes[0].id = 1; map.nodes[1].id = 2;
    map.clusters.resize(1); map.clusters[0].nodes = {1, 2};
    grasp_attention::regions regions;
    regions.assign(map, {{.01f, .01f, .01f}, {.02f, .02f, .02f}}, 0);
    const auto rule = grasp_attention::sampling_rule(1, .5, regions);
    gng_sampling_cell cell;
    for (int dim = 0; dim < 3; ++dim) {cell.min_pos[dim] = 0; cell.max_pos[dim] = .5;}
    const auto score = rule.cell_score(cell, rule.data);
    ASSERT_GT(score.weight, 0); ASSERT_TRUE(score.enable_point_weights);
    const float inside[] = {.015f, .015f, .015f}, outside[] = {.3f, .3f, .3f};
    EXPECT_EQ(rule.point_score(inside, rule.data), 1);
    EXPECT_EQ(rule.point_score(outside, rule.data), 0);
    for (int dim = 0; dim < 3; ++dim) {cell.min_pos[dim] = .012; cell.max_pos[dim] = .018;}
    EXPECT_FALSE(rule.cell_score(cell, rule.data).enable_point_weights);
}

TEST(spatial_sampling, boundary_cell_gate_matches_point_weights) {
    std::mt19937 random(9);
    std::uniform_real_distribution<float> dist(-2, 2), fraction(0, 1);
    std::vector<boundary_attention::point> anchors(300);
    for (auto &anchor : anchors) for (auto &value : anchor) {value = dist(random);}
    boundary_attention::sampling_data data(anchors, .03);
    const auto rule = boundary_attention::sampling_rule(2, .2, data);
    for (const double cell_size : {.001, .03, .5}) {
        for (int iter = 0; iter < 3000; ++iter) {
            gng_sampling_cell cell;
            float point[3];
            for (int dim = 0; dim < 3; ++dim) {
                cell.min_pos[dim] = dist(random); cell.max_pos[dim] = cell.min_pos[dim] + cell_size;
                point[dim] = cell.min_pos[dim] + fraction(random) * cell_size;
                cell.min_pos[dim] = std::min(cell.min_pos[dim], static_cast<double>(point[dim]));
                cell.max_pos[dim] = std::max(cell.max_pos[dim], static_cast<double>(point[dim]));
            }
            const auto direct = data.tree.weight(point, .03 * .03);
            const auto gate = rule.cell_score(cell, rule.data);
            const auto actual = gate.weight > 0 ? rule.point_score(point, rule.data) : 0;
            EXPECT_EQ(actual, direct);
        }
    }
}
