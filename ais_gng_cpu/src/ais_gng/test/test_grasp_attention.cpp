#include <ais_gng/grasp_attention.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include <random>

TEST(grasp_attention, exact_radius_empty_invalid_and_overlap) {
    fuzzrobo::grasp_attention::regions regions;
    const float points[] = {0, 0, 0, 0.02f, 0, 0, 0.04f, 0, 0, -0.02f, 0, 0, NAN, 0, 0};
    EXPECT_TRUE(regions.select(points, 5).empty());
    regions.assign({{0, 0, 0}, {0, 0, 0}, {NAN, 0, 0}}, 0.03);
    EXPECT_EQ(regions.select(points, 5), (std::vector<uint32_t>{0, 1, 3}));
    regions.assign({{0, 0, 0}}, -1);
    EXPECT_TRUE(regions.select(points, 5).empty());
    regions.assign({}, 0.03);
    EXPECT_TRUE(regions.select(points, 5).empty());
}

TEST(grasp_attention, spatial_lookup_matches_full_search) {
    std::mt19937 random(42);
    std::uniform_real_distribution<float> dist(-0.5, 0.5);
    std::vector<std::array<float, 3>> centers(100);
    std::vector<float> points(3000);
    for (auto &p : centers) for (auto &value : p) {value = dist(random);}
    for (auto &value : points) {value = dist(random);}
    fuzzrobo::grasp_attention::regions regions;
    regions.assign(centers, 0.03);
    for (std::size_t idx = 0; idx < points.size(); idx += 3) {
        bool is_near = false;
        for (const auto &center : centers) {
            double dist2 = 0;
            for (int axis = 0; axis < 3; ++axis) {
                const double delta = static_cast<double>(points[idx + axis]) - center[axis];
                dist2 += delta * delta;
            }
            is_near |= dist2 <= 0.03 * 0.03;
        }
        EXPECT_EQ(regions.contains(points.data() + idx), is_near);
    }
    centers.resize(5000);
    points.resize(300000);
    for (auto &p : centers) for (auto &value : p) {value = dist(random);}
    for (auto &value : points) {value = dist(random);}
    const auto start = std::chrono::steady_clock::now();
    regions.assign(centers, 0.03);
    const auto ids = regions.select(points.data(), points.size() / 3);
    std::cout << "5000候補中心・10万入力点の索引構築と抽出[ms]=" <<
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()
        << " 選択点数=" << ids.size() << '\n';
}
