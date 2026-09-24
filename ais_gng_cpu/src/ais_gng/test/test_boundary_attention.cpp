#include <ais_gng/boundary_attention.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include <random>

using namespace fuzzrobo::boundary_attention;

TEST(boundary_attention, gaussian_radius_finite_and_duplicates) {
    const float points[] = {0, 0, 0, .01f, 0, 0, .02f, 0, 0, .031f, 0, 0, NAN, 0, 0};
    const auto weights = make_weights(points, 5, {{0, 0, 0}}, .03);
    EXPECT_FLOAT_EQ(weights[0], 1);
    EXPECT_NEAR(weights[1], std::exp(-.5), 1e-6);
    EXPECT_NEAR(weights[2], std::exp(-2), 1e-6);
    EXPECT_EQ(weights[3], 0);
    EXPECT_EQ(weights[4], 0);
    EXPECT_EQ(weights, make_weights(points, 5, {{0, 0, 0}, {0, 0, 0}, {NAN, 0, 0}}, .03));
    EXPECT_EQ(make_weights(points, 5, {}, .03), std::vector<float>(5, 0));
    EXPECT_EQ(make_weights(points, 5, {{0, 0, 0}}, 0), std::vector<float>(5, 0));
    EXPECT_EQ(make_weights(nullptr, 5, {{0, 0, 0}}, .03), std::vector<float>(5, 0));
}

TEST(boundary_attention, spatial_distance_and_large_coordinates) {
    const float points[] = {-.031f, 0, 0, -.029f, .029f, .029f, 1e30f, 0, 0};
    const auto weights = make_weights(points, 3, {{-.029f, 0, 0}}, .03);
    EXPECT_GT(weights[0], .97);
    EXPECT_EQ(weights[1], 0);
    EXPECT_EQ(weights[2], 0);
}

TEST(boundary_attention, nearest_search_matches_exhaustive_search) {
    std::mt19937 random(17);
    std::uniform_real_distribution<float> dist(-.1, .1);
    std::vector<point> anchors(51);
    std::vector<float> points(900);
    for (auto &anchor : anchors) for (auto &coord : anchor) {coord = dist(random);}
    for (auto &coord : points) {coord = dist(random);}
    const auto weights = make_weights(points.data(), 300, anchors, .03);
    for (std::size_t idx = 0; idx < weights.size(); ++idx) {
        double min_dist_sq = 1;
        for (const auto &anchor : anchors) {
            double dist_sq = 0;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                const double diff = static_cast<double>(points[idx * 3 + axis]) - anchor[axis];
                dist_sq += diff * diff;
            }
            min_dist_sq = std::min(min_dist_sq, dist_sq);
        }
        const double expected = min_dist_sq <= .0009 ? std::exp(-4.5 * min_dist_sq / .0009) : 0;
        EXPECT_NEAR(weights[idx], expected, 1e-6);
    }
}

TEST(boundary_attention, mixture_overlap_and_empty_fallback) {
    auto result = mix({0, 1}, .5, {0, 1, 3, 0}, .2);
    EXPECT_EQ(result.ids, (std::vector<uint32_t>{0, 1, 2}));
    EXPECT_NEAR(result.ratio, .7, 1e-6);
    EXPECT_NEAR(result.weights[0], .25, 1e-6);
    EXPECT_NEAR(result.weights[1], .30, 1e-6);
    EXPECT_NEAR(result.weights[2], .15, 1e-6);
    EXPECT_NEAR(mix({}, .5, {0, 1, 3}, .2).ratio, .2, 1e-6);
    EXPECT_NEAR(mix({0}, .5, {0, 0}, .2).ratio, .5, 1e-6);
    EXPECT_TRUE(mix({}, .5, {0, 0}, .2).ids.empty());
}

TEST(boundary_attention, degenerate_trees_radius_boundary_and_invalid_anchors) {
    const float points[] = {1, 0, 0, std::nextafter(1.f, 2.f), 0, 0,
        -1, 0, 0, 0, 0, 0, INFINITY, 0, 0};
    std::vector<point> anchors(257, {0, 0, 0});
    anchors.push_back({NAN, 0, 0});
    anchors.push_back({0, INFINITY, 0});
    EXPECT_EQ(make_weights(points, 5, anchors, 1),
        (std::vector<float>{static_cast<float>(std::exp(-4.5)), 0,
            static_cast<float>(std::exp(-4.5)), 1, 0}));
    EXPECT_EQ(make_weights(points, 5, {{NAN, 0, 0}}, 1), std::vector<float>(5, 0));
}

TEST(boundary_attention, anisotropic_trees_match_exhaustive_search_exactly) {
    std::mt19937 random(71);
    std::uniform_real_distribution<float> dist(-1, 1);
    // 線状・平面状・体積状、末端境界と多段分割の最近傍一致。
    for (const auto num_anchors : {1U, 16U, 17U, 257U, 2048U}) {
        for (std::size_t num_dims = 1; num_dims <= 3; ++num_dims) {
            std::vector<point> anchors(num_anchors);
            for (auto &anchor : anchors) {
                for (std::size_t dim = 0; dim < num_dims; ++dim) {
                    anchor[dim] = dist(random) * (dim == 0 ? 100.f : 1.f);
                }
            }
            std::vector<float> points(900);
            for (auto &coord : points) {coord = dist(random);}
            for (const double radius : {.03, .3, 100.0}) {
                const auto weights = make_weights(points.data(), 300, anchors, radius);
                for (std::size_t idx = 0; idx < weights.size(); ++idx) {
                    double min_dist_sq = radius * radius;
                    bool has_neighbor = false;
                    for (const auto &anchor : anchors) {
                        double dist_sq = 0;
                        for (std::size_t dim = 0; dim < 3; ++dim) {
                            const double diff = static_cast<double>(points[idx * 3 + dim]) - anchor[dim];
                            dist_sq += diff * diff;
                        }
                        if (dist_sq <= min_dist_sq) {min_dist_sq = dist_sq; has_neighbor = true;}
                    }
                    const float expected = has_neighbor ?
                        static_cast<float>(std::exp(-4.5 * min_dist_sq / (radius * radius))) : 0;
                    ASSERT_EQ(weights[idx], expected) << num_anchors << ":" << num_dims << ":" << radius;
                }
            }
        }
    }
}

TEST(boundary_attention, stale_frame_and_time_guards) {
    EXPECT_TRUE(can_reuse("map", 10, "map", 10.1, .1, .5));
    EXPECT_FALSE(can_reuse("map", 10, "camera", 10.1, .1, .5));
    EXPECT_FALSE(can_reuse("", 10, "", 10.1, .1, .5));
    EXPECT_FALSE(can_reuse("map", 10, "map", 10, .1, .5));
    EXPECT_FALSE(can_reuse("map", 10, "map", 9.9, .1, .5));
    EXPECT_FALSE(can_reuse("map", 10, "map", 10.6, .1, .5));
    EXPECT_FALSE(can_reuse("map", 10, "map", 10.1, .6, .5));
    EXPECT_FALSE(can_reuse("map", 0, "map", .1, .1, .5));
}

TEST(boundary_attention, selection_timing) {
    std::mt19937 random(42);
    std::uniform_real_distribution<float> dist(-1, 1);
    std::vector<point> anchors(2000);
    std::vector<float> points(300000);
    for (auto &anchor : anchors) for (auto &coord : anchor) {coord = dist(random);}
    for (auto &coord : points) {coord = dist(random);}
    const auto start = std::chrono::steady_clock::now();
    const auto weights = make_weights(points.data(), 100000, anchors, .03);
    std::cout << "10万点・境界2000点の選択時間[ms]: " <<
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() << '\n';
    EXPECT_GT(std::count_if(weights.begin(), weights.end(), [](float value) {return value > 0;}), 0);
}
