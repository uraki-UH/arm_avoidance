#include <gtest/gtest.h>

#include "gng/nearest_node_index.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using point_map = std::map<int, Eigen::VectorXf>;
using ranked_points = std::vector<std::pair<float, int>>;

Eigen::VectorXf random_point(int num_dims, std::mt19937 &generator) {
  std::uniform_real_distribution<float> distribution(-3.0f, 3.0f);
  Eigen::VectorXf point(num_dims);
  for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx) {
    point[dim_idx] = distribution(generator);
  }
  return point;
}

void expect_matches_scan(const GNG::nearest_node_index &tree,
                         const point_map &points,
                         const Eigen::VectorXf &query, int num_candidates) {
  ranked_points expected;
  for (const auto &[id, point] : points) {
    expected.emplace_back((query - point).squaredNorm(), id);
  }
  std::sort(expected.begin(), expected.end());
  expected.resize(std::min(expected.size(),
                            static_cast<std::size_t>(std::max(0, num_candidates))));
  ranked_points actual{{-1.0f, -1}};
  const auto calc_dist = [&](int id) { return (query - points.at(id)).squaredNorm(); };
  ASSERT_TRUE(tree.query(query.data(), num_candidates, calc_dist, actual));
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t candidate_idx = 0; candidate_idx < expected.size(); ++candidate_idx) {
    SCOPED_TRACE(candidate_idx);
    EXPECT_EQ(actual[candidate_idx].first, expected[candidate_idx].first);
    EXPECT_EQ(actual[candidate_idx].second, expected[candidate_idx].second);
  }
}

}  // 無名名前空間の終端

TEST(gng_nearest_index, dynamic_updates_sparse_ids_and_reuse_match_scan) {
  for (int num_dims : {3, 7, 14}) {
    SCOPED_TRACE(num_dims);
    GNG::nearest_node_index tree(num_dims);
    ASSERT_TRUE(tree.is_supported());
    point_map points;
    std::mt19937 generator(4391 + num_dims);
    for (int node_idx = 0; node_idx < 257; ++node_idx) {
      const int id = node_idx * 7 + 3;
      points[id] = random_point(num_dims, generator);
      tree.set_point(id, points.at(id).data());
    }
    ASSERT_EQ(tree.size(), points.size());
    ASSERT_TRUE(tree.check_invariants().empty()) << tree.check_invariants();
    for (int iter = 0; iter < 180; ++iter) {
      SCOPED_TRACE(iter);
      const int id = (iter * 37 % 257) * 7 + 3;
      if (iter % 5 == 0) {
        tree.remove_point(id);
        points.erase(id);
        // 削除済み ID に対する重複削除の冪等性。
        tree.remove_point(id);
      } else {
        points[id] = random_point(num_dims, generator);
        tree.set_point(id, points.at(id).data());
      }
      if (iter % 13 == 0) {
        const int inserted_id = 10001 + iter * 19;
        points[inserted_id] = random_point(num_dims, generator);
        tree.set_point(inserted_id, points.at(inserted_id).data());
      }
      const auto query = random_point(num_dims, generator);
      for (int num_candidates : {1, 2, 4, 11}) {
        expect_matches_scan(tree, points, query, num_candidates);
      }
      ASSERT_EQ(tree.size(), points.size());
      ASSERT_TRUE(tree.check_invariants().empty()) << tree.check_invariants();
    }
    const auto query = random_point(num_dims, generator);
    expect_matches_scan(tree, points, query, static_cast<int>(points.size()) + 10);
    tree.clear();
    points.clear();
    ASSERT_EQ(tree.size(), 0U);
    expect_matches_scan(tree, points, query, 4);
    points[50003] = query;
    tree.set_point(50003, query.data());
    expect_matches_scan(tree, points, query, 4);
    EXPECT_TRUE(tree.check_invariants().empty()) << tree.check_invariants();
  }
}

TEST(gng_nearest_index, coincident_points_and_equal_distances_use_smallest_id) {
  for (int num_dims : {3, 7, 14}) {
    SCOPED_TRACE(num_dims);
    GNG::nearest_node_index tree(num_dims);
    point_map points;
    const Eigen::VectorXf zero = Eigen::VectorXf::Zero(num_dims);
    // 逆順挿入による格納順と ID 順の分離。
    for (int id = 600; id >= 0; --id) {
      points[id * 3 + 1] = zero;
      tree.set_point(id * 3 + 1, zero.data());
    }
    expect_matches_scan(tree, points, zero, 4);
    expect_matches_scan(tree, points, zero, 601);
    for (int id : {1, 4, 7, 10}) {
      tree.remove_point(id);
      points.erase(id);
    }
    expect_matches_scan(tree, points, zero, 4);
    tree.clear();
    points.clear();
    for (int dim_idx = 0; dim_idx < num_dims; ++dim_idx) {
      for (int sign : {-1, 1}) {
        const int id = 100 - dim_idx * 2 - (sign + 1) / 2;
        Eigen::VectorXf point = zero;
        point[dim_idx] = static_cast<float>(sign);
        points[id] = point;
        tree.set_point(id, point.data());
      }
    }
    expect_matches_scan(tree, points, zero, 4);
    EXPECT_TRUE(tree.check_invariants().empty()) << tree.check_invariants();
  }
}

TEST(gng_nearest_index, float_rounding_near_ties_match_eigen_callback_order) {
  for (int num_dims : {3, 7, 14}) {
    SCOPED_TRACE(num_dims);
    GNG::nearest_node_index tree(num_dims);
    point_map points;
    const Eigen::VectorXf query = Eigen::VectorXf::Zero(num_dims);
    // double の幾何距離順と float の丸め後 ID 順の不一致候補。
    for (int id = 0; id < 160; ++id) {
      Eigen::VectorXf point = query;
      point[0] = 1.0f;
      point[1] = static_cast<float>(160 - id) * 0.000001f;
      if (num_dims > 3) point[6] = std::nextafter(0.00001f, 1.0f);
      points[id] = point;
      tree.set_point(id, point.data());
    }
    expect_matches_scan(tree, points, query, 4);
    EXPECT_EQ((query - points.at(0)).squaredNorm(),
              (query - points.at(159)).squaredNorm());
    for (int id = 0; id < 160; ++id) {
      points[id][0] = std::nextafter(1.0f, id % 2 == 0 ? 2.0f : 0.0f);
      tree.set_point(id, points.at(id).data());
    }
    expect_matches_scan(tree, points, query, 1);
    expect_matches_scan(tree, points, query, 4);
    expect_matches_scan(tree, points, query, 37);
  }
}

TEST(gng_nearest_index, fixed_three_dimensional_eigen_distance_matches_scan) {
  GNG::nearest_node_index tree(3);
  std::map<int, Eigen::Vector3f> points;
  std::mt19937 generator(7630);
  for (int id = 0; id < 193; ++id) {
    points[id] = random_point(3, generator);
    tree.set_point(id, points.at(id).data());
  }
  for (int iter = 0; iter < 90; ++iter) {
    const Eigen::Vector3f query = random_point(3, generator);
    ranked_points expected;
    for (const auto &[id, point] : points) {
      expected.emplace_back((query - point).squaredNorm(), id);
    }
    std::sort(expected.begin(), expected.end());
    expected.resize(4);
    ranked_points actual;
    ASSERT_TRUE(tree.query(query.data(), 4,
                           [&](int id) { return (query - points.at(id)).squaredNorm(); },
                           actual));
    EXPECT_EQ(actual, expected);
  }
}

TEST(gng_nearest_index, nonfinite_points_request_fallback_and_recover) {
  GNG::nearest_node_index tree(7);
  point_map points;
  points[2] = Eigen::VectorXf::Zero(7);
  points[17] = Eigen::VectorXf::Ones(7);
  tree.set_point(2, points.at(2).data());
  tree.set_point(17, points.at(17).data());
  const Eigen::VectorXf query = Eigen::VectorXf::Zero(7);
  const auto calc_dist = [&](int id) { return (query - points.at(id)).squaredNorm(); };
  for (float value : {std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()}) {
    points[17][3] = value;
    tree.set_point(17, points.at(17).data());
    ranked_points actual;
    EXPECT_FALSE(tree.query(query.data(), 2, calc_dist, actual));
    tree.remove_point(17);
    points.erase(17);
    expect_matches_scan(tree, points, query, 2);
    points[17] = Eigen::VectorXf::Ones(7);
    tree.set_point(17, points.at(17).data());
    expect_matches_scan(tree, points, query, 2);
  }
  tree.set_point(17, nullptr);
  ranked_points actual;
  EXPECT_FALSE(tree.query(query.data(), 2, calc_dist, actual));
  tree.set_point(17, points.at(17).data());
  expect_matches_scan(tree, points, query, 2);
  EXPECT_TRUE(tree.check_invariants().empty()) << tree.check_invariants();
}

TEST(gng_nearest_index, invalid_queries_and_unsupported_dimensions_request_fallback) {
  GNG::nearest_node_index tree(3);
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  tree.set_point(1, point.data());
  const auto calc_dist = [](int) { return 0.0f; };
  ranked_points actual;
  EXPECT_FALSE(tree.query(nullptr, 1, calc_dist, actual));
  for (float value : {std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity()}) {
    point[1] = value;
    EXPECT_FALSE(tree.query(point.data(), 1, calc_dist, actual));
  }
  point.setZero();
  ASSERT_TRUE(tree.query(point.data(), 0, calc_dist, actual));
  EXPECT_TRUE(actual.empty());
  ASSERT_TRUE(tree.query(point.data(), -2, calc_dist, actual));
  EXPECT_TRUE(actual.empty());
  for (int num_dims : {0, 1, 2, 4, 8, 15}) {
    SCOPED_TRACE(num_dims);
    GNG::nearest_node_index unsupported(num_dims);
    EXPECT_FALSE(unsupported.is_supported());
    Eigen::VectorXf query = Eigen::VectorXf::Zero(std::max(1, num_dims));
    unsupported.set_point(1, query.data());
    EXPECT_FALSE(unsupported.query(query.data(), 1, calc_dist, actual));
  }
}
