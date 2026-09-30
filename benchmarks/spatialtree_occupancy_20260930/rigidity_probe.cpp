#include "common.hpp"
#include <limits>

// 同一剛体リンクの代表点間隔と、埋め込み更新後の間隔の比較
float point_dist(const occ::PointT &value, int first_idx, int second_idx) {
  float squared_dist = 0;
  const auto &points = arm::points();
  for (int coordinate_idx = 0; coordinate_idx < 3; ++coordinate_idx) {
    const float first = value[3 * first_idx + coordinate_idx] / std::sqrt(points[first_idx].w);
    const float second = value[3 * second_idx + coordinate_idx] / std::sqrt(points[second_idx].w);
    squared_dist += (first - second) * (first - second);
  }
  return std::sqrt(squared_dist);
}

int main() {
  constexpr int link_idx = 2;
  constexpr int first_idx = 3 * link_idx;
  constexpr int second_idx = first_idx + 2;
  std::array<float, arm::kJoints> zero_angles{};
  const float expected_dist = point_dist(arm::embed(zero_angles.data()), first_idx, second_idx);
  float max_sample_dev = 0;
  std::mt19937 sample_generator(99);
  for (int sample_idx = 0; sample_idx < 20000; ++sample_idx) {
    const auto sample = arm::randomSample(sample_generator);
    max_sample_dev = std::max(max_sample_dev, std::abs(point_dist(sample, first_idx, second_idx) - expected_dist));
  }
  auto graph = occ::makeGNG(1000, 50, 1, 0.08f);
  std::mt19937 generator(7);
  for (int step_idx = 0; step_idx < 50000; ++step_idx) graph->train_step(arm::randomSample(generator));
  float min_node_dist = std::numeric_limits<float>::max();
  float max_node_dist = 0;
  int num_invalid_nodes = 0;
  constexpr float min_dev_th = 0.0001f;
  std::vector<float> node_dist;
  for (const auto *node : graph->getActiveNodes()) {
    const float current_dist = point_dist(node->position, first_idx, second_idx);
    min_node_dist = std::min(min_node_dist, current_dist);
    max_node_dist = std::max(max_node_dist, current_dist);
    num_invalid_nodes += std::abs(current_dist - expected_dist) > min_dev_th;
    node_dist.push_back(current_dist);
  }
  std::sort(node_dist.begin(), node_dist.end());
  std::printf("{\"rule\":2,\"link_idx\":%d,\"num_nodes\":%d,\"expected_dist_m\":%.9f,\"max_sample_dev_m\":%.9f,\"min_node_dist_m\":%.9f,\"median_node_dist_m\":%.9f,\"max_node_dist_m\":%.9f,\"num_invalid_nodes\":%d,\"min_dev_th_m\":%.9f}\n",
              link_idx, graph->getNodesCount(), expected_dist, max_sample_dev,
              min_node_dist, node_dist[node_dist.size()/2], max_node_dist, num_invalid_nodes, min_dev_th);
}
