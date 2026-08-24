// 診断用の一時プログラム。リポジトリには入れない。
// クラスタのmember_countとresidual_ratio(厚み比)の関係を実測する。
// 「大きいクラスタほど精度が上がる」がどこまで、どんな速度で成り立つかを見る。
#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>
#include <vector>
using fuzzrobo::topological_plane::incremental::ClusterOptions;
using fuzzrobo::topological_plane::incremental::Clusterizer;
using Map = ais_gng_msgs::msg::TopologicalMap;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("residual_vs_size_probe");
  Clusterizer c{ClusterOptions{}};
  int frames = 0;
  // member_countのlog2バケットごとにresidual_ratio/planarityを集める。
  std::map<int, std::vector<double>> residual_by_bucket;
  std::map<int, std::vector<double>> planarity_by_bucket;

  auto sub = node->create_subscription<Map>("/topological_map", rclcpp::QoS(1),
    [&](const Map::ConstSharedPtr &m) {
      if (frames >= 300) return;
      auto r = c.update(*m);
      frames++;
      // 定常状態に近いフレームだけ使う(最初の方は過渡状態なので除外)。
      if (frames < 60) return;
      for (const auto &cl : r.clusters.clusters) {
        const int n = static_cast<int>(cl.node_indices.size());
        if (n < 7) continue;
        int bucket = 0;
        int th = 8;
        while (n >= th && bucket < 10) { ++bucket; th *= 2; }
        residual_by_bucket[bucket].push_back(cl.residual_ratio);
        planarity_by_bucket[bucket].push_back(cl.planarity);
      }
    });
  while (rclcpp::ok() && frames < 300) { rclcpp::spin_some(node); }
  std::printf("記録完了: %d フレーム\n", frames);

  std::printf(
    "%-14s %8s %8s %8s %8s %8s\n", "member_count", "n_sample", "res_p10", "res_median",
    "res_p90", "planarity_med");
  const char *labels[] = {
    "7-8", "9-16", "17-32", "33-64", "65-128", "129-256", "257-512",
    "513-1024", "1025-2048", "2049-4096", "4097+"};
  for (auto &[bucket, values] : residual_by_bucket) {
    std::sort(values.begin(), values.end());
    auto &pvals = planarity_by_bucket[bucket];
    std::sort(pvals.begin(), pvals.end());
    if (values.empty()) continue;
    std::printf(
      "%-14s %8zu %8.3f %8.3f %8.3f %8.3f\n",
      labels[std::min<int>(bucket, 10)], values.size(),
      values[values.size() * 10 / 100], values[values.size() * 50 / 100],
      values[std::min(values.size() - 1, values.size() * 90 / 100)],
      pvals[pvals.size() * 50 / 100]);
  }
  rclcpp::shutdown();
  return 0;
}
