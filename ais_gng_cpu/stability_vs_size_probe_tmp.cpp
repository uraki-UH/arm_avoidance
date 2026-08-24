// 診断用の一時プログラム。リポジトリには入れない。
// 同一クラスタIDの、フレーム間の法線角度変化・重心位置変化(ジッター)を追跡し、
// メンバー数でバケット分けして「大きいクラスタほど安定しているか」を確認する。
#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <vector>
using fuzzrobo::topological_plane::incremental::ClusterOptions;
using fuzzrobo::topological_plane::incremental::Clusterizer;
using Map = ais_gng_msgs::msg::TopologicalMap;
static Eigen::Vector3d P(const geometry_msgs::msg::Point32 &p){return {p.x,p.y,p.z};}
static Eigen::Vector3d V(const geometry_msgs::msg::Vector3 &p){return {p.x,p.y,p.z};}

struct PrevState { Eigen::Vector3d normal, centroid; int member_count; };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stability_vs_size_probe");
  Clusterizer c{ClusterOptions{}};
  int frames = 0;
  std::map<std::uint32_t, PrevState> prev;
  // メンバー数バケットごとの (法線角度変化[deg], 重心の法線方向オフセット変化[間隔比]) を集める。
  std::map<int, std::vector<double>> angle_by_bucket;
  std::map<int, std::vector<double>> offset_by_bucket;

  auto sub = node->create_subscription<Map>("/topological_map", rclcpp::QoS(1),
    [&](const Map::ConstSharedPtr &m) {
      if (frames >= 300) return;
      auto r = c.update(*m);
      frames++;
      std::map<std::uint32_t, PrevState> current;
      for (const auto &cl : r.clusters.clusters) {
        const int n = static_cast<int>(cl.node_indices.size());
        if (n < 7) continue;
        const Eigen::Vector3d normal = V(cl.normal).normalized();
        const Eigen::Vector3d centroid = P(cl.centroid);
        current[cl.id] = {normal, centroid, n};

        if (frames >= 60) {  // 定常状態のみ
          const auto it = prev.find(cl.id);
          if (it != prev.end()) {
            double cosang = std::clamp(normal.dot(it->second.normal), -1.0, 1.0);
            double angle_deg = std::acos(std::abs(cosang)) * 180.0 / M_PI;
            // 前フレームの法線方向にみた、重心オフセットの変化量をノード間隔で正規化。
            const double offset =
              std::abs(it->second.normal.dot(centroid - it->second.centroid)) /
              std::max(1e-6, static_cast<double>(cl.local_spacing));
            int bucket = 0;
            int th = 8;
            while (n >= th && bucket < 10) { ++bucket; th *= 2; }
            angle_by_bucket[bucket].push_back(angle_deg);
            offset_by_bucket[bucket].push_back(offset);
          }
        }
      }
      prev = std::move(current);
    });
  while (rclcpp::ok() && frames < 300) { rclcpp::spin_some(node); }
  std::printf("記録完了: %d フレーム\n", frames);

  const char *labels[] = {
    "7-8", "9-16", "17-32", "33-64", "65-128", "129-256", "257-512",
    "513-1024", "1025-2048", "2049-4096", "4097+"};
  std::printf(
    "%-12s %8s | %9s %9s %9s | %9s %9s %9s\n", "member_cnt", "n_sample",
    "angle_p50", "angle_p90", "angle_max", "offs_p50", "offs_p90", "offs_max");
  for (auto &[bucket, vals] : angle_by_bucket) {
    std::sort(vals.begin(), vals.end());
    auto &ovals = offset_by_bucket[bucket];
    std::sort(ovals.begin(), ovals.end());
    if (vals.empty()) continue;
    std::printf(
      "%-12s %8zu | %9.3f %9.3f %9.3f | %9.3f %9.3f %9.3f\n",
      labels[std::min<int>(bucket, 10)], vals.size(),
      vals[vals.size() * 50 / 100], vals[vals.size() * 90 / 100], vals.back(),
      ovals[ovals.size() * 50 / 100], ovals[ovals.size() * 90 / 100], ovals.back());
  }
  rclcpp::shutdown();
  return 0;
}
