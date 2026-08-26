// 診断用の一時プログラム。リポジトリには入れない。
// 残差比が「メンバー数」と「面積(extent_u*extent_v)」のどちらとより強く
// 相関するかを確認する。メンバー数一定でも面積が違えば残差が変わるなら、
// 「実面積が大きいほど本物の凹凸を拾う」という説明を裏付けられる。
#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <numeric>
#include <vector>
using fuzzrobo::topological_plane::incremental::ClusterOptions;
using fuzzrobo::topological_plane::incremental::Clusterizer;
using Map = ais_gng_msgs::msg::TopologicalMap;

double pearson(const std::vector<double> &x, const std::vector<double> &y)
{
  const double n = static_cast<double>(x.size());
  const double mx = std::accumulate(x.begin(), x.end(), 0.0) / n;
  const double my = std::accumulate(y.begin(), y.end(), 0.0) / n;
  double sxy = 0, sxx = 0, syy = 0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    sxy += (x[i] - mx) * (y[i] - my);
    sxx += (x[i] - mx) * (x[i] - mx);
    syy += (y[i] - my) * (y[i] - my);
  }
  return sxy / std::sqrt(sxx * syy);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("residual_vs_area_probe");
  Clusterizer c{ClusterOptions{}};
  int frames = 0;
  std::vector<double> log_n, log_area, residual, spacing_v;
  // 33-64人バケット内だけに絞った比較用。
  std::vector<double> fixed_bucket_area, fixed_bucket_residual;

  auto sub = node->create_subscription<Map>("/topological_map", rclcpp::QoS(1),
    [&](const Map::ConstSharedPtr &m) {
      if (frames >= 300) return;
      auto r = c.update(*m);
      frames++;
      if (frames < 60) return;
      for (const auto &cl : r.clusters.clusters) {
        const int n = static_cast<int>(cl.node_indices.size());
        if (n < 7) continue;
        const double area = std::max(1e-6, static_cast<double>(cl.extent_u) * static_cast<double>(cl.extent_v));
        log_n.push_back(std::log(static_cast<double>(n)));
        log_area.push_back(std::log(area));
        residual.push_back(cl.residual_ratio);
        spacing_v.push_back(cl.local_spacing);
        if (n >= 33 && n <= 64) {
          fixed_bucket_area.push_back(area);
          fixed_bucket_residual.push_back(cl.residual_ratio);
        }
      }
    });
  while (rclcpp::ok() && frames < 300) { rclcpp::spin_some(node); }
  std::printf("記録完了: %d フレーム, サンプル数=%zu\n", frames, residual.size());

  std::printf("相関係数(全体): residual_ratio vs log(member_count) = %.3f\n",
    pearson(log_n, residual));
  std::printf("相関係数(全体): residual_ratio vs log(area)         = %.3f\n",
    pearson(log_area, residual));
  std::printf(
    "相関係数(member_count=33-64に固定): residual_ratio vs area   = %.3f (n=%zu)\n",
    fixed_bucket_area.size() >= 3 ? pearson(fixed_bucket_area, fixed_bucket_residual) : 0.0,
    fixed_bucket_area.size());

  // 面積バケット別の残差比中央値も出す。
  std::map<int, std::vector<double>> residual_by_area_bucket;
  for (std::size_t i = 0; i < residual.size(); ++i) {
    const double area = std::exp(log_area[i]);
    int bucket = static_cast<int>(std::floor(std::log2(std::max(area, 1e-4) / 0.01)));
    bucket = std::clamp(bucket, 0, 12);
    residual_by_area_bucket[bucket].push_back(residual[i]);
  }
  std::printf("面積バケット別 残差比中央値:\n");
  for (auto &[b, vals] : residual_by_area_bucket) {
    std::sort(vals.begin(), vals.end());
    const double area_lo = 0.01 * std::pow(2.0, b);
    std::printf(
      "  area~[%.4f,%.4f)m^2  n=%4zu  median_residual=%.3f\n",
      area_lo, area_lo * 2.0, vals.size(), vals[vals.size() / 2]);
  }
  rclcpp::shutdown();
  return 0;
}
