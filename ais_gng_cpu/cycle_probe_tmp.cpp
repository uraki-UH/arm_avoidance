// 診断用の一時プログラム。リポジトリには入れない。
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
  auto node = std::make_shared<rclcpp::Node>("cycle_probe");
  std::vector<Map> frames;
  auto sub = node->create_subscription<Map>("/topological_map", rclcpp::QoS(1),
    [&](const Map::ConstSharedPtr &m){ if (frames.size() < 120) frames.push_back(*m); });
  while (rclcpp::ok() && frames.size() < 120) { rclcpp::spin_some(node); }

  ClusterOptions o;
  Clusterizer c{o};
  std::map<std::uint32_t, std::vector<std::size_t>> hist;
  std::map<std::uint32_t, std::size_t> prev;
  int died_large = 0, died_small = 0, shown = 0;
  std::size_t alive_total = 0;
  double worst = 0.0;
  std::vector<int> lifetimes;

  for (std::size_t f = 0; f < frames.size(); ++f) {
    const auto t0 = std::chrono::steady_clock::now();
    auto r = c.update(frames[f]);
    worst = std::max(worst, std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-t0).count());
    alive_total += r.clusters.clusters.size();
    std::map<std::uint32_t, std::size_t> now;
    for (const auto &cl : r.clusters.clusters) {
      now[cl.id] = cl.node_indices.size();
      hist[cl.id].push_back(cl.node_indices.size());
    }
    if (f > 10) {
      for (auto &[id, sz] : prev) {
        if (now.count(id)) { continue; }
        lifetimes.push_back(int(hist[id].size()));
        if (sz >= o.min_cluster_nodes) {
          died_large++;
          if (sz >= 60 && shown < 8) {
            shown++;
            std::printf("  id=%-4u 推移: ", id);
            const auto &h = hist[id];
            const std::size_t from = h.size() > 8 ? h.size()-8 : 0;
            for (std::size_t i = from; i < h.size(); ++i) std::printf("%zu ", h[i]);
            std::printf("-> 消滅\n");
          }
        } else { died_small++; }
      }
    }
    prev = now;
  }
  std::sort(lifetimes.begin(), lifetimes.end());
  std::printf("\n消滅 %d 件 (下限以上のまま消滅 %d / 下限を割って消滅 %d)\n",
    died_large + died_small, died_large, died_small);
  std::printf("平均クラスタ数 %.1f / 寿命中央値 %d / 最大 %.1f ms\n",
    double(alive_total)/double(frames.size()),
    lifetimes.empty()?0:lifetimes[lifetimes.size()/2], worst);
  rclcpp::shutdown();
  return 0;
}
