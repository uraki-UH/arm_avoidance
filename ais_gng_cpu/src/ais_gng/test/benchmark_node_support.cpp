#include <ais_gng/node_support.hpp>
#if defined(enable_baseline_build)
#include "../../gng_cpu/src/utils/node.hpp"
#include <unordered_set>
#endif
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <iostream>
#include <unordered_map>

namespace support_model = fuzzrobo::node_support;
using clock_type = std::chrono::steady_clock;

double elapsed_ms(clock_type::time_point start)
{
  return std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
}

int main(int argc, char **argv)
{
  if (argc < 4) {
    std::cerr << "使用法: benchmark_node_support 入力.bin 出力.csv baseline|capture2|support|cumulative|first [診断ディレクトリ]\n";
    return 1;
  }
  const std::string mode = argv[3];
  const bool enable_support = mode != "baseline" && mode != "capture2";
  std::ifstream input(argv[1], std::ios::binary);
  std::ofstream timing(argv[2]);
  if (!input || !timing) {return 2;}
  support_model::options config;
  if (mode == "cumulative") {config.sample_alpha = 0;}
  if (mode == "first") {config.second_weight = 0;}
  std::ofstream legacy_output, node_output, event_output;
  if (argc == 5) {
    const std::filesystem::path directory(argv[4]);
    if (!std::filesystem::create_directories(directory)) {return 7;}
    legacy_output.open(directory / "legacy_nodes.csv");
    node_output.open(directory / "nodes.csv");
    event_output.open(directory / "events.csv");
    if (!legacy_output || !node_output || !event_output) {return 8;}
    legacy_output << "frame,id,generation,px,py,pz,c00,c01,c02,c10,c11,c12,c20,c21,c22\n";
    node_output << "frame,stamp_sec,id,generation,rank,px,py,pz,axis0,axis1,axis2,scale,sample_weight_sum,has_support,c00,c01,c02,c10,c11,c12,c20,c21,c22\n";
    event_output << "frame,id,generation,rank,rx,ry,rz\n";
    legacy_output << std::setprecision(10);
    node_output << std::setprecision(10);
    event_output << std::setprecision(10);
  }
  gng_setParameter("node.num_max", 0, 5000);
  gng_setParameter("node.learning_num", 0, 5000);
  gng_setParameter("node.eta_s1", 0, 0.08);
  gng_setParameter("node.eta_s2", 0, 0.008);
  gng_setParameter("input.point_cloud_num", 0, 100000);
  gng_setParameter("input.voxel_grid_unit", 0, 0.02);
  gng_setParameter("input.x_min", 0, -10);
  gng_setParameter("input.x_max", 0, 10);
  gng_setParameter("input.y_min", 0, -10);
  gng_setParameter("input.y_max", 0, 10);
  gng_setParameter("input.z_min", 0, -10);
  gng_setParameter("input.z_max", 0, 20);
  // 設定後の初期化。ノード設定コピーと入力バッファ容量への反映。
  gng_setParameter("node.covariance_enabled", 0, 1);
  gng_setParameter("node.enable_support", 0, enable_support);
  gng_setParameter("node.support.sample_alpha", 0, config.sample_alpha);
  gng_setParameter("node.support.second_weight", 0, config.second_weight);
  if (gng_init() != SUCCESS) {return 3;}
  gng_setTrainingEventMaxWinnerRank(mode == "baseline" ? 1 : 2);
#if defined(enable_baseline_build)
  gng_setTrainingEventCapture(1);
  struct legacy_state {std::uint32_t generation = 0; node_moments moments;};
  std::unordered_map<std::uint16_t, legacy_state> legacy;
#else
  gng_setTrainingEventCapture(argc == 5 || mode == "capture2");
#endif
  LiDAR_Config lidar; lidar.point_step = 3 * sizeof(float);
  std::uint32_t frame = 0, point_num = 0;
  double checksum = 0;
  timing << "frame,stamp_sec,points,nodes,edges,gng_ms,legacy_ms,support_ms,total_ms,diagnostic_ms,checksum,mean_scale\n";
  timing << std::setprecision(12);
  double stamp_sec = 0;
  while (input.read(reinterpret_cast<char *>(&stamp_sec), sizeof(stamp_sec))) {
    input.read(reinterpret_cast<char *>(&point_num), sizeof(point_num));
    if (point_num > 1000000) {return 4;}
    std::vector<Vec3> points(point_num);
    input.read(reinterpret_cast<char *>(points.data()), point_num * sizeof(Vec3));
    if (!input) {return 5;}
    const auto total_start = clock_type::now();
    gng_setPointCloud(reinterpret_cast<const std::uint8_t *>(points.data()), point_num, &lidar);
    gng_exec();
    const double gng_ms = elapsed_ms(total_start);
    std::uint32_t event_num = 0;
    const auto *events = gng_getTrainingEvents(&event_num);
    const auto legacy_start = clock_type::now();
    const auto map = gng_getTopologicalMap();
    std::vector<gng_node_statistics> statistics(map.node_num);
#if defined(enable_baseline_build)
    for (std::size_t idx = 0; idx < event_num; ++idx) {
      const auto &event = events[idx];
      if (event.winner_rank != 1) {continue;}
      auto &state = legacy[event.winner_node_id];
      if (state.generation != event.winner_node_frame) {state = {}; state.generation = event.winner_node_frame;}
      state.moments.add_residual(Vec3f(event.residual.x, event.residual.y, event.residual.z));
    }
    std::unordered_set<std::uint16_t> active;
#endif
    for (std::size_t idx = 0; idx < map.node_num; ++idx) {
      const auto &node = map.nodes[idx];
      auto &stats = statistics[idx];
#if defined(enable_baseline_build)
      active.insert(node.id);
      const auto it = legacy.find(node.id);
      if (it != legacy.end() && it->second.generation == node.frame) {
        const auto &moments = it->second.moments;
        stats.winner_point_count = moments.count;
        for (int column = 0; column < 9; ++column) {
          stats.winner_point_covariance[column] = moments.covariance[column] / std::max(1.0, moments.count - 1);
        }
      }
#else
      stats = gng_get_node_statistics(node.id);
#endif
      for (double value : stats.winner_point_covariance) {checksum += value;}
    }
#if defined(enable_baseline_build)
    for (auto it = legacy.begin(); it != legacy.end();) {
      if (active.count(it->first) == 0) {it = legacy.erase(it);} else {++it;}
    }
#endif
    const double legacy_ms = elapsed_ms(legacy_start);
    const auto support_start = clock_type::now();
    std::vector<support_model::ellipsoid> shapes;
    if (enable_support) {
      shapes.reserve(map.node_num);
      for (const auto &stats : statistics) {shapes.push_back(support_model::make_ellipsoid(stats, config));}
    }
    const double support_ms = enable_support ? elapsed_ms(support_start) : 0;
    const double total_ms = elapsed_ms(total_start);
    const auto diagnostic_start = clock_type::now();
    if (node_output.is_open() && enable_support) {
      // 数値と入力イベントだけの記録。残差統計はPython側の後処理。
      for (std::size_t idx = 0; idx < map.node_num; ++idx) {
        const auto &node = map.nodes[idx];
        const auto &shape = shapes[idx];
        for (int rank = 1; rank <= 2; ++rank) {
          node_output << frame << ',' << stamp_sec << ',' << node.id << ',' << node.frame << ',' << rank;
          node_output << ',' << node.pos.x << ',' << node.pos.y << ',' << node.pos.z;
          for (int axis = 0; axis < 3; ++axis) {node_output << ',' << shape.axis_std[axis];}
          node_output << ',' << config.base_scale << ',' << statistics[idx].support_weight_sum << ',' << shape.has_support;
          for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {node_output << ',' << shape.moment(row, column);}
          }
          node_output << '\n';
        }
      }
      for (std::size_t idx = 0; idx < event_num; ++idx) {
        const auto &event = events[idx];
        event_output << frame << ',' << event.winner_node_id << ',' << event.winner_node_frame << ',' << event.winner_rank
          << ',' << event.residual.x << ',' << event.residual.y << ',' << event.residual.z
          << '\n';
      }
      for (std::size_t idx = 0; idx < map.node_num; ++idx) {
        const auto &node = map.nodes[idx];
        legacy_output << frame << ',' << node.id << ',' << node.frame << ',' << node.pos.x << ',' << node.pos.y << ',' << node.pos.z;
        for (double value : statistics[idx].winner_point_covariance) {legacy_output << ',' << value;}
        legacy_output << '\n';
      }
    }
    const double diagnostic_ms = elapsed_ms(diagnostic_start);
    double scale_sum = 0;
    std::size_t support_num = 0;
    for (const auto &shape : shapes) {
      if (shape.has_support) {scale_sum += config.base_scale; ++support_num;}
    }
    timing << frame << ',' << stamp_sec << ',' << point_num << ',' << map.node_num << ',' << map.edge_num / 2
      << ',' << gng_ms << ',' << legacy_ms << ',' << support_ms << ',' << total_ms << ',' << diagnostic_ms
      << ',' << checksum
      << ',' << (support_num > 0 ? scale_sum / support_num : 0) << '\n';
    ++frame;
  }
  std::cout << "frames=" << frame << " checksum=" << std::setprecision(12) << checksum << '\n';
  return frame > 0 ? 0 : 7;
}
