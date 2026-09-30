#include "ais_gng/topological_plane/surface_model_tracking.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <set>

namespace surface = fuzzrobo::surface_model;
using json = nlohmann::json;
struct scene {
  ais_gng_msgs::msg::TopologicalMap map;
  ais_gng_msgs::msg::PlaneClusterArray planes;
};
double cpu_ms() {
  timespec value{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value);
  return value.tv_sec * 1000.0 + value.tv_nsec * 1e-6;
}
std::vector<scene> read_scenes(const char *path) {
  std::ifstream stream(path, std::ios::binary);
  const auto records = json::from_cbor(stream);
  std::vector<scene> scenes;
  for (const auto &raw : records) {
    scene data;
    const auto &raw_map = raw.at("map");
    data.map.frame_number = raw_map.at("frame_number");
    data.map.header.frame_id = "map";
    data.map.header.stamp.sec = data.map.frame_number / 10;
    data.map.header.stamp.nanosec = (data.map.frame_number % 10) * 100000000;
    data.planes.header = data.map.header;
    data.planes.frame_number = data.map.frame_number;
    for (const auto &node : raw_map.at("nodes")) {
      ais_gng_msgs::msg::TopologicalNode value;
      value.id = node[0]; value.label = node[1];
      value.pos.x = node[2]; value.pos.y = node[3]; value.pos.z = node[4];
      value.normal.x = node[5]; value.normal.y = node[6]; value.normal.z = node[7];
      value.rho = node[8]; value.boundary_evidence = node[9];
      if (node.size() > 10) value.frame = node[10];
      data.map.nodes.push_back(value);
    }
    data.map.edges = raw_map.at("edges").get<decltype(data.map.edges)>();
    for (const auto &plane : raw.at("planes")) {
      ais_gng_msgs::msg::PlaneCluster value;
      value.id = plane.at("id");
      value.normal.x = plane["normal"][0]; value.normal.y = plane["normal"][1];
      value.normal.z = plane["normal"][2];
      value.local_spacing = plane.at("local_spacing");
      value.node_indices = plane.at("node_indices").get<decltype(value.node_indices)>();
      // 保存形式に未収録の平面統計の補完。読込段階の計算、計測時間には不算入。
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (const auto node_idx : value.node_indices) {
        const auto &point = data.map.nodes.at(node_idx).pos;
        center += Eigen::Vector3d(point.x, point.y, point.z);
      }
      if (!value.node_indices.empty()) {
        center /= static_cast<double>(value.node_indices.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto node_idx : value.node_indices) {
          const auto &point = data.map.nodes.at(node_idx).pos;
          const Eigen::Vector3d delta = Eigen::Vector3d(point.x, point.y, point.z) - center;
          covariance.noalias() += delta * delta.transpose();
        }
        covariance /= static_cast<double>(value.node_indices.size());
        value.centroid.x = center.x(); value.centroid.y = center.y(); value.centroid.z = center.z();
        for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
          value.position_covariance[3 * row + col] = covariance(row, col);
        }
      }
      data.planes.clusters.push_back(value);
    }
    scenes.push_back(std::move(data));
  }
  return scenes;
}
int main(int argc, char **argv) {
  try {
    if (argc != 6) {
      std::cerr << "measure input.cbor frames.jsonl quality.jsonl max_frames before|on|off\n";
      return 2;
    }
    const auto scenes = read_scenes(argv[1]);
    surface::options config;
    config.enable_plane_local_search = true;
    const std::string mode = argv[5];
#ifdef SURFACE_PRIORITY
    config.enable_patch_history = mode == "on";
#endif
    surface::tracker tracking;
    const auto num_frames = std::min(scenes.size(), static_cast<std::size_t>(std::stoul(argv[4])));
    std::ofstream output(argv[2]), quality_output(argv[3]);
    for (std::size_t idx = 0; idx < num_frames; ++idx) {
      const auto &data = scenes[idx];
      const auto begin = std::chrono::steady_clock::now();
      const auto begin_cpu = cpu_ms();
      const auto result = tracking.update(data.map, data.planes, config);
      const auto elapsed_cpu_ms = cpu_ms() - begin_cpu;
      const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
      std::set<std::uint32_t> curved_nodes;
      double residual_sum = 0.0, max_residual = 0.0;
      std::size_t num_residuals = 0, num_curved = 0;
      json regions = json::array();
      for (const auto &region : result.regions) {
        const auto &shape = region.shape;
        if (shape.type == "plane" || shape.type == "unknown") continue;
        ++num_curved;
        curved_nodes.insert(region.node_indices.begin(), region.node_indices.end());
        for (const auto node_idx : region.node_indices) {
          if (node_idx >= data.map.nodes.size()) throw std::runtime_error("node idx out of range");
          const auto dev = surface::model_dev(shape, data.map.nodes[node_idx]);
          if (!std::isfinite(dev.dist)) throw std::runtime_error("nonfinite model residual");
          residual_sum += dev.dist;
          max_residual = std::max(max_residual, dev.dist);
          ++num_residuals;
        }
        regions.push_back({{"id", region.id}, {"nodes", region.node_indices},
          {"type", shape.type}, {"rms", shape.rms}, {"score", shape.score},
          {"radii", {shape.radii[0], shape.radii[1]}}, {"retained", region.is_retained}});
      }
      json row = {{"frame", data.map.frame_number}, {"cpu_ms", elapsed_cpu_ms},
        {"wall_ms", elapsed_ms}, {"curvature_ms", result.curvature_ms},
        {"boundary_ms", result.boundary_ms}, {"retention_ms", result.retention_ms},
        {"support_ms", result.support_ms}, {"fits", result.model_fits},
        {"boundary_fits", result.boundary_fit_num}, {"num_patches", result.patches.size()},
        {"num_nodes", data.map.nodes.size()}, {"num_planes", data.planes.clusters.size()},
        {"num_candidate_nodes", result.num_candidate_nodes}, {"num_curved", num_curved},
        {"num_curved_nodes", curved_nodes.size()},
        {"num_residuals", num_residuals},
        {"mean_residual", num_residuals ? json(residual_sum / num_residuals) : json()},
        {"max_residual", num_residuals ? json(max_residual) : json()}};
#ifdef SURFACE_PRIORITY
      row["num_curvature_fits"] = result.num_curvature_fits;
      row["num_curvature_reused"] = result.num_curvature_reused;
      row["num_curvature_deferred"] = result.num_curvature_deferred;
#endif
      output << row.dump() << '\n';
      quality_output << json({{"frame", data.map.frame_number}, {"regions", regions},
        {"curved_nodes", curved_nodes}}).dump() << '\n';
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
