// 移動状態の周期角埋込みを用いる、共通GNG学習器のオフライン実行。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include "gng/GrowingNeuralGas.hpp"

int main(int argc, char **argv) {
  try {
    if (argc != 3) throw std::invalid_argument("学習入力JSONと出力JSONの指定が必要");
    std::ifstream stream(argv[1]);
    const auto input = nlohmann::json::parse(stream);
    if (input.at("schema") != "mobile_gng_training_v1")
      throw std::invalid_argument("学習入力の形式不正");
    std::vector<Eigen::VectorXf> samples;
    for (const auto &row : input.at("features")) {
      if (row.size() != 6) throw std::invalid_argument("学習特徴の次元不正");
      Eigen::VectorXf sample(6);
      for (int idx = 0; idx < 6; ++idx) sample[idx] = row[idx].get<float>();
      if (!sample.allFinite()) throw std::invalid_argument("学習特徴の非有限値");
      samples.push_back(sample);
    }
    if (samples.size() < 2) throw std::invalid_argument("学習サンプルの不足");
    const auto &config = input.at("config");
    GNG::GngParameters params;
    params.max_node_num = config.at("max_nodes").get<int>();
    params.lambda = config.at("insert_interval").get<int>();
    params.max_edge_age = config.at("max_edge_age").get<int>();
    params.learn_rate_s1 = config.at("learn_rate_winner").get<float>();
    params.learn_rate_s2 = config.at("learn_rate_neighbor").get<float>();
    // 誤差最大ノードの分割によるGNG。距離によるAiS追加は無効。
    params.ais_threshold = std::numeric_limits<float>::max();
    params.enable_nearest_index = true;
    const int max_iter = config.at("max_iter").get<int>();
    if (params.max_node_num < 2 || params.max_node_num > 65535 ||
        params.lambda < 1 || params.max_edge_age < 1 || max_iter < 1 ||
        !(0 < params.learn_rate_s1 && params.learn_rate_s1 <= 1) ||
        !(0 < params.learn_rate_s2 && params.learn_rate_s2 <= 1))
      throw std::invalid_argument("GNG学習設定の不正");
    std::srand(config.at("seed").get<unsigned>());
    GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f> gng(6, 3, nullptr);
    gng.setParams(params);
    gng.setCollisionAware(false);
    gng.setStatsLogPath(input.at("stats_path").get<std::string>());
    std::size_t farthest = 1;
    float max_dist_sq = -1;
    for (std::size_t idx = 1; idx < samples.size(); ++idx) {
      const float dist_sq = (samples[idx] - samples[0]).squaredNorm();
      if (dist_sq > max_dist_sq) { max_dist_sq = dist_sq; farthest = idx; }
    }
    if (max_dist_sq <= 0) throw std::invalid_argument("学習特徴が全て同一");
    gng.initialize_from_samples({samples[0], samples[farthest]});
    const auto began = std::chrono::steady_clock::now();
    gng.gngTrain(samples, max_iter);
    const double train_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - began).count();
    nlohmann::json output;
    output["schema"] = "mobile_gng_prototypes_v1";
    output["train_ms"] = train_ms;
    output["max_iter"] = max_iter;
    output["features"] = nlohmann::json::array();
    output["edges"] = nlohmann::json::array();
    std::unordered_map<int, int> ids;
    for (int old_id : gng.getActiveIndices()) {
      ids[old_id] = ids.size();
      const auto &node = gng.nodeAt(old_id);
      std::vector<float> feature(node.weight_angle.data(), node.weight_angle.data()+6);
      output["features"].push_back(feature);
    }
    for (const auto &[old_id, new_id] : ids) {
      for (int neighbor : gng.getNeighborsAngle(old_id)) {
        if (ids.count(neighbor) && old_id < neighbor && gng.isEdgeActive(old_id, neighbor))
          output["edges"].push_back({new_id, ids.at(neighbor)});
      }
    }
    std::sort(output["edges"].begin(), output["edges"].end());
    std::ofstream out(argv[2]);
    out << output.dump();
    if (!out.good()) throw std::runtime_error("学習結果の保存失敗");
    std::cout << "GNG nodes=" << ids.size() << " edges=" << output["edges"].size()
              << " train_ms=" << train_ms << " iter=" << max_iter << std::endl;
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
