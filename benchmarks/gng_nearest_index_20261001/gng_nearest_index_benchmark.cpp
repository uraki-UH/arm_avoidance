// 実GNGライブラリによる、固定入力と索引更新を含む通常学習の有限比較。
#include "gng/GrowingNeuralGas.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "reachability/joint_sampling.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using gng_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
using json = nlohmann::json;
using clock_type = std::chrono::steady_clock;
using position_list = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using orientation_list = std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>;

static void require(bool is_valid, const std::string &message) {
  if (!is_valid) throw std::runtime_error(message);
}

static double elapsed_sec(clock_type::time_point start) {
  return std::chrono::duration<double>(clock_type::now() - start).count();
}

// 乱数生成を固定入力列へ限定し、FKを元URDFの運動連鎖へ委譲する構成。
class replay_chain final : public kinematics::KinematicChain {
public:
  replay_chain(std::unique_ptr<kinematics::KinematicChain> chain,
               const std::vector<Eigen::VectorXf> &samples)
      : chain_(std::move(chain)), samples_(samples) {}
  void reset() { sample_idx_ = 0; }
  int getTotalDOF() const override { return chain_->getTotalDOF(); }
  std::size_t getArmCount() const override { return chain_->getArmCount(); }
  void sampleRandomJointValues(std::vector<double> &output) const override {
    const auto &sample = samples_.at(sample_idx_++ % samples_.size());
    output.assign(sample.data(), sample.data() + sample.size());
  }
  std::vector<double> sampleRandomJointValues() const override {
    std::vector<double> values;
    sampleRandomJointValues(values);
    return values;
  }
  void updateKinematics(const std::vector<double> &values) override {
    chain_->updateKinematics(values);
  }
  void forwardKinematicsAt(const std::vector<double> &values,
                           position_list &positions, orientation_list &orientations) const override {
    chain_->forwardKinematicsAt(values, positions, orientations);
  }
  const position_list &getLinkPositions() const override { return chain_->getLinkPositions(); }
  const orientation_list &getLinkOrientations() const override { return chain_->getLinkOrientations(); }
  Eigen::Vector3d getEEFPosition() const override { return chain_->getEEFPosition(); }
  Eigen::Quaterniond getEEFOrientation() const override { return chain_->getEEFOrientation(); }
  Eigen::Vector3d getEEFPosition(std::size_t arm_idx) const override { return chain_->getEEFPosition(arm_idx); }
  Eigen::Quaterniond getEEFOrientation(std::size_t arm_idx) const override { return chain_->getEEFOrientation(arm_idx); }
private:
  std::unique_ptr<kinematics::KinematicChain> chain_;
  const std::vector<Eigen::VectorXf> &samples_;
  mutable std::size_t sample_idx_ = 0;
};

// 公開ステータス通知による追加回数の観測。保存ノード情報への書込みなし。
class addition_counter final : public gng_type::IStatusProvider {
public:
  std::vector<GNG::UpdateTrigger> getTriggers() const override {
    return {GNG::UpdateTrigger::NODE_ADDED};
  }
  // 集計のみで、ノード座標も外部保持参照も編集しない通知。
  bool can_modify_node_positions() const override { return false; }
  void update(gng_type::NodeType &, GNG::UpdateTrigger) override { ++num_added; }
  std::size_t num_added = 0;
};

static void write_json(const std::filesystem::path &path, const json &value) {
  std::ofstream stream(path);
  require(bool(stream), "JSON出力先の作成失敗");
  stream << value.dump(2) << '\n';
  stream.close();
  require(bool(stream), "JSON出力の途中失敗");
}

int main(int argc, char **argv) {
  try {
    std::unordered_map<std::string, std::string> values;
    for (int arg_idx = 1; arg_idx < argc; arg_idx += 2) {
      require(arg_idx + 1 < argc, "引数値の不足");
      require(values.emplace(argv[arg_idx], argv[arg_idx + 1]).second, "引数の重複");
    }
    const auto get = [&](const std::string &name) -> const std::string & {
      require(values.count(name) == 1, "必須引数の不足: " + name);
      return values.at(name);
    };
    const auto input = std::filesystem::path(get("--input"));
    const auto output = std::filesystem::path(get("--output"));
    const auto profile = get("--profile");
    require(profile == "standard" || profile == "churn" || profile == "churn_delete", "未知の学習条件");
    require(!std::filesystem::exists(output) && !std::filesystem::is_symlink(output), "既存出力の上書き拒否");
    const std::string flag = get("--enable-nearest-index");
    require(flag == "0" || flag == "1", "索引フラグの不正値");
    const bool enable_nearest_index = flag == "1";
    const auto seed = static_cast<uint32_t>(std::stoul(get("--seed")));
    const int num_angle_iter = std::stoi(get("--num-angle-iter"));
    const int num_coord_iter = std::stoi(get("--num-coord-iter"));
    const int num_samples = std::stoi(get("--num-samples"));
    require(num_angle_iter > 0 && num_angle_iter <= 200000, "角度反復数の範囲外");
    require(num_coord_iter > 0 && num_coord_iter <= 200000, "座標反復数の範囲外");
    require(num_samples > 0 && num_samples <= 65536, "固定入力数の範囲外");
    std::filesystem::create_directories(output);
    const auto preparation_start = clock_type::now();
    gng_type source(14, 3, nullptr);
    require(source.load(input.string()), "入力GNGの読込失敗");
    require(source.getCoordLayerCount() == 2, "左右TCP層数の不一致");
    std::unordered_map<int, Eigen::VectorXf> before_angles;
    std::vector<Eigen::VectorXf> anchors;
    for (int node_id : source.getActiveIndices()) {
      const auto &q = source.nodeAt(node_id).weight_angle;
      require(q.size() == 14 && q.allFinite(), "入力関節角の不正値");
      before_angles.emplace(node_id, q);
      anchors.push_back(q);
    }
    require(!anchors.empty(), "空の入力GNG");
    simulation::RobotModel model = simulation::loadRobotFromUrdf(
        get("--urdf"), get("--resource-root"), get("--mesh-root"));
    auto actual_chain = simulation::createMultiArmKinematicChain(model,
        {{"L_shoulder_mount", "L_tcp", ""}, {"R_shoulder_mount", "R_tcp", ""}});
    require(actual_chain->getTotalDOF() == 14 && actual_chain->getArmCount() == 2, "運動連鎖の次元不一致");
    // 既存到達域生成と共通の関節範囲。上下限のない関節は[-pi, pi]の有限区間。
    const auto limits = robot_sim::reachability::collect_joint_limits(model, *actual_chain);
    std::vector<std::string> expected_names;
    for (const auto *arm : {"L", "R"}) {
      for (int joint_idx = 1; joint_idx <= 7; ++joint_idx) {
        const std::string name = std::string(arm) + "_joint" + std::to_string(joint_idx);
        const auto *joint = model.getJoint(name);
        require(joint != nullptr, "URDF関節の欠落");
        expected_names.push_back(name);
      }
    }
    std::vector<std::string> actual_names;
    for (int joint_idx = 0; joint_idx < actual_chain->getNumJoints(); ++joint_idx) {
      if (actual_chain->getJointDOF(joint_idx) > 0) actual_names.push_back(actual_chain->getJointName(joint_idx));
    }
    require(actual_names == expected_names, "関節順序の不一致");
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<Eigen::VectorXf> samples;
    samples.reserve(static_cast<std::size_t>(num_samples));
    for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
      Eigen::VectorXf q(14);
      const std::size_t anchor_idx = (static_cast<std::size_t>(sample_idx % 64) * 997 + seed) % anchors.size();
      for (int joint_idx = 0; joint_idx < 14; ++joint_idx) {
        const auto bound = limits[joint_idx];
        const double value = profile == "standard"
            ? bound.first + unit(generator) * (bound.second - bound.first)
            : anchors[anchor_idx][joint_idx] + (unit(generator) - .5) * .02;
        q[joint_idx] = static_cast<float>(std::clamp(value, bound.first + 1e-6, bound.second - 1e-6));
      }
      samples.push_back(std::move(q));
    }
    {
      std::ofstream stream(output / "samples.f32", std::ios::binary);
      for (const auto &q : samples) stream.write(reinterpret_cast<const char *>(q.data()), q.size() * sizeof(float));
      require(bool(stream), "固定入力の保存失敗");
    }
    replay_chain chain(std::move(actual_chain), samples);
    const double preparation_sec = elapsed_sec(preparation_start);
    const auto total_start = clock_type::now();
    const auto load_start = clock_type::now();
    gng_type graph(14, 3, &chain);
    GNG::GngParameters params;
    // 通常max/long設定と共通の学習パラメータ。
    params.max_edge_age = 2000;
    params.beta = .0005f;
    params.n_best_candidates = 4;
    params.ais_threshold = 10.0f;
    params.enable_nearest_index = enable_nearest_index;
    params.max_node_num = static_cast<int>(source.getMaxNodeNum());
    if (profile == "churn" || profile == "churn_delete") {
      params.max_edge_age = profile == "churn_delete" ? 0 : 2;
      params.lambda = 25;
      params.ais_threshold = .05f;
    }
    graph.setParams(params);
    require(graph.load(input.string()), "比較GNGの読込失敗");
    graph.setCollisionAware(false);
    graph.setStatsLogPath((output / "distance_stats.dat").string());
    auto additions = std::make_shared<addition_counter>();
    graph.registerStatusProvider(additions);
    const std::size_t num_before = graph.getActiveIndices().size();
    const double load_sec = elapsed_sec(load_start);
    chain.reset();
    std::srand(seed);
    const auto angle_start = clock_type::now();
    graph.gngTrain(samples, num_angle_iter);
    const double angle_sec = elapsed_sec(angle_start);
    const auto angle_save_start = clock_type::now();
    require(graph.save((output / "angle.gng").string()), "角度学習結果の保存失敗");
    const double angle_save_sec = elapsed_sec(angle_save_start);
    std::vector<double> coord_sec;
    std::vector<double> coord_save_sec;
    for (int layer_idx = 0; layer_idx < 2; ++layer_idx) {
      chain.reset();
      const auto start = clock_type::now();
      graph.trainCoordEdgesOnTheFly(num_coord_iter, layer_idx);
      coord_sec.push_back(elapsed_sec(start));
      const auto save_start = clock_type::now();
      const auto name = layer_idx == 0 ? "coord0.gng" : "final.gng";
      require(graph.save((output / name).string()), "TCP辺生成結果の保存失敗");
      coord_save_sec.push_back(elapsed_sec(save_start));
    }
    const double total_sec = elapsed_sec(total_start);
    const std::size_t num_after = graph.getActiveIndices().size();
    require(num_before + additions->num_added >= num_after, "ノード追加削除数の不整合");
    std::size_t num_moved = 0;
    for (int node_id : graph.getActiveIndices()) {
      const auto found = before_angles.find(node_id);
      if (found != before_angles.end() &&
          !(found->second.array() == graph.nodeAt(node_id).weight_angle.array()).all()) ++num_moved;
    }
    std::vector<std::size_t> num_edges;
    for (int layer_idx = 0; layer_idx < 3; ++layer_idx) {
      std::size_t count = 0;
      for (int node_id : graph.getActiveIndices()) {
        const auto &neighbors = layer_idx == 0 ? graph.getNeighborsAngle(node_id)
            : graph.getNeighborsCoord(node_id, layer_idx - 1);
        count += neighbors.size();
      }
      require(count % 2 == 0, "無向辺の片側参照");
      num_edges.push_back(count / 2);
    }
    const std::size_t num_removed = num_before + additions->num_added - num_after;
    json result = {{"seed", seed}, {"profile", profile}, {"enable_nearest_index", enable_nearest_index},
      {"input", input.string()}, {"urdf", get("--urdf")}, {"joint_limits", limits}, {"is_collision_aware", false},
      {"num_input_nodes", num_before}, {"num_output_nodes", num_after}, {"num_added_nodes", additions->num_added},
      {"num_removed_nodes", num_removed}, {"num_moved_existing_nodes", num_moved}, {"num_edges_per_layer", num_edges},
      {"num_angle_iter", num_angle_iter}, {"num_coord_iter_per_layer", num_coord_iter}, {"num_samples", num_samples},
      {"preparation_sec", preparation_sec}, {"load_sec", load_sec}, {"angle_sec", angle_sec},
      {"angle_save_sec", angle_save_sec}, {"coord_sec", coord_sec}, {"coord_save_sec", coord_save_sec},
      {"total_sec", total_sec}, {"max_edge_age", params.max_edge_age}, {"lambda", params.lambda},
      {"ais_dist_th", params.ais_threshold}, {"n_best_candidates", params.n_best_candidates},
      {"beta", params.beta}, {"alpha", params.alpha}, {"learn_rate_s1", params.learn_rate_s1}, {"learn_rate_s2", params.learn_rate_s2},
      {"scope", "衝突検査なしの実GNG更新と実URDF FK。索引構築・更新を含む計測。標準条件と低寿命辺条件の別集計。"}};
    write_json(output / "metrics.json", result);
    write_json(output / "metrics.numeric.json", {{"total_sec", total_sec}, {"load_sec", load_sec},
      {"angle_sec", angle_sec}, {"coord_left_sec", coord_sec[0]}, {"coord_right_sec", coord_sec[1]},
      {"num_input_nodes", num_before}, {"num_output_nodes", num_after}, {"num_added_nodes", additions->num_added},
      {"num_removed_nodes", num_removed}, {"num_moved_existing_nodes", num_moved}});
    std::cout << result.dump() << std::endl;
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "[Benchmark] " << error.what() << std::endl;
    return 1;
  }
}
