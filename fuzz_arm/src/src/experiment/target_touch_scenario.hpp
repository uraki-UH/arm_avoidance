#pragma once
#include "common/config_manager.hpp"
#include "experiment/i_scenario.hpp"
#include "experiment/scenario_definition.hpp"
#include "simulation/planning/scenario_manager.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <vector>

namespace robot_sim {
namespace experiment {

/**
 * @brief 複数地点を順番に回るターゲットタッチ・シナリオ
 */
class TargetTouchScenario : public IScenario {
public:
  TargetTouchScenario() { reset(); }

  void reset() override {
    targets_.clear();
    auto &config = ::common::ConfigManager::Instance();
    int num_targets_conf = config.GetInt("num_targets", 0);

    if (num_targets_conf > 0) {
      for (int i = 1; i <= num_targets_conf; ++i) {
        std::string key = "target_" + std::to_string(i);
        std::string val = config.Get(key, "");
        if (!val.empty()) {
          // Parse "x, y, z"
          std::string item;
          std::stringstream ss(val);
          std::vector<double> coords;
          while (std::getline(ss, item, ',')) {
            try {
              coords.push_back(std::stod(item));
            } catch (...) {
            }
          }
          if (coords.size() == 3) {
            targets_.push_back(
                Eigen::Vector3d(coords[0], coords[1], coords[2]));
          }
        }
      }
    }

    // デフォルト（設定がない場合）
    if (targets_.empty()) {
      targets_ = {{0.5, -0.1, 0.4},
                  {-0.4, 0.5, 0.6},
                  {0.1, -0.7, 0.3},
                  {0.4, 0.4, 0.8},
                  {-0.5, -0.2, 0.5}};
    }
    current_target_idx_ = 0;
    is_completed_ = false;
    total_time_ = 0.0;
  }

  Eigen::Vector3d getCurrentTargetPos() const override {
    if (is_completed_)
      return Eigen::Vector3d::Zero();
    return targets_[current_target_idx_];
  }

  bool onTargetReached() override {
    std::cout << "[Scenario] Target " << current_target_idx_ + 1 << " reached!"
              << std::endl;
    current_target_idx_++;
    if (current_target_idx_ >= targets_.size()) {
      is_completed_ = true;
      std::cout << "[Scenario] All targets reached! Total time: " << total_time_
                << "s" << std::endl;
      return true;
    }
    return false;
  }

  double getTargetTolerance() const override { return 0.02; } // 2cmへ変更

  void update(double dt) override {
    if (!is_completed_) {
      total_time_ += dt;
    }
    // 注: 障害物の移動は DynamicObstacleManager
    // がメインループで更新しているため ここでは時間の累計のみ行う
    // (必要に応じて障害物の操作も可能)
  }

  std::string getName() const override { return "TargetTouchChallenge"; }

  std::string getProgressString() const override {
    if (is_completed_)
      return "COMPLETED";
    return "Target: " + std::to_string(current_target_idx_ + 1) + "/" +
           std::to_string(targets_.size());
  }

  double getTotalTime() const { return total_time_; }

private:
  std::vector<Eigen::Vector3d> targets_;
  size_t current_target_idx_ = 0;
  bool is_completed_ = false;
  double total_time_ = 0.0;
};

/**
 * @brief ステージ制シナリオ定義に基づいた評価タスク
 */
class AdvancedTargetScenario : public IScenario {
public:
  AdvancedTargetScenario(const ScenarioDefinition &def,
                         simulation::ScenarioManager *mgr)
      : def_(def), mgr_(mgr) {
    current_target_idx_ = 0;
  }

  void reset() override {
    current_target_idx_ = 0;
    if (mgr_)
      mgr_->startScenario();
  }

  Eigen::Vector3d getCurrentTargetPos() const override {
    if (current_target_idx_ < (int)def_.stages.size()) {
      return def_.stages[current_target_idx_].target_pos;
    }
    return Eigen::Vector3d::Zero();
  }

  bool onTargetReached() override {
    current_target_idx_++;
    if (mgr_) {
      mgr_->nextStage();
    }
    if (current_target_idx_ >= (int)def_.stages.size()) {
      return true; // 全ステージ完了
    }
    return false; // 次のステージへ
  }

  double getTargetTolerance() const override {
    if (current_target_idx_ < (int)def_.stages.size()) {
      return def_.stages[current_target_idx_].target_tolerance;
    }
    return 0.01;
  }

  std::string getName() const override { return def_.scenario_name; }

  int getCurrentStageIndex() const { return current_target_idx_; }

  void update([[maybe_unused]] double dt) override {
    // Stage-based scenario logic could be added here if needed
  }

  std::string getProgressString() const override {
    std::stringstream ss;
    ss << "Stage " << current_target_idx_ + 1 << "/" << def_.stages.size();
    if (current_target_idx_ < (int)def_.stages.size()) {
      ss << " (" << def_.stages[current_target_idx_].name << ")";
    }
    return ss.str();
  }

private:
  ScenarioDefinition def_;
  simulation::ScenarioManager *mgr_ = nullptr;
  int current_target_idx_ = 0;
};

} // namespace experiment
} // namespace robot_sim
