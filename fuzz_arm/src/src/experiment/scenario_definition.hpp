#pragma once
#include "simulation/safety/influence_management.hpp"
#include "simulation/world/sim_obstacle_controller.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace robot_sim {
namespace experiment {

/**
 * @brief 1つの目標地点とその時の環境設定（障害物など）を定義する構造体
 */
struct ScenarioStage {
  Eigen::Vector3d target_pos = Eigen::Vector3d(0.5, 0.0, 0.5);

  // 障害物設定
  robot_sim::simulation::ObstacleBehavior behavior =
      robot_sim::simulation::ObstacleBehavior::PASSIVE;
  double obstacle_speed = 1.0;
  double obstacle_radius = 0.05;

  // タイムアウト (秒)
  double timeout = 30.0;

  // 到達判定しきい値 (m)
  double target_tolerance = 0.01;

  std::string name = "";
};

/**
 * @brief 複数のステージで構成される実験シナリオの定義
 */
struct ScenarioDefinition {
  std::string scenario_name = "Default Scenario";
  std::vector<ScenarioStage> stages;

  /**
   * @brief JSONファイルからシナリオをロードする
   */
  static ScenarioDefinition loadFromJson(const std::string &filepath);

  /**
   * @brief デフォルトのシナリオを生成する
   */
  static ScenarioDefinition getDefault();
};

} // namespace experiment
} // namespace robot_sim
