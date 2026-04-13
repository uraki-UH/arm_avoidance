#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace robot_sim {
namespace experiment {

/**
 * @brief 動的障害物の設定パラメータ
 */
struct DynamicObstacleConfig {
  double radius = 0.15;              // 障害物の半径 (m)
  double orbit_radius = 0.5;         // 回転軌道の半径 (m)
  double rotation_speed = 0.005;     // 回転速度 (rad/step)
  double z_min = 0.3;                // Z軸最小高度 (m)
  double z_max = 0.7;                // Z軸最大高度 (m)
  double z_oscillation_period = 5.0; // Z軸振動周期 (seconds)
};

/**
 * @brief 実験全体の設定パラメータ
 */
struct ExperimentConfig {
  // ロボット設定
  std::string robot_urdf_path = "custom_robot_cylinder";
  std::string leaf_link_name = "link_7";
  double max_joint_velocity = 1.0; // rad/s

  // ターゲット設定
  std::vector<Eigen::Vector3d> targets;
  double target_reach_threshold = 0.06; // m

  // GNG設定
  std::string gng_map_file = "gng_results/topoarm_final.bin";
  double voxel_size = 0.05;           // m
  double eef_collision_radius = 0.08; // m

  // 動的障害物設定
  DynamicObstacleConfig obstacle;

  // 実験設定
  std::string experiment_name = "DynamicObstacleChallenge";
  std::string log_directory = "experiment_logs";

  /**
   * @brief ファイルから設定を読み込む
   * @param filepath 設定ファイルのパス (.json)
   * @return 読み込まれた設定
   */
  static ExperimentConfig loadFromFile(const std::string &filepath);

  /**
   * @brief 設定をファイルに保存
   * @param filepath 保存先ファイルパス (.json)
   */
  void saveToFile(const std::string &filepath) const;

  /**
   * @brief デフォルト設定を取得 (experiment_settings.mdに基づく)
   */
  static ExperimentConfig getDefault();
};

} // namespace experiment
} // namespace robot_sim
