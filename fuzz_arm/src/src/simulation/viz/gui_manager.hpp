#pragma once
#include <Eigen/Dense>
#include <functional>

namespace robot_sim {
namespace simulation {

class SimulationState;

/**
 * @brief ImGuiパネルを管理するクラス
 *
 * ユーザーインターフェースの描画とユーザー入力の処理を一元管理します。
 */
class GuiManager {
public:
  GuiManager() = default;
  ~GuiManager() = default;

  /**
   * @brief コールバック関数の型定義
   */
  using VoidCallback = std::function<void()>;
  using PathPlanningCallback =
      std::function<void(const Eigen::VectorXf &, const Eigen::VectorXf &)>;

  /**
   * @brief コールバック関数を設定
   */
  void setStartExperimentCallback(
      std::function<void(const std::string &)> callback) {
    start_experiment_callback_ = callback;
  }

  void setResetExperimentCallback(VoidCallback callback) {
    reset_experiment_callback_ = callback;
  }

  void setPickRandomPostureCallback(VoidCallback callback) {
    pick_random_posture_callback_ = callback;
  }

  void setPickRandomGngNodeCallback(VoidCallback callback) {
    pick_random_gng_node_callback_ = callback;
  }

  void setPlanAndExecutePathCallback(PathPlanningCallback callback) {
    plan_and_execute_path_callback_ = callback;
  }

  /**
   * @brief ImGuiパネルを描画
   * @param state シミュレーション状態
   */
  void render(SimulationState &state);

private:
  void renderExperimentPanel(SimulationState &state);
  void renderVisualizationPanel(SimulationState &state);
  void renderAutoNavigationPanel(SimulationState &state);
  void renderPlanningPanel(SimulationState &state);
  void renderSystemPanel(SimulationState &state);

  // Callbacks
  std::function<void(const std::string &)> start_experiment_callback_;
  VoidCallback reset_experiment_callback_;
  VoidCallback pick_random_posture_callback_;
  VoidCallback pick_random_gng_node_callback_;
  PathPlanningCallback plan_and_execute_path_callback_;

  // Trajectory control state (static variables from original)
  int traj_idx_ = 0;
  float traj_scale_ = 0.3f;
  float traj_speed_ = 0.5f;
};

} // namespace simulation
} // namespace robot_sim
