#pragma once
#include "common/config_manager.hpp"
#include "control/trajectory_controller.hpp"
#include "experiment/i_planner.hpp"
#include "experiment/i_scenario.hpp"
#include "gng/GrowingNeuralGas_offline2.hpp"
#include "kinematics/state_adapter.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/safety/safety_management.hpp"
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

namespace robot_sim {
namespace experiment {

/**
 * @brief 実験の実行と計測を管理するクラス
 */
class ExperimentRunner {
public:
  struct Stats {
    double total_time_s;
    bool is_success;
  };

  ExperimentRunner(::simulation::OdeRobotSim &robot,
                   ::control::ITrajectoryController &controller,
                   kinematics::KinematicChain *chain,
                   kinematics::JointStateAdapter *adapter,
                   double base_speed_scale = 1.0)
      : robot_(robot), controller_(controller), chain_(chain),
        adapter_(adapter), base_speed_scale_(base_speed_scale) {
    auto &config = ::common::ConfigManager::Instance();
    target_tolerance_ = config.GetDouble("target_tolerance", 0.02);
    controller_.setSpeedScale(base_speed_scale_);

    // CSV Log Init
    log_file_.open("experiment_log.csv", std::ios::out | std::ios::app);
    if (log_file_.is_open()) {
      log_file_.seekp(0, std::ios::end);
      if (log_file_.tellp() == 0) {
        log_file_ << "Timestamp,Stage,Event,ReplanningCount,PlanTimeMs,Result,"
                     "Nodes,CollChecks,TotalCheckTimeUs,AvgCheckTimeUs,"
                     "ManipulabilityOverheadMs\n";
      }
    }
    logLifecycleEvent("EXPERIMENT_START");
  }

  ~ExperimentRunner() {
    if (log_file_.is_open())
      log_file_.close();
  }

  void setSafetyInfo(
      const GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f> *gng_ptr,
      std::shared_ptr<simulation::SafetyStateManager> safety_manager) {
    gng_ptr_ = gng_ptr;
    safety_state_manager_ = safety_manager;
  }
  void setPlanner(std::shared_ptr<IPlanner> planner) { planner_ = planner; }
  void setScenario(std::shared_ptr<IScenario> scenario) {
    scenario_ = scenario;
  }
  void setValidityChecker(const planner::StateValidityChecker *checker) {
    validity_checker_ = checker;
  }

  /**
   * @brief 周期的な更新処理
   */
  void update(double dt) {
    if (is_completed_ || !scenario_ || !planner_)
      return;

    total_simulation_time_ += dt;

    // クールダウンタイマーの更新
    if (replan_cooldown_timer_ > 0.0) {
      replan_cooldown_timer_ -= dt;
    }

    scenario_->update(dt);
    Eigen::Vector3d target_pos = scenario_->getCurrentTargetPos();
    
    // Current state from physical simulation (8D)
    Eigen::VectorXd q_curr_phys = robot_.getJointPositions();
    // Convert to Planning Domain (6D) for GNG/Planners
    Eigen::VectorXd q_curr = adapter_->toPlanning(q_curr_phys);

    // Update Kinematic Chain with expanded 8D physical values for correct FK
    Eigen::Vector3d eef_pos = Eigen::Vector3d::Zero();
    if (chain_) {
      chain_->updateKinematics(q_curr);
      eef_pos = chain_->getEEFPosition();

      // Debug logging (every ~2.0s)
      static double last_log_time = -10.0;
      if (total_simulation_time_ - last_log_time > 2.0) {
        std::cout << "[ExperimentRunner] Dist: "
                  << (eef_pos - target_pos).norm()
                  << " (Tol: " << target_tolerance_ << ") "
                  << "EEF: " << eef_pos.transpose()
                  << " TGT: " << target_pos.transpose() << std::endl;
        last_log_time = total_simulation_time_;
      }
    }

    // 1.5 リスク適応型速度調整
    // 現在の姿勢の危険度に基づいて、コントローラの速度スケールを動的に調整する
    if (gng_ptr_ && safety_state_manager_) {
      int nearest_id = -1;
      float min_d = 1e10f;
      gng_ptr_->forEachActiveValid([&](int i, const auto &node) {
        // [Follow Adapter] q_curr is now guaranteed 7D, same as node.weight_angle
        float d = (node.weight_angle - q_curr.cast<float>()).norm();
        if (d < min_d) {
          min_d = d;
          nearest_id = i;
        }
      });

      if (nearest_id != -1) {
        float danger = safety_state_manager_->getDangerLevel(nearest_id);
        // 危険度に応じて速度を抑制 (danger=0.0 -> 1.0x, danger=1.0 -> 0.1x)
        float risk_speed_factor = 1.0f / (1.0f + 9.0f * danger);
        controller_.setSpeedScale(base_speed_scale_ * risk_speed_factor);
      }
    }

    // 1. 手先が目標に到達したかチェック
    double dist_to_target = (eef_pos - target_pos).norm();
    double current_tolerance = scenario_->getTargetTolerance();
    if (dist_to_target < current_tolerance) {
      if (scenario_->onTargetReached()) {
        is_completed_ = true;
        controller_.stop(); // 完了時も停止
        logLifecycleEvent("EXPERIMENT_COMPLETE");
        return;
      } else {
        // 次のターゲットへ
        // 現在の動作を停止し、再計画が完了するまで「ホールド」
        controller_.stop();
        current_path_.clear();
        needs_replan_after_reach_ = true; // フラグ管理：計画が必要な状態へ
        
        // [Bug Fix] Update target_pos immediately to the NEW target before planning occurs in the same frame
        target_pos = scenario_->getCurrentTargetPos();
        replan_cooldown_timer_ = 0.0; // Ensure replanning starts immediately for the new target

        logLifecycleEvent("TARGET_REACHED");
      }
    }

    // 2. 経路計画 (ターゲット到達直後、または経路がなくなった/衝突した場合)
    bool need_replan = controller_.isFinished() || needs_replan_after_reach_;

    if (!need_replan) {
      // プランナー自律型の衝突チェック (RRTは幾何学、GNGはノードフラグを監視)
      if (!planner_->isPathValid(q_curr, controller_.getCurrentIndex())) {
        controller_.stop(); // 安全のため停止
        need_replan = true;
      }
    }

    if (need_replan && replan_cooldown_timer_ <= 0.0) {
      replanning_count_++;
      auto path = planner_->plan(q_curr, target_pos);
      if (!path.empty()) {
        // ぐらつき解消のために2点分削る (RRTのみ)
        if (planner_->getName().find("RRT") != std::string::npos) {
          if (path.size() > 2) {
            path.erase(path.begin(), path.begin() + 2);
          } else if (path.size() == 2) {
            path.erase(path.begin());
          }
        }

        // 8次元に変換（グリッパー状態維持）してコントローラに渡す
        std::vector<Eigen::VectorXd> path_phys;
        for (const auto &q_plan : path) {
          path_phys.push_back(adapter_->fromPlanning(q_plan, q_curr_phys));
        }
        controller_.setPath(path_phys);
        needs_replan_after_reach_ = false; // 計画が完了したのでフラグを下ろす

        auto stats = planner_->getLastPlanStats();
        logEvent("PLAN_SUCCESS", stats.planning_time_ms, true, stats.node_count,
                 stats.collision_check_count, stats.total_check_time_us,
                 stats.avg_check_time_us);
      } else {
        // クールタイム設定
        replan_cooldown_timer_ = 0.1;
      }
    }
  }

  Stats getStats() const {
    Stats s;
    s.total_time_s = total_simulation_time_;
    s.is_success = is_completed_;
    return s;
  }

  bool isCompleted() const { return is_completed_; }

private:
  ::simulation::OdeRobotSim &robot_;
  ::control::ITrajectoryController &controller_;
  kinematics::KinematicChain *chain_ = nullptr;
  kinematics::JointStateAdapter *adapter_ = nullptr;
  std::shared_ptr<IPlanner> planner_;
  ::robot_sim::experiment::TargetTouchScenario *touch_scenario_ = nullptr;
  std::shared_ptr<IScenario> scenario_;
  const planner::StateValidityChecker *validity_checker_ = nullptr;

  std::deque<Eigen::VectorXd>
      current_path_; // Legacy, monitoring is now handled by planners
  bool is_completed_ = false;
  bool needs_replan_after_reach_ = false; // ターゲット到達後の思考待ちフラグ
  double total_simulation_time_ = 0.0;
  double replan_cooldown_timer_ = 0.0;
  double target_tolerance_ = 0.02;
  double base_speed_scale_ = 1.0;
  const GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f> *gng_ptr_ =
      nullptr;
  std::shared_ptr<simulation::SafetyStateManager> safety_state_manager_;

  // Logging
  std::ofstream log_file_;
  int replanning_count_ = 0;

  // ---- log helpers (ROS2 移行時は currentTimestamp を rclcpp::now()
  // に差し替え) ----
  std::string currentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d %X");
    return ss.str();
  }

  std::string currentStage(const std::string &override = "") const {
    std::string s =
        override.empty()
            ? (scenario_ ? scenario_->getProgressString() : "UNKNOWN")
            : override;
    std::replace(s.begin(), s.end(), ',', ';');
    return s;
  }

  void writeCsvRow(const std::string &ts, const std::string &stage,
                   const std::string &event, double plan_time, bool success,
                   size_t nodes, size_t checks, double total_us,
                   double avg_us) {
    if (!log_file_.is_open())
      return;
    log_file_ << ts << "," << stage << "," << event << "," << replanning_count_
              << "," << plan_time << "," << (success ? "SUCCESS" : "FAIL")
              << "," << nodes << "," << checks << "," << total_us << ","
              << avg_us << ",0.0\n";
    log_file_.flush();
  }

  void logEvent(const std::string &event, double plan_time, bool success,
                size_t nodes, size_t checks, double total_us, double avg_us) {
    writeCsvRow(currentTimestamp(), currentStage(), event, plan_time, success,
                nodes, checks, total_us, avg_us);
  }

  void logLifecycleEvent(const std::string &event,
                         const std::string &stage_override = "") {
    writeCsvRow(currentTimestamp(), currentStage(stage_override), event, 0.0,
                true, 0, 0, 0.0, 0.0);
  }
};

} // namespace experiment
} // namespace robot_sim
