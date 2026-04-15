#include "simulation/reactive_controller.hpp"
#include "control/linear_trajectory_controller.hpp"
#include "control/trajectory_controller.hpp"
#include "gng/GrowingNeuralGas_offline.hpp"
#include "kinematics/utility.hpp"
#include "planner/RRT/ik_rrt_planner.hpp"
#include "planning/gng_dijkstra_planner.hpp"
#include "simulation/core/simulation_state.hpp"
#include "simulation/planning/ode_state_validity_checker.hpp"
#include "simulation/planning/scenario_manager.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/safety/safety_management.hpp"
#include "simulation/world/dynamic_obstacle_manager.hpp"
#include "spatial/dense_spatial_index.hpp"
#include "kinematics/state_adapter.hpp"
#include <Eigen/Dense>
#include <deque>
#include <drawstuff/drawstuff.h>

namespace robot_sim {
namespace simulation {

/**
 * @brief 反応的追跡モジュール（内部実装クラス）
 *
 * 動的な目標を追跡するための複雑なロジックを実装します。
 * 複数の計画戦略（GNG、RRT、IK）を使用してパンくずリストベースの実行を行います。
 */
class RobotModel;
class OdeRobotSim;
class VoxelGrid;
struct OdeRobotComponent;
class ReactiveTrackingModule {
private:
  struct Breadcrumb {
    Eigen::Vector3d pos; // 手先位置
    Eigen::VectorXf q;
    std::string strategy;
  };

  std::deque<Breadcrumb> breadcrumbs_;
  Eigen::VectorXf last_planned_q_;
  bool initialized_ = false;
  double last_record_time_ = 0.0;
  Eigen::Vector3d last_obs_pos_ = Eigen::Vector3d::Zero();

  // Arrival-based Trajectory Management
  std::deque<Eigen::Vector3d> target_queue_;
  double current_traj_angle_ = 0.0;
  bool was_trajectory_mode_ = false;
  int ik_fail_count_ = 0;
  int stuck_counter_ = 0;
  int current_escape_target_id_ = -1; // Goal Latching
  int last_goal_node_id_ = -1;       // Posture Stability (Hysteresis)

  int findNearestNodeToCartesian(const Eigen::Vector3d &target_pos,
                                 SimulationState &state) {
    if (!state.gng_ptr)
      return -1;
    float min_dist_sq = 1e10f;
    int nearest_id = -1;
    state.gng_ptr->forEachActiveValid([&](int i, const auto &node) {
      float d2 = (node.weight_coord.template cast<double>() - target_pos)
                     .squaredNorm();
      if (d2 < min_dist_sq) {
        min_dist_sq = d2;
        nearest_id = i;
      }
    });
    return nearest_id;
  }

  std::vector<int> findNearestNodesToCartesian(const Eigen::Vector3d &target_pos,
                                               SimulationState &state,
                                               int n_best) {
    if (!state.gng_ptr)
      return {};
    struct Candidate {
      int id;
      double dist_sq;
    };
    std::vector<Candidate> candidates;
    state.gng_ptr->forEachActiveValid([&](int i, const auto &node) {
      double d2 = (node.weight_coord.template cast<double>() - target_pos)
                     .squaredNorm();
      candidates.push_back({i, d2});
    });

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b) {
                return a.dist_sq < b.dist_sq;
              });

    std::vector<int> result;
    int count = std::min((int)candidates.size(), n_best);
    for (int i = 0; i < count; ++i) {
      result.push_back(candidates[i].id);
    }
    return result;
  }

  int findNearestNodeToJoints(const Eigen::VectorXf &q,
                              SimulationState &state) {
    if (!state.gng_ptr)
      return -1;
    float min_dist_sq = 1e10f;
    int nearest_id = -1;
    state.gng_ptr->forEachActiveValid([&](int i, const auto &node) {
      float d2 = (node.weight_angle - q).squaredNorm();
      if (d2 < min_dist_sq) {
        min_dist_sq = d2;
        nearest_id = i;
      }
    });
    return nearest_id;
  }

  bool isNodeTrulySafe(int node_id, SimulationState &state) {
    if (!state.gng_ptr || node_id == -1)
      return false;
    const auto &node = state.gng_ptr->nodeAt(node_id);
    if (node.status.is_danger || node.status.is_colliding)
      return false;

    // node.status.is_danger / is_colliding are already updated by SafetyStateManager
    // using VLUT, which accounts for the entire arm's geometry.

    // Check 1st neighbors
    const auto &n1 = state.gng_ptr->getNeighborsAngle(node_id);
    for (int id1 : n1) {
      const auto &nb1 = state.gng_ptr->nodeAt(id1);
      if (nb1.status.is_danger || nb1.status.is_colliding)
        return false;
    }
    return true;
  }


  int findSafeEscapeNode(int start_node_id, SimulationState &state) {
    if (!state.gng_ptr || start_node_id == -1)
      return -1;

    // Latching: すでに有効な目標があればそれを優先
    if (current_escape_target_id_ != -1) {
      if (isNodeTrulySafe(current_escape_target_id_, state)) {
        return current_escape_target_id_;
      }
    }

    std::queue<int> queue;
    std::unordered_set<int> visited;

    queue.push(start_node_id);
    visited.insert(start_node_id);

    while (!queue.empty()) {
      int u_id = queue.front();
      queue.pop();

      if (isNodeTrulySafe(u_id, state)) {
        // BFS finds the nearest safe node in the graph. 
        // Since node.status.is_danger is 0 here, it's considered safe.
        current_escape_target_id_ = u_id;
        return u_id;
      }

      for (int v_id : state.gng_ptr->getNeighborsAngle(u_id)) {
        if (visited.find(v_id) == visited.end()) {
          visited.insert(v_id);
          queue.push(v_id);
        }
      }

      if (visited.size() > 500)
        break; // 探索爆発防止
    }

    return -1;
  }

  void refillTargetQueue(SimulationState &state) {
    if (!state.scenario_manager_ptr)
      return;
    const double angle_step = 2.0 * M_PI / 63.0; // Optimized for 63 divisions
                                                 // ~7.5cm interval at R=0.15m
    const size_t max_targets =
        20; // Enough for one loop with 0.5 step (2pi/0.5 ~ 12)
    while (target_queue_.size() < max_targets) {
      target_queue_.push_back(
          state.scenario_manager_ptr->getTrajectoryPoint(current_traj_angle_));
      current_traj_angle_ += angle_step;
    }
  }

public:
  void update(SimulationState &state, double dt) {
    // accumulated time for cooldown logic
    static double accumulated_sim_time = 0.0;
    accumulated_sim_time += dt;
    double current_time = accumulated_sim_time;

    bool is_static_evacuation =
        (state.scenario_manager_ptr &&
         state.scenario_manager_ptr->getCurrentScenario() ==
             robot_sim::simulation::ScenarioType::AVOIDANCE_STATIC);

    if (!state.gng_ptr || !state.safety_state_manager_ptr ||
        !state.reactive_trailing_mode) {
      breadcrumbs_.clear();
      initialized_ = false;
      return;
    }

    if (!state.robot_hal_ptr || !state.controller_ptr || !state.fk_chain_ptr) {
      return;
    }

    // Joint state capture for tracking
    auto* adapter = state.state_adapter_ptr.get();
    if (!adapter) return;

    Eigen::VectorXf q_curr =
        state.robot_hal_ptr->getJointPositions().cast<float>();
    
    // [Fix] Convert to Planning Domain (6D) for GNG calculations to prevent Eigen crashes
    Eigen::VectorXf q_curr_plan = adapter->toPlanning(q_curr.cast<double>()).cast<float>();

    int dof = state.fk_chain_ptr->getTotalDOF();
    state.fk_chain_ptr->updateKinematics(
        q_curr_plan.head(std::min((int)q_curr_plan.size(), dof)).cast<double>());
    Eigen::Vector3d eef_curr = state.fk_chain_ptr->getEEFPosition();

    if (!initialized_) {
      last_planned_q_ = adapter->ensurePlanningDim(q_curr_plan.cast<double>()).cast<float>();
      if (state.obstacle_manager_ptr &&
          !state.obstacle_manager_ptr->getObstacles().empty()) {
        last_obs_pos_ = state.obstacle_manager_ptr->getObstacles()[0].position;
      }
      current_escape_target_id_ = -1;
      initialized_ = true;
    }

    // 1. Collision Check (Contacts) -> STOP
    auto names = state.robot_hal_ptr->getJointNames();
    for (const auto &name : names) {
      if (!state.robot_hal_ptr->getContacts(name).empty()) {
        ::control::ControlCommand stop;
        stop.joint_positions = state.robot_hal_ptr->getJointPositions();
        stop.priority = ::control::CommandPriority::HIGH_SAFETY;
        stop.source = "ReactiveTrack:Contact";
        state.controller_ptr->setOverrideCommand(stop);
        return;
      }
    }

    // 1.5 Target Source Management

    Eigen::Vector3d obs_pos = Eigen::Vector3d::Zero();

    if (state.scenario_manager_ptr &&
        state.scenario_manager_ptr->getCurrentScenario() ==
            robot_sim::simulation::ScenarioType::TRAJECTORY) {
      if (!was_trajectory_mode_) {
        target_queue_.clear();
        current_traj_angle_ = 0.0;
        was_trajectory_mode_ = true;
      }
      refillTargetQueue(state);

      if (!target_queue_.empty()) {
        obs_pos = target_queue_.front();

        // 1.5.1 Advance queue based on PLANNED arrival
        // [Fix] Enforce planning dimension for consistency
        Eigen::VectorXd last_q_d = adapter->ensurePlanningDim(last_planned_q_.cast<double>());
        state.fk_chain_ptr->updateKinematics(last_q_d);
        Eigen::Vector3d lp_eef = state.fk_chain_ptr->getEEFPosition();

        double dist_planned_to_target = (lp_eef - obs_pos).norm();
        bool should_advance = (dist_planned_to_target < 0.015);

        if (breadcrumbs_.size() > 200) {
          should_advance = false;
        }

        if (ik_fail_count_ > 10) {
          should_advance = true;
        }

        if (should_advance) {
          target_queue_.pop_front();
          refillTargetQueue(state);
          if (!target_queue_.empty()) {
            obs_pos = target_queue_.front();
          }
          ik_fail_count_ = 0;
        }
      }
    } else {
      was_trajectory_mode_ = false;
      if (state.scenario_manager_ptr &&
          state.scenario_manager_ptr->getCurrentScenario() ==
              robot_sim::simulation::ScenarioType::AVOIDANCE_STATIC) {
        // Static Evacuation Mode:
        // Goal position is restricted to the scenario's fixed target position
        obs_pos = state.scenario_manager_ptr->getTargetPosition();
      } else if (state.obstacle_manager_ptr &&
                 !state.obstacle_manager_ptr->getObstacles().empty()) {
        // Default: Trail the obstacle (TRACKING mode or AVOIDANCE)
        obs_pos = state.obstacle_manager_ptr->getObstacles()[0].position;
      } else {
        return;
      }
    }

    // 2. Trail Generation
    double dist_moved = (obs_pos - last_obs_pos_).norm();
    double move_threshold =
        (is_static_evacuation) ? 0.01 : 0.005; // Tighten for responsiveness
    bool target_moved = (dist_moved > move_threshold) || breadcrumbs_.empty();

    // 隣接ノードおよび2次隣接ノードの危険度チェック (保守的な退避トリガー)
    bool neighbor_danger_trigger = false;
    if (state.gng_ptr) {
      int current_node_id = findNearestNodeToJoints(q_curr_plan, state);
      if (current_node_id != -1) {
        std::unordered_set<int> neighbors_to_check;
        const auto &n1 = state.gng_ptr->getNeighborsAngle(current_node_id);
        neighbors_to_check.insert(n1.begin(), n1.end());
        // Restore 2nd neighbors for robust avoidance
        for (int id1 : n1) {
          const auto &n2 = state.gng_ptr->getNeighborsAngle(id1);
          neighbors_to_check.insert(n2.begin(), n2.end());
        }

        for (int nb_id : neighbors_to_check) {
          const auto &nb_node = state.gng_ptr->nodeAt(nb_id);
          if (nb_node.status.is_danger || nb_node.status.is_colliding) {
            neighbor_danger_trigger = true;
            break;
          }
        }
      }
    }

    bool trigger_planning = target_moved;

    if (!breadcrumbs_.empty()) {
      // Basic validity check: current node shouldn't be dangerous in evacuation
      // mode
      if (is_static_evacuation) {
        int current_node_id = findNearestNodeToJoints(q_curr_plan, state);
        if (!isNodeTrulySafe(current_node_id, state)) {
          trigger_planning = true;
        }
      }
    } else if (breadcrumbs_.empty()) {
      // Must plan if we have no path
      trigger_planning = true;
    }

    bool cooldown_passed = (current_time - last_record_time_ > 0.01);
    if (is_static_evacuation) {
      cooldown_passed = true;
    }

    if (trigger_planning && cooldown_passed) {
      if (is_static_evacuation) {
        std::cout << "[ReactiveTrack] Triggering PLANNING for avoidance..."
                  << std::endl;
      }
      std::string strategy = "HOLD";
      std::vector<Eigen::VectorXd> segment;

      if (state.current_planning_method == PlanningMethod::IK_RRT_CONNECT &&
          state.rrt_planner_ptr && state.validity_checker_ptr) {

        Eigen::VectorXd start_q_d;
        // if trigger was due to target move, we want to start from current
        // position to overwrite
        if (target_moved || breadcrumbs_.empty()) {
          start_q_d = adapter->ensurePlanningDim(q_curr_plan.cast<double>());
          // Note: we don't update last_planned_q_ yet
        } else {
          start_q_d = adapter->ensurePlanningDim(last_planned_q_.cast<double>());
        }

        // 1. First, try simple IK + Linear Interpolation (Fast)
        // 使用するパラメータを追従用に軽量化 (通常プランニングには影響しない)
        robot_sim::planner::RRTParams fast_params;
        fast_params.max_ik_iterations = 300;
        fast_params.max_ik_samples = 3;
        fast_params.max_planning_time_ms = 5.0; // RRTにも制限をかける

        std::vector<Eigen::VectorXd> goals =
            state.rrt_planner_ptr->sampleGoalStates(
                start_q_d, obs_pos, *state.validity_checker_ptr, &fast_params);

        bool linear_success = false;
        if (!goals.empty()) {
          Eigen::VectorXd best_goal = goals[0];
          double dist = (best_goal - start_q_d).norm();

          if (dist < 0.5) {
            int steps = std::max(1, (int)(dist / 0.01));
            double step = 1.0 / steps;
            for (double t = step; t <= 1.1; t += step) {
              // [Fix] Enforce dimension for arithmetic
              Eigen::VectorXd q_target_6d = adapter->ensurePlanningDim(best_goal);
              Eigen::VectorXd start_6d = adapter->ensurePlanningDim(start_q_d);
              Eigen::VectorXd q = start_6d + std::min(t, 1.0) * (q_target_6d - start_6d);
              segment.push_back(q);
            }

            if (true) {
              strategy = "LINEAR_TRAIL";
              ik_fail_count_ = 0;
              last_record_time_ = current_time;
              last_obs_pos_ = obs_pos;
              linear_success = true;
            }
          }
        }

        // 2. If Linear failed, fallback to RRT (Slow but robust)
        if (!linear_success) {
          auto rrt_path = state.rrt_planner_ptr->plan(
              start_q_d, goals, *state.validity_checker_ptr, &fast_params);

          if (!rrt_path.empty()) {
            for (const auto &q : rrt_path) {
              segment.push_back(q);
            }
            strategy = "RRT_TRAIL";
            ik_fail_count_ = 0;
            last_record_time_ = current_time;
            last_obs_pos_ = obs_pos;
          } else {
            ik_fail_count_++;
          }
        }
      } else if (state.current_planning_method ==
                 PlanningMethod::GNG_DIJKSTRA) {
        // [Ver. 4] 目標位置に近いノードを複数候補として挙げ、その中から「真に安全」なものを優先的に選択
        // [Ver. 5] その中からさらに「現在姿勢(q_curr)に最も近い」ものを選ぶことで姿勢を安定させる（ヒステリシス）
        std::vector<int> goal_candidates =
            findNearestNodesToCartesian(obs_pos, state, 20);
        std::vector<int> safe_candidates;
        for (int cid : goal_candidates) {
          if (isNodeTrulySafe(cid, state)) {
            safe_candidates.push_back(cid);
          }
        }

        int goal_node_id = -1;
        if (!safe_candidates.empty()) {
          // ヒステリシス：前回のゴールが依然として安全なら継続（チャタリング防止）
          bool last_goal_still_safe = false;
          if (last_goal_node_id_ != -1) {
            for (int scid : safe_candidates) {
              if (scid == last_goal_node_id_) {
                last_goal_still_safe = true;
                break;
              }
            }
          }

          if (last_goal_still_safe) {
            goal_node_id = last_goal_node_id_;
          } else {
            // 新規選定：安全な候補の中で、現在の関節姿勢に最も近いものを選択（連続性の維持）
            float min_q_dist_sq = 1e10f;
            for (int scid : safe_candidates) {
              float q_d2 =
                  (state.gng_ptr->nodeAt(scid).weight_angle - q_curr_plan).squaredNorm();
              if (q_d2 < min_q_dist_sq) {
                min_q_dist_sq = q_d2;
                goal_node_id = scid;
              }
            }
          }
        } else if (!goal_candidates.empty()) {
          // 全滅の場合は最も近いものを暫定目標に（後の退避ロジックで判定）
          goal_node_id = goal_candidates[0];
        }

        last_goal_node_id_ = goal_node_id;

        // Escape logic for Static Evacuation:
        // 2次近傍内に「真に安全（自身と隣接が安全）」なノードがあるか確認
        if (is_static_evacuation) {
          bool can_engage_goal = false;
          int curr_node_id = findNearestNodeToJoints(q_curr_plan, state);
          if (curr_node_id != -1) {
            if (isNodeTrulySafe(curr_node_id, state)) {
              can_engage_goal = true;
            } else {
              // 1次、2次近傍を探索
              for (int n1_id : state.gng_ptr->getNeighborsAngle(curr_node_id)) {
                if (isNodeTrulySafe(n1_id, state)) {
                  can_engage_goal = true;
                  break;
                }
                for (int n2_id : state.gng_ptr->getNeighborsAngle(n1_id)) {
                  if (isNodeTrulySafe(n2_id, state)) {
                    can_engage_goal = true;
                    break;
                  }
                }
                if (can_engage_goal) break;
              }
            }
          }

          // 「2次近傍まで安全なノード」が見つからない、または目標自体が危険な場合は、待機/避難を優先
          if (!can_engage_goal || !isNodeTrulySafe(goal_node_id, state)) {
            int start_node_id = (curr_node_id != -1) ? curr_node_id : findNearestNodeToJoints(q_curr, state);
            int escape_node_id = findSafeEscapeNode(start_node_id, state);
            if (escape_node_id != -1) {
              goal_node_id = escape_node_id;
            }
          } else {
            // [改良] 目標が安全ならラッチを解除して直進する
            current_escape_target_id_ = -1;
          }
        }

        if (goal_node_id != -1 && state.gng_planner_ptr) {
          // Use q_curr as start when in static evacuation or drifting
          int start_node_id = findNearestNodeToJoints(q_curr_plan, state);
          if (start_node_id != -1) {
            std::vector<Eigen::VectorXf> gng_path = state.gng_planner_ptr->plan(
                q_curr_plan, state.gng_ptr->nodeAt(goal_node_id).weight_angle,
                *state.gng_ptr);

            if (!gng_path.empty()) {
              // [追加チェック] パスが「真に安全なノード」を初期段階で含んでいるか確認
              bool start_is_risky = !isNodeTrulySafe(start_node_id, state);
              bool path_reaches_safety = false;
              if (start_is_risky && is_static_evacuation) {
                int check_num = std::min((int)gng_path.size(), 3);
                for (int i = 0; i < check_num; ++i) {
                  int nid = findNearestNodeToJoints(gng_path[i], state);
                  if (isNodeTrulySafe(nid, state)) {
                    path_reaches_safety = true;
                    break;
                  }
                }
              } else {
                path_reaches_safety = true;
              }

              if (path_reaches_safety) {
                for (const auto &q : gng_path) {
                  segment.push_back(q.cast<double>());
                }
                strategy = "GNG_TRAIL";
                last_obs_pos_ = obs_pos;
              } else {
                // 危険地帯を抜け出せないパスは却下し、避難ノードを再検索
                std::cout << "[ReactiveTrack] Path rejected: Does not reach safety promptly." << std::endl;
                int escape_node_id = findSafeEscapeNode(start_node_id, state);
                if (escape_node_id != -1 && escape_node_id != goal_node_id) {
                   // 再計画 (次のフレームで処理される)
                }
              }
            } else {
              if (is_static_evacuation) {
                std::cout << "[ReactiveTrack] GNG Plan failed from "
                          << start_node_id << " to " << goal_node_id
                          << " (Path Empty)" << std::endl;
              }
            }
          } else {
            if (is_static_evacuation) {
              std::cout << "[ReactiveTrack] GNG Plan failed: Start node near "
                           "current joints not found!"
                        << std::endl;
            }
          }
        } else {
          if (is_static_evacuation) {
            std::cout << "[ReactiveTrack] GNG Plan failed: Goal node near "
                         "target not found!"
                      << std::endl;
          }
          ik_fail_count_++;
        }

        if (!segment.empty()) {
          ik_fail_count_ = 0;
          last_record_time_ = current_time;
          last_obs_pos_ = obs_pos;
        } else {
          ik_fail_count_++;
          // [修正] 計画に失敗し、かつパンくずが空の場合は、現在位置を「最後に計画された位置」として
          // 保持することで、以前の（あるいは初期の0の）位置に引き戻されるのを防ぐ
          if (breadcrumbs_.empty()) {
            last_planned_q_ = q_curr;
          }
        }
      }

      // Add segment to breadcrumbs
      if (!segment.empty()) {
        // --- NEW: Clear only if we have a successful new plan ---
        if (target_moved || neighbor_danger_trigger) {
          breadcrumbs_.clear();
        }

        if (breadcrumbs_.size() <= 600) {
          for (size_t i = 0; i < segment.size(); ++i) {
            Eigen::VectorXd q_next = segment[i];
            int total_dof = state.fk_chain_ptr->getTotalDOF();
            state.fk_chain_ptr->updateKinematics(
                q_next.head(std::min((int)q_next.size(), total_dof)));
            Eigen::Vector3d p_eef = state.fk_chain_ptr->getEEFPosition();
            breadcrumbs_.push_back({p_eef, segment.back().head(6).cast<float>(), strategy});
          }
          last_planned_q_ = segment.back().head(6).cast<float>();
          last_obs_pos_ = obs_pos;
          last_record_time_ = current_time;
        }
      }
    }

    // 3. Execution & Skip Logic

    Eigen::VectorXf target_q = q_curr_plan;
    if (!initialized_)
      target_q = q_curr_plan;

    if (!breadcrumbs_.empty()) {
      while (!breadcrumbs_.empty()) {
        // [改良] 動的ルックアヘッド:
        // スピードスケールに応じてパンくずのスキップ距離を広げる
        // これにより、高速移動時に「一歩先のパンくず」に固執せず流れるように動く
        double skip_distance = 0.02 * (double)state.speed_scale;
        double dist_to_breadcrumb =
            (eef_curr - breadcrumbs_.front().pos).norm();

        if (dist_to_breadcrumb < skip_distance) {
          breadcrumbs_.pop_front();
          stuck_counter_ = 0;
          // 到着確認: 目標ノードに十分近づいたらラッチ解除
          if (breadcrumbs_.empty()) {
            current_escape_target_id_ = -1;
          }
        } else if (dist_to_breadcrumb > 0.40) {
          std::cout << "[ReactiveTrack] DRIFT DETECTED (" << dist_to_breadcrumb << "m). ";
          if (is_static_evacuation) {
             std::cout << "Static Evacuation: Clearing breadcrumbs but maintaining mode to retry planning." << std::endl;
             breadcrumbs_.clear();
             state.hold_positions = q_curr.cast<double>();
          } else {
             std::cout << "Disabling reactive mode for safety." << std::endl;
             breadcrumbs_.clear();
             state.hold_positions = q_curr.cast<double>(); // 停止位置を同期
             state.reactive_trailing_mode = false;
             state.auto_mode = false;
          }
          break;
        } else {
          if (++stuck_counter_ > 100) {
            breadcrumbs_.pop_front();
            stuck_counter_ = 0;
          }
          break;
        }
      }

      if (!breadcrumbs_.empty()) {
        target_q = breadcrumbs_.front().q;
        // state.target_sphere_pos = breadcrumbs_.front().pos; // Removed to
        // prevent goal overwrite
      } else {
        target_q = last_planned_q_;
      }

      // 4. Command Output (REFACTORED: Use Velocity Limits instead of simple
      // P-Gain) Original logic: next_q = q_curr + (diff / dq_norm) *
      // min(dq_norm, max_dq); New logic: Check per-joint velocity limits.

      Eigen::VectorXf next_q = q_curr;
      // [Fix] Final arithmetic safeguard
      Eigen::VectorXf q6 = adapter->ensurePlanningDim(q_curr_plan.cast<double>()).cast<float>();
      Eigen::VectorXf diff = target_q.head(6) - q6;

      // Handle Wraparound
      ::kinematics::applyWraparound(diff);

      // [改良] 速度計算の修正:
      // 予算(v_max)をスケールさせ、到達倍率(scale)は1.0でクランプする
      // これにより、目標地点を飛び越える（オーバーシュート）ことによるカクつきを防ぐ
      float v_max_effective = 2.4f * state.speed_scale;
      if (v_max_effective > 4.8f) v_max_effective = 4.8f; // Cap to joint physical limits
      float scale = ::kinematics::calculateVelocityScale(diff, v_max_effective,
                                                         (float)dt);
      if (scale > 1.0f)
        scale = 1.0f; // Clamp to target to prevent oscillation

      next_q = q_curr_plan + diff * scale;

      ::control::ControlCommand cmd;
      // Convert Planning (6D) back to Physical (8D) using current state as base for gripper safety
      cmd.joint_positions = adapter->fromPlanning(next_q.cast<double>(), q_curr.cast<double>());
      cmd.priority = ::control::CommandPriority::HIGH_SAFETY;
      cmd.source = "ReactiveTrack:Standardized";
      state.controller_ptr->setOverrideCommand(cmd);
    }
  }

  void draw(const SimulationState &state) const {
    if (!state.reactive_trailing_mode || !state.show_reactive_tracking_viz ||
        state.is_replay_mode)
      return;

    dReal R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    int total = breadcrumbs_.size();
    int i = 0;
    for (const auto &b : breadcrumbs_) {
      float alpha = 0.05f + 0.45f * (float)(i + 1) / total;
      dsSetColorAlpha(0.0f, 1.0f, 0.0f, alpha);
      float bpos[3] = {(float)b.pos.x(), (float)b.pos.y(), (float)b.pos.z()};
      float R_float[12];
      for (int k = 0; k < 12; ++k)
        R_float[k] = (float)R[k];
      dsDrawSphere(bpos, R_float, 0.01f);
      i++;
    }

    // Target Queue Visualization
    dsSetColor(1.0f, 0.0f, 1.0f);
    float R_float[12];
    for (int k = 0; k < 12; ++k)
      R_float[k] = (float)R[k];
    for (const auto &tp : target_queue_) {
      float pos[3] = {(float)tp.x(), (float)tp.y(), (float)tp.z()};
      dsDrawSphere(pos, R_float, 0.005f);
    }

    // 現在の目標ノードを表示 (大きなマゼンタ球)
    if (current_escape_target_id_ != -1 && state.gng_ptr) {
      dsSetColor(1.0f, 0.0f, 1.0f);
      const auto &node = state.gng_ptr->nodeAt(current_escape_target_id_);
      float pos[3] = {node.weight_coord.x(), node.weight_coord.y(),
                      node.weight_coord.z()};
      dsDrawSphere(pos, R_float, 0.03f);
    }
  }
};

// ReactiveController implementation
ReactiveController::ReactiveController()
    : reactive_mode_enabled_(false),
      tracking_module_(std::make_unique<ReactiveTrackingModule>()) {}

ReactiveController::~ReactiveController() = default;

void ReactiveController::update(double dt, SimulationState &state) {
  // Profiling
  auto start_time = std::chrono::high_resolution_clock::now();

  if (!reactive_mode_enabled_ || !state.reactive_trailing_mode) {
    return;
  }

  if (tracking_module_ && state.robot_hal_ptr && state.controller_ptr) {
    tracking_module_->update(state, dt);
  } else {
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double elapsed_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

  // Log if update takes too long (> 1ms)
  if (elapsed_ms > 1.0) {
  }
}

void ReactiveController::handleDynamicObstaclePrediction(
    SimulationState &state) {
  if (!state.enable_prediction_field || !state.obstacle_manager_ptr ||
      !state.safety_state_manager_ptr || !state.gng_ptr) {
    return;
  }

  double dt = 0.05;
  auto dense_index =
      std::dynamic_pointer_cast<robot_sim::analysis::DenseSpatialIndex>(
          state.spatial_index_ptr);

  const auto &obstacles = state.obstacle_manager_ptr->getObstacles();
  double max_obs_v = 0.0;

  for (auto &obs : obstacles) {
    auto &mutable_obs =
        const_cast<robot_sim::simulation::DynamicObstacle &>(obs);

    Eigen::Vector3d velocity =
        state.obstacle_manager_ptr->estimateVelocityGlobal(mutable_obs, dt);

    double v_norm = velocity.norm();
    if (v_norm > max_obs_v)
      max_obs_v = v_norm;

    if (dense_index && state.enable_prediction_field) {
      state.safety_state_manager_ptr->updateDangerField(
          dense_index, mutable_obs.position, mutable_obs.radius, velocity);
    }

    if (v_norm > 0.001 && state.eef_log_counter % 30 == 0) {
    }
  }
  state.safety_state_manager_ptr->update((float)dt, (float)max_obs_v);
}

void ReactiveController::draw(const SimulationState &state) const {
  if (tracking_module_) {
    tracking_module_->draw(state);
  }
}

} // namespace simulation
} // namespace robot_sim
