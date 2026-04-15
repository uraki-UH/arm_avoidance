#include "simulation/viz/gui_manager.hpp"
#include "control/linear_trajectory_controller.hpp"
#include "simulation/safety/influence_management.hpp"
#include "simulation/world/sim_obstacle_controller.hpp"

#include "simulation/core/simulation_state.hpp"
#include "simulation/core/udp_command_bridge.hpp"
#include "simulation/planning/planning_coordinator.hpp"
#include "simulation/planning/scenario_manager.hpp"
#include "simulation/reactive_controller.hpp"
#include "simulation/robot/ode/ode_robot_sim.hpp"
#include "simulation/safety/risk_aware_cost_evaluator.hpp" // Added
#include "simulation/safety/safety_management.hpp"
#include <algorithm>
#include <imgui.h>

namespace robot_sim {
namespace simulation {

// Helper to ensure clean state transition
static void resetControlModes(SimulationState &state) {
  state.experiment_active = false;
  state.auto_mode = false;
  state.reactive_trailing_mode = false;
  state.manual_udp_mode = false;

  if (state.controller_ptr) {
    state.controller_ptr->stop();
    // Overrideコマンドも解除しておく
    ::control::ControlCommand empty_cmd;
    state.controller_ptr->setOverrideCommand(empty_cmd);
  }

  if (state.reactive_controller_ptr) {
    state.reactive_controller_ptr->setReactiveMode(false);
  }

  state.clearPathVisualizations();
}

void GuiManager::render(SimulationState &state) {
  if (!state.gui_show_panel)
    return;

  // Ensure window position and size
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiCond_FirstUseEver);

  ImGui::Begin("Simulation Control", &state.gui_show_panel);

  renderExperimentPanel(state);
  renderVisualizationPanel(state);
  renderAutoNavigationPanel(state);
  renderPlanningPanel(state);
  renderSystemPanel(state);

  ImGui::End();

  // ========================================
  // Timestamp Overlay (Draggable / Resizable)
  // ========================================
  {
    // 初回のみデフォルト位置に配置
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 240, 20),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(220, 80), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.45f);
    // ドラッグ・リサイズ可能にするため NoMove / NoDecoration / AlwaysAutoResize
    // は外す
    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Sim Time", nullptr,
                     window_flags)) { // nullptr = 閉じるボタンなし
      double display_time =
          state.is_replay_mode ? state.replay_time : state.sim_time;

      if (state.is_replay_mode) {
        const auto *frame =
            state.replay_manager_ptr->getFrameAt(state.replay_playback_idx);
        if (frame)
          display_time = frame->timestamp;
        ImGui::TextColored(ImVec4(1, 1, 0, 1), "[ REPLAY ]");
      } else if (state.replay_manager_ptr &&
                 state.replay_manager_ptr->isRecording()) {
        ImGui::TextColored(ImVec4(1, 0.2f, 0.2f, 1), "[ REC ]  %zu frames",
                           state.replay_manager_ptr->getFrameCount());
      }

      // フォントスケールはウィンドウサイズに合わせて自動調節
      float win_w = ImGui::GetContentRegionAvail().x;
      float font_scale = std::max(0.8f, std::min(2.5f, win_w / 110.0f));
      ImGui::SetWindowFontScale(font_scale);
      ImGui::Text("%.3f s", display_time);
      ImGui::SetWindowFontScale(1.0f);
    }
    ImGui::End();
  }
}

void GuiManager::renderExperimentPanel(SimulationState &state) {
  if (ImGui::CollapsingHeader("Experiment", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Start GNG Experiment", ImVec2(-1, 0))) {
      resetControlModes(state); // Reset before start
      if (start_experiment_callback_)
        start_experiment_callback_("GNG");
    }
    if (ImGui::Button("Start RRT Experiment", ImVec2(-1, 0))) {
      resetControlModes(state); // Reset before start
      if (start_experiment_callback_)
        start_experiment_callback_("RRT");
    }
    if (ImGui::Button("Reset Experiment", ImVec2(-1, 0))) {
      resetControlModes(state);
      if (reset_experiment_callback_)
        reset_experiment_callback_();
    }

    ImGui::Separator();
    // [NEW] RRT Topology Verification Button
    ImGui::PushStyleColor(ImGuiCol_Button, state.rrt_verify_mode_active ? ImVec4(0.8f, 0, 0, 1) : ImGui::GetStyle().Colors[ImGuiCol_Button]);
    if (ImGui::Button(state.rrt_verify_mode_active ? "STOP RRT Verification" : "START RRT Verification", ImVec2(-1, 0))) {
        state.rrt_verify_mode_active = !state.rrt_verify_mode_active;
        std::cerr << "\n[GUI EVENT] RRT Verification Toggled: " << (state.rrt_verify_mode_active ? "ON" : "OFF") << std::endl;
    }
    ImGui::PopStyleColor();

    ImGui::Separator();
    if (ImGui::Button("Start Static Evacuation", ImVec2(-1, 0))) {
      resetControlModes(state);
      if (state.scenario_manager_ptr) {
        state.scenario_manager_ptr->setScenario(
            robot_sim::simulation::ScenarioType::AVOIDANCE_STATIC);
        state.scenario_manager_ptr->setObstacleBehavior(
            robot_sim::simulation::ObstacleBehavior::LINEAR_RECIPROCATE);
        state.scenario_manager_ptr->setObstacleSpeed(0.5);

        // Use the current target position (set by sliders or snap button)
        state.scenario_manager_ptr->setTargetPosition(state.target_sphere_pos);

        state.auto_mode = true;
        state.reactive_trailing_mode = true;
        if (state.reactive_controller_ptr) {
          state.reactive_controller_ptr->setReactiveMode(true);
        }
        std::cout << "[GUI] Started STATIC EVACUATION scenario.\n";
      }
    }

    // --- Marker Control (Target Sphere) - Always visible for setup ---
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0, 0.8f, 1.0f, 1),
                       "Avoidance Target (Blue Sphere):");
    float tpos[3] = {(float)state.target_sphere_pos.x(),
                     (float)state.target_sphere_pos.y(),
                     (float)state.target_sphere_pos.z()};
    if (ImGui::DragFloat3("Target Pos##marker_exp", tpos, 0.005f)) {
      state.target_sphere_pos = Eigen::Vector3d(tpos[0], tpos[1], tpos[2]);
      state.show_target_sphere = true;
      if (state.scenario_manager_ptr) {
        state.scenario_manager_ptr->setTargetPosition(state.target_sphere_pos);
      }
    }
    if (ImGui::Button("Snap Target to EEF", ImVec2(-1, 0))) {
      if (state.kinematic_chain_ptr) {
        state.target_sphere_pos = state.kinematic_chain_ptr->getEEFPosition();
        state.show_target_sphere = true;
        if (state.scenario_manager_ptr) {
          state.scenario_manager_ptr->setTargetPosition(
              state.target_sphere_pos);
        }
      }
    }
    ImGui::Checkbox("Show Sphere##marker_exp", &state.show_target_sphere);
  }
}

void GuiManager::renderVisualizationPanel(SimulationState &state) {
  if (ImGui::CollapsingHeader("Visualization",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::TreeNode("GNG & Graphs")) {
      ImGui::Checkbox("Show Edges", &state.show_edges);
      ImGui::Checkbox("Show Coord Graph", &state.show_coord_graph);
      ImGui::Checkbox("Show Only Target", &state.show_only_target);
      if (ImGui::Checkbox("Show Candidate Paths",
                          &state.show_candidate_paths)) {
        // Toggle logic if needed
      }
      if (ImGui::Button("Clear All Paths", ImVec2(-1, 0))) {
        state.clearPathVisualizations();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Environment & Safety")) {
      ImGui::Checkbox("Show Danger Points", &state.show_danger_points);
      ImGui::Checkbox("Show PC Robot Structure",
                      &state.show_point_cloud_structure); // Added for Phase 8
      ImGui::Checkbox("Show Danger Field", &state.show_danger_field);
      ImGui::Checkbox("Show Danger Trends", &state.show_danger_trends);
      ImGui::Checkbox("Dual View (Snapshot)", &state.show_dual_view);
      ImGui::TreePop();
    }

    ImGui::Checkbox("Show Obstacle Voxels", &state.show_obstacle_voxels);
    ImGui::Checkbox("Simplified Visual", &state.use_simplified_visual);

    const char *manip_modes[] = {"OFF", "CURRENT_ONLY", "ALL_NODES"};
    int current_manip_idx = static_cast<int>(state.manip_viz_mode);
    if (ImGui::Combo("Manip. Visualization", &current_manip_idx, manip_modes,
                     IM_ARRAYSIZE(manip_modes))) {
      state.manip_viz_mode =
          static_cast<ManipulabilityVizMode>(current_manip_idx);
    }
    if (state.manip_viz_mode != ManipulabilityVizMode::OFF) {
      ImGui::SliderFloat("Manip. Scale", &state.manip_ellipsoid_scale, 0.1f,
                         10.0f);
    }

    const char *node_modes[] = {"ALL",
                                "ACTIVE_ONLY",
                                "INACTIVE_ONLY",
                                "DANGER_ONLY",
                                "COLLISION_ONLY",
                                "HAZARD_ONLY",
                                "INFLUENCE",
                                "PATH_ONLY",
                                "TOPOLOGY",
                                "ISLAND_ONLY",
                                "OFF"};
    int current_mode_idx = static_cast<int>(state.node_display_mode);
    if (ImGui::Combo("Node Display", &current_mode_idx, node_modes,
                     IM_ARRAYSIZE(node_modes))) {
      state.node_display_mode = static_cast<NodeDisplayMode>(current_mode_idx);
    }
    ImGui::Checkbox("Strict Topology Mode", &state.topology_strict_mode);
    ImGui::Checkbox("Show Surface Only", &state.show_surface_only);
  }
}

void GuiManager::renderAutoNavigationPanel(SimulationState &state) {
  if (ImGui::CollapsingHeader("Auto Navigation (Tracking)",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    // PAUSE / RESUME Button
    if (state.is_paused) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.6f, 1.0f, 1.0f));
      if (ImGui::Button("RESUME SIMULATION", ImVec2(-1, 0))) {
        state.is_paused = false;
      }
      ImGui::PopStyleColor();
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "Simulation PAUSED");
    } else {
      if (ImGui::Button("PAUSE SIMULATION", ImVec2(-1, 0))) {
        state.is_paused = true;
      }
    }
    ImGui::Separator();

    // Status Indicator
    if (state.auto_mode) {
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f),
                         "Status: MOVING (One-shot)");
    } else if (state.reactive_trailing_mode) {
      ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f),
                         "Status: CONTINUOUS TRACKING");
    } else {
      ImGui::Text("Status: IDLE");
    }

    if (ImGui::Checkbox("Continuous Tracking", &state.reactive_trailing_mode)) {
      if (!state.reactive_trailing_mode && state.reactive_controller_ptr) {
        state.reactive_controller_ptr->setReactiveMode(false);
      }
    }
    ImGui::Checkbox("Show Tracking Viz (Green/Purple Spheres)", &state.show_reactive_tracking_viz);

    if (ImGui::SliderFloat("Trajectory Height", &state.trajectory_height, 0.05f,
                           1.0f)) {
      if (state.scenario_manager_ptr) {
        state.scenario_manager_ptr->setTrajectoryHeight(
            state.trajectory_height);
      }
    }

    if (ImGui::Checkbox("Enable Dynamic Obstacle (Blue Ball)",
                        &state.enable_dynamic_obstacle)) {
    }

    if (state.enable_dynamic_obstacle) {
      const char *behavior_names[] = {
          "Passive", "Aggressive",        "Evasive",
          "Random",  "Trajectory",        "Hybrid",
          "Bounce",  "Linear (Approach)", "Pause (This)"};
      int behavior_idx =
          static_cast<int>(state.scenario_manager_ptr->getObstacleBehavior());
      if (ImGui::Combo("Behavior", &behavior_idx, behavior_names,
                       IM_ARRAYSIZE(behavior_names))) {
        if (state.scenario_manager_ptr) {
          state.scenario_manager_ptr->setObstacleBehavior(
              static_cast<robot_sim::simulation::ObstacleBehavior>(
                  behavior_idx));
        }
      }
    }

    ImGui::Checkbox("Show Target Sphere", &state.show_target_sphere);

    ImGui::Separator();
    ImGui::Text("Target Selection:");
    if (ImGui::Button("Pick Random Posture",
                      ImVec2(ImGui::GetContentRegionAvail().x * 0.5f, 0))) {
      if (pick_random_posture_callback_)
        pick_random_posture_callback_();
    }
    ImGui::SameLine();
    if (ImGui::Button("Pick Random GNG Node", ImVec2(-1, 0))) {
      if (pick_random_gng_node_callback_)
        pick_random_gng_node_callback_();
    }

    if (ImGui::Button("Plan & Move to Target (G)", ImVec2(-1, 0))) {
      resetControlModes(state); // Reset before plan
      // Plan mode sets auto_mode internally later, but we ensure clean slate

      if (state.robot_hal_ptr && plan_and_execute_path_callback_) {
        Eigen::VectorXf start_q =
            state.robot_hal_ptr->getJointPositions().cast<float>();
        plan_and_execute_path_callback_(start_q, state.global_target_goal_q);
      }
    }

    // EEF Monitoring
    if (state.kinematic_chain_ptr) {
      Eigen::Vector3d eef = state.kinematic_chain_ptr->getEEFPosition();
      float dist = (eef - state.target_sphere_pos).norm();
      ImGui::Text("EEF Dist to Target: %.3f m", dist);
      if (dist < 0.05f) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "[REACHED]");
      }
    }

    if (ImGui::SliderFloat("Speed Scale", &state.speed_scale, 0.1f, 10.0f)) {
      if (state.controller_ptr)
        state.controller_ptr->setSpeedScale(
            static_cast<double>(state.speed_scale));
    }
    ImGui::Checkbox("Enable Prediction Field (Heavy)",
                    &state.enable_prediction_field);

    if (ImGui::Checkbox("Enable Influence Tracking",
                        &state.enable_influence_tracking)) {
      if (state.enable_influence_tracking) {
        if (state.influence_manager_ptr && state.gng_ptr &&
            state.robot_hal_ptr) {
          Eigen::VectorXf current_q =
              state.robot_hal_ptr->getJointPositions().cast<float>();
          state.influence_manager_ptr->initialize(current_q, *state.gng_ptr,
                                                  0.5f);
        }
      } else {
        if (state.influence_manager_ptr) {
          state.influence_manager_ptr->clear();
        }
      }
    }

    // Trajectory Control
    if (ImGui::CollapsingHeader("Trajectory Control",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("TRAJECTORY Mode", ImVec2(-1, 0))) {
        resetControlModes(state); // Reset before trajectory mode

        if (state.scenario_manager_ptr) {
          state.scenario_manager_ptr->setScenario(
              robot_sim::simulation::ScenarioType::TRAJECTORY);

          state.reactive_trailing_mode = true;
          if (state.reactive_controller_ptr) {
            state.reactive_controller_ptr->setReactiveMode(true);
          }
        }
      }

      const char *traj_names[] = {"Circle", "Ellipse", "Infinity", "Square"};
      if (ImGui::Combo("Pattern", &traj_idx_, traj_names,
                       IM_ARRAYSIZE(traj_names))) {
        if (state.scenario_manager_ptr) {
          state.scenario_manager_ptr->setTrajectoryType(
              static_cast<robot_sim::simulation::TrajectoryType>(traj_idx_));
        }
      }

      if (ImGui::SliderFloat("Scale (m)", &traj_scale_, 0.05f, 0.6f)) {
        if (state.scenario_manager_ptr)
          state.scenario_manager_ptr->setTrajectoryScale(traj_scale_);
      }

      if (ImGui::SliderFloat("Speed (rad/s)", &traj_speed_, 0.1f, 2.0f)) {
        if (state.scenario_manager_ptr)
          state.scenario_manager_ptr->setTrajectorySpeed(traj_speed_);
      }
    }
  }
}

void GuiManager::renderPlanningPanel(SimulationState &state) {
  if (ImGui::CollapsingHeader("Planning & Metrics",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Hazard Monitoring (VLUT Dilation)");
    ImGui::SliderFloat("Hazard Voxel Dilation (m)", &state.danger_voxel_dilation,
                       0.005f, 0.15f, "%.3f");

    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Pre-baked max: 0.1m (0.05m margin + 0.05m base)");
    }

    // ソリッドモード切り替え
    bool is_instant =
        state.safety_state_manager_ptr
            ? state.safety_state_manager_ptr->isInstantaneousMode()
            : false;
    if (ImGui::Checkbox("Solid Danger Mode (Instant)", &is_instant)) {
      if (state.safety_state_manager_ptr) {
        state.safety_state_manager_ptr->setInstantaneousMode(is_instant);
      }
    }
    // 点群膨張切り替え
    if (is_instant) { // Only show if solid mode is active
      ImGui::Indent();
      bool use_dilation = state.safety_state_manager_ptr
                              ? state.safety_state_manager_ptr->getParameters()
                                    .use_point_cloud_dilation
                              : true;
      if (ImGui::Checkbox("Point Cloud Dilation (Stamp)", &use_dilation)) {
        if (state.safety_state_manager_ptr) {
          auto p = state.safety_state_manager_ptr->getParameters();
          p.use_point_cloud_dilation = use_dilation;
          state.safety_state_manager_ptr->setParameters(p);
        }
      }
      ImGui::Unindent();
    }
    ImGui::Separator();

    // ========================================
    // Replay / Recording GUI
    // ========================================
    ImGui::Text("Simulation Replay & Log");
    if (state.replay_manager_ptr) {
      // 録画制御
      if (!state.is_replay_mode) {
        if (!state.replay_manager_ptr->isRecording()) {
          if (ImGui::Button("Start Recording")) {
            state.replay_manager_ptr->startRecording();
          }
        } else {
          ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1, 0, 0, 1));
          if (ImGui::Button("Stop Recording")) {
            state.replay_manager_ptr->stopRecording();
          }
          ImGui::PopStyleColor();
          ImGui::SameLine();
          ImGui::Text("Rec: %zu frames",
                      state.replay_manager_ptr->getFrameCount());
        }
      }

      ImGui::Separator();

      // リプレイ制御
      if (ImGui::Checkbox("Enable Replay Mode", &state.is_replay_mode)) {
        if (state.is_replay_mode) {
          state.replay_manager_ptr->stopRecording();
        }
      }

      if (state.is_replay_mode) {
        size_t frame_count = state.replay_manager_ptr->getFrameCount();
        if (frame_count > 0) {
          int idx = state.replay_playback_idx;
          if (ImGui::SliderInt("Timeline", &idx, 0, (int)frame_count - 1)) {
            state.replay_playback_idx = idx;
          }
          ImGui::SameLine();
          ImGui::Checkbox("Auto Play", &state.replay_is_playing);
          const auto *frame = state.replay_manager_ptr->getFrameAt(idx);
          if (frame) {
            ImGui::Text("Time: %.3f s", frame->timestamp);
            ImGui::Text("Status: %s", frame->is_danger ? "DANGER" : "SAFE");
          }
        } else {
          ImGui::TextDisabled("No frames recorded.");
        }
      }

      if (ImGui::Button("Save Log to CSV")) {
        state.replay_manager_ptr->saveToCSV("simulation_replay_log.csv");
      }
      ImGui::SameLine();
      if (ImGui::Button("Save Replay File (.rply)")) {
        state.replay_manager_ptr->saveToFile("simulation_replay.rply");
      }

      // ファイルロード (入力フィールド + ボタン)
      static char replay_load_path[256] = "simulation_replay.rply";
      ImGui::SetNextItemWidth(200);
      ImGui::InputText("##rply_path", replay_load_path,
                       sizeof(replay_load_path));
      ImGui::SameLine();
      if (ImGui::Button("Load Replay File")) {
        if (state.replay_manager_ptr->loadFromFile(replay_load_path)) {
          state.is_replay_mode = true;
          state.replay_playback_idx = 0;
          state.replay_time = 0.0;
        }
      }
    }
    ImGui::Separator();

    // Multi-Target GNG Options
    if (state.current_planning_method ==
        robot_sim::simulation::PlanningMethod::GNG_DIJKSTRA) {
      ImGui::Text("GNG Multi-Goal");
      ImGui::Checkbox("Use Multi-Target", &state.gng_use_multi_target);
      if (state.gng_use_multi_target) {
        ImGui::SliderInt("Candidates", &state.gng_multi_target_count, 1, 10);
      }
      ImGui::Separator();
    }

    // Planning Method Selection
    const char *method_names[] = {"GNG-Dijkstra", "IK-RRT Connect"};
    int current_method_idx = static_cast<int>(state.current_planning_method);
    if (ImGui::Combo("Trajectory Planner", &current_method_idx, method_names,
                     IM_ARRAYSIZE(method_names))) {
      state.current_planning_method =
          static_cast<robot_sim::simulation::PlanningMethod>(
              current_method_idx);
    }

    ImGui::Separator();
    ImGui::Checkbox("Strict Goal Collision Check",
                    &state.strict_goal_collision_check);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "If enabled, the planner will fail if the goal node is in collision. "
          "If disabled (default), goal collision is allowed.");
    }

    // Cost Metric Selection
    const char *metric_names[] = {"Distance (Joint)",
                                  "Kinematic Manipulability",
                                  "Dynamic Manipulability",
                                  "Directional Manipulability",
                                  "EEF Distance",
                                  "Risk (Danger Field)",
                                  "L1-Norm",
                                  "L-Inf Norm"};
    int current_metric_idx = static_cast<int>(state.current_metric);
    if (ImGui::Combo("Cost Metric", &current_metric_idx, metric_names,
                     IM_ARRAYSIZE(metric_names))) {
      state.current_metric = static_cast<robot_sim::simulation::PlanningMetric>(
          current_metric_idx);

      // Apply to Coordinator
      if (state.planning_coordinator_ptr) {
        std::shared_ptr<
            planning::ICostEvaluator<Eigen::VectorXf, Eigen::Vector3f>>
            evaluator = nullptr;

        if (state.current_metric ==
            robot_sim::simulation::PlanningMetric::RISK) {
          // Create Risk Evaluator
          if (state.safety_state_manager_ptr) {
            // Default risk weight 10.0
            evaluator =
                std::make_shared<robot_sim::simulation::RiskAwareCostEvaluator<
                    Eigen::VectorXf, Eigen::Vector3f>>(
                    state.safety_state_manager_ptr, 10.0f);
          }
        }

        state.planning_coordinator_ptr->setCostMetric(state.current_metric,
                                                      state, evaluator);
      }
    }

    // Additional metrics UI could go here (omitted for brevity in this port,
    // can be added if needed)
  }
}

void GuiManager::renderSystemPanel(SimulationState &state) {
  if (ImGui::CollapsingHeader("External Control & System",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("UDP Control:");
    ImGui::Indent();
    if (ImGui::Checkbox("Real-time Posture Sync (Sync to Real)",
                        &state.manual_udp_mode)) {
      if (state.manual_udp_mode) {
        state.auto_mode = false;
        state.reactive_trailing_mode = false;
        state.experiment_active = false;
      }
    }
    if (state.udp_bridge_ptr) {
      bool running = state.udp_bridge_ptr->isRunning();
      ImGui::Text("UDP Listener: %s", running ? "RUNNING" : "STOPPED");
      if (running) {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Port: 12345");
      }
    }
    ImGui::Unindent();

    if (ImGui::Checkbox("Geometry Only (Visual-Only)",
                        &state.geometry_only_mode)) {
      auto *ode_hal =
          dynamic_cast<::simulation::OdeRobotSim *>(state.robot_hal_ptr.get());
      if (ode_hal) {
        ode_hal->setKinematicMode(state.geometry_only_mode);
      }
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("rviz-like mode: Bypasses physics and directly renders "
                        "commanded joint states.");
    }
  }
}

} // namespace simulation
} // namespace robot_sim
