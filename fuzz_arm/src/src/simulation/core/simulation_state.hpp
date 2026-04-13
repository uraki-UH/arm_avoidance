#pragma once

#include "planner/RRT/rrt_params.hpp"
#include "simulation/viz/gui_manager.hpp"
#include "visualization/color_scheme.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <ode/ode.h>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include "simulation/core/replay_manager.hpp"
#include "hardware/twin_manager.hpp"

namespace kinematics {
class KinematicChain;
class JointStateAdapter;
}

namespace robot_sim {
namespace simulation {
class DynamicObstacleManager;
class OdeStateValidityChecker;
class ScenarioManager;
class ReactiveController;
class SafetyStateManager;
class VisualizationManager;
class UdpCommandBridge;
class InfluenceManager;
class PlanningCoordinator;
class MeshCache;
struct MeshEntry;

} // namespace simulation

namespace analysis {
class GraphTopologyAnalyzer;
} // namespace analysis

} // namespace robot_sim

namespace simulation {
class OdeRobotSim;
class OdeGngVisualizer;
class RobotModel;
class VoxelGrid;
struct OdeRobotComponent;
class CollisionManager;
class EnvironmentManager;
class OdeRobotCollisionModel;
class PointCloudCollisionChecker;
class ExperimentLogger;
} // namespace simulation

namespace GNG {

template <typename WeightType, typename StatusType> class GrowingNeuralGas2;
}

namespace planning {
template <typename T_angle, typename T_coord, typename T_GNG>
class GngDijkstraPlanner;
}

namespace control {
class LinearTrajectoryController;
}

namespace robot_sim {

namespace planner {
class IKRRTPlanner;

} // namespace planner

namespace analysis {
class ISpatialIndex;
}

namespace status {
template <typename WeightType, typename StatusType> class GNGStatusUpdater;
}

namespace visualization {
class JointAnglePublisher;
} // namespace visualization

namespace experiment {
class ExperimentRunner;
class IScenario;
class TargetTouchScenario;
} // namespace experiment

namespace simulation {

/**
 * @brief プランニング手法の列挙
 */
enum class PlanningMethod { GNG_DIJKSTRA, IK_RRT_CONNECT };

/**
 * @brief ノード表示モードの列挙
 */
enum class NodeDisplayMode {
  ALL,
  ACTIVE_ONLY,
  INACTIVE_ONLY,
  DANGER_ONLY,
  COLLISION_ONLY,
  HAZARD_ONLY,
  INFLUENCE,
  PATH_ONLY,
  TOPOLOGY,
  ISLAND_ONLY,
  OFF
};

/**
 * @brief マニピュラビリティ可視化モードの列挙
 */
enum class ManipulabilityVizMode { OFF, CURRENT_ONLY, ALL_NODES };

/**
 * @brief プランニングメトリクスの列挙
 */
enum class PlanningMetric {
  DISTANCE,
  KINEMATIC_MANIPULABILITY,
  DYNAMIC_MANIPULABILITY,
  DIRECTIONAL_MANIPULABILITY,
  EEF_DISTANCE,
  RISK,
  DISTANCE_L_INF
};

/**
 * @brief 候補パス可視化用の構造体
 */
struct CandidatePathVisualization {
  std::vector<Eigen::VectorXf> path;
  float cost;
  Eigen::Vector3f color;
  int node_id = -1;
  bool is_selected = false;
};

/**
 * @brief シミュレーション全体の状態を管理するクラス
 *
 * online_integration_main.cpp の50個以上のグローバル変数を
 * 構造化されたクラスに集約します。
 */
class SimulationState {
public:
  SimulationState();
  ~SimulationState();

  // コピー禁止
  SimulationState(const SimulationState &) = delete;
  SimulationState &operator=(const SimulationState &) = delete;

  // ムーブ許可
  SimulationState(SimulationState &&) = default;
  SimulationState &operator=(SimulationState &&) = default;

  /**
   * @brief 状態のクリーンアップ
   */
  void cleanup();

  /**
   * @brief パス可視化のクリア（実行中のパスや候補パスの表示を消去）
   */
  void clearPathVisualizations();

  // ========================================
  // ODE Physics
  // ========================================
  dWorldID world = nullptr;
  dSpaceID space = nullptr;
  dWorldID sc_world = nullptr; // For self-collision checking
  dSpaceID sc_space = nullptr; // For self-collision checking
  dJointGroupID contactgroup = nullptr;
  dGeomID ground = nullptr;

  // ========================================
  // Robot
  // ========================================
  std::unique_ptr<::simulation::OdeRobotSim> robot_hal_ptr;
  std::unique_ptr<kinematics::KinematicChain> fk_chain_ptr;
  kinematics::KinematicChain *kinematic_chain_ptr =
      nullptr; // Raw pointer for shared access
  std::shared_ptr<::simulation::RobotModel> robot_model_ptr;
  std::unique_ptr<kinematics::JointStateAdapter> state_adapter_ptr;
  std::map<std::string, ::simulation::OdeRobotComponent> robot_components_map;

  // Target sphere for visualization
  dGeomID target_sphere_geom = nullptr;
  dBodyID target_sphere_body = nullptr;
  Eigen::Vector3d target_sphere_pos = Eigen::Vector3d(0.25, 0.0, 0.35);
  Eigen::Quaterniond target_sphere_quat = Eigen::Quaterniond::Identity();

  // Robot state
  bool is_robot_safe = true;
  Eigen::VectorXd hold_positions;
  Eigen::Vector3d target_eef_pos = Eigen::Vector3d(0.25, 0.0, 0.35);
  Eigen::Vector3d node_cartesian_goal = Eigen::Vector3d::Zero();
  Eigen::VectorXf global_target_goal_q;

  // ========================================
  // GNG
  // ========================================
  std::unique_ptr<GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>
      gng_ptr;
  std::shared_ptr<analysis::ISpatialIndex> spatial_index_ptr;
  std::unique_ptr<status::GNGStatusUpdater<Eigen::VectorXf, Eigen::Vector3f>>
      status_updater_ptr;
  std::unique_ptr<robot_sim::analysis::GraphTopologyAnalyzer>
      topology_analyzer_ptr; // Added

  // GNG visualization
  // GNG visualization
  std::unique_ptr<::simulation::OdeGngVisualizer> gng_viz_ptr;

  int target_node_id = -1;
  bool show_only_target = false;
  bool show_edges = false;
  bool show_coord_graph = false;
  bool show_surface_only = false;
  NodeDisplayMode node_display_mode = NodeDisplayMode::ALL;
  bool topology_strict_mode = false;
  visualization::ColorScheme node_color_scheme;

  // Point Cloud Grid
  // Point Cloud Grid
  std::unique_ptr<::simulation::VoxelGrid> point_cloud_grid_ptr;

  // ========================================
  // Planning
  // ========================================
  std::unique_ptr<planning::GngDijkstraPlanner<
      Eigen::VectorXf, Eigen::Vector3f,
      GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>>
      gng_planner_ptr;
  std::unique_ptr<planner::IKRRTPlanner> rrt_planner_ptr;
  std::shared_ptr<PlanningCoordinator> planning_coordinator_ptr; // Added
  PlanningMethod current_planning_method = PlanningMethod::GNG_DIJKSTRA;
  planner::RRTParams global_rrt_params;

  // GNG Multi-Goal Settings
  bool gng_use_multi_target = false;
  int gng_multi_target_count = 5;

  PlanningMetric current_metric = PlanningMetric::DISTANCE;
  bool avoid_collision_mode = true;         // Enable by default for safety
  bool strict_goal_collision_check = false; // Added for Phase 7

  // Candidate paths visualization
  std::vector<CandidatePathVisualization> candidate_paths_viz;
  bool show_candidate_paths = false;

  // ========================================
  // Control
  // ========================================
  std::unique_ptr<::control::LinearTrajectoryController> controller_ptr;
  std::unique_ptr<robot_sim::simulation::ReactiveController>
      reactive_controller_ptr;
  bool auto_mode = false;
  float speed_scale = 1.0f;
  float trajectory_height = 0.3f;
  bool is_paused = false;

  // Execution
  std::deque<Eigen::VectorXd> active_path;
  std::vector<int> current_path_node_ids;
  bool is_executing_path = false;

  // ========================================
  // Collision Detection
  // ========================================
  std::unique_ptr<::simulation::CollisionManager> collision_manager_ptr;
  std::unique_ptr<::simulation::CollisionManager>
      sc_manager_ptr; // For self-collision
  std::shared_ptr<::simulation::OdeRobotCollisionModel>
      robot_collision_model_ptr;
  std::unique_ptr<::simulation::PointCloudCollisionChecker>
      pc_collision_checker_ptr;
  std::unique_ptr<::simulation::EnvironmentManager> env_manager_ptr;

  std::unique_ptr<robot_sim::simulation::MeshCache> mesh_cache_ptr; // Added
  bool use_mesh_collision = true; // Toggle for mesh-accurate vs primitive collision detection

  // ========================================
  // Dynamic Obstacles
  // ========================================
  std::unique_ptr<DynamicObstacleManager> obstacle_manager_ptr;
  std::shared_ptr<robot_sim::simulation::SafetyStateManager>
      safety_state_manager_ptr;
  bool enable_dynamic_obstacle = true;
  Eigen::Vector3d demo_obstacle_pos = Eigen::Vector3d(0.4, 0.0, 0.5);
  double demo_obstacle_radius = 0.05;

  // Reactive control
  bool reactive_trailing_mode = false;
  float online_danger_threshold =
      0.025f; // Suggested by user for better sensitivity
  bool risk_aware_mode = true;
  bool enable_prediction_field = false;
  bool use_point_cloud_dilation =
      true; // 点群の膨張処理 (Stamping) を有効にするか

  // Collision tracking
  std::vector<int> node_collision_counts;
  std::vector<int> node_danger_counts;

  // Validity Checker
  std::unique_ptr<robot_sim::simulation::OdeStateValidityChecker>
      validity_checker_ptr;

  // ========================================
  // Experiment System
  // ========================================
  std::unique_ptr<experiment::ExperimentRunner> experiment_runner_ptr;
  std::shared_ptr<robot_sim::experiment::IScenario> current_scenario_ptr;
  bool experiment_active = false;
  bool rrt_verify_mode_active = false;

  // ========================================
  // Visualization & UI
  // ========================================
  bool show_obstacle_voxels = false;
  bool show_danger_field = false;
  bool show_danger_points = false;        // Disabled by default to reduce clutter
  bool show_point_cloud_structure = true; // Added for Phase 8
  bool show_target_axes = false;
  bool show_target_sphere = false;    // Hide target sphere until explicitly set
  bool use_simplified_visual = false; // Toggle for simplified rendering mode
  bool gui_show_panel = true;
  std::unique_ptr<GuiManager> gui_manager_ptr; // Use unique_ptr
  bool enable_influence_tracking = false;
  bool show_danger_trends = false;
  bool show_dual_view = false;
  bool show_reactive_tracking_viz = false;
  Eigen::Vector3d dual_view_offset = Eigen::Vector3d(0, 1.0, 0);

  ManipulabilityVizMode manip_viz_mode = ManipulabilityVizMode::OFF;
  float manip_ellipsoid_scale = 1.0f;

  // ========================================
  // Utilities
  // ========================================
  std::unique_ptr<visualization::JointAnglePublisher> joint_angle_publisher_ptr;
  std::unique_ptr<robot_sim::simulation::VisualizationManager> viz_manager_ptr;

  std::unique_ptr<robot_sim::simulation::UdpCommandBridge> udp_bridge_ptr;
  std::unique_ptr<robot_sim::simulation::InfluenceManager> influence_manager_ptr;
  std::shared_ptr<robot_sim::simulation::ScenarioManager> scenario_manager_ptr;

  bool manual_udp_mode = false;
  bool geometry_only_mode = false;

  std::unique_ptr<robot_sim::digital_twin::DigitalTwinManager> digital_twin_manager;

  // Logging
  std::unique_ptr<::simulation::ExperimentLogger> logger_ptr;

  int eef_log_counter = 0;
  bool enable_csv_flush = false; // Control CSV flush behavior
  double sim_time = 0.0;        // Current simulation time

  // ========================================
  // Replay System
  // ========================================
  std::unique_ptr<ReplayManager> replay_manager_ptr;
  bool is_replay_mode = false;
  bool replay_is_playing = true; // Added for Auto Play control
  int replay_playback_idx = 0;
  double replay_time = 0.0;

  // Analysis / Paper measurement
  double vlut_update_time_ms = 0.0;
  double topology_analysis_time_ms = 0.0;
  bool topology_was_connected = true;
  double topology_disconnection_time = -1.0;
  double last_lead_time = -1.0;

  // ========================================
  // Helper Methods
  // ========================================

  bool hasGng() const { return gng_ptr != nullptr; }
  bool hasRobot() const { return robot_hal_ptr != nullptr; }
  bool isExperimentActive() const { return experiment_active; }

  PlanningMethod getPlanningMethod() const { return current_planning_method; }

  void setPlanningMethod(PlanningMethod method) {
    current_planning_method = method;
  }
};

} // namespace simulation
} // namespace robot_sim
