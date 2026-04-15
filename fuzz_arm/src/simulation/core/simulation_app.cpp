#define _USE_MATH_DEFINES
#include "simulation/core/simulation_app.hpp"
#include "simulation/robot/geometry_management.hpp"

#include "simulation/core/simulation_state.hpp"
#include "simulation/planning/ode_state_validity_checker.hpp"
#include <chrono>
#include <cmath>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

#include <Eigen/Dense>
#include <drawstuff/drawstuff.h>
#include <iostream>
#include <memory>
#include <ode/ode.h>

// アンブレラヘッダー群
#include "kinematics/state_adapter.hpp"
#include "simulation/planning/simulation_planning.hpp"
#include "simulation/robot/simulation_robot.hpp"
#include "simulation/safety/simulation_safety.hpp"
#include "simulation/sensing/simulation_sensing.hpp"
#include "simulation/viz/simulation_viz.hpp"
#include "simulation/world/simulation_world.hpp"

// その他 (個別に必要なもの)
#include "collision/ode/ode_robot_collision_model.hpp"
#include "collision/point_cloud_collision_checker.hpp"
#include "control/gng_data_adapter.hpp"
#include "simulation/reactive_controller.hpp"
#include "spatial/surface_node_classifier.hpp"
#include "status/graph_topology_analyzer.hpp"
#include "simulation/core/udp_command_bridge.hpp"
#include "simulation/core/experiment_logger.hpp"
#include "common/resource_utils.hpp"

// ImGui
#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace robot_sim {
namespace simulation {

using namespace ::simulation;

// 静的メンバの定義
dsFunctions SimulationApp::fn_;

// ヘルパー関数: nearCallback (Cスタイル)

// ここでは簡単のため、SimulationAppクラス内で定義された static
// コールバックから シングルトンの private
// メンバにアクセスできることを利用する。 ただしこの nearCallback
// はファイルスコープのstatic関数として定義しようとしているため、
// SimulationAppのフレンドにするか、SimulationAppのstaticメンバにする必要がある。
// ヘッダーでの定義に従い、SimulationAppのロジック内に組み込む。

// SimulationStateへのアクセサがないため、friend化するか、
// SimulationApp::nearCallback を実装してそれを呼ぶ形にする。
// ここでは、SimulationApp::step() 内で dSpaceCollide を呼ぶ際に、
// ラムダやローカル関数は使えない(関数ポインタが必要)ため、
// SimulationAppの静的メンバ関数として `staticNearCallback`
// を用意するのがベスト。 しかしヘッダーには入れていなかった。
// cppファイル内で解決するため、SimulationApp::instance()の参照を返す関数を利用する。
// ただし state_ は private。
// 今回は実装を簡単にするため、SimulationAppがCollisionManagerへの委譲を行う
// static public メソッドを持つか、あるいは instance() が state
// への参照を返すアクセサを持つか。
// ヘッダー修正なしでいくなら、Cスタイルのコールバックは SimulationApp
// の外にある必要があり、 かつ SimulationApp の private
// メンバにはアクセスできない。
// -> SimulationApp.hpp に `friend void nearCallback(...)`
// を書くのが定石だが、 編集済みなので、SimulationState struct
// の定義が公開されている(ヘッダーインクルードされている)
// ことを利用し、SimulationApp に `SimulationState& getState() { return
// state_; }` を追加するのが最も手っ取り早い。 しかしヘッダーは既に書いた。
// よって、SimulationApp.cpp 内で SimulationApp::stepCallback から呼ばれる
// step() の中で ローカルな static 関数を使う。SimulationApp
// のメンバ関数内であれば private メンバにアクセスできるが、 dSpaceCollide
// に渡す関数ポインタは static である必要がある。 妥協案:
// SimulationApp::instance().handleCollision(o1, o2) を呼ぶ static 関数を
// SimulationAppの内部クラスまたはfriendとして定義したいが、
// ここは「SimulationAppのstaticメンバ関数」として定義し、ヘッダーには載せていないが
// クラス定義外の関数からアクセス...はできない。

// 解決策: SimulationState は struct
// であり、SimulationApp内でpublicに保持...していない、privateだ。
// 今回のヘッダー設計では `state_` は private。
// よって、`initialize` や `step` など内部のロジックで state_ を使う。
// dSpaceCollide に渡すコールバック関数だけが問題。
// SimulationApp.cpp 内で、クラスの static メンバではない普通の関数として
// nearCallback を定義し、 それがアクセスできるように、SimulationApp に
// `friend void nearCallback(void*, dGeomID, dGeomID);`
// を追加宣言(前方宣言)したいが、ヘッダーを変えないといけない。

// 強引だが、SimulationApp のインスタンスはシングルトンなので、
// グローバル変数としてポインタを持たせておく(init時にセット)手もあるが、汚い。

// 最も正統法は、SimulationApp::step() の中で dSpaceCollide を呼ぶとき、
// state_.collision_manager_ptr を使うわけだが、
// CollisionManager::nearCallback は static
// なので、それをラップする関数が必要。 SimulationApp.cpp 内で `static
// SimulationState* g_state_ptr = nullptr;` を作り、 initialize
// でセット、nearCallback はそれを使う。これが実用的。

// 内部用グローバルポインタ (コールバック用)
static SimulationState *g_state_ptr = nullptr;

static void globalNearCallback(void * /*data*/, dGeomID o1, dGeomID o2) {
  if (g_state_ptr && g_state_ptr->collision_manager_ptr) {
    CollisionManager::nearCallback(g_state_ptr->collision_manager_ptr.get(), o1,
                                   o2);
  }
}

SimulationApp &SimulationApp::instance() {
  static SimulationApp app;
  return app;
}

SimulationApp::SimulationApp() {
  // コンストラクタ
  // グローバルポインタの設定
  g_state_ptr = &state_;
}

SimulationApp::~SimulationApp() {
  finalize();
  g_state_ptr = nullptr;
}

void SimulationApp::startCallback() {
  // Viewpoint 設定
  float xyz[3] = {1.5f, 0.5f, 1.0f};
  float hpr[3] = {-150.0f, -20.0f, 0.0f};
  dsSetViewpoint(xyz, hpr);

  // Initialize ImGui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();

  ImGui_ImplGLUT_Init();
  ImGui_ImplOpenGL2_Init();
  ImGui_ImplGLUT_InstallFuncs();

  // GuiManager Callbacks 設定
  auto &app = instance();

  // Safety State Manager initial status log
  if (app.state_.safety_state_manager_ptr) {
    std::cout << "[App] Safety Mode: "
              << (app.state_.safety_state_manager_ptr->isInstantaneousMode()
                      ? "Instantaneous"
                      : "Time-Decay")
              << std::endl;
  }
}

void SimulationApp::stepCallback(int pause) {
  instance().updateCamera();
  instance().update(pause);
  instance().draw();
  instance().renderGui();
}

void SimulationApp::commandCallback(int cmd) { instance().handleCommand(cmd); }

void SimulationApp::stopCallback() {
  // 必要なら実装
}

void SimulationApp::run(int argc, char **argv) {
  // --replay <file> 引数の解析 (config より前に確認)
  std::string replay_file_to_load;
  for (int i = 1; i < argc - 1; ++i) {
    if (std::string(argv[i]) == "--replay") {
      replay_file_to_load = argv[i + 1];
      break;
    }
  }

  std::string config_file = (argc > 1) ? argv[1] : "config.txt";
  if (!initialize(config_file)) {
    return;
  }

  // --replay が指定されていた場合、ファイルからロードしてリプレイモードを有効化
  if (!replay_file_to_load.empty() && state_.replay_manager_ptr) {
    if (state_.replay_manager_ptr->loadFromFile(replay_file_to_load)) {
      state_.is_replay_mode = true;
      state_.replay_playback_idx = 0;
      state_.replay_time = 0.0;
      std::cout << "[App] Replay mode enabled. "
                << state_.replay_manager_ptr->getFrameCount()
                << " frames loaded from: " << replay_file_to_load << "\n";
    }
  }

  // DrawStuff 設定
  fn_.version = DS_VERSION;
  fn_.start = &startCallback;
  fn_.step = &stepCallback;
  fn_.command = &commandCallback;
  fn_.stop = nullptr;

  // robot_sim::common::resolvePath -> ::robot_sim::common::resolvePath
  static std::string resolved_texture_path =
      ::robot_sim::common::resolvePath("drawstuff/textures");
  fn_.path_to_textures = resolved_texture_path.c_str();

  // 影の無効化引数追加
  std::vector<char *> new_argv;
  for (int i = 0; i < argc; ++i) {
    new_argv.push_back(argv[i]);
  }
  char noshadow_arg[] = "-noshadow";
  new_argv.push_back(noshadow_arg);
  int new_argc = static_cast<int>(new_argv.size());

  // シミュレーションループ開始
  dsSimulationLoop(new_argc, new_argv.data(), 800, 600, &fn_);
}

bool SimulationApp::initialize(const std::string &config_file) {
  dInitODE();
  srand(static_cast<unsigned int>(time(NULL)));

  if (!::common::ConfigManager::Instance().Load(config_file)) {
    std::cerr << "Failed to load config file: " << config_file << std::endl;
    return false;
  }
  auto &config = ::common::ConfigManager::Instance();

  // 実験設定のロード
  std::string exp_settings_path = ::robot_sim::common::resolvePath("experiment_settings/experiment_settings.txt");
  if (config.Load(exp_settings_path)) {
    state_.demo_obstacle_radius =
        config.GetDouble("radius", state_.demo_obstacle_radius);
    state_.danger_voxel_dilation = (float)config.GetDouble(
        "danger_threshold", state_.danger_voxel_dilation);
    state_.strict_goal_collision_check =
        config.GetBool("strict_goal_collision_check", false);
    state_.avoid_collision_mode = config.GetBool("avoid_collision_mode", true);

    // [MODIFIED] Centralize management: config.txt should override
    // experiment_settings.txt
    state_.danger_voxel_dilation = (float)config.GetDouble(
        "danger_threshold", state_.danger_voxel_dilation);
    state_.use_mesh_collision = config.GetBool("use_mesh_collision", true);
  }

  // ODE 初期化
  state_.world = dWorldCreate();
  state_.space = dHashSpaceCreate(0);
  state_.contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(state_.world, 0, 0, -9.81);
  state_.ground = dCreatePlane(state_.space, 0, 0, 1, 0);

  // Increase solver stability
  dWorldSetQuickStepNumIterations(state_.world, 100);
  dWorldSetContactMaxCorrectingVel(state_.world, 2.0); // Prevent explosive push-back
  dWorldSetContactSurfaceLayer(state_.world, 0.002);   // Allow small overlap to reduce jitter

  // ターゲット球 (Visual Only)
  state_.target_sphere_body = dBodyCreate(state_.world);
  dMass mass;
  dMassSetSphere(&mass, 0.001, 0.025);
  dBodySetMass(state_.target_sphere_body, &mass);
  state_.target_sphere_geom = dCreateSphere(0, 0.025);
  dGeomSetBody(state_.target_sphere_geom, state_.target_sphere_body);

  // ロボット構築
  state_.collision_manager_ptr =
      std::make_unique<CollisionManager>(state_.world, state_.contactgroup);

  std::string urdf_path = config.Get("robot_urdf_path", "custom_robot");
  auto model_obj = loadRobotFromUrdf(urdf_path);
  state_.robot_model_ptr = std::make_shared<RobotModel>(model_obj);

  // Mesh Cache Initialization
  state_.mesh_cache_ptr = std::make_unique<MeshCache>();

  OdeRobotBuilder builder(state_.world, state_.space,
                          state_.collision_manager_ptr.get(),
                          state_.mesh_cache_ptr.get());
  builder.setUseMesh(state_.use_mesh_collision); // Apply collision toggle setting

  state_.robot_components_map =
      builder.build(*state_.robot_model_ptr, CollisionCategory::ROBOT_PART,
                    CollisionCategory::ALL & ~CollisionCategory::ROBOT_PART,
                    Eigen::Vector3d(0, 0, 0));

  // 環境 & 衝突設定
  state_.env_manager_ptr =
      std::make_unique<EnvironmentManager>(state_.world, state_.space);
  if (state_.env_manager_ptr->load("env_config.txt")) {
    std::cout << "[App] Environment loaded from env_config.txt\n";
  }

  state_.viz_manager_ptr =
      std::make_unique<robot_sim::simulation::VisualizationManager>();

  state_.collision_manager_ptr->registerGeom(state_.ground, "ground",
                                             CollisionCategory::GROUND,
                                             CollisionCategory::ROBOT_PART);

  setupDefaultCollisionExclusions(*state_.collision_manager_ptr);
  setupAdjacentCollisionExclusions(*state_.robot_model_ptr,
                                   *state_.collision_manager_ptr);

  // キネマティクスチェーン
  std::string leaf_link = config.Get("leaf_link_name", "link_7");
  std::vector<std::string> target_joint_names;

  kinematics::KinematicChain temp_chain = createKinematicChainFromModel(
      *state_.robot_model_ptr, leaf_link, Eigen::Vector3d(0, 0, 0));
  state_.fk_chain_ptr =
      std::make_unique<kinematics::KinematicChain>(temp_chain);

  // 簡略化ビジュアルの生成
  state_.use_simplified_visual = config.GetBool("use_simplified_visual", false);
  generateSimplifiedRobotVisuals();

  state_.kinematic_chain_ptr = state_.fk_chain_ptr.get();

  state_.point_cloud_grid_ptr = std::make_unique<::simulation::VoxelGrid>(0.02);

  int num_joints = temp_chain.getNumJoints();
  for (int i = 0; i < num_joints; ++i) {
    if (temp_chain.getJointDOF(i) > 0) {
      target_joint_names.push_back(temp_chain.getLinkName(i));
    }
  }

  // Add gripper joints manually if they exist and are not in the main chain
  for (const auto &pair : state_.robot_model_ptr->getJoints()) {
    const auto &j = pair.second;
    if (j.name.find("gripper") != std::string::npos &&
        j.type != kinematics::JointType::Fixed) {
      if (std::find(target_joint_names.begin(), target_joint_names.end(),
                    j.child_link) == target_joint_names.end()) {
        target_joint_names.push_back(j.child_link);
      }
    }
  }

  // OdeRobotSim & collision checker
  state_.robot_hal_ptr = std::make_unique<OdeRobotSim>(
      state_.world, state_.collision_manager_ptr.get(), *state_.robot_model_ptr,
      state_.robot_components_map, target_joint_names,
      state_.geometry_only_mode);
  state_.pc_collision_checker_ptr =
      std::make_unique<PointCloudCollisionChecker>(*state_.robot_model_ptr,
                                                   *state_.kinematic_chain_ptr);

  // --- JointStateAdapter Initialization ---
  state_.state_adapter_ptr = std::make_unique<kinematics::JointStateAdapter>();
  std::vector<std::string> logical_names;
  for (int i = 0; i < state_.fk_chain_ptr->getNumJoints(); ++i) {
    if (state_.fk_chain_ptr->getJointDOF(i) > 0) {
      logical_names.push_back(state_.fk_chain_ptr->getLinkName(i));
    }
  }
  std::vector<std::string> physical_names = state_.robot_hal_ptr->getJointNames();
  state_.state_adapter_ptr->init(logical_names, physical_names);

  // 初期姿勢設定 (Piper用に展開姿勢を設定)
  // Joint limits: J1(-2.6, 2.6), J2(0, 3.14), J3(-2.9, 0), J4?, J5?, J6?
  // 0.0 だと畳まれて表示が乱れる可能性があるため、少し展開する。
  std::vector<double> initial_pose_cmd = {0.0, 0.2, -0.2, 0.0, 0.0, 0.0};

  if (target_joint_names.size() >= 6) {
    std::string urdf_name = config.Get("robot_urdf_path", "");
    if (urdf_name == "piper_mesh" || urdf_name == "piper_refined") {
      // Piper Specific Pose
      initial_pose_cmd = {0.0, 0.5, -0.5, 0.0, 0.5, 0.0};
    }
  }

  for (size_t i = 0; i < target_joint_names.size(); ++i) {
    double val = (i < initial_pose_cmd.size()) ? initial_pose_cmd[i] : 0.0;
    state_.robot_hal_ptr->setJointCommand(target_joint_names[i], val,
                                          ::control::ControlMode::POSITION);
  }
  dWorldStep(state_.world, 0.01);
  state_.hold_positions = state_.robot_hal_ptr->getJointPositions();

  Eigen::VectorXd initial_q_double = state_.robot_hal_ptr->getJointPositions();
  int total_dof = state_.kinematic_chain_ptr->getTotalDOF();
  state_.kinematic_chain_ptr->updateKinematics(
      initial_q_double.head(std::min((int)initial_q_double.size(), total_dof)));

  // 可視化・ログ初期化
  state_.joint_angle_publisher_ptr =
      std::make_unique<robot_sim::visualization::JointAnglePublisher>();

  manipulability_csv_.open("manipulability_data.csv",
                           std::ios::out | std::ios::trunc);
  if (manipulability_csv_.is_open()) {
    manipulability_csv_ << "timestamp,manipulability,condition_number,min_"
                           "singular_value,ellipsoid_volume,manip_x,manip_y,"
                           "manip_z\n";
  }

  reachability_csv_.open("reachability_stats.dat",
                         std::ios::out | std::ios::trunc);
  if (reachability_csv_.is_open()) {
    reachability_csv_ << "# time mainland_size island_size is_robot_safe "
                         "total_active_nodes vlut_time_ms lead_time\n";
  }

  robustness_csv_.open("robustness_sweep.dat", std::ios::out | std::ios::trunc);
  if (robustness_csv_.is_open()) {
    robustness_csv_ << "# time margin mainland_size island_size is_robot_safe "
                       "total_active_nodes\n";
  }

  // GNG 初期化
  state_.gng_ptr = std::make_unique<
      GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>(
      temp_chain.getTotalDOF(), 3, state_.kinematic_chain_ptr);

  // 自己衝突判定用環境
  state_.sc_world = dWorldCreate();
  state_.sc_space = dHashSpaceCreate(0);
  state_.sc_manager_ptr =
      std::make_unique<CollisionManager>(state_.sc_world, state_.contactgroup);

  static dGeomID sc_ground = dCreatePlane(state_.sc_space, 0, 0, 1, 0);
  state_.sc_manager_ptr->registerGeom(sc_ground, "sc_ground",
                                      CollisionCategory::GROUND,
                                      CollisionCategory::ROBOT_PART);

  setupDefaultCollisionExclusions(*state_.sc_manager_ptr);
  setupAdjacentCollisionExclusions(*state_.robot_model_ptr,
                                   *state_.sc_manager_ptr);

  state_.robot_collision_model_ptr =
      std::make_unique<::simulation::OdeRobotCollisionModel>(
          *state_.robot_model_ptr, state_.sc_world, state_.sc_space,
          state_.sc_manager_ptr.get(), *state_.kinematic_chain_ptr,
          state_.mesh_cache_ptr.get(),
          state_.use_mesh_collision); // Pass the toggle flag

  auto sc_provider = std::make_shared<
      GNG::SelfCollisionProvider<Eigen::VectorXf, Eigen::Vector3f>>(
      state_.robot_collision_model_ptr.get(), state_.kinematic_chain_ptr);
  state_.gng_ptr->registerStatusProvider(sc_provider);

  state_.gng_ptr->registerStatusProvider(
      std::make_shared<
          GNG::EEDirectionProvider<Eigen::VectorXf, Eigen::Vector3f>>(
          state_.kinematic_chain_ptr));
  state_.gng_ptr->registerStatusProvider(
      std::make_shared<
          GNG::ManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(
          state_.kinematic_chain_ptr));
  state_.gng_ptr->registerStatusProvider(
      std::make_shared<
          GNG::DynamicManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(
          state_.kinematic_chain_ptr, state_.robot_model_ptr.get()));

  // Node Color Scheme
  /*
  state_.node_color_scheme.addRule(
      robot_sim::visualization::NodeAttribute::ISLAND,
      robot_sim::visualization::NodeAttribute::NONE,
      robot_sim::visualization::Color::fromRGB(200, 0, 255), 95); // Purple
  */
  state_.node_color_scheme.addRule(
      robot_sim::visualization::NodeAttribute::MAINLAND,
      robot_sim::visualization::NodeAttribute::NONE,
      robot_sim::visualization::Color::fromRGB(0, 255, 0),
      55); // Vibrant Green (Mainland)
  state_.node_color_scheme.addRule(
      robot_sim::visualization::NodeAttribute::DANGER,
      robot_sim::visualization::NodeAttribute::NONE,
      robot_sim::visualization::Color::fromRGB(255, 255, 0), 82); // Yellow
  state_.node_color_scheme.addRule(
      robot_sim::visualization::NodeAttribute::COLLIDING,
      robot_sim::visualization::NodeAttribute::NONE,
      robot_sim::visualization::Color::fromRGB(255, 255, 0), 85); // Yellow (was Red)


  // UDP Bridge
  state_.udp_bridge_ptr =
      std::make_unique<robot_sim::simulation::UdpCommandBridge>(12345);
  state_.udp_bridge_ptr->start();

  // GNGパラメータ・マップロード
  state_.gng_ptr->loadParameters(::robot_sim::common::resolvePath("gng_online.cfg"));
  std::cout << "[App] Loaded GNG config: gng_online.cfg\n";
  std::string data_dir = config.Get("data_directory", "gng_results");
  std::string exp_id = config.Get("experiment_id", "default_experiment");
  std::string map_suffix = config.Get("online_input_suffix", "_phase2");
  
  // Automatically construct spatial index path based on experiment
  std::string spatial_file = data_dir + "/" + exp_id + "/gng_spatial_correlation.bin";
  config.Set("spatial_correlation_file", spatial_file);
  std::cout << "[App] Auto-resolved spatial index path: " << spatial_file << std::endl;

  std::string map_file = data_dir + "/" + exp_id + "/" + exp_id + map_suffix + ".bin";
  
  if (state_.gng_ptr->load(map_file)) {
    std::cout << "[App] Loaded GNG map: " << map_file << "\n";
    state_.gng_ptr->refresh_coord_weights();
    std::cout << "[App] Loaded " << state_.gng_ptr->getMaxNodeNum() << " nodes.\n";
  } else {
    std::cerr << "[App] Failed to load map: " << map_file << "\n";
  }

  // Dynamic Obstacle / Planning Coordinator init
  state_.planning_coordinator_ptr = std::make_shared<PlanningCoordinator>();
  if (state_.planning_coordinator_ptr->initializeAnalysisSystem(state_)) {
    state_.status_updater_ptr = std::make_unique<
        robot_sim::status::GNGStatusUpdater<Eigen::VectorXf, Eigen::Vector3f>>(
        state_.gng_ptr.get());

    state_.topology_analyzer_ptr =
        std::make_unique<robot_sim::analysis::GraphTopologyAnalyzer>();

    state_.node_collision_counts.assign(state_.gng_ptr->getMaxNodeNum(), 0);
    state_.node_danger_counts.assign(state_.gng_ptr->getMaxNodeNum(), 0);
  } else {
    state_.enable_dynamic_obstacle = false;
  }

  // Safety State Manager
  state_.safety_state_manager_ptr =
      std::make_shared<robot_sim::simulation::SafetyStateManager>();
  state_.safety_state_manager_ptr->resize(state_.gng_ptr->getMaxNodeNum());

  // Load Parameters
  SafetyParameters safety_params;
  safety_params.instantaneous_mode = config.GetBool(
      "safety_mode_instantaneous", safety_params.instantaneous_mode);
  safety_params.danger_voxel_dilation = (float)config.GetDouble(
      "safety_danger_threshold", state_.danger_voxel_dilation);
  safety_params.use_point_cloud_dilation = config.GetBool(
      "safety_use_dilation", safety_params.use_point_cloud_dilation);

  state_.safety_state_manager_ptr->setParameters(safety_params);

  state_.gng_viz_ptr = std::make_unique<simulation::OdeGngVisualizer>();

  // Digital Twin Initialization
  state_.digital_twin_manager = std::make_unique<robot_sim::digital_twin::DigitalTwinManager>(5005, 5006);
  state_.digital_twin_manager->start();
  // Default mode for now
  state_.digital_twin_manager->setMode(robot_sim::digital_twin::SyncMode::RECEPTOR_SYNC);

  // Planning & Control Initialization
  state_.planning_coordinator_ptr->initializePlanningSystem(state_);

  if (!state_.validity_checker_ptr) {
    state_.validity_checker_ptr =
        std::make_unique<robot_sim::simulation::OdeStateValidityChecker>(
            state_.robot_collision_model_ptr.get(),
            state_.pc_collision_checker_ptr.get(),
            state_.point_cloud_grid_ptr.get(), state_.env_manager_ptr.get(),
            state_.kinematic_chain_ptr);
  }

  state_.planning_coordinator_ptr->initializeControlSystem(state_);

  // GuiManager とコールバックの設定
  state_.gui_manager_ptr = std::make_unique<GuiManager>(); // ここで初期化
  // コールバック設定
  state_.gui_manager_ptr->setStartExperimentCallback(
      [this](const std::string &method) { this->startExperiment(method); });
  state_.gui_manager_ptr->setResetExperimentCallback(
      [this]() { this->resetExperiment(); });
  state_.gui_manager_ptr->setPickRandomPostureCallback(
      [this]() { this->handlePickRandomPosture(); });
  state_.gui_manager_ptr->setPickRandomGngNodeCallback(
      [this]() { this->handlePickRandomGngNode(); });
  state_.gui_manager_ptr->setPlanAndExecutePathCallback(
      [this](const Eigen::VectorXf &s, const Eigen::VectorXf &g) {
        this->planAndExecutePath(s, g);
      });

  return true;
}

void SimulationApp::finalize() {
  if (state_.logger_ptr)
    state_.logger_ptr->close();
  if (manipulability_csv_.is_open())
    manipulability_csv_.close();
  if (reachability_csv_.is_open())
    reachability_csv_.close();
  if (robustness_csv_.is_open())
    robustness_csv_.close();

  dJointGroupDestroy(state_.contactgroup);
  if (state_.space)
    dSpaceDestroy(state_.space);
  if (state_.world)
    dWorldDestroy(state_.world);
  if (state_.sc_space)
    dSpaceDestroy(state_.sc_space);
  if (state_.sc_world)
    dWorldDestroy(state_.sc_world);
  dCloseODE();
}

void SimulationApp::handleCommand(int cmd) {
  if (ImGui::GetIO().WantCaptureKeyboard)
    return;
  /*
  if (cmd == 'f' || cmd == 'F') {
    state_.enable_csv_flush = !state_.enable_csv_flush;
    std::cout << "[UI] CSV Flush (Real-time): " << (state_.enable_csv_flush ?
  "ON" : "OFF") << std::endl; return;
  }
  */

  // マニュアルUDPモード切り替え ('u'キー)
  if (cmd == 'u' || cmd == 'U') {
    state_.manual_udp_mode = !state_.manual_udp_mode;
    std::cout << "[App] UDP Manual Mode: "
              << (state_.manual_udp_mode ? "ON" : "OFF")
              << " (Polling Override)" << std::endl;
    if (state_.manual_udp_mode) {
      state_.auto_mode = false;
      if (state_.robot_hal_ptr)
        state_.hold_positions = state_.robot_hal_ptr->getJointPositions();
    }
  }

  // [DEBUG] トポロジー検証モード切り替え ('i'キー)
  if (cmd == 'i' || cmd == 'I') {
    state_.rrt_verify_mode_active = !state_.rrt_verify_mode_active;
    std::cerr << "\n[KEY EVENT] RRT Verification Toggled (via Key): "
              << (state_.rrt_verify_mode_active ? "ON" : "OFF") << std::endl;
  }

  if (cmd == 27) { // ESC
    exit(0);
  }
}

void SimulationApp::update(int pause) {
  // GUIや入力の更新
  if (ImGui::IsKeyPressed(ImGuiKey_U)) {
    handleCommand('u');
  }
  // (RRT Verification logic moved to topology analysis block for better synchronization)


  // ========================================
  // Replay System (Post-GUI, Pre-Sim)
  // ========================================
  if (state_.is_replay_mode && state_.replay_manager_ptr) {
    const auto *frame =
        state_.replay_manager_ptr->getFrameAt(state_.replay_playback_idx);
    if (frame) {
      // --- ロボット・障害物・目標の復元 ---
      if (state_.robot_hal_ptr) {
        state_.robot_hal_ptr->setJointPositions(
            frame->joint_positions.cast<double>());
      }
      state_.demo_obstacle_pos = frame->obstacle_pos;
      state_.target_sphere_pos = frame->target_pos;
      state_.is_robot_safe = !frame->is_danger;

      // --- GNG ノードの全状態復元 ---
      if (state_.gng_ptr && !frame->node_snapshots.empty()) {
        for (const auto &snap : frame->node_snapshots) {
          try {
            auto &node = state_.gng_ptr->nodeAt(snap.id);
            node.status.valid = snap.valid;
            node.status.active = snap.active;
            node.status.is_colliding = snap.is_colliding;
            node.status.is_danger = snap.is_danger;
            node.status.is_mainland = snap.is_mainland;
            node.status.topology_group_id = snap.topology_group_id;
          } catch (...) {
          }
        }
      }

      // --- パス・表示・障害物アームの復元 ---
      if (!frame->path_node_ids.empty()) {
        state_.current_path_node_ids = frame->path_node_ids;
      }
      if (state_.env_manager_ptr && !frame->robot_obstacle_states.empty()) {
        state_.env_manager_ptr->setRobotObstacleStates(
            frame->robot_obstacle_states);
      }
      state_.replay_time = frame->timestamp;

      // --- 物理ボディの強制同期 (テレポーション) ---
      // リプレイ時は物理ステップ(dWorldStep)をスキップするため、
      // 順運動学(FK)の結果を直接ODE Bodyに適用する。
      if (state_.robot_hal_ptr && state_.fk_chain_ptr) {
        Eigen::VectorXd q_replayed = frame->joint_positions.cast<double>();

        // 1. 内部の仮想関節角を更新 (GUI/同期用)
        if (auto *ode_hal = dynamic_cast<::simulation::OdeRobotSim *>(
                state_.robot_hal_ptr.get())) {
          ode_hal->setJointPositions(q_replayed);
        }

        // 2. FK計算
        int rep_dof = state_.fk_chain_ptr->getTotalDOF();
        state_.fk_chain_ptr->updateKinematics(
            q_replayed.head(std::min((int)q_replayed.size(), rep_dof)));
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
            positions;
        std::vector<Eigen::Quaterniond,
                    Eigen::aligned_allocator<Eigen::Quaterniond>>
            orientations;
        state_.fk_chain_ptr->forwardKinematicsAt(q_replayed, positions,
                                                 orientations);

        // 3. 全リンクの絶対座標トランスフォームを構築
        std::map<std::string, Eigen::Isometry3d> link_transforms;
        state_.fk_chain_ptr->buildAllLinkTransforms(
            positions, orientations, state_.robot_model_ptr->getFixedLinkInfo(),
            link_transforms);

        // 4. ODE Bodyへの反映
        if (auto *ode_hal = dynamic_cast<::simulation::OdeRobotSim *>(
                state_.robot_hal_ptr.get())) {
          for (const auto &pair : link_transforms) {
            ode_hal->setLinkPose(pair.first, pair.second.translation(),
                                 Eigen::Quaterniond(pair.second.rotation()));
          }
        }
      }
      // --- 再生インデックスの進展 ---
      if (!pause && !state_.is_paused && state_.replay_is_playing) {
        state_.replay_playback_idx++;
        if (state_.replay_playback_idx >=
            (int)state_.replay_manager_ptr->getFrameCount()) {
          state_.replay_playback_idx = 0; // ループ再生
        }
      }
    }

    // 可視化（FKチェーン、EEF位置等）の同期
    syncKinematics();

    // リプレイ中は物理演算や通常の更新をスキップ
    return;
  }

  // 実験ランナーの更新
  if ((!pause && !state_.is_paused) || state_.geometry_only_mode) {
    // auto frame_start_time = std::chrono::high_resolution_clock::now();
    static auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - start_time).count();

    static double sim_time = 0.0;
    sim_time += 0.01;
    state_.sim_time = sim_time; // Sync with state for GUI

    // タイマーリセット
    safety_update_time_ms_ = 0.0;
    env_update_time_ms_ = 0.0;

    auto q = state_.robot_hal_ptr->getJointPositions();
    Eigen::VectorXd dq(q.size());
    auto joint_names = state_.robot_hal_ptr->getJointNames();
    for (size_t i = 0; i < joint_names.size(); ++i) {
      dq[i] = state_.robot_hal_ptr->getJointState(joint_names[i]).velocity;
    }

    double min_dist = -1.0;
    Eigen::Vector3d obs_pos = Eigen::Vector3d::Zero();

    // 障害物情報の更新とログ
    if (state_.obstacle_manager_ptr) {
      const auto &obstacles = state_.obstacle_manager_ptr->getObstacles();
      if (!obstacles.empty()) {
        obs_pos = obstacles[0].position;
        min_dist = (state_.target_eef_pos - obs_pos).norm();
      }

      if (state_.logger_ptr) {
        for (const auto &obs : obstacles) {
          state_.logger_ptr->logObstacle(sim_time, obs.id, obs.position,
                                         obs.filtered_velocity, obs.radius);
        }
      }
    }

    // ロボット障害物のログ
    if (state_.env_manager_ptr && state_.logger_ptr) {
      auto robot_states = state_.env_manager_ptr->getRobotObstacleStates();
      state_.logger_ptr->logRobotObstacles(sim_time, robot_states);
    }

    // 関節トルクの取得
    Eigen::VectorXf tau(q.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      tau[i] = static_cast<float>(
          state_.robot_hal_ptr->getJointState(joint_names[i]).effort);
    }

    // 関節角度のパブリッシュ (可視化用)
    if (state_.joint_angle_publisher_ptr &&
        state_.joint_angle_publisher_ptr->isReady()) {
      Eigen::VectorXd q_double = state_.robot_hal_ptr->getJointPositions();
      state_.joint_angle_publisher_ptr->publish(sim_time, q_double);
    }

    // マニピュラリティの計算とログ
    if (manipulability_csv_.is_open() && state_.kinematic_chain_ptr) {
      try {
        int eef_index = state_.kinematic_chain_ptr->getNumJoints() - 1;
        Eigen::MatrixXd jacobian =
            state_.kinematic_chain_ptr->calculateJacobian(eef_index);

        if (jacobian.rows() >= 3 && jacobian.cols() > 0) {
          Eigen::MatrixXd J_trans = jacobian.topRows(3);
          auto metrics = Manipulability::calculateFullMetrics(J_trans);

          manipulability_csv_
              << sim_time << "," << metrics.manipulability << ","
              << metrics.condition_number << "," << metrics.min_singular_value
              << "," << metrics.volume << "," << metrics.manip_x << ","
              << metrics.manip_y << "," << metrics.manip_z << "\n";
          manipulability_csv_.flush();
        }
      } catch (const std::exception &e) {
        // エラーは無視
      }
    }

    // シナリオ情報の準備
    std::string scenarioStr = "IDLE";
    std::string modeStr = "NONE";
    if (state_.scenario_manager_ptr) {
      scenarioStr = state_.scenario_manager_ptr->getScenarioName();
      modeStr = state_.scenario_manager_ptr->getModeName();
    }

    // 簡易衝突チェック (ログ用)
    bool is_collision = false;
    if (min_dist >= 0 && min_dist < 0.15) {
      is_collision = true;
    }

    // ステップログ
    if (state_.logger_ptr) {
      int active_nodes = 0;
      int touched_nodes = 0;
      if (state_.safety_state_manager_ptr) {
        active_nodes =
            (int)state_.safety_state_manager_ptr->getActiveNodeCount();
        touched_nodes =
            (int)state_.safety_state_manager_ptr->getTouchedNodeCount();
      }

      state_.logger_ptr->logStep(
          sim_time, q.cast<float>(), dq.cast<float>(), tau, obs_pos, min_dist,
          active_nodes, touched_nodes, 0.0, last_env_time_, last_safety_time_,
          scenarioStr, modeStr, is_collision);
    }
    // ========================================
    // Replay / Record Logic (Full State)
    // ========================================
    if (state_.is_replay_mode && state_.replay_manager_ptr) {
      const auto *frame =
          state_.replay_manager_ptr->getFrameAt(state_.replay_playback_idx);
      if (frame) {
        // --- ロボット・障害物・目標の復元 ---
        if (state_.robot_hal_ptr) {
          state_.robot_hal_ptr->setJointPositions(
              frame->joint_positions.cast<double>());
        }
        state_.demo_obstacle_pos = frame->obstacle_pos;
        state_.target_sphere_pos = frame->target_pos;
        state_.is_robot_safe = !frame->is_danger;

        // --- GNG ノードの全状態復元 ---
        // 可視化フラグ(show_path,
        // NodeDisplayMode等)は既存のGUIで制御されるため、
        // ここでは状態データのみを復元する。描画はSimulationApp::drawが担う。
        if (state_.gng_ptr && !frame->node_snapshots.empty()) {
          for (const auto &snap : frame->node_snapshots) {
            try {
              auto &node = state_.gng_ptr->nodeAt(snap.id);
              node.status.valid = snap.valid;
              node.status.active = snap.active;
              node.status.is_colliding = snap.is_colliding;
              node.status.is_danger = snap.is_danger;
              node.status.is_mainland = snap.is_mainland;
              node.status.topology_group_id = snap.topology_group_id;
            } catch (...) {
            } // 存在しないノードIDはスキップ
          }
        }

        // --- パス復元 (表示は show_path フラグで GUI 制御) ---
        if (!frame->path_node_ids.empty()) {
          state_.current_path_node_ids = frame->path_node_ids;
        }

        // --- 障害物アームの復元 ---
        if (state_.env_manager_ptr && !frame->robot_obstacle_states.empty()) {
          state_.env_manager_ptr->setRobotObstacleStates(
              frame->robot_obstacle_states);
        }

        // リプレイ時間の更新
        state_.replay_time = frame->timestamp;
      }
      // リプレイ中は物理演算や通常の更新をスキップ
      return;
    }

    // --- 通常時の記録 ---
    if (state_.replay_manager_ptr && state_.replay_manager_ptr->isRecording()) {
      robot_sim::simulation::LogFrame log_frame;
      log_frame.timestamp = sim_time;
      log_frame.joint_positions =
          state_.robot_hal_ptr->getJointPositions().cast<float>();
      log_frame.target_pos = state_.target_sphere_pos;
      log_frame.obstacle_pos = state_.demo_obstacle_pos;
      log_frame.is_danger = !state_.is_robot_safe;

      // GNG 全ノードのスナップショット収集
      if (state_.gng_ptr) {
        state_.gng_ptr->forEachActiveValid([&](int id, const auto &node) {
          robot_sim::simulation::NodeSnapshot snap;
          snap.id = id;
          snap.valid = node.status.valid;
          snap.active = node.status.active;
          snap.is_colliding = node.status.is_colliding;
          snap.is_danger = node.status.is_danger;
          snap.is_mainland = node.status.is_mainland;
          snap.topology_group_id = node.status.topology_group_id;
          log_frame.node_snapshots.push_back(snap);
        });
      }

      // 現在のパス
      log_frame.path_node_ids = state_.current_path_node_ids;

      // 障害物アームの状態
      if (state_.env_manager_ptr) {
        log_frame.robot_obstacle_states =
            state_.env_manager_ptr->getRobotObstacleStates();
      }

      state_.replay_manager_ptr->addFrame(std::move(log_frame));
    }

    // 1. 危険場の更新準備
    state_.node_collision_counts.assign(state_.node_collision_counts.size(), 0);
    state_.node_danger_counts.assign(state_.node_danger_counts.size(), 0);

    if (state_.scenario_manager_ptr) {
      state_.scenario_manager_ptr->update(
          0.01, state_.kinematic_chain_ptr->getEEFPosition(),
          state_.node_collision_counts, state_.node_danger_counts,
          state_.danger_voxel_dilation, state_.enable_dynamic_obstacle);

      // Digital Twin Sync (Node/Receptor mode)
      if (state_.digital_twin_manager) {
        state_.digital_twin_manager->update(state_.node_collision_counts, state_.spatial_index_ptr.get());
      }

      // Global Delta Bridge
      if (state_.enable_dynamic_obstacle) {
        if (state_.obstacle_manager_ptr && state_.safety_state_manager_ptr &&
            state_.spatial_index_ptr) {
          auto t_vlut_start = std::chrono::high_resolution_clock::now();
          auto delta = state_.obstacle_manager_ptr->updateUnifiedOccupancy();
          state_.safety_state_manager_ptr->applyOccupancyDeltas(
              state_.spatial_index_ptr, delta,
              state_.obstacle_manager_ptr->getVoxelSize());
          auto t_vlut_end = std::chrono::high_resolution_clock::now();
          state_.vlut_update_time_ms =
              std::chrono::duration<double, std::milli>(t_vlut_end -
                                                        t_vlut_start)
                  .count();
        }
      } else {
        state_.vlut_update_time_ms = 0.0;
        // If disabled, we still check if there are lingering unified voxels.
        // ScenarioManager::update already called clearObstacles if enabled was
        // false.
        if (state_.obstacle_manager_ptr && state_.safety_state_manager_ptr &&
            state_.spatial_index_ptr &&
            !state_.obstacle_manager_ptr->getUnifiedOccupiedVoxels().empty()) {
          auto delta = state_.obstacle_manager_ptr->updateUnifiedOccupancy();
          state_.safety_state_manager_ptr->applyOccupancyDeltas(
              state_.spatial_index_ptr, delta,
              state_.obstacle_manager_ptr->getVoxelSize());
        }
      }
      state_.demo_obstacle_pos =
          state_.scenario_manager_ptr->getObstaclePosition();
      if (!state_.reactive_trailing_mode) {
        state_.target_sphere_pos =
            state_.scenario_manager_ptr->getTargetPosition();
      }
    }

    if (state_.safety_state_manager_ptr) {
      auto t_safety_start = std::chrono::high_resolution_clock::now();
      state_.safety_state_manager_ptr->prepareUpdate();
      state_.safety_state_manager_ptr->update(0.01); // 減衰
      auto t_safety_end = std::chrono::high_resolution_clock::now();
      safety_update_time_ms_ += std::chrono::duration<double, std::milli>(
                                    t_safety_end - t_safety_start)
                                    .count();
    }

    // 2. 環境 & 点群の更新
    if (state_.env_manager_ptr) {
      auto t_env_start = std::chrono::high_resolution_clock::now();
      state_.env_manager_ptr->update(t);
      auto t_env_end = std::chrono::high_resolution_clock::now();
      env_update_time_ms_ =
          std::chrono::duration<double, std::milli>(t_env_end - t_env_start)
              .count();

      const auto &pc = state_.env_manager_ptr->getCombinedPointCloud();

      // フェーズ1衝突チェック用VoxelGridへの入力
      if (state_.point_cloud_grid_ptr) {
        state_.point_cloud_grid_ptr->clear();
        for (const auto &p : pc) {
          state_.point_cloud_grid_ptr->insert(p);
        }
      }

      // 2.5 点群からの危険度更新
      if (state_.safety_state_manager_ptr && state_.spatial_index_ptr) {
        auto t_safety_2_start = std::chrono::high_resolution_clock::now();

        auto dense_index =
            std::dynamic_pointer_cast<robot_sim::analysis::DenseSpatialIndex>(
                state_.spatial_index_ptr);
        if (dense_index) {
          state_.safety_state_manager_ptr->updateDangerFieldFromPoints(
              dense_index, pc, state_.danger_voxel_dilation,
              state_.use_point_cloud_dilation);

          // 2.6 動的障害物 (球体デモ) の危険度フィールド評価 (マージン計算を含む)
          if (state_.enable_dynamic_obstacle) {
             state_.safety_state_manager_ptr->updateDangerField(
                 dense_index, state_.demo_obstacle_pos, 0.05, Eigen::Vector3d::Zero());
          }
        }
        state_.safety_state_manager_ptr->finalizeUpdate(0.01);
        auto t_safety_2_end = std::chrono::high_resolution_clock::now();
        safety_update_time_ms_ += std::chrono::duration<double, std::milli>(
                                      t_safety_2_end - t_safety_2_start)
                                      .count();
      }
    }

    // 3. 危険場更新の完了 (速度計算など)
    if (state_.safety_state_manager_ptr) {
      state_.safety_state_manager_ptr->finalizeUpdate(0.01);

      // 3.5 GNGノード状態の同期
      auto t_topo_start = std::chrono::high_resolution_clock::now();
      if (state_.status_updater_ptr) {
        // Discrete Counts (Scenario + Digital Twin)
        state_.status_updater_ptr->applyCollisionCounts(
            state_.gng_ptr.get(), state_.node_collision_counts,
            state_.node_danger_counts);
            
        // Continuous Danger Levels (Voxel/Pointcloud)
        state_.status_updater_ptr->applySafetyStatus(
            state_.gng_ptr.get(),
            state_.safety_state_manager_ptr->getDangerLevels(),
            state_.safety_state_manager_ptr->getTouchedIndices(), 0.8f, 0.1f);
      }

      // 3.6 C-Space Topology 判定 (Robot-Centric)
      if (state_.topology_analyzer_ptr && state_.gng_ptr) {
        // ロボットが現在衝突しているかどうかを確認
        Eigen::VectorXf raw_q =
            state_.robot_hal_ptr->getJointPositions().cast<float>();
        int cur_dof = state_.kinematic_chain_ptr ? state_.kinematic_chain_ptr->getTotalDOF() : raw_q.size();
        Eigen::VectorXf current_q = raw_q.head(std::min((int)raw_q.size(), cur_dof));
        
        bool is_robot_safe = true;
        Eigen::Vector3d current_eef_pos = Eigen::Vector3d::Zero();
        if (state_.kinematic_chain_ptr) {
            state_.kinematic_chain_ptr->updateKinematics(current_q.cast<double>());
            current_eef_pos = state_.kinematic_chain_ptr->getEEFPosition();
            
            // VLUT-based check (High Speed)
            if (state_.safety_state_manager_ptr && state_.spatial_index_ptr) {
                is_robot_safe = !state_.safety_state_manager_ptr->isCollidingAt(
                    state_.spatial_index_ptr.get(), current_eef_pos);
            } else if (state_.validity_checker_ptr) {
                // Fallback to ODE
                is_robot_safe = state_.validity_checker_ptr->isValid(current_q.cast<double>());
            }
        }
        state_.is_robot_safe = is_robot_safe;

        // ロボットにもっとも近いノードを探す (VLUT-Accelerated Reachability Check)
        int robot_node_id = -1;
        if (is_robot_safe && state_.gng_planner_ptr) {
          robot_node_id = state_.gng_planner_ptr->findNearestReachableNode(
              current_q, current_eef_pos, *state_.gng_ptr, state_.spatial_index_ptr.get(), 
              state_.safety_state_manager_ptr.get());
        }

        // Lambda for validation: valid, active, non-colliding
        auto is_valid = [&](int node_id) {
          const auto &node = state_.gng_ptr->nodeAt(node_id);
          // Actual collisions always break topology.
          if (!node.status.valid || !node.status.active ||
              node.status.is_colliding)
            return false;

          // In strict mode, danger (proximity to obstacles) also breaks
          // topology.
          if (state_.topology_strict_mode && node.status.is_danger)
            return false;

          return true;
        };

        // Lambda for neighbors
        auto get_neighbors = [&](int node_id) -> const std::vector<int> & {
          return state_.gng_ptr->getNeighborsAngle(node_id);
        };

        // BFSによる本土・島解析 兼 ステータス更新 (Hyper-Fast / Single Pass)
        auto t_bfs_start = std::chrono::high_resolution_clock::now();
        auto topo_stats = state_.topology_analyzer_ptr->analyzeIntegrated(
                                              state_.gng_ptr->getNodes(),
                                              state_.gng_ptr->getActiveIndices(), 
                                              state_.gng_ptr->getMaxNodeNum(),
                                              is_valid, get_neighbors,
                                              is_robot_safe ? robot_node_id : -1, false);

        int active_count = topo_stats.active_count;
        int mainland_count = topo_stats.mainland_count;
        int island_count = topo_stats.island_count;

        auto t_topo_end = std::chrono::high_resolution_clock::now();
        state_.topology_analysis_time_ms =
            std::chrono::duration<double, std::milli>(t_topo_end - t_topo_start)
                .count();
        double status_update_time = std::chrono::duration<double, std::milli>(
                                        t_bfs_start - t_topo_start)
                                        .count();
        double bfs_pure_time =
            std::chrono::duration<double, std::milli>(t_topo_end - t_bfs_start)
                .count();

        // 統計用デバッグ出力 (一定間隔)
        static int topo_print_counter = 0;
        if (++topo_print_counter >= 100) {
          topo_print_counter = 0;
          std::cout << "[DEBUG] Total: " << std::fixed << std::setprecision(3)
                    << state_.topology_analysis_time_ms << "ms "
                    << "(Update: " << status_update_time
                    << "ms, BFS: " << bfs_pure_time << "ms) "
                    << "| Nodes: " << active_count
                    << ", Island: " << island_count << std::endl;
        }

        // --- RRT Reachability Verification Integration ---
        if (state_.rrt_verify_mode_active) {
          static auto last_rrt_test_time = std::chrono::steady_clock::now();
          static int total_trials = 0;
          static int success_10ms = 0;
          static int success_50ms = 0;

          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration<double>(now - last_rrt_test_time).count() > 0.5) {
            last_rrt_test_time = now;

            std::vector<int> island_node_indices;
            state_.gng_ptr->forEachActive([&](int i, const auto &mnode) {
              if (!mnode.status.is_mainland && !mnode.status.is_colliding && mnode.status.valid) {
                island_node_indices.push_back(i);
              }
            });

            if (!island_node_indices.empty()) {
              total_trials++;
              int target_id = island_node_indices[rand() % island_node_indices.size()];
              
              Eigen::VectorXd raw_q_d = state_.robot_hal_ptr->getJointPositions();
              int cur_dof = state_.kinematic_chain_ptr ? state_.kinematic_chain_ptr->getTotalDOF() : (int)raw_q_d.size();
              Eigen::VectorXd current_q = raw_q_d.head(std::min((int)raw_q_d.size(), cur_dof));
              
              std::vector<Eigen::VectorXd> goal_states = {state_.gng_ptr->nodeAt(target_id).weight_angle.cast<double>()};

              // 10ms verification
              planner::RRTParams p10 = state_.global_rrt_params;
              p10.max_planning_time_ms = 10.0;
              auto path10 = state_.rrt_planner_ptr->plan(current_q, goal_states, *state_.validity_checker_ptr, &p10);
              bool s10 = !path10.empty();
              if (s10) success_10ms++;

              // 50ms verification
              planner::RRTParams p50 = state_.global_rrt_params;
              p50.max_planning_time_ms = 50.0;
              auto path50 = state_.rrt_planner_ptr->plan(current_q, goal_states, *state_.validity_checker_ptr, &p50);
              bool s50 = !path50.empty();
              if (s50) success_50ms++;

              std::cerr << "\n[RRT VERIFY] Trial: " << total_trials 
                        << " | Node: " << target_id
                        << " | 10ms: " << (s10 ? "SUCCESS" : "FAILED ") 
                        << " | 50ms: " << (s50 ? "SUCCESS" : "FAILED ") 
                        << "\n[RRT STATS] Success Rate (n/N): "
                        << "10ms=" << std::fixed << std::setprecision(2) << (100.0 * success_10ms / total_trials) << "% (" << success_10ms << "/" << total_trials << "), "
                        << "50ms=" << (100.0 * success_50ms / total_trials) << "% (" << success_50ms << "/" << total_trials << ")" << std::endl;
            } else {
              // Only print when active and island nodes exist to reduce noise
            }
          }
        }

        // --- Lead Time Measurement ---
        if (state_.topology_was_connected && island_count > 0 &&
            mainland_count > 0) {
          // Disconnection detected for the first time
          state_.topology_disconnection_time = sim_time;
          state_.topology_was_connected = false;
          std::cout << "[ANALYSIS] Topology Disconnected at " << sim_time
                    << "s! Monitoring lead time..." << std::endl;
        }

        if (!state_.topology_was_connected && !is_robot_safe &&
            state_.last_lead_time < 0) {
          // Collision occurred after disconnection
          state_.last_lead_time = sim_time - state_.topology_disconnection_time;
          std::cout << "[ANALYSIS] Collision reached. Lead Time since "
                       "disconnection: "
                    << state_.last_lead_time << " seconds." << std::endl;
        }

        // Reset if we become safe and connected again (manual reset or move)
        if (island_count == 0 && is_robot_safe &&
            !state_.topology_was_connected) {
          state_.topology_was_connected = true;
          state_.topology_disconnection_time = -1.0;
          state_.last_lead_time = -1.0;
        }

        // 統計データの記録
        if (reachability_csv_.is_open()) {
          reachability_csv_
              << sim_time << " " << mainland_count << " " << island_count << " "
              << (is_robot_safe ? 1 : 0) << " " << active_count << " "
              << state_.vlut_update_time_ms << " " << state_.last_lead_time
              << "\n";
          if (state_.enable_csv_flush)
            reachability_csv_.flush();
        }

        // --- Robustness Perturbation Sweep ---
        // 本土サイズが減少し、リスクが高いと判断された場合にスイープを実行
        // (ここでは簡易的に 本土ノード < 200 を「リスク」とする)
        if (robustness_csv_.is_open() && island_count > 0 &&
            mainland_count < 300 && mainland_count > 0) {
          std::vector<double> margins = {0.005, 0.010};

          // 0mmの結果をまず記録
          robustness_csv_ << sim_time << " 0.000 " << mainland_count << " "
                          << island_count << " " << (is_robot_safe ? 1 : 0)
                          << " " << active_count << "\n";

          for (double margin : margins) {
            // マージンを考慮した validity checker
            auto is_valid_m = [&](int node_id) {
              const auto &node = state_.gng_ptr->nodeAt(node_id);
              if (!node.status.valid || !node.status.active)
                return false;

              // PointCloud衝突判定にのみマージンを適用する簡易実装
              // 本来は validity_checker->isValid(q, margin) が望ましい
              Eigen::VectorXd q_node = node.weight_angle.cast<double>();
              bool safe = true;
              if (auto *checker = dynamic_cast<OdeStateValidityChecker *>(
                      state_.validity_checker_ptr.get())) {
                safe = checker->isValid(q_node, margin);
              } else {
                safe = state_.validity_checker_ptr->isValid(q_node);
              }
              return safe;
            };

            // ロボット自身の安全性もマージン付きで再判定
            bool robot_safe_m = true;
            if (auto *checker = dynamic_cast<OdeStateValidityChecker *>(
                    state_.validity_checker_ptr.get())) {
              robot_safe_m = checker->isValid(current_q.cast<double>(), margin);
            }

            int robot_node_id_m = -1;
            if (robot_safe_m) {
              robot_node_id_m = state_.gng_planner_ptr->findNearestNode(
                  current_q, *state_.gng_ptr);
            }

            analysis::GraphTopologyAnalyzer temp_analyzer;
            temp_analyzer.analyze(state_.gng_ptr->getMaxNodeNum(), is_valid_m,
                                  get_neighbors, robot_node_id_m);

            int m_count = 0;
            int i_count = 0;
            state_.gng_ptr->forEachActive([&](int i, const auto & /*node*/) {
              if (temp_analyzer.isMainland(i))
                m_count++;
              else if (temp_analyzer.getGroupId(i) != -1)
                i_count++;
            });

            robustness_csv_ << sim_time << " " << margin << " " << m_count
                            << " " << i_count << " " << (robot_safe_m ? 1 : 0)
                            << " " << active_count << "\n";
          }
          if (state_.enable_csv_flush)
            robustness_csv_.flush();
        }
      }
    }

    // --- Influence Manager & Receptor Layer Update ---
    if (state_.enable_influence_tracking && state_.influence_manager_ptr &&
        state_.gng_ptr) {
      Eigen::VectorXf raw_q =
          state_.robot_hal_ptr->getJointPositions().cast<float>();
      int cur_dof = state_.kinematic_chain_ptr ? state_.kinematic_chain_ptr->getTotalDOF() : raw_q.size();
      Eigen::VectorXf current_q = raw_q.head(std::min((int)raw_q.size(), cur_dof));

      // Update robot collision model with current joint positions
      if (state_.robot_collision_model_ptr) {
        state_.robot_collision_model_ptr->updateBodyPoses(
            state_.kinematic_chain_ptr->getLinkPositions(),
            state_.kinematic_chain_ptr->getLinkOrientations());
        state_.robot_collision_model_ptr->updateCollisionStatus();
      }


      state_.influence_manager_ptr->update(current_q, *state_.gng_ptr, 1.0f);
    }

    if (state_.experiment_active && state_.experiment_runner_ptr) {
      state_.experiment_runner_ptr->update(0.01);
      if (state_.experiment_runner_ptr->isCompleted()) {
        auto stats = state_.experiment_runner_ptr->getStats();
        std::cout << "\n========= Experiment Results =========" << std::endl;
        std::cout << "Method: COMPLETED" << std::endl;
        if (state_.current_scenario_ptr) {
          std::cout << "Task: " << state_.current_scenario_ptr->getName()
                    << std::endl;
        }
        std::cout << "SUCCESS: " << (stats.is_success ? "YES" : "NO")
                  << std::endl;
        std::cout << "======================================\n" << std::endl;
        if (state_.logger_ptr)
          state_.logger_ptr->close();
        state_.experiment_active = false;
        state_.clearPathVisualizations();
      }
    }

    // 制御ループ
    if (!state_.is_paused &&
        (state_.auto_mode || state_.reactive_trailing_mode ||
         state_.experiment_active)) {
      if (state_.reactive_trailing_mode && !state_.experiment_active) {
        if (state_.reactive_controller_ptr) {
          state_.reactive_controller_ptr->update(0.01, state_);
          // [修正] 追従モード中もホールド位置を同期しておく（強制解除時にジャンプするのを防ぐ）
          state_.hold_positions = state_.robot_hal_ptr->getJointPositions();
        }
      }

      if (state_.controller_ptr)
        state_.controller_ptr->update(0.01, *state_.robot_hal_ptr);

      // [修正] 実験中や自動モード中もホールド位置を同期しておく（終了時に急激に初期姿勢へ戻って爆散するのを防ぐ）
      state_.hold_positions = state_.robot_hal_ptr->getJointPositions();

      if (state_.auto_mode && state_.controller_ptr &&
          state_.controller_ptr->isFinished()) {
        bool is_static_avoidance =
            (state_.scenario_manager_ptr &&
             state_.scenario_manager_ptr->getCurrentScenario() ==
                 robot_sim::simulation::ScenarioType::AVOIDANCE_STATIC);

        if (!is_static_avoidance) {
          std::cout
              << "[DEBUG App] Goal reached. Setting auto_mode to false.\n";
          state_.auto_mode = false;

          if (state_.global_target_goal_q.size() > 0) {
            state_.hold_positions = state_.global_target_goal_q.cast<double>();
          } else {
            state_.hold_positions = state_.robot_hal_ptr->getJointPositions();
          }
          state_.current_path_node_ids.clear();
        }
      }
    } else {
      // ホールドモード（位置維持）
      std::vector<std::string> control_joint_names =
          state_.robot_hal_ptr->getJointNames();

      if (state_.manual_udp_mode && state_.udp_bridge_ptr) {
        robot_sim::simulation::JointCommand udp_cmd;
        if (state_.udp_bridge_ptr->getLatestCommand(udp_cmd)) {
          // Construct 7D communication vector (6 arm + 1 gripper)
          Eigen::VectorXd comm_q(7);
          for (int i = 0; i < 6; ++i) comm_q[i] = udp_cmd.joint_positions[i];
          comm_q[6] = udp_cmd.gripper_position;

          // Convert 7D Communication -> 8D Physical using adapter
          Eigen::VectorXd phys_q = state_.state_adapter_ptr->fromCommunication(comm_q);

          // Update hold positions
          if (state_.hold_positions.size() == (size_t)phys_q.size()) {
            for (int i = 0; i < phys_q.size(); ++i) {
              state_.hold_positions[i] = phys_q[i];
            }
          }

          static int app_udp_log_cnt = 0;
          if (app_udp_log_cnt++ % 50 == 0) {
            std::cout << "[App] Applying UDP command (7D->8D) to robot." << std::endl;
          }
        }
      }

      size_t min_size =
          std::min(control_joint_names.size(),
                   static_cast<size_t>(state_.hold_positions.size()));
      for (size_t i = 0; i < min_size; ++i) {
        state_.robot_hal_ptr->setJointCommand(control_joint_names[i],
                                              state_.hold_positions[i],
                                              ::control::ControlMode::POSITION);
      }
    }

    // 物理演算ステップ
    if (state_.geometry_only_mode) {
      if (state_.robot_hal_ptr) {
        // Visual-Only Implementation: Inject poses directly from KinematicChain
        if (state_.kinematic_chain_ptr && state_.robot_model_ptr) {
          Eigen::VectorXd angles = state_.robot_hal_ptr->getJointPositions();

          std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>>
              positions;
          std::vector<Eigen::Quaterniond,
                      Eigen::aligned_allocator<Eigen::Quaterniond>>
              orientations;
          state_.kinematic_chain_ptr->forwardKinematicsAt(angles, positions,
                                                          orientations);

          std::map<std::string, Eigen::Isometry3d> link_transforms;
          state_.kinematic_chain_ptr->buildAllLinkTransforms(
              positions, orientations,
              state_.robot_model_ptr->getFixedLinkInfo(), link_transforms);

          auto *ode_hal = dynamic_cast<::simulation::OdeRobotSim *>(
              state_.robot_hal_ptr.get());
          if (ode_hal) {
            for (const auto &pair : link_transforms) {
              ode_hal->setLinkPose(pair.first, pair.second.translation(),
                                   Eigen::Quaterniond(pair.second.rotation()));
            }
          }
        }
        state_.robot_hal_ptr->update(0.01);
      }
    } else {
      // 外部定義したラッパーコールバックを使用
      dSpaceCollide(state_.space, 0, &globalNearCallback);
      dWorldStep(state_.world, 0.01);
      dJointGroupEmpty(state_.contactgroup);

      if (state_.robot_hal_ptr) {
        state_.robot_hal_ptr->update(0.01);
      }
    }
    /*
    auto frame_end_time = std::chrono::high_resolution_clock::now();
    double frame_time_ms = std::chrono::duration<double, std::milli>(
                               frame_end_time - frame_start_time)
                               .count();
    */

    static int frame_count = 0;
    if (frame_count++ % 10 == 0) {
      // std::cout << "[DEBUG Profiler] Simulation Update Time: " <<
      // frame_time_ms
      //           << " ms" << std::endl;
    }
  }

  // FKチェーンの更新とEEFログ
  syncKinematics();

  // last_env_time_ = env_update_time_ms_;
  // //これらの変数が定義されていないエラーもあるため、一旦コメントアウトまたは削除が安全だが、メンバ変数ならOK。
  // しかし view_file では宣言が見えない。エラーログには 'use of undeclared
  // identifier' とあるので、これも削除対象。
}

void SimulationApp::draw() {
  if (state_.ground) {
    drawGeom(state_.ground, {0.5, 0.5, 0.5, 0.5}, true);

    // 白いタイル (2mm厚) を貼る
    dReal tile_pos[3] = {0.0, 0.0, 0.0011}; // 少しだけ浮かせる(Z=0の床に対して)
    dMatrix3 tile_R = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    dVector3 tile_sides = {10.0, 10.0, 0.0022}; // 2mm厚
    dsSetColorAlpha(0.95f, 0.95f, 0.95f, 1.0f);
    dsSetTexture(DS_NONE);
    dsDrawBox(tile_pos, tile_R, tile_sides);
  }

  // GNG可視化
  if (state_.gng_ptr && state_.gng_viz_ptr) {
    std::vector<simulation::VisualNode> vnodes;
    std::vector<simulation::VisualEdge> vedges;

    std::unordered_set<int> influenced_ids;
    if (state_.influence_manager_ptr) {
      for (const auto &in : state_.influence_manager_ptr->getActiveNodes()) {
        influenced_ids.insert(in.node_id);
      }
    }
    std::unordered_set<int> path_ids(state_.current_path_node_ids.begin(),
                                     state_.current_path_node_ids.end());

    ::simulation::GngDataAdapter::convert(
        *state_.gng_ptr, vnodes, vedges, influenced_ids, path_ids,
        Eigen::Vector3d(0, 0, 0), state_.show_coord_graph);

    // ImGui controls removed (handled in GuiManager)

    // フィルタリング処理 (Node Display Mode Filtering)
    std::vector<simulation::VisualNode> filtered_nodes;
    if (state_.show_only_target && state_.target_node_id != -1) {
      for (const auto &vn : vnodes) {
        if (vn.id == state_.target_node_id) {
          filtered_nodes.push_back(vn);
          break;
        }
      }
    } else {
      for (const auto &vn_src : vnodes) {
        bool include = false;
        switch (state_.node_display_mode) {
        case robot_sim::simulation::NodeDisplayMode::ALL:
          include = true;
          break;
        case robot_sim::simulation::NodeDisplayMode::ACTIVE_ONLY:
          include = vn_src.active;
          break;
        case robot_sim::simulation::NodeDisplayMode::INACTIVE_ONLY:
          include = !vn_src.active;
          break;
        case robot_sim::simulation::NodeDisplayMode::COLLISION_ONLY:
          include = vn_src.is_collision;
          break;
        case robot_sim::simulation::NodeDisplayMode::DANGER_ONLY:
          include = vn_src.is_hazard; // is_hazard matches node.status.is_danger
          break;
        case robot_sim::simulation::NodeDisplayMode::HAZARD_ONLY:
          // Hazard mode shows both actual collisions and proximity-based danger
          include = vn_src.is_hazard || vn_src.is_collision;
          break;
        case robot_sim::simulation::NodeDisplayMode::INFLUENCE:
          include = vn_src.is_influence;
          break;
        case robot_sim::simulation::NodeDisplayMode::PATH_ONLY:
          include = vn_src.is_path;
          break;
        case robot_sim::simulation::NodeDisplayMode::TOPOLOGY:
          include = true; // Show all nodes for topology analysis
          break;
        case robot_sim::simulation::NodeDisplayMode::ISLAND_ONLY: {
          const auto &node = state_.gng_ptr->nodeAt(vn_src.id);
          int mainland_id = state_.topology_analyzer_ptr->getMainlandId();
          // An island is a valid, non-colliding, non-danger component that is NOT the mainland.
          include = (mainland_id != -1) && node.status.valid &&
                    !node.status.is_colliding && !node.status.is_danger &&
                    (node.status.topology_group_id != -1) &&
                    !node.status.is_mainland;
          break;
        }
        case robot_sim::simulation::NodeDisplayMode::OFF:
          include = false;
          break;
        default:
          include = true;
          break;
        }

        if (include && state_.show_surface_only && !vn_src.is_surface) {
          include = false;
        }

        if (include) {
          filtered_nodes.push_back(vn_src);
        }
      }
      vnodes = filtered_nodes;

      // Filter edges: Only keep edges where both nodes are in the filtered set
      std::unordered_set<int> filtered_ids;
      for (const auto &vn : vnodes)
        filtered_ids.insert(vn.id);

      std::vector<simulation::VisualEdge> filtered_edges;
      for (const auto &ve : vedges) {
        if (filtered_ids.count(ve.node1_id) &&
            filtered_ids.count(ve.node2_id)) {
          filtered_edges.push_back(ve);
        }
      }
      vedges = filtered_edges;
    }

    // --- Unified Coloring Pass (Priorities & Trends) ---
    for (auto &vn : vnodes) {
      const auto &node = state_.gng_ptr->nodeAt(vn.id);
      robot_sim::visualization::DynamicColorParams params;
      params.is_topology_mode =
          (state_.node_display_mode ==
               robot_sim::simulation::NodeDisplayMode::TOPOLOGY ||
           state_.node_display_mode ==
               robot_sim::simulation::NodeDisplayMode::ISLAND_ONLY);

      // Basic Attributes
      if (vn.active)
        params.attributes |= robot_sim::visualization::NodeAttribute::ACTIVE;
      else
        params.attributes |= robot_sim::visualization::NodeAttribute::INACTIVE;
      if (vn.is_surface)
        params.attributes |= robot_sim::visualization::NodeAttribute::SURFACE;

      // Topology attributes mapping (Only apply in topology-related modes)
      if (params.is_topology_mode && node.status.valid &&
          !node.status.is_colliding) {
        // We reuse the is_robot_safe calculated earlier if possible, but
        // for thread safety/clarity in visualization pass we can check nodes.
        // Actually, we already updated node.status.is_mainland.
        int mainland_id = state_.topology_analyzer_ptr->getMainlandId();
        if (node.status.is_mainland) {
          params.attributes |=
              robot_sim::visualization::NodeAttribute::MAINLAND;
        } else if (mainland_id != -1 && node.status.topology_group_id != -1) {
          // If a node belongs to a component but isn't Mainland, it's an Island
          // (Only valid if a Mainland actually exists)
          params.attributes |= robot_sim::visualization::NodeAttribute::ISLAND;
        }
      }

      // Dynamic Parameters from Safety State
      if (state_.safety_state_manager_ptr) {
        const auto &levels = state_.safety_state_manager_ptr->getDangerLevels();
        const auto &vels =
            state_.safety_state_manager_ptr->getDangerVelocities();
        params.danger_level =
            (vn.id < (int)levels.size()) ? levels[vn.id] : 0.0f;
        params.danger_velocity =
            (vn.id < (int)vels.size()) ? vels[vn.id] : 0.0f;
      }

      params.show_trends = state_.show_danger_trends;
      params.is_colliding = node.status.is_colliding;
      params.is_target = (vn.id == state_.target_node_id);
      params.is_danger_status = node.status.is_danger;
      if (params.is_danger_status) {
        params.attributes |= robot_sim::visualization::NodeAttribute::DANGER;
      }

      // Get Color from Scheme
      auto color = state_.node_color_scheme.getColor(params);
      vn.color = {color.r, color.g, color.b, color.a};

      // Update Active State (Force active if significant events occur)
      bool force_active =
          params.is_colliding || params.is_target || params.is_danger_status;
      if (params.show_trends && params.danger_level > 0.05f)
        force_active = true;
      if (params.danger_level > 0.1f)
        force_active = true;

      if (force_active)
        vn.active = true;
    }

    if (!state_.show_edges && !state_.show_coord_graph)
      vedges.clear();

    // Update Visualizer State
    state_.gng_viz_ptr->setHighlightedPath(state_.current_path_node_ids);

    ::simulation::IGngVisualizer::FilterSettings filter;
    if (state_.node_display_mode ==
        robot_sim::simulation::NodeDisplayMode::PATH_ONLY) {
      filter.show_only_path = true;
    }
    state_.gng_viz_ptr->setFilter(filter);

    state_.gng_viz_ptr->update(vnodes, vedges);
    Eigen::Vector3d offset = state_.show_dual_view ? state_.dual_view_offset
                                                   : Eigen::Vector3d::Zero();
    state_.gng_viz_ptr->draw(offset);
  }

  if (state_.experiment_active && state_.current_scenario_ptr) {
    state_.target_eef_pos = state_.current_scenario_ptr->getCurrentTargetPos();
    state_.target_sphere_pos = state_.target_eef_pos;
  } else if (state_.scenario_manager_ptr &&
             state_.scenario_manager_ptr->getCurrentScenario() ==
                 robot_sim::simulation::ScenarioType::TRAJECTORY) {
    state_.target_eef_pos = state_.scenario_manager_ptr->getTargetPosition();
    state_.target_sphere_pos = state_.target_eef_pos;
  }

  // 障害物描画
  if (state_.enable_dynamic_obstacle) {
    bool is_sim = !state_.scenario_manager_ptr ||
                  state_.scenario_manager_ptr->getOperationMode() ==
                      robot_sim::simulation::OperationMode::SIMULATION;
    if (is_sim) {
      dsSetColor(0.0f, 1.0f,
                 1.0f); // Cyan (Reverted to match user's existing screenshots)
      const dReal pos[3] = {(dReal)state_.demo_obstacle_pos.x(),
                            (dReal)state_.demo_obstacle_pos.y(),
                            (dReal)state_.demo_obstacle_pos.z()};
      const dReal R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
      dsDrawSphere(pos, R, (dReal)state_.demo_obstacle_radius);
    }

    if (state_.show_obstacle_voxels && state_.obstacle_manager_ptr) {
      // Use the unified set for visualization
      std::vector<long> unified_voxels;
      for (long vid : state_.obstacle_manager_ptr->getUnifiedOccupiedVoxels()) {
        unified_voxels.push_back(vid);
      }
      state_.viz_manager_ptr->drawObstacleVoxels(
          unified_voxels, (float)state_.obstacle_manager_ptr->getVoxelSize());
    }
  }

  // 点群障害物
  if (state_.env_manager_ptr) {
    // Revert structure color to Red (default) or make it distinct
    // The user said "not the color of the steel frame", implying they want the
    // POINTS to be magenta. Let's make the structure Red again (default) or
    // maybe Grey.
    float pc_robot_structure_color[3] = {1.0f, 0.2f, 0.2f}; // Red-ish
    state_.env_manager_ptr->draw(state_.show_point_cloud_structure,
                                 pc_robot_structure_color);

    if (state_.show_danger_points) {
      const auto &points = state_.env_manager_ptr->getCombinedPointCloud();
      // Magenta for Point Cloud
      dsSetColorAlpha(1.0f, 0.0f, 1.0f, 1.0f);
      for (const auto &p : points) {
        dReal pos[3] = {(dReal)p.x(), (dReal)p.y(), (dReal)p.z()};
        dReal R[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
        dsDrawSphere(pos, R, 0.015f);
      }
    }
  }

  // 危険場ボクセル
  if (state_.viz_manager_ptr) {
    state_.viz_manager_ptr->drawDangerField(state_);
  }

  // ターゲット球
  if (state_.show_target_sphere && state_.target_sphere_geom &&
      state_.target_sphere_body) {
    dBodySetPosition(state_.target_sphere_body, state_.target_sphere_pos.x(),
                     state_.target_sphere_pos.y(),
                     state_.target_sphere_pos.z());

    state_.viz_manager_ptr->drawTargetSphere(state_.target_sphere_pos,
                                             state_.target_sphere_quat, 0.025,
                                             state_.show_target_axes);
  }

  // 候補パスの可視化
  if (state_.show_candidate_paths && !state_.candidate_paths_viz.empty() &&
      state_.viz_manager_ptr) {
    using namespace robot_sim::visualization;

    // シミュレーションの状態を壊さないように現在の関節値を保存
    std::vector<double> saved_joints =
        state_.kinematic_chain_ptr->getJointValues();

    Eigen::Vector3d offset = state_.show_dual_view ? state_.dual_view_offset
                                                   : Eigen::Vector3d::Zero();

    for (const auto &candidate : state_.candidate_paths_viz) {
      // Use the color set by the planner (e.g. Green for GNG/RRT)
      float path_color[4] = {candidate.color.x(), candidate.color.y(),
                             candidate.color.z(), 0.8f};

      // デバッグ描画
      dsSetColorAlpha(path_color[0], path_color[1], path_color[2],
                      path_color[3]);

      std::vector<Eigen::Vector3d> path_points;
      path_points.reserve(candidate.path.size());

      for (const auto &q_f : candidate.path) {
        int path_dof = state_.kinematic_chain_ptr->getTotalDOF();
        state_.kinematic_chain_ptr->updateKinematics(
            q_f.head(std::min((int)q_f.size(), path_dof)));
        path_points.push_back(state_.kinematic_chain_ptr->getEEFPosition() +
                              offset);
      }

      // 線の描画
      for (size_t i = 0; i + 1 < path_points.size(); ++i) {
        dsDrawLine(path_points[i].data(), path_points[i + 1].data());
      }
      // ノードの描画 (オプション)
      for (const auto &p : path_points) {
        const dReal R_identity[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
        dsDrawSphere(p.data(), R_identity, 0.01f);
      }
    }

    // 元の姿勢を復元
    syncKinematics();
  }

  // ロボット描画
  if (state_.use_simplified_visual && state_.robot_model_ptr) {
    dsSetTexture(DS_NONE);

    int link_index = 0;
    for (const auto &[name, comp] : state_.robot_components_map) {
      dReal4 current_color;
      if (link_index % 2 == 0) {
        current_color[0] = 0.2f;
        current_color[1] = 0.2f;
        current_color[2] = 0.2f;
        current_color[3] = 1.0f; // Black-ish
      } else {
        current_color[0] = 0.6f;
        current_color[1] = 0.4f;
        current_color[2] = 0.2f;
        current_color[3] = 1.0f; // Brown
      }
      link_index++;

      dBodyID b = comp.body_id;
      if (!b)
        continue;

      const dReal *p = dBodyGetPosition(b);
      const dReal *R = dBodyGetRotation(b);

      // Link state check: Skip drawing if body state is invalid (NaN/Inf)
      // to prevent OpenGL driver hangs on macOS
      if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
          !std::isfinite(p[2])) {
        continue;
      }

      Eigen::Isometry3d link_transform = Eigen::Isometry3d::Identity();
      link_transform.translation() << p[0], p[1], p[2];
      link_transform.linear() << R[0], R[1], R[2], R[4], R[5], R[6], R[8], R[9],
          R[10];

      const auto *link_props = state_.robot_model_ptr->getLink(name);
      bool simplified_drawn = false;
      if (link_props) {
        for (const auto &visual : link_props->visuals) {
          if (visual.has_simplified) {
            Eigen::Isometry3d v_transform =
                link_transform * visual.simplified_offset_transform;

            dVector3 v_pos;
            dMatrix3 v_R;
            v_pos[0] = (dReal)v_transform.translation().x();
            v_pos[1] = (dReal)v_transform.translation().y();
            v_pos[2] = (dReal)v_transform.translation().z();

            bool v_finite = true;
            for (int i = 0; i < 3; ++i) {
              if (!std::isfinite(v_pos[i]))
                v_finite = false;
              for (int j = 0; j < 3; ++j) {
                v_R[i * 4 + j] = (dReal)v_transform.linear()(i, j);
                if (!std::isfinite(v_R[i * 4 + j]))
                  v_finite = false;
              }
              v_R[i * 4 + 3] = 0;
            }

            dVector3 sides = {(dReal)visual.simplified_geometry.size.x(),
                              (dReal)visual.simplified_geometry.size.y(),
                              (dReal)visual.simplified_geometry.size.z()};
            if (!std::isfinite(sides[0]) || !std::isfinite(sides[1]) ||
                !std::isfinite(sides[2]))
              v_finite = false;

            // Size safety check & NaN safety
            if (v_finite && sides[0] > 1e-6 && sides[1] > 1e-6 &&
                sides[2] > 1e-6) {
              dsSetColorAlpha(current_color[0], current_color[1],
                              current_color[2], 1.0f);
              dsDrawBox(v_pos, v_R, sides);
              simplified_drawn = true;
            }
          }
        }
      }

      // Draw non-mesh collision geoms (or fallback if no simplified visuals
      // were drawn for this link)
      for (dGeomID g : comp.geom_ids) {
        if (g) {
          int g_class = dGeomGetClass(g);
          auto *gData = static_cast<::simulation::GeomData *>(dGeomGetData(g));
          bool is_mesh_replacement = (gData && gData->visual_mesh_id);

          // Skip if we already drew a simplified visual for this mesh,
          // or if it's the original TriMesh class which we handle separately
          if (!simplified_drawn || (!is_mesh_replacement && g_class != dTriMeshClass)) {
            drawGeom(g, current_color);
          }
        }
      }
    }
  } else {
    int link_index = 0;
    for (const auto &[name, comp] : state_.robot_components_map) {
      dReal4 current_color;
      if (link_index % 2 == 0) {
        current_color[0] = 0.2f;
        current_color[1] = 0.2f;
        current_color[2] = 0.2f;
        current_color[3] = 1.0f; // Black
      } else {
        current_color[0] = 0.6f;
        current_color[1] = 0.4f;
        current_color[2] = 0.2f;
        current_color[3] = 1.0f; // Brown
      }
      link_index++;

      for (dGeomID g : comp.geom_ids) {
        if (g) {
          drawGeom(g, current_color);
        }
      }
    }
  }

  // マニピュラリティ可視化
  drawManipulabilityViz();

  if (state_.reactive_controller_ptr) {
    state_.reactive_controller_ptr->draw(state_);
  }

  if (state_.viz_manager_ptr) {
    state_.viz_manager_ptr->drawInfluenceNodes(state_);
  }
}

void SimulationApp::drawGeom(dGeomID g, const dReal4 &color, bool use_texture) {
  if (!g)
    return;

  // Check for visual override in GeomData
  auto *gData = static_cast<::simulation::GeomData *>(dGeomGetData(g));
  if (gData && gData->visual_mesh_id && state_.mesh_cache_ptr) {
    auto entry = state_.mesh_cache_ptr->getMeshEntry(gData->visual_mesh_id);
    if (entry && !entry->flattened_vertices.empty()) {
      const dReal *pos = dGeomGetPosition(g);
      const dReal *R = dGeomGetRotation(g);

      // Adjust drawing position based on mesh center (inverse of the simplification offset)
      Eigen::Vector3d geom_pos(pos[0], pos[1], pos[2]);
      Eigen::Matrix3d geom_rot;
      geom_rot << R[0], R[1], R[2],
                  R[4], R[5], R[6],
                  R[8], R[9], R[10];
      
      Eigen::Vector3d visual_pos = geom_pos - geom_rot * gData->visual_mesh_center;
      dVector3 final_pos = { (dReal)visual_pos.x(), (dReal)visual_pos.y(), (dReal)visual_pos.z() };

      dsSetColorAlpha(color[0], color[1], color[2], color[3]);
      if (use_texture)
        dsSetTexture(DS_WOOD);
      else
        dsSetTexture(DS_NONE);

      for (size_t i = 0; i < entry->flattened_vertices.size(); i += 9) {
        const dReal *v0 = &entry->flattened_vertices[i];
        const dReal *v1 = &entry->flattened_vertices[i + 3];
        const dReal *v2 = &entry->flattened_vertices[i + 6];
#ifdef dDOUBLE
        dsDrawTriangleD(final_pos, R, v0, v1, v2, 1);
#else
        dsDrawTriangle(final_pos, R, v0, v1, v2, 1);
#endif
      }
      return; // Mesh drawn, skip default geom drawing
    }
  }

  int type = dGeomGetClass(g);
  if (type == dPlaneClass || type == dRayClass)
    return; // Skip non-placeable geoms

  const dReal *pos = dGeomGetPosition(g);
  const dReal *R = dGeomGetRotation(g);

  dsSetColorAlpha(color[0], color[1], color[2], color[3]);
  if (use_texture)
    dsSetTexture(DS_WOOD);
  else
    dsSetTexture(DS_NONE);

  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths(g, sides);
    dsDrawBox(pos, R, sides);
  } else if (type == dSphereClass) {
    dsDrawSphere(pos, R, dGeomSphereGetRadius(g));
  } else if (type == dCylinderClass) {
    dReal radius, length;
    dGeomCylinderGetParams(g, &radius, &length);
    dsDrawCylinder(pos, R, length, radius);
  } else if (type == dCapsuleClass) {
    dReal radius, length;
    dGeomCapsuleGetParams(g, &radius, &length);
    dsDrawCapsule(pos, R, length, radius);
  } else if (type == dTriMeshClass) {
    if (state_.mesh_cache_ptr) {
      dTriMeshDataID data_id = dGeomTriMeshGetData(g);
      auto entry = state_.mesh_cache_ptr->getMeshEntry(data_id);
      if (entry && !entry->flattened_vertices.empty()) {
        for (size_t i = 0; i < entry->flattened_vertices.size(); i += 9) {
          const dReal *v0 = &entry->flattened_vertices[i];
          const dReal *v1 = &entry->flattened_vertices[i + 3];
          const dReal *v2 = &entry->flattened_vertices[i + 6];
#ifdef dDOUBLE
          dsDrawTriangleD(pos, R, v0, v1, v2, 1);
#else
          dsDrawTriangle(pos, R, v0, v1, v2, 1);
#endif
        }
      }
    }
  }
}

void SimulationApp::updateCamera() {
  ImGuiIO &io = ImGui::GetIO();
  static float last_x = 0, last_y = 0;
  if (io.WantCaptureMouse) {
    last_x = io.MousePos.x;
    last_y = io.MousePos.y;
    return;
  }

  float dx = io.MousePos.x - last_x;
  float dy = io.MousePos.y - last_y;
  last_x = io.MousePos.x;
  last_y = io.MousePos.y;

  if (io.MouseDown[0] || io.MouseDown[1] || io.MouseDown[2]) {
    float xyz[3], hpr[3];
    dsGetViewpoint(xyz, hpr);

    if (io.MouseDown[0]) { // Rotate
      hpr[0] += dx * 0.2f;
      hpr[1] -= dy * 0.2f;
    } else if (io.MouseDown[1]) { // Zoom
      float rad = hpr[0] * M_PI / 180.0f;
      float tilt = hpr[1] * M_PI / 180.0f;
      xyz[0] += cos(rad) * cos(tilt) * dy * 0.01f;
      xyz[1] += sin(rad) * cos(tilt) * dy * 0.01f;
      xyz[2] += sin(tilt) * dy * 0.01f;
    } else if (io.MouseDown[2]) { // Pan
      float rad = (hpr[0] + 90.0f) * M_PI / 180.0f;
      xyz[0] += cos(rad) * dx * 0.005f;
      xyz[1] += sin(rad) * dx * 0.005f;
      xyz[2] -= dy * 0.005f;
    }
    dsSetViewpoint(xyz, hpr);
  }
}

void SimulationApp::renderGui() {
  if (!state_.gui_show_panel)
    return;

  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();
  ImGui::NewFrame();

  if (state_.gui_manager_ptr) {
    state_.gui_manager_ptr->render(state_);
  }

  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

void SimulationApp::handlePickRandomPosture() {
  if (!state_.robot_hal_ptr || !state_.robot_model_ptr ||
      !state_.kinematic_chain_ptr)
    return;

  Eigen::VectorXf random_q(state_.robot_hal_ptr->getJointPositions().size());
  for (int i = 0; i < random_q.size(); ++i) {
    std::string joint_name = state_.robot_hal_ptr->getJointNames()[i];
    const simulation::JointProperties *joint_props =
        state_.robot_model_ptr->getJoint(joint_name);
    if (joint_props && joint_props->has_limits) {
      random_q[i] = joint_props->limits.lower +
                    (static_cast<float>(rand()) / static_cast<double>(RAND_MAX)) *
                        (joint_props->limits.upper - joint_props->limits.lower);
    } else {
      random_q[i] =
          (static_cast<float>(rand()) / static_cast<double>(RAND_MAX)) * 2.0f * M_PI - M_PI;
    }
  }
  state_.global_target_goal_q = random_q;
  int rand_dof = state_.kinematic_chain_ptr->getTotalDOF();
  state_.kinematic_chain_ptr->updateKinematics(random_q.cast<double>().head(
      std::min((int)random_q.size(), rand_dof)));
  Eigen::Vector3f pos =
      state_.kinematic_chain_ptr->getEEFPosition().cast<float>();
  state_.target_eef_pos = pos.cast<double>();
  state_.target_sphere_pos = pos.cast<double>();
  state_.target_sphere_quat = state_.kinematic_chain_ptr->getEEFOrientation();
  state_.show_target_axes = true;

  // Sync with ScenarioManager and stop any movement
  if (state_.scenario_manager_ptr) {
    state_.scenario_manager_ptr->setTargetPosition(state_.target_sphere_pos);
    state_.scenario_manager_ptr->setScenario(
        robot_sim::simulation::ScenarioType::IDLE);
  }

  state_.show_target_sphere = true; // Show the target sphere
  std::cout << "[App] Target Set (Random Posture): " << pos.transpose() << "\n";
}

void SimulationApp::handlePickRandomGngNode() {
  if (!state_.gng_ptr || !state_.kinematic_chain_ptr)
    return;

  int nid = -1;
  std::vector<int> active_indices;
  state_.gng_ptr->forEachActive(
      [&](int i, const auto & /*node*/) { active_indices.push_back(i); });
  if (!active_indices.empty()) {
    nid = active_indices[rand() % active_indices.size()];
  }

  if (nid == -1)
    return;

  const auto &node = state_.gng_ptr->nodeAt(nid);
  Eigen::VectorXf node_q = node.weight_angle;

  state_.global_target_goal_q = node_q;

  if (node_q.size() > 0) {
    int node_dof = state_.kinematic_chain_ptr->getTotalDOF();
    state_.kinematic_chain_ptr->updateKinematics(node_q.cast<double>().head(
        std::min((int)node_q.size(), node_dof)));
    Eigen::Vector3f pos =
        state_.kinematic_chain_ptr->getEEFPosition().cast<float>();
    state_.target_eef_pos = pos.cast<double>();
    state_.target_sphere_pos = pos.cast<double>();
    state_.target_sphere_quat = state_.kinematic_chain_ptr->getEEFOrientation();
    state_.show_target_axes = true;
  } else {
    state_.target_sphere_pos = node.weight_coord.cast<double>();
    state_.target_eef_pos = state_.target_sphere_pos;
    state_.show_target_axes = false;
  }

  state_.show_target_sphere = true; // Show the target sphere

  if (state_.scenario_manager_ptr) {
    state_.scenario_manager_ptr->setTargetPosition(state_.target_sphere_pos);
    state_.scenario_manager_ptr->setScenario(
        robot_sim::simulation::ScenarioType::IDLE);
  }

  std::cout << "[App] Target Set (GNG Node " << node.id
            << "): " << state_.target_sphere_pos.transpose() << "\n";
}

void SimulationApp::planAndExecutePath(const Eigen::VectorXf &start,
                                       const Eigen::VectorXf &goal) {
  if (!state_.planning_coordinator_ptr) {
    std::cout << "[App] Planning coordinator not initialized." << std::endl;
    return;
  }

  state_.planning_coordinator_ptr->planAndExecute(start, goal, state_);
}

void SimulationApp::startExperiment(const std::string &method) {
  if (!state_.robot_hal_ptr || !state_.controller_ptr ||
      !state_.kinematic_chain_ptr) {
    std::cerr << "[App] Error: Components not ready for experiment."
              << std::endl;
    return;
  }

  if (state_.experiment_active) {
    std::cout << "[App] Experiment is already active. Stop first.\n";
    return;
  }

  // Determine Scenario Folder
  std::string scenario_path = ::robot_sim::common::resolvePath(
      "experiment_settings/progressive_challenge.json");
  std::ifstream f(scenario_path);
  if (f.good()) {
    if (state_.scenario_manager_ptr) {
      state_.scenario_manager_ptr->loadScenario(scenario_path);
      state_.current_scenario_ptr =
          std::make_shared<robot_sim::experiment::AdvancedTargetScenario>(
              state_.scenario_manager_ptr->getScenarioDefinition(),
              state_.scenario_manager_ptr.get());
      std::cout << "[App] Loaded Advanced JSON Scenario: " << scenario_path
                << std::endl;
      state_.scenario_manager_ptr->startScenario();
    }
  }

  // Fallback to simple scenario if not loaded
  if (!state_.current_scenario_ptr) {
    state_.current_scenario_ptr =
        std::make_shared<robot_sim::experiment::TargetTouchScenario>();
  }

  state_.experiment_runner_ptr =
      std::make_unique<robot_sim::experiment::ExperimentRunner>(
          *state_.robot_hal_ptr, *state_.controller_ptr,
          state_.kinematic_chain_ptr, state_.state_adapter_ptr.get(),
          static_cast<double>(state_.speed_scale));
  state_.experiment_runner_ptr->setScenario(state_.current_scenario_ptr);

  // Initialize Experiment Logging (New Run Folder)
  int dof = state_.robot_hal_ptr->getJointNames().size();
  if (state_.logger_ptr) {
    state_.logger_ptr->initialize(dof, "../experiment_logs");
    std::cout << "[App] Logger Initialized." << std::endl;
  }

  if (!state_.validity_checker_ptr) {
    if (!state_.robot_collision_model_ptr) {
      state_.robot_collision_model_ptr =
          std::make_unique<::simulation::OdeRobotCollisionModel>(
              *state_.robot_model_ptr, state_.sc_world, state_.sc_space,
              state_.sc_manager_ptr.get(), *state_.kinematic_chain_ptr,
              state_.mesh_cache_ptr.get(), state_.use_mesh_collision);
    }
    state_.validity_checker_ptr =
        std::make_unique<robot_sim::simulation::OdeStateValidityChecker>(
            state_.robot_collision_model_ptr.get(),
            state_.pc_collision_checker_ptr.get(),
            state_.point_cloud_grid_ptr.get(), state_.env_manager_ptr.get(),
            state_.kinematic_chain_ptr);
  }
  state_.experiment_runner_ptr->setValidityChecker(
      state_.validity_checker_ptr.get());

  auto gng_planner_shared = std::shared_ptr<planning::GngDijkstraPlanner<
      Eigen::VectorXf, Eigen::Vector3f,
      GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f>>>(
      state_.gng_planner_ptr.get(), [](void *) {});

  if (gng_planner_shared) {
    gng_planner_shared->setAvoidCollisions(true);
  }

  if (method == "GNG") {
    state_.experiment_runner_ptr->setPlanner(
        std::make_shared<robot_sim::experiment::GNGPlannerWrapper>(
            gng_planner_shared, *state_.gng_ptr, *state_.kinematic_chain_ptr,
            state_.state_adapter_ptr.get(), state_,
            state_.validity_checker_ptr.get()));
    std::cout << "[App] Started GNG Challenge" << std::endl;
  } else if (method == "RRT") {
    state_.experiment_runner_ptr->setPlanner(
        std::make_shared<robot_sim::experiment::RRTPlannerWrapper>(
            *state_.fk_chain_ptr, state_.validity_checker_ptr.get(),
            state_.state_adapter_ptr.get(), state_,
            state_.global_rrt_params));
    std::cout << "[App] Started RRT Challenge" << std::endl;
  }

  state_.reactive_trailing_mode = false;
  state_.auto_mode = false;
  state_.experiment_active = true;
  state_.show_target_sphere = true;
}

void SimulationApp::resetExperiment() {
  state_.experiment_active = false;
  if (state_.controller_ptr)
    state_.controller_ptr->stop();
  state_.experiment_runner_ptr.reset();
  state_.current_scenario_ptr.reset();
  if (state_.logger_ptr)
    state_.logger_ptr->close();
  std::cout << "[App] Experiment RESET." << std::endl;
  state_.clearPathVisualizations();
}

void SimulationApp::drawManipulabilityViz() {
  // Stub implementation
}

void SimulationApp::generateSimplifiedRobotVisuals() {
  if (!state_.robot_model_ptr || !state_.mesh_cache_ptr)
    return;

  std::cout << "[App] Generating simplified visuals for robot meshes...\n";

  for (auto &link_pair : const_cast<std::map<std::string, LinkProperties> &>(
           state_.robot_model_ptr->getLinks())) {
    for (auto &visual : link_pair.second.visuals) {
      if (visual.geometry.type == GeometryType::MESH) {
        dTriMeshDataID data_id = state_.mesh_cache_ptr->getMesh(
            visual.geometry.mesh_filename, visual.geometry.size);
        if (data_id) {
          const auto &vertices =
              state_.mesh_cache_ptr->getOriginalVertices(data_id);
          if (!vertices.empty()) {
            robot_sim::simulation::MeshData temp_mesh;
            temp_mesh.vertices = vertices;

            Eigen::Vector3d min_c, max_c;
            robot_sim::simulation::GeometrySimplifier::calculateAABB(
                temp_mesh, min_c, max_c);

            visual.simplified_geometry.type = ::simulation::GeometryType::BOX;

            visual.simplified_geometry.size =
                GeometrySimplifier::getSize(min_c, max_c);
            visual.simplified_offset =
                GeometrySimplifier::getCenter(min_c, max_c);

            // Pre-calculate the combined visual offset transform
            visual.simplified_offset_transform =
                visual.origin * Eigen::Translation3d(visual.simplified_offset);

            visual.has_simplified = true;
          }
        }
      }
    }
  }
  std::cout << "[App] Simplified visuals generated.\n";
}
void SimulationApp::syncKinematics() {
  if (state_.kinematic_chain_ptr && state_.robot_hal_ptr) {
    int cur_dof = state_.kinematic_chain_ptr->getTotalDOF();
    state_.kinematic_chain_ptr->updateKinematics(
        state_.robot_hal_ptr->getJointPositions().head(
            std::min((int)state_.robot_hal_ptr->getJointPositions().size(),
                     cur_dof)));
  }
}

} // namespace simulation
} // namespace robot_sim
