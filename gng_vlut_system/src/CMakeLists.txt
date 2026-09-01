# --- 1. Common Setup ---
include_directories(
    core
    nodes
    offline_tools
    core/safety_engine # Base for internal includes
)

# Standard ROS 2 Dependencies
set(DEPS_ROS rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs tf2_eigen visualization_msgs voxel_msgs ament_index_cpp nlohmann_json)
# System libraries
set(EXTRA_LIBS m OpenMP::OpenMP_CXX)

# --- 2. Core Libraries (STATIC) ---

# Common
add_library(common_lib STATIC core/common/geometry_utils.cpp)
ament_target_dependencies(common_lib Eigen3 ament_index_cpp)

# Kinematics
add_library(kinematics_lib STATIC core/kinematics/kinematic_chain.cpp)
target_link_libraries(kinematics_lib common_lib)
ament_target_dependencies(kinematics_lib Eigen3 urdf)

# Robot Model (Old: description)
add_library(robot_model_lib STATIC 
    core/robot_model/urdf_loader.cpp
    core/robot_model/stl_loader.cpp
    core/robot_model/geometry_management.cpp
    core/robot_model/kinematic_adapter.cpp
)
target_link_libraries(robot_model_lib common_lib kinematics_lib)
ament_target_dependencies(robot_model_lib urdf urdfdom urdfdom_headers Eigen3)

# Safety Engine (Old: safety_engine)
add_library(safety_engine STATIC core/gng/GrowingNeuralGas.cpp)
target_link_libraries(safety_engine common_lib kinematics_lib)
ament_target_dependencies(safety_engine Eigen3)

# Visualization-only 3D GNG and compact persistence format
add_library(visualization_gng_lib STATIC
    core/visualization/visualization_gng.cpp
)
ament_target_dependencies(visualization_gng_lib Eigen3)

# Safety Engine Offline (For training)
add_library(safety_engine_offline STATIC 
    core/safety_engine/runtime/influence_management.cpp
    core/safety_engine/runtime/safety_management.cpp
)
target_link_libraries(safety_engine_offline safety_engine robot_model_lib)

# Collision
add_library(collision_lib STATIC 
    core/collision/collision_detector.cpp
    core/collision/environment_collision_checker.cpp
    core/collision/geometric_self_collision_checker.cpp
    core/collision/point_cloud_collision_checker.cpp
    core/collision/self_collision_checker.cpp
    core/collision/voxel_grid.cpp
    core/collision/voxel_collision_checker.cpp
)
if(USE_FCL)
    target_sources(collision_lib PRIVATE core/collision/fcl/fcl_collision_detector.cpp)
    add_definitions(-DUSE_FCL)
    find_package(fcl REQUIRED)
    find_package(ccd REQUIRED)
    find_package(octomap REQUIRED)
endif()
target_link_libraries(collision_lib common_lib kinematics_lib robot_model_lib ${EXTRA_LIBS})
ament_target_dependencies(collision_lib Eigen3 urdfdom)
if(USE_FCL)
    target_link_libraries(collision_lib fcl)
    ament_target_dependencies(collision_lib fcl)
endif()

# Metrics (Old: status)
add_library(metrics_lib STATIC 
    core/metrics/manipulability.cpp
    core/metrics/dynamic_manipulability.cpp
)
target_link_libraries(metrics_lib common_lib kinematics_lib safety_engine)
ament_target_dependencies(metrics_lib Eigen3)

# Analysis (Old: spatial)
add_library(analysis_lib STATIC core/analysis/robot_model_analyzer.cpp)
target_link_libraries(analysis_lib safety_engine common_lib kinematics_lib)
ament_target_dependencies(analysis_lib Eigen3)

# Collection of all core libs
set(LIBS_ALL_CORE common_lib kinematics_lib robot_model_lib safety_engine visualization_gng_lib collision_lib metrics_lib analysis_lib safety_engine_offline)

# --- 3. ROS 2 Nodes ---
# URDFモデルの配信・可視化
add_executable(robot_description_player_node nodes/self_recognition/robot_description_player.cpp)
add_executable(rviz_robot_visual_marker_node nodes/self_recognition/rviz_robot_visual_marker_node.cpp)
add_executable(initial_joint_state_publisher_node nodes/self_recognition/initial_joint_state_publisher_node.cpp)

# 自己認識ボクセルの生成・フィルタ
add_executable(self_recognition_viz_node nodes/self_recognition/self_recognition_viz_node.cpp)
add_executable(self_recognition_filter_node nodes/self_recognition/self_recognition_filter_node.cpp)
add_executable(self_voxel_filter_node nodes/self_recognition/self_voxel_filter_node.cpp)
add_executable(voxel_spherized_robot_viewer_node nodes/bridge/voxel_spherized_robot_viewer_node.cpp)

# ボクセルからVLUTへの橋渡し
add_executable(voxel_to_vlut_node nodes/bridge/voxel_to_vlut_node.cpp)

# オフライン学習ツール
add_executable(offline_urdf_trainer offline_tools/offline_urdf_trainer.cpp)
add_executable(visualization_gng_trainer offline_tools/visualization_gng_trainer.cpp)
add_executable(offline_gng_main5_clean offline_tools/offline_gng_main5_clean.cpp)
add_executable(offline_gng_main6_island_pruning offline_tools/offline_gng_main6_island_pruning.cpp)
add_executable(offline_gng_status_updater offline_tools/offline_gng_status_updater.cpp)

# オフライン検証ツール
add_subdirectory(offline_tools/tools)

# 全ノードのリンクとインストール
set(NODES
    robot_description_player_node
    rviz_robot_visual_marker_node
    initial_joint_state_publisher_node
    self_recognition_viz_node
    self_recognition_filter_node
    self_voxel_filter_node
    voxel_spherized_robot_viewer_node
    voxel_to_vlut_node
    offline_urdf_trainer
    visualization_gng_trainer
    offline_gng_main5_clean
    offline_gng_main6_island_pruning
    offline_gng_status_updater
)
foreach(node ${NODES})
  target_link_libraries(${node} ${LIBS_ALL_CORE} ${EXTRA_LIBS})
  ament_target_dependencies(${node} ${DEPS_ROS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

target_link_libraries(voxel_spherized_robot_viewer_node ${LIBS_ALL_CORE} ${EXTRA_LIBS} tinyxml2)

# 共有コンポーネント
add_library(robot_description_player_component SHARED nodes/self_recognition/robot_description_player.cpp)

set(COMPONENTS robot_description_player_component)
foreach(comp ${COMPONENTS})
  target_link_libraries(${comp} ${LIBS_ALL_CORE} ${EXTRA_LIBS})
  ament_target_dependencies(${comp} ${DEPS_ROS})
  install(TARGETS ${comp} DESTINATION lib/${PROJECT_NAME})
endforeach()

rclcpp_components_register_nodes(robot_description_player_component "robot_sim::self_recognition::RobotDescriptionPlayer")
