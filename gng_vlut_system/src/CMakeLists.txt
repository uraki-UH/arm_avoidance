# --- 1. Common Setup ---
include_directories(
    core
    nodes
    offline_tools
    core/safety_engine # Base for internal includes
)

# Standard ROS 2 Dependencies
set(DEPS_ROS rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs tf2_eigen visualization_msgs ais_gng_msgs ament_index_cpp nlohmann_json fcl)
# System libraries
set(EXTRA_LIBS fcl ccd octomap octomath m)

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
add_library(safety_engine STATIC core/safety_engine/gng/GrowingNeuralGas.cpp)
target_link_libraries(safety_engine common_lib kinematics_lib)
ament_target_dependencies(safety_engine Eigen3)

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
    core/collision/fcl/fcl_collision_detector.cpp
)
add_definitions(-DUSE_FCL)
target_link_libraries(collision_lib common_lib kinematics_lib robot_model_lib ${EXTRA_LIBS})
ament_target_dependencies(collision_lib Eigen3 urdfdom)

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
set(LIBS_ALL_CORE common_lib kinematics_lib robot_model_lib safety_engine collision_lib metrics_lib analysis_lib safety_engine_offline)

# --- 3. ROS 2 Nodes ---
if(USE_ROS2)
  # Basic Nodes
  add_executable(robot_bridge_node nodes/bridge/robot_bridge_node.cpp)
  add_executable(joint_state_mux_node nodes/bridge/joint_state_mux_node.cpp)
  add_executable(topoarm_udp_bridge nodes/bridge/topoarm_udp_bridge.cpp)
  add_executable(topofuzzy_bridge_node nodes/bridge/topofuzzy_bridge_node.cpp)
  add_executable(robot_viewer_bridge_node nodes/bridge/robot_viewer_bridge_node.cpp)
  
  # Grouped Nodes
  add_executable(robot_description_player_node nodes/description/robot_description_player.cpp)
  add_executable(safety_monitor_node nodes/safety_monitor/safety_monitor_node.cpp)
  add_executable(self_recognition_viz_node nodes/self_recognition/self_recognition_viz_node.cpp)

  # Link & Installation for all nodes
  set(NODES robot_bridge_node joint_state_mux_node topoarm_udp_bridge topofuzzy_bridge_node robot_viewer_bridge_node safety_monitor_node robot_description_player_node self_recognition_viz_node)
  foreach(node ${NODES})
    target_link_libraries(${node} ${LIBS_ALL_CORE} ${EXTRA_LIBS})
    ament_target_dependencies(${node} ${DEPS_ROS})
    install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
  endforeach()

  # Shared Components
  add_library(robot_viewer_bridge_node_component SHARED nodes/bridge/robot_viewer_bridge_node.cpp)
  add_library(robot_description_player_component SHARED nodes/description/robot_description_player.cpp)
  
  set(COMPONENTS robot_viewer_bridge_node_component robot_description_player_component)
  foreach(comp ${COMPONENTS})
    target_link_libraries(${comp} ${LIBS_ALL_CORE} ${EXTRA_LIBS})
    ament_target_dependencies(${comp} ${DEPS_ROS})
    install(TARGETS ${comp} DESTINATION lib/${PROJECT_NAME})
  endforeach()
  
  rclcpp_components_register_nodes(robot_viewer_bridge_node_component "robot_sim::bridge::RobotViewerBridgeNode")
  rclcpp_components_register_nodes(robot_description_player_component "robot_sim::self_recognition::RobotDescriptionPlayer")

  # --- 4. Offline Training Tools ---
  set(TOOLS offline_urdf_trainer offline_gng_main5_clean offline_gng_main6_island_pruning offline_gng_status_updater)
  foreach(tool ${TOOLS})
    add_executable(${tool} offline_tools/${tool}.cpp)
    target_compile_options(${tool} PRIVATE $<$<CONFIG:Release>:-O3 -DNDEBUG> $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    target_link_libraries(${tool} ${LIBS_ALL_CORE} ${EXTRA_LIBS})
    ament_target_dependencies(${tool} Eigen3 urdfdom rclcpp rmw_implementation_cmake)
    install(TARGETS ${tool} DESTINATION lib/${PROJECT_NAME})
  endforeach()
endif()
