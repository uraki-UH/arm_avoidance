cmake_minimum_required(VERSION 3.10)
project(gng_vlut_system)

# Jazzy/HarmonicのPython実行環境。既定OFFによる従来C++ビルドの維持。
option(enable_harmonic_runtime_only "Harmonic実行部品だけのインストール" OFF)
if(enable_harmonic_runtime_only)
  find_package(ament_cmake REQUIRED)
  install(FILES
    launch/dual_arm_tasks.launch.py
    launch/dual_arm_gz.launch.py
    launch/dual_arm_effort_config.py
    launch/simulation_scenario.py
    DESTINATION share/${PROJECT_NAME}/launch)
  install(FILES config/dual_arm_effort.yaml DESTINATION share/${PROJECT_NAME}/config)
  install(DIRECTORY config/simulation DESTINATION share/${PROJECT_NAME}/config)
  install(FILES scripts/task_program.py DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/task_executor.py scripts/harmonic_launcher.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY
    ../urdf/topo_dual_arm_max ../urdf/topo_dual_arm_max_long
    DESTINATION share/${PROJECT_NAME}/urdf)
  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_task_program test/test_task_program.py)
    ament_add_pytest_test(test_harmonic_launcher test/test_harmonic_launcher.py)
  endif()
  ament_package()
  return()
endif()

# Set default build type to Release for optimized performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- 1. Try to find ROS 2 (ament_cmake) ---
find_package(ament_cmake QUIET)

# --- 1. ROS 2 Build ---
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(arrow_visualization REQUIRED)
include_directories(${arrow_visualization_INCLUDE_DIRS})

message(STATUS "ROS 2 (Humble) environment detected.")

# Gazebo依存のJsonCppをPCL/VTKより先に解決する初期化順序
find_package(gazebo REQUIRED)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ais_gng_msgs REQUIRED)
find_package(ais_gng_feature_msgs REQUIRED)
find_package(voxel_msgs REQUIRED)#追加
find_package(rclcpp_components REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(fcl QUIET)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenMP REQUIRED)
find_package(voxel_idx REQUIRED)
find_package(grasping_system REQUIRED)
find_package(gng_control_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters people sample_consensus search segmentation)

set(_eigen_include_dirs "${Eigen3_INCLUDE_DIRS}")
if(NOT _eigen_include_dirs AND EIGEN3_INCLUDE_DIR)
    set(_eigen_include_dirs "${EIGEN3_INCLUDE_DIR}")
endif()
if(NOT _eigen_include_dirs)
    set(_eigen_include_dirs "/usr/include/eigen3")
endif()
message(STATUS "Eigen3_INCLUDE_DIRS: ${_eigen_include_dirs}")
include_directories(${_eigen_include_dirs})

set(_urdfdom_headers_include_dirs "${urdfdom_headers_INCLUDE_DIRS}")
if(NOT _urdfdom_headers_include_dirs)
    set(_urdfdom_headers_include_dirs "/opt/ros/humble/include/urdfdom_headers")
endif()
include_directories(${_urdfdom_headers_include_dirs})

set(_urdfdom_include_dirs "${urdfdom_INCLUDE_DIRS}")
if(NOT _urdfdom_include_dirs)
    set(_urdfdom_include_dirs "/opt/ros/humble/include/urdfdom")
endif()
include_directories(${_urdfdom_include_dirs})

find_package(ament_index_cpp REQUIRED)
include_directories(${ament_index_cpp_INCLUDE_DIRS})
find_package(rmw_implementation_cmake REQUIRED)

set(_rmw_include_dirs "${rmw_implementation_cmake_INCLUDE_DIRS}")
if(NOT _rmw_include_dirs)
    set(_rmw_include_dirs "/opt/ros/humble/include")
endif()
message(STATUS "rmw_implementation_cmake_INCLUDE_DIRS: ${_rmw_include_dirs}")
include_directories(${_rmw_include_dirs})
include_directories(${voxel_idx_INCLUDE_DIRS})
include_directories(${grasping_system_INCLUDE_DIRS})
include_directories(${gng_control_msgs_INCLUDE_DIRS})
include_directories(${ais_gng_feature_msgs_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Standard definitions
add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if(fcl_FOUND OR FCL_FOUND)
    add_definitions(-DUSE_FCL)
    set(USE_FCL ON)
    message(STATUS "FCL Support Enabled.")
else()
    set(USE_FCL OFF)
    message(WARNING "FCL Support Disabled (Not found).")
endif()

# Include directories
include_directories(src)
include_directories(src/core/safety_engine)

# Gazebo専用の有限トルクモータ
find_package(gazebo_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
add_library(bounded_gazebo_system SHARED src/simulation/bounded_gazebo_system.cpp)
target_include_directories(bounded_gazebo_system PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(bounded_gazebo_system ${GAZEBO_LIBRARIES} nlohmann_json::nlohmann_json)
ament_target_dependencies(bounded_gazebo_system gazebo_ros2_control pluginlib rclcpp std_msgs std_srvs)
pluginlib_export_plugin_description_file(gazebo_ros2_control gazebo_system_plugins.xml)
install(TARGETS bounded_gazebo_system LIBRARY DESTINATION lib)

# Subdirectories
add_subdirectory(src)
rclcpp_components_register_nodes(world_index_to_voxel_component "robot_sim::bridge::WorldIndexToVoxelNode")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_viewer_status test/test_viewer_status.cpp)
  ament_target_dependencies(test_viewer_status rclcpp std_msgs)
  foreach(test_name test_gng_nearest_index test_gng_nearest_training)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE src/core)
    target_link_libraries(${test_name} safety_engine collision_lib kinematics_lib)
    ament_target_dependencies(${test_name} Eigen3 rclcpp urdfdom)
  endforeach()
  if(USE_FCL)
    foreach(test_name test_self_collision_policy test_solid_voxel_geometry test_geometric_solid_containment)
      ament_add_gtest(${test_name} test/${test_name}.cpp)
      target_include_directories(${test_name} PRIVATE src/core)
      target_link_libraries(${test_name} collision_lib robot_model_lib kinematics_lib)
      ament_target_dependencies(${test_name} Eigen3 rclcpp fcl urdfdom)
    endforeach()
    ament_add_gtest(test_gng_collision_filter test/test_gng_collision_filter.cpp)
    target_include_directories(test_gng_collision_filter PRIVATE src/core)
    target_link_libraries(test_gng_collision_filter safety_engine collision_lib kinematics_lib)
    ament_target_dependencies(test_gng_collision_filter Eigen3 rclcpp fcl urdfdom)
    ament_add_gtest(test_reachability_collision test/test_reachability_collision.cpp)
    target_include_directories(test_reachability_collision PRIVATE src/core)
    target_compile_definitions(test_reachability_collision PRIVATE
      workspace_source_dir="${CMAKE_CURRENT_SOURCE_DIR}/..")
    target_link_libraries(test_reachability_collision collision_lib robot_model_lib kinematics_lib)
    ament_target_dependencies(test_reachability_collision Eigen3 rclcpp fcl urdfdom)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../benchmarks/gng_nearest_index_20261001/gng_nearest_index_benchmark.cpp")
    add_executable(gng_nearest_index_benchmark
      ../benchmarks/gng_nearest_index_20261001/gng_nearest_index_benchmark.cpp)
    target_include_directories(gng_nearest_index_benchmark PRIVATE src/core)
    target_link_libraries(gng_nearest_index_benchmark safety_engine collision_lib robot_model_lib kinematics_lib)
    ament_target_dependencies(gng_nearest_index_benchmark Eigen3 rclcpp urdfdom)
  endif()

  ament_add_pytest_test(test_dual_arm_collision_geometry test/test_dual_arm_collision_geometry.py)
  ament_add_pytest_test(test_dual_arm_launch test/test_dual_arm_launch.py)
  ament_add_pytest_test(test_avoidance_motion test/test_avoidance_motion.py)
  ament_add_pytest_test(test_task_program test/test_task_program.py)
  ament_add_pytest_test(test_harmonic_launcher test/test_harmonic_launcher.py)
  ament_add_pytest_test(test_pointcloud_avoidance test/test_pointcloud_avoidance.py)
  ament_add_pytest_test(test_dual_arm_limits test/test_dual_arm_limits.py)
  ament_add_pytest_test(test_gazebo_stop_keyboard test/test_gazebo_stop_keyboard.py)
  ament_add_pytest_test(test_dual_arm_mode_model test/test_dual_arm_mode_model.py)
  ament_add_pytest_test(test_dual_arm_control test/test_dual_arm_control.py)
  ament_add_pytest_test(test_dual_arm_control_keyboard test/test_dual_arm_control_keyboard.py)
  ament_add_pytest_test(test_dual_arm_udp_output test/test_dual_arm_udp_output.py)
  ament_add_pytest_test(test_dual_arm_control_udp test/test_dual_arm_control_udp.py)
  ament_add_pytest_test(test_dynamixel_neck_torque test/test_dynamixel_neck_torque.py)
  ament_add_gtest(test_goal_tasks test/test_goal_tasks.cpp)
  target_include_directories(test_goal_tasks PRIVATE src)
  ament_add_gtest(test_joint_command_mux test/test_joint_command_mux.cpp)
  target_include_directories(test_joint_command_mux PRIVATE src)
  ament_add_gtest(test_gazebo_stop_latch test/test_gazebo_stop_latch.cpp)
  target_include_directories(test_gazebo_stop_latch PRIVATE src)
  ament_add_pytest_test(test_joint_command_model test/test_joint_command_model.py)
  ament_add_gtest(test_joint_state_utils test/test_joint_state_utils.cpp)
  target_include_directories(test_joint_state_utils PRIVATE src)
  ament_target_dependencies(test_joint_state_utils Eigen3 sensor_msgs)
  ament_add_gtest(test_motion_smoothing_cpp test/test_motion_smoothing.cpp)
  target_include_directories(test_motion_smoothing_cpp PRIVATE src/core)
  ament_target_dependencies(test_motion_smoothing_cpp Eigen3)
  ament_add_pytest_test(test_motion_smoothing test/test_motion_smoothing.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  ament_add_gtest(test_grasp_refinement test/test_grasp_refinement.cpp)
  target_include_directories(test_grasp_refinement PRIVATE src)
  ament_target_dependencies(test_grasp_refinement Eigen3)
  ament_add_gtest(test_visualization_gng test/test_visualization_gng.cpp)
  target_include_directories(test_visualization_gng PRIVATE src/core)
  target_link_libraries(test_visualization_gng visualization_gng_lib)
  ament_target_dependencies(test_visualization_gng Eigen3 rclcpp tf2_ros tf2_eigen
    ais_gng_msgs ais_gng_feature_msgs geometry_msgs)
  ament_add_gtest(test_candidate_metric_availability
    test/test_candidate_metric_availability.cpp
    test/test_candidate_path_batch.cpp)
  if(TARGET test_candidate_metric_availability)
    target_include_directories(test_candidate_metric_availability PRIVATE src src/core src/legacy)
    target_link_libraries(test_candidate_metric_availability metrics_lib safety_engine collision_lib)
    ament_target_dependencies(test_candidate_metric_availability
      rclcpp sensor_msgs geometry_msgs ais_gng_msgs ais_gng_feature_msgs gng_control_msgs)
  endif()
  ament_add_gtest(test_grasp_candidate_reachability test/test_grasp_candidate_reachability.cpp)
  target_include_directories(test_grasp_candidate_reachability PRIVATE src ../bsp3d/include)
  ament_target_dependencies(test_grasp_candidate_reachability
    tf2 ais_gng_msgs ais_gng_feature_msgs gng_control_msgs)
  ament_add_gtest(
    test_reachability_voxel_accumulator
    test/test_reachability_voxel_accumulator.cpp
    test/test_world_point_bucket_index.cpp
    test/test_persistent_depth_world_index.cpp
    test/test_voxelizer_engine.cpp
  )
  if(TARGET test_reachability_voxel_accumulator)
    target_include_directories(
      test_reachability_voxel_accumulator PRIVATE src src/core
    )
    ament_target_dependencies(
      test_reachability_voxel_accumulator Eigen3 sensor_msgs std_msgs voxel_msgs voxel_idx
    )
  endif()
endif()

# --- 3. Resource Installation ---
install(DIRECTORY config launch gng_results worlds
  DESTINATION share/${PROJECT_NAME}
)

# 旧配置のURDFが存在するworkspaceとの互換性
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
endif()

install(FILES scripts/dual_arm_avoidance_geometry.py scripts/motion_smoothing.py
  scripts/task_program.py
  scripts/joint_motion_state.py
  scripts/local_qp.py
  scripts/joint_command_model.py
  scripts/dual_arm_mode_model.py
  scripts/dual_arm_udp_output.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/task_executor.py
  scripts/harmonic_launcher.py
  scripts/joint_motion_observer.py
  scripts/dynamixel_neck_torque.py
  scripts/dynamixel_sim_output.py
  scripts/dynamixel_sim_keyboard.py
  scripts/start_gazebo_physics.py
  scripts/avoidance_motion.py
  scripts/gng_avoidance_planner.py
  scripts/dual_arm_gng_lidar_demo.py
  scripts/external_pointcloud_bridge.py
  scripts/dual_arm_avoidance_demo.py
  scripts/joint_command_output.py
  scripts/dual_arm_gazebo_demo.py
  scripts/gazebo_stop_keyboard.py
  scripts/dual_arm_control.py
  scripts/dual_arm_control_keyboard.py
  scripts/video_file_publisher.py
  scripts/yolo_person_detector.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../dummy_joint_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
