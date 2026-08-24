cmake_minimum_required(VERSION 3.8)
project(grasping_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ais_gng_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(voxel_idx REQUIRED)
find_package(voxel_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(test_rigid_grasp_vlut src/test_rigid_grasp_vlut.cpp)
ament_target_dependencies(test_rigid_grasp_vlut geometry_msgs tf2 tf2_geometry_msgs voxel_idx Eigen3)
add_executable(gripper_volume_graph_node src/gripper_volume_graph_node.cpp)
ament_target_dependencies(
  gripper_volume_graph_node
  ais_gng_msgs geometry_msgs rclcpp std_msgs Eigen3
)
add_executable(grasp_voxel_matcher_node src/grasp_voxel_matcher_node.cpp)
ament_target_dependencies(
  grasp_voxel_matcher_node
  ais_gng_msgs geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  voxel_idx voxel_msgs Eigen3
)
install(TARGETS
  test_rigid_grasp_vlut
  gripper_volume_graph_node
  grasp_voxel_matcher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_gripper_volume_graph test/test_gripper_volume_graph.cpp)
  ament_target_dependencies(
    test_gripper_volume_graph
    ais_gng_msgs geometry_msgs std_msgs Eigen3
  )
  add_test(NAME gripper_volume_graph COMMAND test_gripper_volume_graph)

  add_executable(
    test_grasp_pose_occupancy_evaluator
    test/test_grasp_pose_occupancy_evaluator.cpp
  )
  ament_target_dependencies(
    test_grasp_pose_occupancy_evaluator
    geometry_msgs Eigen3
  )
  add_test(
    NAME grasp_pose_occupancy_evaluator
    COMMAND test_grasp_pose_occupancy_evaluator
  )

  add_executable(
    test_grasp_voxel_matcher
    test/test_grasp_voxel_matcher.cpp
  )
  ament_target_dependencies(
    test_grasp_voxel_matcher
    voxel_idx Eigen3
  )
  add_test(
    NAME grasp_voxel_matcher
    COMMAND test_grasp_voxel_matcher
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  ais_gng_msgs geometry_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros voxel_idx
  voxel_msgs Eigen3
)

ament_package()
