cmake_minimum_required(VERSION 3.8)
project(grasping_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(voxel_indexing_common REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

add_executable(test_rigid_grasp_vlut src/tools/test_rigid_grasp_vlut.cpp)
ament_target_dependencies(test_rigid_grasp_vlut geometry_msgs tf2 tf2_geometry_msgs voxel_indexing_common Eigen3)
install(TARGETS test_rigid_grasp_vlut
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs std_msgs tf2 tf2_geometry_msgs voxel_indexing_common)

ament_package()
