cmake_minimum_required(VERSION 3.8)
project(fuzzy_voxel_grid)

cmake_policy(SET CMP0069 NEW)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ais_gng_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(topological_node_marker_node src/topological_node_marker_node.cpp)

target_include_directories(topological_node_marker_node PRIVATE
  include
)

ament_target_dependencies(topological_node_marker_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  ais_gng_msgs
  std_srvs
)

add_executable(voxel_grid_node src/main.cpp src/voxel_grid_node.cpp)
add_executable(topo_visualizar src/topo_visualizar.cpp)

target_include_directories(voxel_grid_node PRIVATE
  include
)

target_include_directories(topo_visualizar PRIVATE
  include
)

ament_target_dependencies(voxel_grid_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  ais_gng_msgs
  std_srvs
)

ament_target_dependencies(topo_visualizar
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  ais_gng_msgs
  std_srvs
)


# Release系だけ最適化
target_compile_options(voxel_grid_node PRIVATE
  $<$<CONFIG:Release>:-O3 -DNDEBUG>
  $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
)

# IPO / LTO
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
  set_property(TARGET voxel_grid_node PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
  set_property(TARGET voxel_grid_node PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
else()
  message(STATUS "IPO/LTO not supported: ${ipo_error}")
endif()

install(TARGETS
  voxel_grid_node
  topo_visualizar
  topological_node_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()