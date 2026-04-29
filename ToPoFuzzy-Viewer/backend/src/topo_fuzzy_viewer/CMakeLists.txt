cmake_minimum_required(VERSION 3.16)
project(topo_fuzzy_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -march=x86-64 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(ais_gng_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

set(ROS2_DEPENDENCIES_BASE
  rclcpp
  std_msgs
)

set(ROS2_DEPENDENCIES_PC
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)

set(ROS2_DEPENDENCIES_GRAPH
  rclcpp
  ament_index_cpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  ais_gng_msgs
  tf2_msgs
)

set(ROS2_DEPENDENCIES_EDIT
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.10 REQUIRED COMPONENTS
  common io visualization filters segmentation
  kdtree search surface octree features sample_consensus
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

FetchContent_Declare(
  uSockets
  GIT_REPOSITORY https://github.com/uNetworking/uSockets.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(uSockets)

FetchContent_Declare(
  uWebSockets
  GIT_REPOSITORY https://github.com/uNetworking/uWebSockets.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(uWebSockets)

file(GLOB_RECURSE USOCKETS_SOURCES "${usockets_SOURCE_DIR}/src/*.c" "${usockets_SOURCE_DIR}/src/*.cpp")
add_library(uSockets STATIC ${USOCKETS_SOURCES})
target_compile_definitions(uSockets PUBLIC LIBUS_USE_OPENSSL)
target_include_directories(uSockets PUBLIC ${usockets_SOURCE_DIR}/src)

find_package(OpenSSL REQUIRED)
target_link_libraries(uSockets OpenSSL::SSL OpenSSL::Crypto)

find_package(Threads REQUIRED)

add_executable(viewer_ws_gateway_node
  src/nodes/viewer_ws_gateway_node.cpp
  src/common/pcl_converter.cpp
)
target_include_directories(viewer_ws_gateway_node PRIVATE
  ${uwebsockets_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(viewer_ws_gateway_node
  ${PCL_LIBRARIES}
  uSockets
  z
  Threads::Threads
)
ament_target_dependencies(viewer_ws_gateway_node ${ROS2_DEPENDENCIES_GRAPH})

add_executable(viewer_source_node
  src/nodes/viewer_source_node.cpp
  src/common/pcl_converter.cpp
)
target_include_directories(viewer_source_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_source_node ${PCL_LIBRARIES} Threads::Threads)
ament_target_dependencies(viewer_source_node ${ROS2_DEPENDENCIES_GRAPH})

add_executable(viewer_file_node
  src/nodes/viewer_file_node.cpp
  src/services/pointcloud_file_manager.cpp
  src/common/pcl_converter.cpp
)
target_include_directories(viewer_file_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_file_node ${PCL_LIBRARIES} Threads::Threads)
ament_target_dependencies(viewer_file_node ${ROS2_DEPENDENCIES_PC})

add_executable(viewer_rosbag_node
  src/nodes/viewer_rosbag_node.cpp
  src/services/rosbag_manager.cpp
)
target_include_directories(viewer_rosbag_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_rosbag_node Threads::Threads)
ament_target_dependencies(viewer_rosbag_node ${ROS2_DEPENDENCIES_BASE})

add_executable(viewer_gng_node
  src/nodes/viewer_gng_node.cpp
  src/services/gng_process_manager.cpp
)
target_include_directories(viewer_gng_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_gng_node Threads::Threads)
ament_target_dependencies(viewer_gng_node ${ROS2_DEPENDENCIES_BASE})

add_executable(viewer_param_node
  src/nodes/viewer_param_node.cpp
  src/services/ros2_parameter_manager.cpp
)
target_include_directories(viewer_param_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_param_node Threads::Threads)
ament_target_dependencies(viewer_param_node ${ROS2_DEPENDENCIES_BASE})

add_executable(viewer_edit_node
  src/nodes/viewer_edit_node.cpp
  src/common/pcl_converter.cpp
)
target_include_directories(viewer_edit_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(viewer_edit_node ${PCL_LIBRARIES} Threads::Threads)
ament_target_dependencies(viewer_edit_node ${ROS2_DEPENDENCIES_EDIT})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  viewer_ws_gateway_node
  viewer_source_node
  viewer_file_node
  viewer_rosbag_node
  viewer_gng_node
  viewer_param_node
  viewer_edit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
