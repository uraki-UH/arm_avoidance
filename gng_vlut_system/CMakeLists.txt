cmake_minimum_required(VERSION 3.10)
project(gng_vlut_system)

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

message(STATUS "ROS 2 (Humble) environment detected.")

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

# Subdirectories
add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
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
install(DIRECTORY urdf config launch gng_results worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/video_file_publisher.py
  scripts/yolo_person_detector.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../dummy_joint_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
