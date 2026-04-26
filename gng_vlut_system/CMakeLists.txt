cmake_minimum_required(VERSION 3.10)
project(gng_vlut_system)

# Set default build type to Release for optimized performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. Try to find ROS 2 (ament_cmake) ---
find_package(ament_cmake QUIET)

# --- 1. ROS 2 Build ---
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

message(STATUS "ROS 2 (Humble) environment detected.")
set(USE_ROS2 ON)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ais_gng_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)

set(_eigen_include_dirs "${Eigen3_INCLUDE_DIRS}")
if(NOT _eigen_include_dirs AND EIGEN3_INCLUDE_DIR)
    set(_eigen_include_dirs "${EIGEN3_INCLUDE_DIR}")
endif()
if(NOT _eigen_include_dirs)
    set(_eigen_include_dirs "/usr/include/eigen3")
endif()
message(STATUS "Eigen3_INCLUDE_DIRS: ${_eigen_include_dirs}")
include_directories(${_eigen_include_dirs})

find_package(ament_index_cpp REQUIRED)
include_directories(${ament_index_cpp_INCLUDE_DIRS})
find_package(rmw_implementation_cmake REQUIRED)

set(_rmw_include_dirs "${rmw_implementation_cmake_INCLUDE_DIRS}")
if(NOT _rmw_include_dirs)
    set(_rmw_include_dirs "/opt/ros/humble/include")
endif()
message(STATUS "rmw_implementation_cmake_INCLUDE_DIRS: ${_rmw_include_dirs}")
include_directories(${_rmw_include_dirs})

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
include_directories(src/core_safety)

# Subdirectories
add_subdirectory(src)

# --- 3. Executables ---
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_auto_package()
