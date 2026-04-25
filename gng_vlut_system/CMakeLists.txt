cmake_minimum_required(VERSION 3.10)
project(gng_safety)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. Try to find ROS 2 (ament_cmake) ---
find_package(ament_cmake QUIET)

# --- 1. ROS 2 Build ---
find_package(ament_cmake REQUIRED)

message(STATUS "ROS 2 (Humble) environment detected.")
add_definitions(-DUSE_ROS2)

# ROS 2 packages
find_package(rclcpp REQUIRED)
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
include_directories(${Eigen3_INCLUDE_DIRS})
find_package(ament_index_cpp REQUIRED)
include_directories(${ament_index_cpp_INCLUDE_DIRS})

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
install(FILES temp_robot.urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
