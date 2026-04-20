cmake_minimum_required(VERSION 3.10)
project(core_safety)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 REQUIRED)

set(_eigen_include_dirs "${Eigen3_INCLUDE_DIRS}")
if(NOT _eigen_include_dirs AND EIGEN3_INCLUDE_DIR)
    set(_eigen_include_dirs "${EIGEN3_INCLUDE_DIR}")
endif()
if(NOT _eigen_include_dirs)
    set(_eigen_include_dirs "/usr/include/eigen3")
endif()

# Include directories
# We include '..' so that headers can use #include "core_safety/..." 
# or just '.' for local includes.
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${_eigen_include_dirs}
)

# Core Safety Library
add_library(${PROJECT_NAME} STATIC
    gng/GrowingNeuralGas.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>
)

# If ROS 2 is present, handle dependencies
if(ament_cmake_FOUND)
    ament_target_dependencies(${PROJECT_NAME} Eigen3)
    target_link_libraries(${PROJECT_NAME} common_lib kinematics_lib)
else()
    target_link_libraries(${PROJECT_NAME} Eigen3::Eigen common_lib kinematics_lib)
endif()

# Since most of our logic is in headers, we primarily need to ensure 
# GrowingNeuralGas.cpp can be compiled into the shared lib.
# All other headers (spatial, analysis, persistence) are included via search path.
add_subdirectory(management)
