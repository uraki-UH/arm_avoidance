cmake_minimum_required(VERSION 3.10)
project(GenericSpatialTree LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force SpatialTree to build as Release even when included from a parent
# project that configures another build type.
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -march=native -ffast-math)
endif()

# Find OpenCV (Optional for Visualization)
find_package(OpenCV QUIET)

# Include directories
include_directories(include)

if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_definitions(-DUSE_OPENCV)
  message(STATUS "OpenCV found. Visualization examples enabled.")
endif()

# Core Benchmarks 
# add_executable(2d_benchmark examples/2d_benchmark.cpp)
# add_executable(3d_benchmark examples/3d_benchmark.cpp)
# add_executable(5d_benchmark examples/5d_benchmark.cpp)
# add_executable(7d_benchmark examples/7d_benchmark.cpp)
# add_executable(brute_force_benchmark examples/brute_force_benchmark.cpp)
# add_executable(update_mechanism_benchmark examples/update_mechanism_benchmark.cpp)

# Optional Visualization Examples
if(OpenCV_FOUND)

  add_executable(dynamic_3d_cross_learning examples/dynamic_3d_cross_learning.cpp)
  target_link_libraries(dynamic_3d_cross_learning ${OpenCV_LIBS})

  add_executable(dynamic_2d_cross_learning examples/dynamic_2d_cross_learning.cpp)
  target_link_libraries(dynamic_2d_cross_learning ${OpenCV_LIBS})
endif()
