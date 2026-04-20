cmake_minimum_required(VERSION 3.8)
project(ais_gng)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_add_executable(visualizar src/visualizar.cpp)
ament_auto_add_executable(reverse_z_pointcloud src/rev/reverse_z.cpp)
ament_auto_add_executable(cluster_summary_node src/cluster_summary_node.cpp)

ament_auto_add_library(ais_gng SHARED src/ais_gng.cpp src/cluster_classification.cpp src/downsampling.cpp src/visualize_filter.cpp)
include_directories(ais_gng ${CMAKE_CURRENT_LIST_DIR}/libgng/include)

rclcpp_components_register_node(ais_gng PLUGIN "fuzzrobo::AiSGNG" EXECUTABLE ais_gng_node)

install(
 DIRECTORY include config launch
 DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
