cmake_minimum_required(VERSION 3.8)
project(fuzzy_classifier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ais_gng_msgs REQUIRED)

add_executable(cluster_summary_node
  src/cluster_summary_node.cpp
)
add_executable(cluster_relabel_node
  src/cluster_relabel_node.cpp
)

ament_target_dependencies(cluster_summary_node
  rclcpp
  std_msgs
  visualization_msgs
  ais_gng_msgs
)
ament_target_dependencies(cluster_relabel_node
  rclcpp
  ais_gng_msgs
)

install(TARGETS cluster_summary_node cluster_relabel_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
