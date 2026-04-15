cmake_minimum_required(VERSION 3.10)
project(fuzz_gng_result_player)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ais_gng_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(gng_result_player src/gng_result_player.cpp)
ament_target_dependencies(gng_result_player
  rclcpp
  ais_gng_msgs
  visualization_msgs
)
target_link_libraries(gng_result_player Eigen3::Eigen)

add_executable(udp_joint_state_player src/udp_joint_state_player.cpp)
ament_target_dependencies(udp_joint_state_player
  rclcpp
  sensor_msgs
)

add_executable(robot_description_player src/robot_description_player.cpp)
ament_target_dependencies(robot_description_player
  rclcpp
  std_msgs
)

install(TARGETS gng_result_player
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS udp_joint_state_player
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS robot_description_player
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
