cmake_minimum_required(VERSION 3.8)
project(gng_update_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ais_gng_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TopologicalMapUpdate.msg"
  DEPENDENCIES std_msgs ais_gng_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
