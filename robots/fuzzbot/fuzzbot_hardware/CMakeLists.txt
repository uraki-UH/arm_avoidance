cmake_minimum_required(VERSION 3.8)
project(fuzzbot_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(wheel_handler_node src/wheel_handler_node.cpp)
# ament_auto_add_executable(pantilt_handler_node src/pantilt_handler_node.cpp)

install(
  DIRECTORY #include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  wheel_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()