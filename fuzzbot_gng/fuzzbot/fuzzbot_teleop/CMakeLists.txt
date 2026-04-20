cmake_minimum_required(VERSION 3.8)
project(fuzzbot_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(teleop_key_node src/teleop_key_node.cpp src/teleop_key.cpp src/keyboard_reader.cpp)
ament_auto_add_executable(teleop_joy_node src/teleop_joy_node.cpp src/teleop_joy.cpp)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  teleop_key_node
  teleop_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()