cmake_minimum_required(VERSION 3.8)
project(pantilt_transform)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

ament_auto_add_executable(transform src/transform.cpp)

include_directories(PCL_CONVERSIONS_INCLUDE_DIRS)

install(
 DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  transform
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
