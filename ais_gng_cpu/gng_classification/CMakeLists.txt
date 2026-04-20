cmake_minimum_required(VERSION 3.8)
project(gng_classification)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(ais_gng_msgs REQUIRED)

#--- for Python
ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
  scripts/sampling_dataset.py
  DESTINATION lib/${PROJECT_NAME}
)

#--- Install directories
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
