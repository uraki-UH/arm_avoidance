cmake_minimum_required(VERSION 3.8)
project(voxel_idx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
add_library(point_cloud_store SHARED src/point_cloud_store.cpp)
target_compile_features(point_cloud_store PUBLIC cxx_std_17)
target_include_directories(point_cloud_store PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(point_cloud_store PUBLIC Eigen3::Eigen)
install(TARGETS point_cloud_store EXPORT export_point_cloud_store LIBRARY DESTINATION lib)
ament_export_targets(export_point_cloud_store HAS_LIBRARY_TARGET)
ament_export_libraries(point_cloud_store)
ament_export_dependencies(Eigen3)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_point_cloud_store test/test_point_cloud_store.cpp)
  target_link_libraries(test_point_cloud_store point_cloud_store)
endif()

ament_package()
