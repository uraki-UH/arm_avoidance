cmake_minimum_required(VERSION 3.8)
project(ais_gng)

# 単一構成ジェネレータにおける既定のリリースビルド。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(voxel_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gng_cpu REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(PNG REQUIRED)
find_package(ZLIB REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_topological_grid_assignment
    test/test_topological_grid_assignment.cpp
    src/topological_grid/topological_grid_assignment.cpp
  )
  target_include_directories(test_topological_grid_assignment PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_topological_grid_assignment ais_gng_msgs)

endif()

# visualizar
ament_auto_add_executable(visualizar src/visualizar.cpp)
ament_auto_add_executable(object_gng_dataset_exporter_node src/object_gng_dataset_exporter.cpp)
ament_target_dependencies(object_gng_dataset_exporter_node sensor_msgs pcl_conversions)
target_include_directories(object_gng_dataset_exporter_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(object_gng_dataset_exporter_node ${PCL_LIBRARIES} PNG::PNG ZLIB::ZLIB)
ament_auto_add_executable(topological_query_node src/topological_query/topological_query_node.cpp src/topological_query/topological_reachability.cpp)
ament_auto_add_executable(topological_grid_node src/topological_grid/topological_grid_node.cpp src/topological_grid/topological_grid_assignment.cpp)
ament_target_dependencies(topological_grid_node voxel_msgs)

# 増分方式の平面クラスタ生成。
ament_auto_add_library(
  plane_cluster_incremental
  SHARED
  src/topological_plane/plane_cluster_incremental.cpp
)
target_include_directories(plane_cluster_incremental PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(plane_cluster_incremental ais_gng_msgs Eigen3)

ament_auto_add_executable(
  plane_cluster_incremental_node
  src/topological_plane/plane_cluster_incremental_node.cpp
)
target_link_libraries(plane_cluster_incremental_node plane_cluster_incremental)
ament_target_dependencies(
  plane_cluster_incremental_node
  ais_gng_msgs rclcpp visualization_msgs Eigen3
)

# 平面クラスタ未所属nodeの連結成分抽出。
ament_auto_add_library(
  nonplane_component_extractor
  SHARED
  src/topological_plane/nonplane_component_extractor.cpp
)
target_include_directories(nonplane_component_extractor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(nonplane_component_extractor ais_gng_msgs)

ament_auto_add_executable(
  nonplane_component_node
  src/topological_plane/nonplane_component_node.cpp
)
target_link_libraries(nonplane_component_node nonplane_component_extractor)
ament_target_dependencies(
  nonplane_component_node
  ais_gng_msgs rclcpp visualization_msgs
)

if(BUILD_TESTING)
  ament_add_gtest(
    test_plane_cluster_incremental
    test/test_plane_cluster_incremental.cpp
  )
  target_link_libraries(test_plane_cluster_incremental plane_cluster_incremental)
  ament_target_dependencies(test_plane_cluster_incremental ais_gng_msgs Eigen3)

  ament_add_gtest(
    test_nonplane_component_extractor
    test/test_nonplane_component_extractor.cpp
  )
  target_link_libraries(test_nonplane_component_extractor nonplane_component_extractor)
  ament_target_dependencies(test_nonplane_component_extractor ais_gng_msgs)

  add_executable(
    benchmark_plane_cluster_incremental
    test/benchmark_plane_cluster_incremental.cpp
  )
  target_link_libraries(benchmark_plane_cluster_incremental plane_cluster_incremental)
  ament_target_dependencies(benchmark_plane_cluster_incremental ais_gng_msgs Eigen3)
endif()

set(_gng_component_sources
  src/ais_gng_component.cpp
  src/handle_label_utils.cpp
  src/point_selection.cpp
  src/topological_query/topological_reachability.cpp
  src/plugin/cluster_classification.cpp
  src/plugin/downsampling.cpp
  src/plugin/visualize_filter.cpp
)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _gng_system_processor)
if(_gng_system_processor STREQUAL "x86_64" OR _gng_system_processor STREQUAL "amd64")
  set(_gng_arch_dir "lib_x86_64")
elseif(_gng_system_processor STREQUAL "aarch64" OR _gng_system_processor STREQUAL "arm64")
  set(_gng_arch_dir "lib_aarch64")
else()
  message(FATAL_ERROR "ais_gng: unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}'")
endif()

set(_gng_lib_dir "${CMAKE_CURRENT_LIST_DIR}/libgng/${_gng_arch_dir}")
if(NOT IS_DIRECTORY "${_gng_lib_dir}")
  message(FATAL_ERROR "ais_gng: architecture library directory not found: ${_gng_lib_dir}")
endif()

set(_available_gng_backends "")

set(_torch_available OFF)
execute_process(
    COMMAND python3 -c "import torch; print(torch.utils.cmake_prefix_path)"
    OUTPUT_VARIABLE TORCH_CMAKE_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYTHON_RESULT
)
if(PYTHON_RESULT EQUAL 0)
    message(STATUS "Automatically found PyTorch CMake path: ${TORCH_CMAKE_PATH}")
    list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PATH}")
    find_package(Torch QUIET)
    if(Torch_FOUND)
        set(_torch_available ON)
    else()
        message(WARNING "PyTorch was imported, but CMake package could not be found. GNG backends will be skipped.")
    endif()
else()
    message(WARNING "PyTorch is not available in this Python environment. GNG backends will be skipped, but utility nodes can still build.")
endif()

foreach(_backend IN ITEMS cpu gpu)
  set(_backend_lib "")
  if(_backend STREQUAL "cpu")
    set(_backend_lib gng_cpu::gng_cpu)
  elseif(EXISTS "${_gng_lib_dir}/libgng_${_backend}.so")
    set(_backend_lib "${_gng_lib_dir}/libgng_${_backend}.so")
  endif()

  if(_backend_lib AND _torch_available)
    string(TOUPPER "${_backend}" _backend_upper)
    set(_library_target "ais_gng_component_${_backend}")
    set(_executable_name "ais_gng_${_backend}")
    ament_auto_add_library(${_library_target} SHARED ${_gng_component_sources})
    if(_backend STREQUAL "gpu")
      target_include_directories(${_library_target} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/libgng/include)
    else()
      target_link_libraries(${_library_target} plane_cluster_incremental)
    endif()
    target_link_libraries(${_library_target} "${_backend_lib}" ykpiv ${TORCH_LIBRARIES})
    target_compile_definitions(${_library_target} PRIVATE AIS_GNG_BACKEND_${_backend_upper}=1)
    rclcpp_components_register_node(${_library_target} PLUGIN "fuzzrobo::AiSGNGComponent" EXECUTABLE ${_executable_name})
    list(APPEND _available_gng_backends ${_backend})
    target_compile_options(${_library_target} PRIVATE
      $<$<CONFIG:Debug>:-O0 -g -fno-omit-frame-pointer>
      $<$<CONFIG:Release>:-O3 -DNDEBUG>
    )
  elseif(_backend_lib)
    message(STATUS "ais_gng: skipping ${_backend} backend because PyTorch CMake package is unavailable")
  else()
    message(STATUS "ais_gng: skipping ${_backend} backend because libgng_${_backend}.so was not found in ${_gng_arch_dir}")
  endif()
endforeach()

if(_available_gng_backends STREQUAL "")
  message(WARNING "ais_gng: no libgng backend components were built")
endif()

install(
 DIRECTORY include config launch
 DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/save_object_gng_dataset
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
