cmake_minimum_required(VERSION 3.8)
project(ais_gng)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(voxel_msgs REQUIRED)
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
ament_auto_add_executable(topological_query_node src/topological_query/topological_query_node.cpp src/topological_query/topological_reachability.cpp)
ament_auto_add_executable(topological_grid_node src/topological_grid/topological_grid_node.cpp src/topological_grid/topological_grid_assignment.cpp)
ament_target_dependencies(topological_grid_node voxel_msgs)

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
  if(EXISTS "${_gng_lib_dir}/libgng_${_backend}.so")
    set(_backend_lib "${_gng_lib_dir}/libgng_${_backend}.so")
  endif()

  if(_backend_lib AND _torch_available)
    set(_library_target "ais_gng_component_${_backend}")
    set(_executable_name "ais_gng_${_backend}")
    ament_auto_add_library(${_library_target} SHARED ${_gng_component_sources})
    target_include_directories(${_library_target} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/libgng/include)
    target_link_libraries(${_library_target} "${_backend_lib}" ykpiv ${TORCH_LIBRARIES})
    target_compile_definitions(${_library_target} PRIVATE AIS_GNG_BACKEND_${_backend}=1)
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

ament_auto_package()
