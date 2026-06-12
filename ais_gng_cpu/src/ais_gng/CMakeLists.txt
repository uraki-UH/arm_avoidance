cmake_minimum_required(VERSION 3.8)
project(ais_gng)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# visualizar
ament_auto_add_executable(visualizar src/visualizar.cpp)

# Pytorchを探す
execute_process(
    COMMAND python3 -c "import torch; print(torch.utils.cmake_prefix_path)"
    OUTPUT_VARIABLE TORCH_CMAKE_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYTHON_RESULT
)
# パスの取得が成功したかを確認
if(PYTHON_RESULT EQUAL 0)
    # 取得したパスを CMAKE_PREFIX_PATH に追加
    message(STATUS "Automatically found PyTorch CMake path: ${TORCH_CMAKE_PATH}")
    list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PATH}")
else()
    message(FATAL_ERROR "Failed to find PyTorch CMake path using Python. Please check your Python environment or manually set CMAKE_PREFIX_PATH.")
endif()
find_package(Torch REQUIRED)

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

foreach(_backend IN ITEMS cpu gpu)
  set(_backend_lib "")
  if(EXISTS "${_gng_lib_dir}/libgng_${_backend}.so")
    set(_backend_lib "${_gng_lib_dir}/libgng_${_backend}.so")
  endif()

  if(_backend_lib)
    set(_library_target "ais_gng_component_${_backend}")
    set(_executable_name "ais_gng_${_backend}")
    ament_auto_add_library(${_library_target} SHARED src/ais_gng_component.cpp src/handle_label_utils.cpp src/plugin/cluster_classification.cpp src/plugin/downsampling.cpp src/plugin/visualize_filter.cpp)
    target_include_directories(${_library_target} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/libgng/include)
    target_link_libraries(${_library_target} "${_backend_lib}" ykpiv ${TORCH_LIBRARIES})
    target_compile_definitions(${_library_target} PRIVATE AIS_GNG_BACKEND_${_backend}=1)
    rclcpp_components_register_node(${_library_target} PLUGIN "fuzzrobo::AiSGNGComponent" EXECUTABLE ${_executable_name})
    list(APPEND _available_gng_backends ${_backend})
    target_compile_options(${_library_target} PRIVATE
      $<$<CONFIG:Debug>:-O0 -g -fno-omit-frame-pointer>
      $<$<CONFIG:Release>:-O3 -DNDEBUG>
    )
  else()
    message(STATUS "ais_gng: skipping ${_backend} backend because libgng_${_backend}.so was not found in ${_gng_arch_dir}")
  endif()
endforeach()

if(_available_gng_backends STREQUAL "")
  message(FATAL_ERROR "ais_gng: no libgng backend libraries were found under ${_gng_lib_dir}")
endif()

install(
 DIRECTORY include config launch
 DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
