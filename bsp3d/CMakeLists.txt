cmake_minimum_required(VERSION 3.10)
project(bsp3d LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -march=native)
endif()

# ヘッダのみのライブラリ
add_library(bsp3d INTERFACE)
target_include_directories(bsp3d INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(bsp3d INTERFACE cxx_std_17)

add_executable(bsp3d_basic examples/basic.cpp)
target_link_libraries(bsp3d_basic PRIVATE bsp3d)
add_executable(bsp3d_gng examples/gng.cpp)
target_link_libraries(bsp3d_gng PRIVATE bsp3d)

# assertを有効にした回帰試験
add_executable(bsp3d_test examples/test_bsp3d.cpp)
target_link_libraries(bsp3d_test PRIVATE bsp3d)
target_compile_options(bsp3d_test PRIVATE -UNDEBUG)
enable_testing()
add_test(NAME bsp3d_test COMMAND bsp3d_test)

# 非有限値の入力検査を含む範囲検索の回帰。fast-mathの無効化。
add_executable(bsp3d_aabb_test examples/test_aabb.cpp)
target_link_libraries(bsp3d_aabb_test PRIVATE bsp3d)
add_test(NAME bsp3d_aabb_test COMMAND bsp3d_aabb_test)
