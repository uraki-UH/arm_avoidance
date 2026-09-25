cmake_minimum_required(VERSION 3.8)
project(gng_cpu LANGUAGES CXX C)

find_package(ament_cmake REQUIRED)

# C++のバージョンを指定
set(CMAKE_CXX_STANDARD 20)

# 単一構成ジェネレータにおける既定のリリースビルド
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "ビルド構成" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
else()
    message(WARNING "Build type: ✅ ${CMAKE_BUILD_TYPE}")
endif()

# GNG VERSION
if(NOT DEFINED GNG_VERSION)
  set(GNG_VERSION 0)
else()
  message(WARNING "GNG: ✅ ${GNG_VERSION}, (0: MOVE, 1: STATIC, 2: MAP)")
endif()

option(TARGET_JETSON "build for jetson" OFF)
option(GNG_ENABLE_AUTHENTICATION "Enable binary signature and YubiKey authentication" OFF)
option(GNG_ENABLE_FRAME_LOG "Enable per-frame GNG summary logging" ON)
option(GNG_BUILD_BENCHMARKS "APIテストとベンチマークのビルド" OFF)
option(allow_external_sampler "社内開発用の外部サンプラーAPIと拡張ヘッダーの公開" ON)
option(enable_voxel_framework "共有セル属性・評価基盤の有効化" OFF)
option(enable_voxel_fuzzy "共有基盤のファジィ評価の有効化" OFF)
option(enable_voxel_history "共有基盤のセル履歴の有効化" OFF)
if((enable_voxel_fuzzy OR enable_voxel_history) AND NOT enable_voxel_framework)
  message(FATAL_ERROR "enable_voxel_fuzzy / enable_voxel_history requires enable_voxel_framework")
endif()

# ROS依存なしの共有ヘッダー。利用先への構成伝播と、実行時ライブラリ依存の分離。
add_library(voxel_framework INTERFACE)
target_compile_features(voxel_framework INTERFACE cxx_std_17)
target_include_directories(voxel_framework INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_definitions(voxel_framework INTERFACE
  enable_voxel_framework_build=$<BOOL:${enable_voxel_framework}>
  enable_voxel_fuzzy_build=$<BOOL:${enable_voxel_fuzzy}>
  enable_voxel_history_build=$<BOOL:${enable_voxel_history}>)

# --- 警告オプション ---
add_compile_options(-Wall -Wextra -Wpedantic)


# srcディレクトリにある.cppファイルの一覧を取得
file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/*.hpp
)

# ターゲット作成時のLTO設定の適用
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  cmake_policy(SET CMP0069 NEW)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE voxel_framework)
target_compile_definitions(${PROJECT_NAME} PUBLIC allow_external_sampler_build=$<BOOL:${allow_external_sampler}>)
if(allow_external_sampler)
  target_link_libraries(${PROJECT_NAME} PUBLIC voxel_framework)
endif()
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 不要なシンボルを削除
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_STRIP} $<TARGET_FILE:${PROJECT_NAME}>
  )
endif()

# ステップ1 & 3のコンパイラオプション
target_compile_options(${PROJECT_NAME} PRIVATE
    -fvisibility=hidden
    -ffunction-sections
    -fdata-sections
    # Debugビルドの時だけ -g を付ける
    $<$<CONFIG:Debug>:-O0 -g -fno-omit-frame-pointer>
    # Releaseビルドの時だけ -O3 -DNDEBUG を付ける
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
)

# $<CONFIG:Release:...> でReleaseビルドの時だけ...を適用する
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:_RELEASE>
    $<$<BOOL:${GNG_ENABLE_AUTHENTICATION}>:GNG_ENABLE_AUTHENTICATION>
    $<$<BOOL:${GNG_ENABLE_FRAME_LOG}>:GNG_ENABLE_FRAME_LOG>
    GNG_VERSION=${GNG_VERSION}
)

# ステップ3のリンカオプション
# INTERFACE をつけると、このライブラリをリンクする側にもこのオプションが伝播することがある
# PRIVATE の方が安全な場合が多い
target_link_options(${PROJECT_NAME} PRIVATE
    -Wl,--gc-sections
)

find_package(Boost REQUIRED)
find_package(OpenSSL REQUIRED)

target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
# find_package(ykpiv
#              NAMES libykpiv.a # 拡張子まで指定して静的ライブラリを探させる
#              PATHS /usr/local/lib/)


target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenSSL::SSL
    OpenSSL::Crypto     # 手順2で見つけた静的ライブラリのフルパス
    ykpiv# 共有ライブラリ
)

#  C++のランタイムも静的リンクする
target_link_options(${PROJECT_NAME} PRIVATE -static-libstdc++)

# 配布構成ごとの公開シンボル。製品版には外部登録・任意重点入力の入口なし。
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.v" symbol_map)
if(NOT allow_external_sampler)
  foreach(symbol gng_set_sampling_rules gng_set_priority_input gng_set_weighted_priority_input)
    string(REPLACE "    ${symbol};\n" "" symbol_map "${symbol_map}")
  endforeach()
endif()
set(VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.v")
file(GENERATE OUTPUT "${VERSION_FILE}" CONTENT "${symbol_map}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.v"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/fuzzrobo/libgng/api.h")

# ターゲットにリンカフラグとしてバージョンファイルを設定
set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_FLAGS "-Wl,--version-script=${VERSION_FILE}"
    LINK_DEPENDS "${VERSION_FILE}"
)
if(NOT allow_external_sampler)
  # 開発用ファイルが残る配布先への書込み前の拒否。既存ファイルの自動削除なし。
  install(CODE [[
    foreach(header voxel_framework.hpp builtin_sampling.hpp)
      if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/fuzzrobo/libgng/${header}" OR
          IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/fuzzrobo/libgng/${header}")
        message(FATAL_ERROR "製品版には開発用ヘッダーのない別のinstall先が必要")
      endif()
    endforeach()
  ]])
endif()
set(export_targets ${PROJECT_NAME})
if(allow_external_sampler)
  list(APPEND export_targets voxel_framework)
endif()
install(
  TARGETS ${export_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(allow_external_sampler)
  install(DIRECTORY include/ DESTINATION include)
else()
  # API定義の一元管理。製品SDKから拡張用の型・関数宣言も除外。
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/fuzzrobo/libgng/api.h" product_api)
  set(begin_marker "// external_sampler_begin")
  set(end_marker "// external_sampler_end")
  string(LENGTH "${end_marker}" end_length)
  while(product_api MATCHES "${begin_marker}")
    string(FIND "${product_api}" "${begin_marker}" begin_pos)
    string(FIND "${product_api}" "${end_marker}" end_pos)
    if(end_pos LESS begin_pos)
      message(FATAL_ERROR "外部サンプラーAPIの区間指定不正")
    endif()
    math(EXPR end_pos "${end_pos} + ${end_length}")
    string(SUBSTRING "${product_api}" 0 ${begin_pos} prefix)
    string(SUBSTRING "${product_api}" ${end_pos} -1 suffix)
    set(product_api "${prefix}${suffix}")
  endwhile()
  string(REPLACE "#define allow_external_sampler_build 1" "#define allow_external_sampler_build 0" product_api "${product_api}")
  file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/product_api/api.h" CONTENT "${product_api}")
  install(DIRECTORY include/ DESTINATION include
    PATTERN "api.h" EXCLUDE PATTERN "voxel_framework.hpp" EXCLUDE PATTERN "builtin_sampling.hpp" EXCLUDE)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/product_api/api.h" DESTINATION include/fuzzrobo/libgng)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Boost OpenSSL)

if(GNG_BUILD_BENCHMARKS)
  enable_testing()
  add_executable(gng_builtin_sampling_api_test test/gng_builtin_sampling_api_test.cpp)
  target_link_libraries(gng_builtin_sampling_api_test PRIVATE gng_cpu ${CMAKE_DL_LIBS})
  add_test(NAME gng_builtin_sampling_api_test COMMAND gng_builtin_sampling_api_test)
  if(allow_external_sampler)
    add_executable(voxel_framework_test test/voxel_framework_test.cpp)
    target_link_libraries(voxel_framework_test PRIVATE gng_cpu)
    add_test(NAME voxel_framework_test COMMAND voxel_framework_test)
  endif()
  add_executable(gng_static_age_api_test test/gng_static_age_api_test.cpp)
  target_link_libraries(gng_static_age_api_test PRIVATE gng_cpu)
  add_test(NAME gng_static_age_default COMMAND gng_static_age_api_test 100)
  add_test(NAME gng_static_age_configured COMMAND gng_static_age_api_test 3 3)
  add_test(NAME gng_static_age_runtime COMMAND gng_static_age_api_test 7 3 7)
  add_executable(gng_parameter_update_api_test test/gng_parameter_update_api_test.cpp)
  target_link_libraries(gng_parameter_update_api_test PRIVATE gng_cpu)
  add_test(NAME gng_parameter_update_api_test COMMAND gng_parameter_update_api_test)
  add_executable(voxel_filter_test test/voxel_filter_test.cpp src/cpu/voxel_grid.cpp
    src/utils/param.cpp src/utils/vec3f.cpp)
  target_include_directories(voxel_filter_test PRIVATE src include ${Boost_INCLUDE_DIRS})
  target_compile_definitions(voxel_filter_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME voxel_filter_test COMMAND voxel_filter_test)
  add_executable(radix_sort_test test/radix_sort_test.cpp)
  target_include_directories(radix_sort_test PRIVATE src include)
  target_compile_definitions(radix_sort_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME radix_sort_test COMMAND radix_sort_test)
  add_executable(normal_cache_test test/normal_cache_test.cpp src/cpu/cugng.cpp src/cpu/labelling.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(normal_cache_test PRIVATE src include)
  target_compile_definitions(normal_cache_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME normal_cache_test COMMAND normal_cache_test)
  add_executable(incremental_updates_test test/incremental_updates_test.cpp test/edge_dense_reference/cugng.cpp
    src/cpu/cugng.cpp src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(incremental_updates_test PRIVATE src include)
  target_compile_definitions(incremental_updates_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME incremental_updates_test COMMAND incremental_updates_test)
  add_executable(search_cache_test test/search_cache_test.cpp src/cpu/cugng.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(search_cache_test PRIVATE src include)
  target_compile_definitions(search_cache_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME search_cache_test COMMAND search_cache_test)
  add_executable(cluster_bookkeeping_test test/cluster_bookkeeping_test.cpp
    src/cpu/clustering.cpp src/cpu/cugng.cpp src/utils/cluster.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(cluster_bookkeeping_test PRIVATE src include)
  target_compile_definitions(cluster_bookkeeping_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME cluster_bookkeeping_test COMMAND cluster_bookkeeping_test)
  add_executable(sparse_edges_test test/sparse_edges_test.cpp test/edge_dense_reference/cugng.cpp
    src/cpu/cugng.cpp src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(sparse_edges_test PRIVATE src include)
  target_compile_definitions(sparse_edges_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME sparse_edges_test COMMAND sparse_edges_test)
  add_executable(attention_lookup_test test/attention_lookup_test.cpp ${SRC_FILES})
  target_include_directories(attention_lookup_test PRIVATE src include ${Boost_INCLUDE_DIRS})
  target_compile_definitions(attention_lookup_test PRIVATE GNG_VERSION=${GNG_VERSION}
    ATTENTION_LOOKUP_VARIANT=0)
  target_link_libraries(attention_lookup_test PRIVATE OpenSSL::SSL OpenSSL::Crypto ykpiv)
  add_test(NAME attention_lookup_test COMMAND attention_lookup_test)
  add_executable(node_id_reuse_test test/node_id_reuse_test.cpp src/cpu/cugng.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(node_id_reuse_test PRIVATE src include)
  target_compile_definitions(node_id_reuse_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME node_id_reuse_test COMMAND node_id_reuse_test)
  add_executable(gng_spatial_coverage_api_test test/gng_spatial_coverage_api_test.cpp)
  target_link_libraries(gng_spatial_coverage_api_test PRIVATE gng_cpu)
  add_test(NAME gng_spatial_coverage_api_test COMMAND gng_spatial_coverage_api_test)
  add_executable(gng_cluster_members_api_test test/gng_cluster_members_api_test.cpp)
  target_link_libraries(gng_cluster_members_api_test PRIVATE gng_cpu)
  add_test(NAME gng_cluster_members_api_test COMMAND gng_cluster_members_api_test)
  add_executable(gng_cluster_labels_api_test test/gng_cluster_labels_api_test.cpp)
  target_link_libraries(gng_cluster_labels_api_test PRIVATE gng_cpu)
  add_test(NAME gng_cluster_labels_human COMMAND gng_cluster_labels_api_test human)
  add_test(NAME gng_cluster_labels_car COMMAND gng_cluster_labels_api_test car)
  if(allow_external_sampler)
    add_executable(gng_priority_input_api_test test/gng_priority_input_api_test.cpp)
    target_link_libraries(gng_priority_input_api_test PRIVATE gng_cpu)
    add_test(NAME gng_priority_input_api_test COMMAND gng_priority_input_api_test)
    add_executable(gng_sampling_api_test test/gng_sampling_api_test.cpp)
    target_link_libraries(gng_sampling_api_test PRIVATE gng_cpu)
    add_test(NAME gng_sampling_api_test COMMAND gng_sampling_api_test)
  endif()
  add_executable(sampling_test test/sampling_test.cpp src/cpu/voxel_grid.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(sampling_test PRIVATE src include ${Boost_INCLUDE_DIRS})
  target_compile_definitions(sampling_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME sampling_test COMMAND sampling_test)
  add_executable(gng_neighbors_api_test test/gng_neighbors_api_test.cpp)
  target_link_libraries(gng_neighbors_api_test PRIVATE gng_cpu)
  add_test(NAME gng_neighbors_api_test COMMAND gng_neighbors_api_test)
  add_executable(observation_angle_range_test test/observation_angle_range_test.cpp)
  target_include_directories(observation_angle_range_test PRIVATE include)
  add_test(NAME observation_angle_range_test COMMAND observation_angle_range_test)
  add_executable(gng_observation_api_test test/gng_observation_api_test.cpp)
  target_link_libraries(gng_observation_api_test PRIVATE gng_cpu)
  add_test(NAME gng_observation_api_test COMMAND gng_observation_api_test)

  add_executable(
    gng_training_event_benchmark
    test/gng_training_event_benchmark.cpp
  )
  target_include_directories(
    gng_training_event_benchmark
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_options(gng_training_event_benchmark PRIVATE -O3 -DNDEBUG)

  add_executable(
    gng_training_event_api_test
    test/gng_training_event_api_test.cpp
  )
  target_link_libraries(gng_training_event_api_test PRIVATE gng_cpu)
  target_compile_options(gng_training_event_api_test PRIVATE -O3 -DNDEBUG)

  add_executable(
    gng_map_delta_api_test
    test/gng_map_delta_api_test.cpp
  )
  target_link_libraries(gng_map_delta_api_test PRIVATE gng_cpu)
  target_compile_options(gng_map_delta_api_test PRIVATE -O3 -DNDEBUG)

endif()

ament_package()
