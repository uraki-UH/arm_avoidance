cmake_minimum_required(VERSION 3.8)
project(gng_cpu LANGUAGES CXX C)

find_package(ament_cmake REQUIRED)

# C++のバージョンを指定
set(CMAKE_CXX_STANDARD 20)

# デフォルトはDebugビルド
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
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

# --- 警告オプション ---
add_compile_options(-Wall -Wextra -Wpedantic)


# srcディレクトリにある.cppファイルの一覧を取得
file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/*.hpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
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
  # LTOの新しい挙動を有効にするポリシーを設定
  cmake_policy(SET CMP0069 NEW)
  #  LTO有効化
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
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

# バージョンファイルへのパス
set(VERSION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.v)

# ターゲットにリンカフラグとしてバージョンファイルを設定
set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_FLAGS "-Wl,--version-script=${VERSION_FILE}"
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Boost OpenSSL)
ament_package()
