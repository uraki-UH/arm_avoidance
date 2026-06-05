cmake_minimum_required(VERSION 3.8)
project(gng_cpu LANGUAGES CXX C)

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

find_package(ament_cmake REQUIRED)

# LTOの新しい挙動を有効にするポリシーを設定
cmake_policy(SET CMP0069 NEW)

# --- 警告オプション ---
add_compile_options(-Wall -Wextra -Wpedantic)

#  LTO有効化
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)

# srcディレクトリにある.cppファイルの一覧を取得
file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/*.hpp
)

include_directories(include)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

# 不要なシンボルを削除
add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_STRIP} $<TARGET_FILE:${PROJECT_NAME}>
)

# ステップ1 & 3のコンパイラオプション
target_compile_options(${PROJECT_NAME} PRIVATE
    -fvisibility=hidden
    -ffunction-sections
    -fdata-sections
    # Debugビルドの時だけ -g を付ける
    $<$<CONFIG:Debug>:-O3 -g>
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

# OpenSSL 静的ライブラリでinclude
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(WARNING "build target 🎯 aarch64")
    set(OpenSSL_DIR "/usr/local/openssl-aarch64/lib/cmake/OpenSSL") # arm64
    include_directories(/usr/local/openssl-aarch64/include) # arm64
    if(TARGET_JETSON)
        message(WARNING "build target 🎯 Jetson")
        target_compile_options(${PROJECT_NAME} PRIVATE -march=armv8.2-a -mtune=cortex-a78ae)
    endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message(WARNING "build target 🎯 x86_64")
    # set(OpenSSL_DIR "/usr/local/openssl-x86_64/lib64/cmake/OpenSSL") #x86
    # include_directories(/usr/local/openssl-x86_64/include) #x86
endif()
# set(OpenSSL_USE_STATIC_LIBS TRUE) # Prefer dynamic linking for ROS packages
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
find_package(OpenSSL REQUIRED)
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

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()