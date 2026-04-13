cmake_minimum_required(VERSION 3.10)
project(gng)

# C++17を使用
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# デバッグビルド用の設定
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# --- Project Source Directory Definition ---
add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Homebrewのパスを追加（macOSの場合）
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
    add_definitions(-DGL_SILENCE_DEPRECATION)
endif()

# パッケージの検索
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(urdfdom REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads REQUIRED)

# --- Optional FCL Support ---
option(USE_FCL "Use FCL for collision detection" OFF)
if(USE_FCL)
    find_package(FCL QUIET)
    if(FCL_FOUND)
        message(STATUS "FCL found. Collision detection with FCL is enabled.")
        add_definitions(-DUSE_FCL)
    else()
        message(WARNING "FCL requested but not found. FCL support will be disabled.")
        set(USE_FCL OFF)
    endif()
endif()

# Manually find ODE
find_path(ODE_INCLUDE_DIR NAMES ode/ode.h HINTS /usr/local/include /opt/homebrew/include)
find_library(ODE_LIBRARY NAMES ode HINTS /usr/local/lib /opt/homebrew/lib)
if(NOT ODE_INCLUDE_DIR OR NOT ODE_LIBRARY)
    message(FATAL_ERROR "ODE library or headers not found!")
endif()

# drawstuffライブラリのパスを検索
find_library(DRAWSTUFF_LIBRARY NAMES drawstuff HINTS /usr/local/lib /opt/homebrew/lib)
if(NOT DRAWSTUFF_LIBRARY)
    message(FATAL_ERROR "drawstuff library not found!")
endif()

# 外部ライブラリ設定 (ImGuiなど)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${urdfdom_INCLUDE_DIRS}
    ${ODE_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${GLUT_INCLUDE_DIR}
    ${FCL_INCLUDE_DIRS}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glut.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
)

# --- Subdirectories (Library Modules) ---
add_subdirectory(src)

# --- Executables ---

# Main Online Integration
add_executable(online_integration ${IMGUI_SOURCES} src/simulation/online_integration_main.cpp)
target_link_libraries(online_integration PRIVATE simulation_lib hardware_lib visualization_lib control_lib experiment_lib)
if(APPLE)
    target_link_libraries(online_integration PRIVATE "-framework GLUT" "-framework OpenGL")
endif()

# Offline GNG Tools
add_executable(offline_gng_main2 src/offline_gng_main2.cpp)
target_link_libraries(offline_gng_main2 PRIVATE simulation_lib collision_lib gng_lib)

add_executable(offline_gng_main3 src/offline_gng_main3.cpp)
target_link_libraries(offline_gng_main3 PRIVATE simulation_lib collision_lib gng_lib)

add_executable(offline_gng_main4 src/offline_gng_main4_env.cpp)
target_link_libraries(offline_gng_main4 PRIVATE simulation_lib collision_lib gng_lib)

add_executable(offline_gng_main5 src/offline_gng_main5_clean.cpp)
target_link_libraries(offline_gng_main5 PRIVATE simulation_lib collision_lib gng_lib)

add_executable(offline_gng_main6 src/offline_gng_main6_island_pruning.cpp)
target_link_libraries(offline_gng_main6 PRIVATE simulation_lib collision_lib gng_lib)

add_executable(offline_gng_status_updater src/offline_gng_status_updater.cpp)
target_link_libraries(offline_gng_status_updater PRIVATE simulation_lib collision_lib gng_lib)

add_executable(offline_gng_spatial_analyzer src/offline_gng_spatial_analyzer.cpp)
target_link_libraries(offline_gng_spatial_analyzer PRIVATE simulation_lib collision_lib gng_lib spatial_lib)

add_executable(offline_gng_main_integrated src/offline_gng_main_integrated.cpp)
target_link_libraries(offline_gng_main_integrated PRIVATE simulation_lib collision_lib gng_lib status_lib)

# Diagnostic Tools
add_executable(inspect_gng_data src/inspect_gng_data.cpp)
target_link_libraries(inspect_gng_data PRIVATE simulation_lib gng_lib)

# Tests & Verifications
add_executable(test_collision_integration test/test_collision_integration.cpp)
target_link_libraries(test_collision_integration PRIVATE simulation_lib collision_lib)

# (Other test executables can be added similarly linking to their required libs)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
