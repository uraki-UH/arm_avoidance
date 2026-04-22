cmake_minimum_required(VERSION 3.10)
project(gng_safety)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 1. Try to find ROS 2 (ament_cmake) ---
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    message(STATUS "ROS 2 (Humble) environment detected.")
    add_definitions(-DUSE_ROS2)
    set(BUILD_WITH_ROS2 ON)
    
    # ROS 2 packages
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(ais_gng_msgs REQUIRED)
    find_package(urdf REQUIRED)
    find_package(urdfdom REQUIRED)
    find_package(tinyxml2_vendor REQUIRED)
    find_package(fcl REQUIRED)
    find_package(Eigen3 REQUIRED)
    if(Eigen3_INCLUDE_DIRS)
        set(_eigen_include_dirs "${Eigen3_INCLUDE_DIRS}")
    elseif(EIGEN3_INCLUDE_DIR)
        set(_eigen_include_dirs "${EIGEN3_INCLUDE_DIR}")
    elseif(EXISTS "/usr/include/eigen3")
        set(_eigen_include_dirs "/usr/include/eigen3")
    else()
        set(_eigen_include_dirs "${PIXI_PROJECT_ROOT}/.pixi/envs/default/include/eigen3")
        if(NOT EXISTS "${_eigen_include_dirs}")
            set(_eigen_include_dirs "/Users/kubotamacbookair/pixi_ros2/.pixi/envs/default/include/eigen3")
        endif()
    endif()
    set(Eigen3_INCLUDE_DIRS "${_eigen_include_dirs}")
    message(STATUS "Final Eigen3_INCLUDE_DIRS: ${Eigen3_INCLUDE_DIRS}")
    include_directories(${Eigen3_INCLUDE_DIRS})
    find_package(ament_index_cpp REQUIRED)
else()
    message(STATUS "Standard CMake environment detected (No ROS 2).")
    set(BUILD_WITH_ROS2 OFF)
    
    # macOS/Homebrew paths
    if(APPLE)
        list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
        add_definitions(-DGL_SILENCE_DEPRECATION)
    endif()

    # Standard package search
    find_package(Eigen3 REQUIRED)
    find_package(tinyxml2 REQUIRED)
    find_package(console_bridge REQUIRED)
    find_package(urdfdom REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(Threads REQUIRED)
    
    # Manual FCL search for standalone
    find_package(FCL QUIET)
endif()

# --- 2. Optional ODE / Drawstuff (Common for both) ---
option(USE_ODE "Use ODE for physical simulation" ON)
if(BUILD_WITH_ROS2)
    set(USE_ODE OFF)
    set(USE_DRAWSTUFF OFF)
    message(STATUS "ROS 2 Build: Forcing USE_ODE=OFF, USE_DRAWSTUFF=OFF")
endif()

if(USE_ODE)
    find_path(ODE_INCLUDE_DIR NAMES ode/ode.h HINTS /usr/local/include /opt/homebrew/include)
    find_library(ODE_LIBRARY NAMES ode HINTS /usr/local/lib /opt/homebrew/lib)
    if(ODE_INCLUDE_DIR AND ODE_LIBRARY)
        message(STATUS "ODE found: ${ODE_LIBRARY}")
        add_definitions(-DUSE_ODE)
    else()
        message(WARNING "ODE not found!")
        set(USE_ODE OFF)
    endif()
endif()

option(USE_DRAWSTUFF "Use Drawstuff for visualization" ON)
if(USE_DRAWSTUFF)
    find_library(DRAWSTUFF_LIBRARY NAMES drawstuff HINTS /usr/local/lib /opt/homebrew/lib)
    if(DRAWSTUFF_LIBRARY)
        message(STATUS "Drawstuff found: ${DRAWSTUFF_LIBRARY}")
        add_definitions(-DUSE_DRAWSTUFF)
    else()
        message(WARNING "Drawstuff not found!")
        set(USE_DRAWSTUFF OFF)
    endif()
endif()

# ImGui setup (for online_integration)
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glut.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
)

# Standard definitions
add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if(fcl_FOUND OR FCL_FOUND)
    add_definitions(-DUSE_FCL)
    set(USE_FCL ON)
    message(STATUS "FCL Support Enabled.")
else()
    set(USE_FCL OFF)
    message(WARNING "FCL Support Disabled (Not found).")
endif()

# Include directories
include_directories(src)
include_directories(src/core_safety)
include_directories(${IMGUI_DIR} ${IMGUI_DIR}/backends)

if(NOT BUILD_WITH_ROS2)
    include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS}
        ${ODE_INCLUDE_DIR}
        ${FCL_INCLUDE_DIRS}
    )
endif()

# Subdirectories
add_subdirectory(src)

# --- 3. Executables ---

# 3.1 Offline Trainer (disabled for ROS 2 viz build - binary already exists)
# add_executable(offline_urdf_trainer src/offline_urdf_trainer.cpp)
# target_link_libraries(offline_urdf_trainer
#     core_safety core_safety_offline description_lib collision_lib status_lib spatial_lib
# )

# 3.2 Online Integration (Simulator)
# if(USE_ODE AND USE_DRAWSTUFF)
#     add_executable(online_integration ${IMGUI_SOURCES} src/simulation/online_integration_main.cpp)
#     target_link_libraries(online_integration 
#         core_safety
#         simulation_full_lib 
#         simulation_lib 
#         hardware_lib 
#         visualization_lib 
#         control_lib 
#         experiment_lib
#         status_lib
#         collision_lib
#         ${ODE_LIBRARY}
#         ${DRAWSTUFF_LIBRARY}
#     )
#     if(NOT BUILD_WITH_ROS2)
#         target_link_libraries(online_integration PRIVATE ${OpenCV_LIBRARIES} Eigen3::Eigen)
#         if(APPLE)
#             target_link_libraries(online_integration PRIVATE "-framework GLUT" "-framework OpenGL")
#         else()
#             target_link_libraries(online_integration PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
#         endif()
#     endif()
# endif()

if(BUILD_WITH_ROS2)
    install(FILES temp_robot.urdf DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
    ament_package()
endif()
