cmake_minimum_required(VERSION 3.10)
project(gng_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    message(STATUS "ROS 2 (Humble) environment detected.")
    add_definitions(-DUSE_ROS2)
    set(BUILD_WITH_ROS2 ON)

    find_package(rclcpp REQUIRED)
    find_package(urdf REQUIRED)
    find_package(urdfdom REQUIRED)
    find_package(tinyxml2_vendor REQUIRED)
    find_package(fcl REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(ament_index_cpp REQUIRED)
else()
    message(STATUS "Standard CMake environment detected (No ROS 2).")
    set(BUILD_WITH_ROS2 OFF)

    find_package(Eigen3 REQUIRED)
    find_package(tinyxml2 REQUIRED)
    find_package(console_bridge REQUIRED)
    find_package(urdfdom REQUIRED)
    find_package(Threads REQUIRED)
    find_package(FCL QUIET)
endif()

add_definitions(-DPROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if(FCL_FOUND OR BUILD_WITH_ROS2)
    add_definitions(-DUSE_FCL)
    set(USE_FCL ON)
endif()

include_directories(src)
add_subdirectory(src)

add_executable(offline_urdf_trainer src/offline_urdf_trainer.cpp)

target_link_libraries(offline_urdf_trainer
    collision_lib
    simulation_lib
    gng_lib
    spatial_lib
    status_lib
)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(offline_urdf_trainer
        rclcpp
        urdf
        urdfdom
        fcl
        Eigen3
        ament_index_cpp
    )
    target_link_libraries(offline_urdf_trainer
        fcl
    )
    install(TARGETS offline_urdf_trainer DESTINATION lib/${PROJECT_NAME})
else()
    target_link_libraries(offline_urdf_trainer
        Eigen3::Eigen
        ${urdfdom_LIBRARIES}
    )
    if(USE_FCL)
        target_link_libraries(offline_urdf_trainer
            ${FCL_LIBRARIES}
        )
    endif()
endif()

if(BUILD_WITH_ROS2)
    ament_package()
endif()
