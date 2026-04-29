# Collision Module
set(COLLISION_SRCS
    collision_detector.cpp
    environment_collision_checker.cpp
    geometric_self_collision_checker.cpp
    point_cloud_collision_checker.cpp
    self_collision_checker.cpp
    voxel_grid.cpp
)

# Force FCL usage for safety monitor
list(APPEND COLLISION_SRCS fcl/fcl_collision_detector.cpp)
add_definitions(-DUSE_FCL)

add_library(collision_lib STATIC ${COLLISION_SRCS})
target_include_directories(collision_lib PUBLIC .)

target_link_libraries(collision_lib common_lib kinematics_lib robot_model_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(collision_lib 
        fcl 
        Eigen3 
        urdfdom
    )
else()
    target_link_libraries(collision_lib 
        ${FCL_LIBRARIES} 
        Eigen3::Eigen 
        ${urdfdom_LIBRARIES}
    )
endif()
