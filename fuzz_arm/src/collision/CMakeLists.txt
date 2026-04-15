# Collision Module
set(COLLISION_SRCS
    collision_detector.cpp
    environment_collision_checker.cpp
    geometric_self_collision_checker.cpp
    point_cloud_collision_checker.cpp
    self_collision_checker.cpp
)

if(USE_FCL)
    list(APPEND COLLISION_SRCS fcl/fcl_collision_detector.cpp)
endif()

if(USE_ODE)
    list(APPEND COLLISION_SRCS ode/ode_robot_collision_model.cpp)
endif()

add_library(collision_lib STATIC ${COLLISION_SRCS})
target_include_directories(collision_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(collision_lib  common_lib kinematics_lib description_lib)

if(USE_ODE)
    target_link_libraries(collision_lib  ${ODE_LIBRARY})
endif()

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
