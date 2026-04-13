add_library(collision_lib STATIC
    collision_detector.cpp
    environment_collision_checker.cpp
    geometric_self_collision_checker.cpp
    point_cloud_collision_checker.cpp
    self_collision_checker.cpp
    ode/ode_robot_collision_model.cpp
)
if(USE_FCL)
    target_sources(collision_lib PRIVATE fcl/fcl_collision_detector.cpp)
    target_link_libraries(collision_lib PUBLIC ${FCL_LIBRARIES})
endif()
target_link_libraries(collision_lib PUBLIC common_lib kinematics_lib description_lib ${ODE_LIBRARY} Eigen3::Eigen)
target_include_directories(collision_lib PUBLIC .)
