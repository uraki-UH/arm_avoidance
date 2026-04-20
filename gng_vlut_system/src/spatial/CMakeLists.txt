# spatial_lib core has moved to core_safety.
# robot_model_analyzer depends on simulation, so it stays here for now.
add_library(spatial_lib STATIC
    robot_model_analyzer.cpp
)
target_include_directories(spatial_lib PUBLIC .)
target_link_libraries(spatial_lib core_safety common_lib kinematics_lib)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(spatial_lib Eigen3)
else()
    target_link_libraries(spatial_lib Eigen3::Eigen)
endif()
