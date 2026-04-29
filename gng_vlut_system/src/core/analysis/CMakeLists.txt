# analysis_lib core has moved to safety_engine.
# robot_model_analyzer depends on simulation, so it stays here for now.
add_library(analysis_lib STATIC
    robot_model_analyzer.cpp
)
target_include_directories(analysis_lib PUBLIC .)
target_link_libraries(analysis_lib safety_engine common_lib kinematics_lib)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(analysis_lib Eigen3)
else()
    target_link_libraries(analysis_lib Eigen3::Eigen)
endif()
