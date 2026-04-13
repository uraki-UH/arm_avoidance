add_library(spatial_lib STATIC
    robot_model_analyzer.cpp
)
target_link_libraries(spatial_lib PUBLIC common_lib kinematics_lib Eigen3::Eigen)
target_include_directories(spatial_lib PUBLIC .)
