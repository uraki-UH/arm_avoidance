add_library(control_lib STATIC
    trajectory_safety_manager.cpp
)
target_link_libraries(control_lib PUBLIC common_lib kinematics_lib gng_lib Eigen3::Eigen)
target_include_directories(control_lib PUBLIC .)
