add_library(status_lib STATIC
    manipulability.cpp
    dynamic_manipulability.cpp
)
target_link_libraries(status_lib PUBLIC common_lib kinematics_lib gng_lib Eigen3::Eigen)
target_include_directories(status_lib PUBLIC .)
