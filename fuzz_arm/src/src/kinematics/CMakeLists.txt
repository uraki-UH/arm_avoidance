add_library(kinematics_lib STATIC
    kinematic_chain.cpp
)
target_link_libraries(kinematics_lib PUBLIC common_lib Eigen3::Eigen)
target_include_directories(kinematics_lib PUBLIC .)
