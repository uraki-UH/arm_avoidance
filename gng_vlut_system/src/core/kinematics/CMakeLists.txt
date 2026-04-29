add_library(kinematics_lib STATIC
    kinematic_chain.cpp
)
target_include_directories(kinematics_lib PUBLIC .)
target_link_libraries(kinematics_lib common_lib)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(kinematics_lib Eigen3)
else()
    target_link_libraries(kinematics_lib Eigen3::Eigen)
endif()
