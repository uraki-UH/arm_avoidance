add_library(gng_lib STATIC
    GrowingNeuralGas_offline.cpp
)
target_include_directories(gng_lib PUBLIC .)
target_link_libraries(gng_lib PUBLIC common_lib kinematics_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(gng_lib Eigen3)
else()
    target_link_libraries(gng_lib PUBLIC Eigen3::Eigen)
endif()
