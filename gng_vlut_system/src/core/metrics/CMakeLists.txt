add_library(metrics_lib STATIC
    manipulability.cpp
    dynamic_manipulability.cpp
)
target_include_directories(metrics_lib PUBLIC .)
target_link_libraries(metrics_lib common_lib kinematics_lib safety_engine)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(metrics_lib Eigen3)
else()
    target_link_libraries(metrics_lib Eigen3::Eigen)
endif()
