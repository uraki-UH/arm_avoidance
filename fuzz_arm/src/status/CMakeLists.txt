add_library(status_lib STATIC
    manipulability.cpp
    dynamic_manipulability.cpp
)
target_include_directories(status_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(status_lib  common_lib kinematics_lib gng_lib)
if(BUILD_WITH_ROS2)
    ament_target_dependencies(status_lib Eigen3)
else()
    target_link_libraries(status_lib  Eigen3::Eigen)
endif()
