# Planner is currently a header-only module
add_library(planner_lib INTERFACE)
target_include_directories(planner_lib INTERFACE .)
target_link_libraries(planner_lib INTERFACE common_lib kinematics_lib collision_lib)

if(BUILD_WITH_ROS2)
    ament_target_dependencies(planner_lib Eigen3)
else()
    target_link_libraries(planner_lib INTERFACE Eigen3::Eigen)
endif()
