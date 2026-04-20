# Safety Monitor Node
add_executable(safety_monitor_node safety_monitor_node.cpp)
target_link_libraries(safety_monitor_node
    core_safety
    collision_lib
)
ament_target_dependencies(safety_monitor_node
    rclcpp
    sensor_msgs
    visualization_msgs
    Eigen3
)
install(TARGETS safety_monitor_node DESTINATION lib/${PROJECT_NAME})
