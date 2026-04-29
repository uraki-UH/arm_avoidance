# Safety Monitor Node (Component)
add_library(safety_monitor_node_component SHARED safety_monitor_node.cpp)
target_link_libraries(safety_monitor_node_component
    safety_engine
    collision_lib
    common_lib
)
ament_target_dependencies(safety_monitor_node_component
    rclcpp
    rclcpp_components
    sensor_msgs
    visualization_msgs
    ais_gng_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    Eigen3
)
rclcpp_components_register_nodes(safety_monitor_node_component "robot_sim::safety::SafetyMonitorNode")

# Safety Monitor Node (Executable)
add_executable(safety_monitor_node src/safety_monitor_node_main.cpp)
target_link_libraries(safety_monitor_node safety_monitor_node_component)

install(TARGETS 
    safety_monitor_node_component
    safety_monitor_node
    DESTINATION lib/${PROJECT_NAME}
)
