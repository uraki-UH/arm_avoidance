# 1. Robot Bridge Node
add_executable(robot_bridge_node robot_bridge_node.cpp)
target_link_libraries(robot_bridge_node core_safety)
ament_target_dependencies(robot_bridge_node rclcpp sensor_msgs std_msgs)

# 2. Joint State Mux Node
add_executable(joint_state_mux_node joint_state_mux_node.cpp)
target_link_libraries(joint_state_mux_node core_safety)
ament_target_dependencies(joint_state_mux_node rclcpp sensor_msgs)

# 3. TopoArm UDP Bridge
add_executable(topoarm_udp_bridge topoarm_udp_bridge.cpp)
target_link_libraries(topoarm_udp_bridge core_safety)
ament_target_dependencies(topoarm_udp_bridge rclcpp sensor_msgs)

# 4. ToPoFuzzy Bridge Node
add_executable(topofuzzy_bridge_node topofuzzy_bridge_node.cpp)
target_link_libraries(topofuzzy_bridge_node core_safety)
ament_target_dependencies(topofuzzy_bridge_node rclcpp sensor_msgs std_msgs geometry_msgs ais_gng_msgs)

# 5. Robot Viewer Bridge Node (Component)
add_library(robot_viewer_bridge_node_component SHARED robot_viewer_bridge_node.cpp)
target_link_libraries(robot_viewer_bridge_node_component description_lib kinematics_lib common_lib)
ament_target_dependencies(robot_viewer_bridge_node_component
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    ament_index_cpp
    nlohmann_json
)
rclcpp_components_register_nodes(robot_viewer_bridge_node_component "robot_sim::bridge::RobotViewerBridgeNode")

# 5b. Robot Viewer Bridge Node (Executable)
add_executable(robot_viewer_bridge_node robot_viewer_bridge_node_main.cpp)
target_link_libraries(robot_viewer_bridge_node robot_viewer_bridge_node_component)

# Installation
install(TARGETS
    robot_bridge_node
    joint_state_mux_node
    topoarm_udp_bridge
    topofuzzy_bridge_node
    robot_viewer_bridge_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    robot_viewer_bridge_node_component
    DESTINATION lib/${PROJECT_NAME}
)
