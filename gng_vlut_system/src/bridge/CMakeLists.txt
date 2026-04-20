# 1. Robot Bridge Node
add_executable(robot_bridge_node robot_bridge_node.cpp)
target_link_libraries(robot_bridge_node
    core_safety
)
ament_target_dependencies(robot_bridge_node
    rclcpp
    sensor_msgs
    std_msgs
)
install(TARGETS robot_bridge_node DESTINATION lib/${PROJECT_NAME})

# 2. Joint State Mux Node
add_executable(joint_state_mux_node joint_state_mux_node.cpp)
target_link_libraries(joint_state_mux_node
    core_safety
)
ament_target_dependencies(joint_state_mux_node
    rclcpp
    sensor_msgs
)
install(TARGETS joint_state_mux_node DESTINATION lib/${PROJECT_NAME})

# 3. TopoArm Joint State Player
add_executable(topoarm_joint_state_player topoarm_joint_state_player.cpp)
target_link_libraries(topoarm_joint_state_player
    core_safety
)
ament_target_dependencies(topoarm_joint_state_player
    rclcpp
    sensor_msgs
)
install(TARGETS topoarm_joint_state_player DESTINATION lib/${PROJECT_NAME})
