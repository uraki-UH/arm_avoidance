# 1. Robot Description Player
add_executable(robot_description_player robot_description_player.cpp)
ament_target_dependencies(robot_description_player
    rclcpp
    std_msgs
    ament_index_cpp
)
install(TARGETS robot_description_player DESTINATION lib/${PROJECT_NAME})

# 2. Safety Monitor Node
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

# 3. Self Recognition Viz Node
add_executable(self_recognition_viz_node self_recognition_viz_node.cpp)
target_link_libraries(self_recognition_viz_node
    core_safety
    kinematics_lib
    description_lib # For URDF loader/StlLoader
)
ament_target_dependencies(self_recognition_viz_node
    rclcpp
    sensor_msgs
    visualization_msgs
    Eigen3
)
install(TARGETS self_recognition_viz_node DESTINATION lib/${PROJECT_NAME})
