# 1. Robot Description Player
add_executable(robot_description_player robot_description_player.cpp)
ament_target_dependencies(robot_description_player
    rclcpp
    std_msgs
    ament_index_cpp
)
install(TARGETS robot_description_player DESTINATION lib/${PROJECT_NAME})

# 2. Self Recognition Viz Node
add_executable(self_recognition_viz_node self_recognition_viz_node.cpp)
target_link_libraries(self_recognition_viz_node
    core_safety
    kinematics_lib
    description_lib
)
ament_target_dependencies(self_recognition_viz_node
    rclcpp
    sensor_msgs
    visualization_msgs
    Eigen3
)
install(TARGETS self_recognition_viz_node DESTINATION lib/${PROJECT_NAME})
