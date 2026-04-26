# 1. Robot Description Player
add_library(robot_description_player_component SHARED robot_description_player.cpp)
target_link_libraries(robot_description_player_component common_lib)
ament_target_dependencies(robot_description_player_component
    rclcpp
    rclcpp_components
    std_msgs
    ament_index_cpp
)
rclcpp_components_register_nodes(robot_description_player_component "robot_sim::self_recognition::RobotDescriptionPlayer")

# 2. Self Recognition Viz Node
add_library(self_recognition_viz_node_component SHARED self_recognition_viz_node.cpp)
target_link_libraries(self_recognition_viz_node_component
    core_safety
    kinematics_lib
    description_lib
    common_lib
)
ament_target_dependencies(self_recognition_viz_node_component
    rclcpp
    rclcpp_components
    sensor_msgs
    visualization_msgs
    Eigen3
)
rclcpp_components_register_nodes(self_recognition_viz_node_component "robot_sim::self_recognition::SelfRecognitionVizNode")

# 3. Executables (for backwards compatibility or standalone use)
add_executable(robot_description_player_node src/robot_description_player_main.cpp)
target_link_libraries(robot_description_player_node robot_description_player_component)

add_executable(self_recognition_viz_node src/self_recognition_viz_node_main.cpp)
target_link_libraries(self_recognition_viz_node self_recognition_viz_node_component)

install(TARGETS 
    robot_description_player_component
    self_recognition_viz_node_component
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
    robot_description_player_node
    self_recognition_viz_node
    DESTINATION lib/${PROJECT_NAME}
)
