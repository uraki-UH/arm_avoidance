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

# 2. Executables (for backwards compatibility or standalone use)
add_executable(robot_description_player_node src/robot_description_player_main.cpp)
target_link_libraries(robot_description_player_node robot_description_player_component)

install(TARGETS 
    robot_description_player_component
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
    robot_description_player_node
    DESTINATION lib/${PROJECT_NAME}
)
