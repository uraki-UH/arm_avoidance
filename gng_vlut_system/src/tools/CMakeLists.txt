add_executable(voxel_status_test_publisher voxel_status_test_publisher.cpp)
ament_target_dependencies(voxel_status_test_publisher
    rclcpp
    std_msgs
    Eigen3
)
target_include_directories(voxel_status_test_publisher PRIVATE ${Eigen3_INCLUDE_DIRS})
install(TARGETS voxel_status_test_publisher DESTINATION lib/${PROJECT_NAME})
