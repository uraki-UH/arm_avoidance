add_executable(voxel_status_test_publisher voxel_status_test_publisher.cpp)
ament_target_dependencies(voxel_status_test_publisher
    rclcpp
    std_msgs
    Eigen3
)
target_include_directories(voxel_status_test_publisher PRIVATE ${Eigen3_INCLUDE_DIRS})
install(TARGETS voxel_status_test_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(voxel_link_mask_validator voxel_link_mask_validator.cpp)
ament_target_dependencies(voxel_link_mask_validator
    rclcpp
    Eigen3
    tf2
    tf2_ros
    tf2_geometry_msgs
)
target_link_libraries(voxel_link_mask_validator ${LIBS_ALL_CORE} ${EXTRA_LIBS})
target_include_directories(voxel_link_mask_validator PRIVATE ${Eigen3_INCLUDE_DIRS})
install(TARGETS voxel_link_mask_validator DESTINATION lib/${PROJECT_NAME})

add_executable(voxel_spherizer_preview voxel_spherizer_preview.cpp)
ament_target_dependencies(voxel_spherizer_preview
    rclcpp
    ament_index_cpp
    Eigen3
)
target_link_libraries(voxel_spherizer_preview ${LIBS_ALL_CORE} ${EXTRA_LIBS})
target_include_directories(voxel_spherizer_preview PRIVATE ${Eigen3_INCLUDE_DIRS})
install(TARGETS voxel_spherizer_preview DESTINATION lib/${PROJECT_NAME})
