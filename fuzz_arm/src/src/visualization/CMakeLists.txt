add_library(visualization_lib STATIC
    joint_angle_publisher.cpp
    voxel_wireframe_generator.cpp
)
target_link_libraries(visualization_lib PUBLIC common_lib status_lib Eigen3::Eigen)
target_include_directories(visualization_lib PUBLIC .)
