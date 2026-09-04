from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("profile_name", default_value="left_arm"),
        DeclareLaunchArgument("output_path", default_value="reachability_voxel_map.bin"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("max_sample_count", default_value="200000"),
        DeclareLaunchArgument("max_no_new_voxel_samples", default_value="50000"),
        Node(
            package="gng_vlut_system",
            executable="reachability_voxel_builder",
            name="reachability_voxel_builder",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "reachability_voxel.profile_name": LaunchConfiguration("profile_name"),
                    "reachability_voxel.output_path": LaunchConfiguration("output_path"),
                    "reachability_voxel.voxel_size": LaunchConfiguration("voxel_size"),
                    "reachability_voxel.max_sample_count": LaunchConfiguration("max_sample_count"),
                    "reachability_voxel.max_no_new_voxel_samples": LaunchConfiguration("max_no_new_voxel_samples"),
                },
            ],
        ),
    ])
