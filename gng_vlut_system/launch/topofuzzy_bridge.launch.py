import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml")),
        DeclareLaunchArgument("gng_model_path", default_value=""),
        DeclareLaunchArgument("vlut_path", default_value=""),
        DeclareLaunchArgument("publish_hz", default_value="20.0"),
        DeclareLaunchArgument("edge_mode", default_value="-1"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("occupied_voxels_topic", default_value="/occupied_voxels"),
        DeclareLaunchArgument("danger_voxels_topic", default_value="/danger_voxels"),
        
        Node(
            package="gng_vlut_system",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "gng_model_path": LaunchConfiguration("gng_model_path"),
                    "vlut_path": LaunchConfiguration("vlut_path"),
                    "publish_hz": LaunchConfiguration("publish_hz"),
                    "edge_mode": LaunchConfiguration("edge_mode"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "occupied_voxels_topic": LaunchConfiguration("occupied_voxels_topic"),
                    "danger_voxels_topic": LaunchConfiguration("danger_voxels_topic"),
                },
            ],
        )
    ])
