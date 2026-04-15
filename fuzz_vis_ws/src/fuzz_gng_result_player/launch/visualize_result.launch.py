import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("fuzz_gng_result_player")
    rviz_config_path = os.path.join(package_dir, "config", "gng_result_player.rviz")

    experiment_id = DeclareLaunchArgument(
        "experiment_id",
        default_value="topoarm_real_v1",
        description="Experiment id used under gng_results/<experiment_id>",
    )
    data_directory = DeclareLaunchArgument(
        "data_directory",
        default_value="gng_results",
        description="Directory that contains offline_urdf_trainer outputs",
    )
    phase2_suffix = DeclareLaunchArgument(
        "phase2_suffix",
        default_value="_phase2",
        description="Suffix used for the saved GNG binary",
    )
    config_file = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Optional config.txt path with key=value overrides",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="camera_color_optical_frame",
        description="Frame id used in TopologicalMap header",
    )
    poll_ms = DeclareLaunchArgument(
        "poll_ms",
        default_value="250",
        description="Polling interval for reloading the result file",
    )
    publish_edges = DeclareLaunchArgument(
        "publish_edges",
        default_value="false",
        description="Whether to show GNG edges in RViz",
    )
    republish_ms = DeclareLaunchArgument(
        "republish_ms",
        default_value="1000",
        description="How often to republish the latest MarkerArray",
    )

    result_player = Node(
        package="fuzz_gng_result_player",
        executable="gng_result_player",
        name="gng_result_player",
        output="screen",
        parameters=[{
            "experiment_id": LaunchConfiguration("experiment_id"),
            "data_directory": LaunchConfiguration("data_directory"),
            "phase2_suffix": LaunchConfiguration("phase2_suffix"),
            "config_file": LaunchConfiguration("config_file"),
            "frame_id": LaunchConfiguration("frame_id"),
            "poll_ms": LaunchConfiguration("poll_ms"),
            "publish_edges": LaunchConfiguration("publish_edges"),
            "republish_ms": LaunchConfiguration("republish_ms"),
        }],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path, "-f", LaunchConfiguration("frame_id")],
        output="screen",
    )

    return LaunchDescription([
        experiment_id,
        data_directory,
        phase2_suffix,
        config_file,
        frame_id,
        poll_ms,
        publish_edges,
        republish_ms,
        result_player,
        rviz2,
    ])
