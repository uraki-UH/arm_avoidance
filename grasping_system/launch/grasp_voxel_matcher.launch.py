import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_matcher(context):
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    parameters = [params_file] if params_file else []
    return [
        Node(
            package="grasping_system",
            executable="grasp_voxel_matcher_node",
            name=LaunchConfiguration("node_name"),
            output="screen",
            parameters=parameters,
        ),
        Node(
            package="gng_vlut_system",
            executable="grasp_pose_marker_bridge_node",
            name="grasp_pose_marker_bridge_node",
            output="screen",
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("grasping_system")
    graph_launch = os.path.join(
        package_share, "launch", "gripper_volume_graph.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument(
                "node_name", default_value="left_grasp_voxel_matcher"
            ),
            DeclareLaunchArgument(
                "enable_gripper_volume_graph", default_value="false"
            ),
            DeclareLaunchArgument("grippers_file", default_value=""),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("cache_directory", default_value=""),
            DeclareLaunchArgument("cache_mode", default_value="use"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(graph_launch),
                condition=IfCondition(
                    LaunchConfiguration("enable_gripper_volume_graph")
                ),
                launch_arguments={
                    "grippers_file": LaunchConfiguration("grippers_file"),
                    "tf_prefix": LaunchConfiguration("tf_prefix"),
                    "cache_directory": LaunchConfiguration("cache_directory"),
                    "cache_mode": LaunchConfiguration("cache_mode"),
                }.items(),
            ),
            OpaqueFunction(function=_launch_matcher),
        ]
    )
