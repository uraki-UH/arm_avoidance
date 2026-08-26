from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument(
                "node_name", default_value="left_grasp_voxel_matcher"
            ),
            OpaqueFunction(function=_launch_matcher),
        ]
    )
