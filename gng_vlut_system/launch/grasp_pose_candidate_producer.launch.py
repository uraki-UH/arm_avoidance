from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PARAMETER_TYPES = {
    "input_topic": str,
    "pose_topic": str,
    "target_frame_id": str,
    "enable_input_frame_passthrough": bool,
    "voxel_size": float,
    "x_shift": int,
    "y_shift": int,
    "z_shift": int,
    "offset": int,
    "max_candidates": int,
    "min_points_per_voxel": int,
    "approach_offset": float,
}


def _launch_setup(context):
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, value_type in _PARAMETER_TYPES.items()
    }
    return [Node(
        package="gng_vlut_system",
        executable="grasp_pose_candidate_producer_node",
        name="grasp_pose_candidate_producer_node",
        output="screen",
        parameters=[parameters],
    )]


def generate_launch_description():
    defaults = {
        "input_topic": "/topo_points",
        "pose_topic": "/grasp_pose_cands",
        "target_frame_id": "world",
        "enable_input_frame_passthrough": "false",
        "voxel_size": "0.01",
        "x_shift": "42",
        "y_shift": "21",
        "z_shift": "0",
        "offset": "1000000",
        "max_candidates": "256",
        "min_points_per_voxel": "1",
        "approach_offset": "0.06",
    }
    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in defaults.items()
        ],
        OpaqueFunction(function=_launch_setup),
    ])
