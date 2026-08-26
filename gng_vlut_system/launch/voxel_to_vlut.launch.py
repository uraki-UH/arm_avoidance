from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    return [
        Node(
            package="gng_vlut_system",
            executable="voxel_to_vlut_node",
            name="voxel_to_vlut_node",
            namespace=LaunchConfiguration("robot_name"),
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "occupied_voxels_topic": "occupied_voxels",
                "danger_voxels_topic": "danger_voxels",
                "target_frame_id": LaunchConfiguration("target_frame_id"),
                "danger_inflation": LaunchConfiguration("danger_inflation"),
                "output_voxel_size": LaunchConfiguration("output_voxel_size"),
                "publish_hz": LaunchConfiguration("publish_hz"),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="ToPoDualArm",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="self_recognition/voxel_mask",
        ),
        DeclareLaunchArgument(
            "target_frame_id",
            default_value="",
        ),
        DeclareLaunchArgument(
            "danger_inflation",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "output_voxel_size",
            default_value="0.02",
        ),
        DeclareLaunchArgument(
            "publish_hz",
            default_value="30.0",
        ),
        OpaqueFunction(function=launch_setup),
    ])
