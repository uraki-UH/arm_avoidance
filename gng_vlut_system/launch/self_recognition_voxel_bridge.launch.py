from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    return [
        Node(
            package="gng_vlut_system",
            executable="self_recognition_voxel_bridge_node",
            name="self_recognition_voxel_bridge_node",
            namespace=LaunchConfiguration("robot_name"),
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "occupied_voxels_topic": LaunchConfiguration("occupied_voxels_topic"),
                "danger_voxels_topic": LaunchConfiguration("danger_voxels_topic"),
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
            "occupied_voxels_topic",
            default_value="occupied_voxels",
        ),
        DeclareLaunchArgument(
            "danger_voxels_topic",
            default_value="danger_voxels",
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
