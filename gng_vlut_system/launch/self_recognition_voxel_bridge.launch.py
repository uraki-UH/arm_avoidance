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
            default_value="topoarm_dual",
            description="ロボットの名前空間",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="self_recognition/voxel_mask",
            description="自己認識ボクセルの入力トピック",
        ),
        DeclareLaunchArgument(
            "occupied_voxels_topic",
            default_value="occupied_voxels",
            description="占有ボクセルトピック",
        ),
        DeclareLaunchArgument(
            "danger_voxels_topic",
            default_value="danger_voxels",
            description="危険ボクセルトピック",
        ),
        DeclareLaunchArgument(
            "danger_inflation",
            default_value="0.05",
            description="danger_voxels の外殻を膨張させる量 [m]",
        ),
        DeclareLaunchArgument(
            "output_voxel_size",
            default_value="0.02",
            description="occupied/danger voxel を再量子化する解像度 [m]",
        ),
        DeclareLaunchArgument(
            "publish_hz",
            default_value="30.0",
            description="再配信頻度 [Hz]",
        ),
        OpaqueFunction(function=launch_setup),
    ])
