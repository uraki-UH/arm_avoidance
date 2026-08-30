from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_publisher_node(context):
    dataset_dir = LaunchConfiguration("dataset_dir").perform(context).rstrip("/")
    dataset_id = LaunchConfiguration("dataset_id").perform(context)
    dataset_path = f"{dataset_dir}/{dataset_id}_gng_template_v1.json.gz"
    return [Node(
        package="gng_vlut_system",
        executable="object_template_map_publisher_node",
        name="object_template_map_publisher_node",
        output="screen",
        parameters=[{
            "dataset_path": dataset_path,
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_hz": LaunchConfiguration("publish_hz"),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dataset_dir", default_value="/datasets"),
        DeclareLaunchArgument("dataset_id"),
        DeclareLaunchArgument("frame_id", default_value="object_template"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        OpaqueFunction(function=create_publisher_node),
    ])
