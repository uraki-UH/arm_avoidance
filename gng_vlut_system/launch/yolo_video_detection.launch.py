import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    default_params_file = os.path.join(
        package_share, "config", "yolo_person_detection.yaml"
    )
    image_topic = LaunchConfiguration("image_topic")
    return LaunchDescription([
        DeclareLaunchArgument("video_path"),
        DeclareLaunchArgument("image_topic", default_value="/video/image_raw"),
        DeclareLaunchArgument("frame_id", default_value="video_optical_frame"),
        DeclareLaunchArgument("enable_loop", default_value="true"),
        DeclareLaunchArgument("publish_hz", default_value="0.0"),
        DeclareLaunchArgument("model_path", default_value="yolo11n.pt"),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        Node(
            package="gng_vlut_system",
            executable="video_file_publisher.py",
            name="video_file_publisher",
            output="screen",
            parameters=[{
                "video_path": LaunchConfiguration("video_path"),
                "image_topic": image_topic,
                "frame_id": LaunchConfiguration("frame_id"),
                "enable_loop": ParameterValue(
                    LaunchConfiguration("enable_loop"), value_type=bool
                ),
                "publish_hz": ParameterValue(
                    LaunchConfiguration("publish_hz"), value_type=float
                ),
            }],
        ),
        Node(
            package="gng_vlut_system",
            executable="yolo_person_detector.py",
            name="yolo_person_detector",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "image_topic": image_topic,
                    "model_path": LaunchConfiguration("model_path"),
                },
            ],
        ),
    ])
