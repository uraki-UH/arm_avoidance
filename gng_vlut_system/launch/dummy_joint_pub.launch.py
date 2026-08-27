"""URDF可動関節由来のダミー姿勢配信launch。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """ダミーpublisher引数構築。"""
    del args, kwargs

    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context).strip("/")
    is_static = LaunchConfiguration("is_static").perform(context).lower() in {
        "1", "true", "yes", "on"}

    topic_prefix = f"/{robot_name}" if robot_name else ""
    publisher_args = [
        "--urdf", urdf_path,
        "--topic", f"{topic_prefix}/dummy_joint_commands",
        "--claim-topic", f"{topic_prefix}/control_claims",
    ]
    if is_static:
        publisher_args.append("--static")

    return [
        Node(
            package="gng_vlut_system",
            executable="dummy_joint_pub.py",
            name="dummy_joint_publisher",
            output="screen",
            arguments=publisher_args,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_path",
            default_value="/ros2_ws/src/dual_arm_urdf/dual_arm_robot.urdf"),
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("is_static", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
