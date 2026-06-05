from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    if not robot_name:
        robot_name = "ToPoDualArm"

    return [
        Node(
            package="gng_vlut_system",
            executable="joint_state_mux_node",
            name="joint_state_mux_node",
            namespace=robot_name,
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        OpaqueFunction(function=launch_setup),
    ])
