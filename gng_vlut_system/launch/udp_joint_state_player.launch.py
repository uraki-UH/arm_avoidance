from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    udp_port = DeclareLaunchArgument(
        "udp_port",
        default_value="12345",
        description="UDP port that feeds the topoarm joint state player",
    )

    # frame_id will be passed from parent launch, so it needs to be declared here
    # or assumed to be available from the context of the including launch file.
    # For now, let's assume it's passed.
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="world", # Default value for standalone use
        description="Frame id for the joint state player",
    )

    topoarm_joint_state_player = Node(
        package="gng_safety",
        executable="topoarm_joint_state_player",
        name="topoarm_joint_state_player",
        output="screen",
        parameters=[{
            "udp_port": LaunchConfiguration("udp_port"),
            "base_frame": LaunchConfiguration("frame_id"),
            "output_topic": "/joint_states_external",
        }],
    )

    return LaunchDescription([
        udp_port,
        frame_id, # Declare frame_id here as well
        topoarm_joint_state_player,
    ])
