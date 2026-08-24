from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/grasp_pose_cands"),
        DeclareLaunchArgument("output_topic", default_value="/grasp_pose_markers"),
        DeclareLaunchArgument("marker_namespace", default_value="grasp_pose"),
        DeclareLaunchArgument("arrow_length", default_value="0.12"),
        DeclareLaunchArgument("shaft_diameter", default_value="0.006"),
        DeclareLaunchArgument("head_diameter", default_value="0.012"),
        DeclareLaunchArgument("color_r", default_value="0.0"),
        DeclareLaunchArgument("color_g", default_value="0.8"),
        DeclareLaunchArgument("color_b", default_value="0.2"),
        DeclareLaunchArgument("color_a", default_value="1.0"),
        Node(
            package="gng_vlut_system",
            executable="grasp_pose_marker_bridge_node",
            name="grasp_pose_marker_bridge_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "marker_namespace": LaunchConfiguration("marker_namespace"),
                "arrow_length": LaunchConfiguration("arrow_length"),
                "shaft_diameter": LaunchConfiguration("shaft_diameter"),
                "head_diameter": LaunchConfiguration("head_diameter"),
                "color_r": LaunchConfiguration("color_r"),
                "color_g": LaunchConfiguration("color_g"),
                "color_b": LaunchConfiguration("color_b"),
                "color_a": LaunchConfiguration("color_a"),
            }],
        ),
    ])
