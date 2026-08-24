from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pose_topic", default_value="/grasp_pose_cands"),
        DeclareLaunchArgument("score_topic", default_value="/grasp_pose_cand_scores"),
        DeclareLaunchArgument("marker_topic", default_value="/grasp_pose_markers"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("publish_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("candidate_count", default_value="1"),
        DeclareLaunchArgument("center_x", default_value="0.04"),
        DeclareLaunchArgument("center_y", default_value="0.0"),
        DeclareLaunchArgument("center_z", default_value="0.13"),
        DeclareLaunchArgument("spread_x", default_value="0.05"),
        DeclareLaunchArgument("spread_y", default_value="0.05"),
        DeclareLaunchArgument("spread_z", default_value="0.0"),
        DeclareLaunchArgument("base_yaw_deg", default_value="0.0"),
        Node(
            package="gng_vlut_system",
            executable="grasp_pose_dummy_publisher_node",
            name="grasp_pose_dummy_publisher_node",
            output="screen",
            parameters=[{
                "pose_topic": LaunchConfiguration("pose_topic"),
                "score_topic": LaunchConfiguration("score_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "candidate_count": LaunchConfiguration("candidate_count"),
                "center_x": LaunchConfiguration("center_x"),
                "center_y": LaunchConfiguration("center_y"),
                "center_z": LaunchConfiguration("center_z"),
                "spread_x": LaunchConfiguration("spread_x"),
                "spread_y": LaunchConfiguration("spread_y"),
                "spread_z": LaunchConfiguration("spread_z"),
                "base_yaw_deg": LaunchConfiguration("base_yaw_deg"),
            }],
        ),
    ])
