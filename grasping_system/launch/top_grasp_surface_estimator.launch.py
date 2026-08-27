from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("gng_vlut_system"), "config", "ToPoDualArm.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "plane_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ais_gng"), "config", "plane_cluster_incremental.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_plane_cluster", default_value="true"),
            DeclareLaunchArgument("topological_map_topic", default_value="/topological_map"),
            DeclareLaunchArgument(
                "planar_clusters_topic",
                default_value="/topological_planar_clusters_incremental",
            ),
            DeclareLaunchArgument(
                "candidate_topic", default_value="/top_grasp_pose_cands"
            ),
            DeclareLaunchArgument(
                "score_topic", default_value="/top_grasp_pose_cand_scores"
            ),
            DeclareLaunchArgument(
                "summary_topic", default_value="/top_grasp_pose_cands/summary"
            ),
            DeclareLaunchArgument(
                "marker_topic", default_value="/top_grasp_pose_markers"
            ),
            Node(
                package="ais_gng",
                executable="plane_cluster_incremental_node",
                name="plane_cluster_incremental_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_plane_cluster")),
                parameters=[
                    LaunchConfiguration("plane_params_file"),
                    {
                        "input_topic": LaunchConfiguration("topological_map_topic"),
                        "output_topic": LaunchConfiguration("planar_clusters_topic"),
                    },
                ],
            ),
            Node(
                package="grasping_system",
                executable="top_grasp_surface_estimator_node",
                name="top_grasp_surface_estimator",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "topological_map_topic": LaunchConfiguration(
                            "topological_map_topic"
                        ),
                        "planar_clusters_topic": LaunchConfiguration(
                            "planar_clusters_topic"
                        ),
                        "candidate_topic": LaunchConfiguration("candidate_topic"),
                        "score_topic": LaunchConfiguration("score_topic"),
                        "summary_topic": LaunchConfiguration("summary_topic"),
                    },
                ],
            ),
            Node(
                package="gng_vlut_system",
                executable="grasp_pose_marker_bridge_node",
                name="top_grasp_pose_marker_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("candidate_topic"),
                        "score_topic": LaunchConfiguration("score_topic"),
                        "output_topic": LaunchConfiguration("marker_topic"),
                        "marker_namespace": "top_grasp_pose",
                    }
                ],
            ),
        ]
    )
