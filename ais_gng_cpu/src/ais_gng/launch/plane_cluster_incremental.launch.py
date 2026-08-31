"""増分方式の平面クラスタ生成ノードを起動する。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ais_gng"), "config", "plane_cluster_incremental.yaml"]
        ),
        description="増分方式のしきい値をまとめた設定ファイル。",
    )
    input_topic = DeclareLaunchArgument("input_topic", default_value="/topological_map")
    output_topic = DeclareLaunchArgument(
        "output_topic", default_value="/topological_plane_clusters_incremental"
    )
    return LaunchDescription(
        [
            params_file,
            input_topic,
            output_topic,
            Node(
                package="ais_gng",
                executable="plane_cluster_incremental_node",
                name="plane_cluster_incremental_node",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    },
                ],
                output="screen",
            ),
        ]
    )
