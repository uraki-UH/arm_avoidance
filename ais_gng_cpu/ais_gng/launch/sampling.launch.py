import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

package_dir = get_package_share_directory("ais_gng")
rviz_config_path = os.path.join(package_dir, "config", "rviz", "sampling.rviz")

def generate_launch_description():
    declar_lidar = DeclareLaunchArgument(
        "lidar",
        default_value="kimura.yaml",
        description="config",
    )
    declar_cluster_nodes_alpha = DeclareLaunchArgument(
        "cluster_nodes_alpha",
        default_value="0.2",
        description="visual.cluster_nodes.alpha for visualizar",
    )
    declar_cluster_nodes_scale = DeclareLaunchArgument(
        "cluster_nodes_scale",
        default_value="0.08",
        description="visual.cluster_nodes.scale for visualizar",
    )
    gng_config_path = PathJoinSubstitution([package_dir, "config", "gng", LaunchConfiguration("lidar")])

    visualizar = Node(
        package="ais_gng",
        executable="visualizar",
        parameters=[
            gng_config_path,
            {
                "visual.cluster_nodes.alpha": ParameterValue(LaunchConfiguration("cluster_nodes_alpha"), value_type=float),
                "visual.cluster_nodes.scale": ParameterValue(LaunchConfiguration("cluster_nodes_scale"), value_type=float),
            },
        ],
        output="screen"
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path]
    )
    
    return LaunchDescription([
        declar_lidar,
        declar_cluster_nodes_alpha,
        declar_cluster_nodes_scale,
        visualizar,
        rviz2
    ])
