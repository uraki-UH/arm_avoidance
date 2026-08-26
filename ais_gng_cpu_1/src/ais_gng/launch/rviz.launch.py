import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

package_dir = get_package_share_directory("ais_gng")
rviz_config_path = os.path.join(package_dir, "config", "rviz", "ais_gng.rviz")

def generate_launch_description():
    visualizar = Node(
        package="ais_gng",
        executable="visualizar",
        #output="screen",
        parameters=[{"marker_size": 0.05}, {"node_alpha": 0.0}],
        # remappings=[("/topological_map", "/topological_map/merged")],
        remappings=[("/topological_map", "/topological_map/merged")],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path, '--ros-args', '--log-level', 'WARN'],
    )
    
    return LaunchDescription([
        visualizar,
        rviz2
    ])