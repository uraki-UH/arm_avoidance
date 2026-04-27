from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gng_topics',
            default_value=['/viewer/internal/stream/graph', '/topological_map', '/topological_map_transformed'],
            description='List of GNG topics to subscribe to'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_ws_gateway_node',
            name='viewer_ws_gateway_node',
            output='screen',
            parameters=[{
                'gng_topics': LaunchConfiguration('gng_topics')
            }]
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_source_node',
            name='viewer_source_node',
            output='screen'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_file_node',
            name='viewer_file_node',
            output='screen'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_rosbag_node',
            name='viewer_rosbag_node',
            output='screen'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_gng_node',
            name='viewer_gng_node',
            output='screen'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_param_node',
            name='viewer_param_node',
            output='screen'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_edit_node',
            name='viewer_edit_node',
            output='screen'
        ),
    ])
