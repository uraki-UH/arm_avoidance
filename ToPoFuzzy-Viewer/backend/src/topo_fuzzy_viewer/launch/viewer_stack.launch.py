from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_port',
            default_value='9001',
            description='WebSocket port for the viewer gateway'
        ),
        DeclareLaunchArgument(
            'pointcloud_max_points',
            default_value='100000',
            description='Maximum number of uniformly sampled points sent per viewer frame'
        ),
        DeclareLaunchArgument(
            'pointcloud_max_hz',
            default_value='10.0',
            description='Maximum PointCloud2 forwarding rate for the viewer'
        ),
        DeclareLaunchArgument(
            'websocket_max_backpressure_bytes',
            default_value='8388608',
            description='WebSocket send queue safety limit in bytes'
        ),
        Node(
            package='topo_fuzzy_viewer',
            executable='viewer_ws_gateway_node',
            name='viewer_ws_gateway_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('ws_port'),
                'pointcloud_max_points': LaunchConfiguration('pointcloud_max_points'),
                'pointcloud_max_hz': LaunchConfiguration('pointcloud_max_hz'),
                'websocket_max_backpressure_bytes': LaunchConfiguration(
                    'websocket_max_backpressure_bytes'
                )
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
