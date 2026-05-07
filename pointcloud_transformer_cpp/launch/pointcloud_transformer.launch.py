import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Correct way: find the configuration file relative to the package installation
    package_dir = get_package_share_directory('pointcloud_transformer_cpp')
    default_config_path = os.path.join(package_dir, 'config', 'realsense_calibration.yaml')

    return LaunchDescription([
        # Allow overriding parameters via launch arguments
        DeclareLaunchArgument('config_file', default_value=default_config_path),
        DeclareLaunchArgument('input_topic', default_value='/camera/camera/depth/color/points'),
        DeclareLaunchArgument('output_topic', default_value='/camera/transformed_points'),

        Node(
            package='pointcloud_transformer_cpp',
            executable='pointcloud_transformer_node',
            name='pointcloud_transformer',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                }
            ],
            output='screen'
        )
    ])
