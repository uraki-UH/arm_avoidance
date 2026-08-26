import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Correct way: find the configuration file relative to the package installation
    package_dir = get_package_share_directory('pointcloud_transformer_cpp')
    default_config_path = os.path.join(package_dir, 'config', 'realsense_calibration.yaml')

    return LaunchDescription([
        # Allow overriding parameters via launch arguments
        DeclareLaunchArgument('config_file', default_value=default_config_path),
        DeclareLaunchArgument('input_topic', default_value='/camera/camera/depth/color/points'),
        DeclareLaunchArgument('output_topic', default_value='/camera/transformed_points'),
        DeclareLaunchArgument('input_queue_depth', default_value='1'),
        DeclareLaunchArgument('reliable_input', default_value='false'),

        Node(
            package='pointcloud_transformer_cpp',
            executable='pointcloud_transformer_node',
            name='pointcloud_transformer',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                    'input_queue_depth': ParameterValue(
                        LaunchConfiguration('input_queue_depth'), value_type=int
                    ),
                    'reliable_input': ParameterValue(
                        LaunchConfiguration('reliable_input'), value_type=bool
                    ),
                }
            ],
            output='screen'
        )
    ])
