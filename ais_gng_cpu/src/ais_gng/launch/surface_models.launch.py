from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/topological_map'),
        DeclareLaunchArgument('plane_clusters_topic', default_value='/plane_clusters'),
        DeclareLaunchArgument('output_topic', default_value='/curved_surface_clusters'),
        DeclareLaunchArgument('enable_markers', default_value='true'),
        DeclareLaunchArgument('enable_graph', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('ais_gng'), 'config', 'surface_model.yaml'])),
        Node(package='ais_gng', executable='plane_cluster_incremental_node',
             name='surface_model_node', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     'input_topic': LaunchConfiguration('input_topic'),
                     'clusters_input_topic': LaunchConfiguration('plane_clusters_topic'),
                     'surface_model.output_topic': LaunchConfiguration('output_topic'),
                     'surface_model.enable_markers': ParameterValue(LaunchConfiguration('enable_markers'), value_type=bool),
                     'surface_model.enable_graph': ParameterValue(LaunchConfiguration('enable_graph'), value_type=bool),
                     'enable_plane_markers': False,
                     'enable_nonplane_markers': False,
                 }]),
    ])
