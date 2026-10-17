import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    glim_config_path = PathJoinSubstitution([FindPackageShare('fuzzbot_slam'),'config','glim'])

    node_glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        parameters=[{'config_path': glim_config_path}],
        output='screen'
    )

    FUZZBOT_MODEL = os.environ['FUZZBOT_MODEL']
    if FUZZBOT_MODEL == 'fuzzbot_pro_normal' or FUZZBOT_MODEL == 'fuzzbot_pro_tilt':
        tf_args = ['0.0', '0.0', '0.215', '0.0', '0.0', '0.0', 'map', 'glim_map']
    else:
        tf_args = ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'glim_map']
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=tf_args,
    )

    nodes = [
        node_glim,
        static_transform_publisher,
    ]

    return LaunchDescription(nodes)