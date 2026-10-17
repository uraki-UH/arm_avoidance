import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    glim_share_dir = get_package_share_directory("fuzzbot_slam")
    rviz_config_path = PathJoinSubstitution([FindPackageShare('fuzzbot_slam'),'rviz','glim.rviz'])

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path])

    launch_glim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([glim_share_dir + "/launch/glim.launch.py"])
    )

    nodes = [
        rviz2
    ]
    launch = [
        launch_glim
    ]
    
    return LaunchDescription(nodes+launch)