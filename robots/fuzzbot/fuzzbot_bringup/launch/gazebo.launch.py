#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def is_valid_to_launch():
    # Path includes model name of Raspberry Pi series
    path = '/sys/firmware/devicetree/base/model'
    return not os.path.exists(path)


def generate_launch_description():
    if not is_valid_to_launch():
        print('Can not launch fake robot in Raspberry Pi')
        return LaunchDescription([])
    
    FUZZBOT_MODEL = os.environ['FUZZBOT_MODEL']
    fuzzbot_urdf_name = FUZZBOT_MODEL + '.urdf.xacro'
    entity_name = 'fuzzbot_system'

    # ---- LaunchConfigurations ----
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')

    world = LaunchConfiguration(
        'world',
        default=PathJoinSubstitution(
            [FindPackageShare('gazebo_ros'), 'worlds', 'empty.world']
        ),
    )

    x_pose = LaunchConfiguration('x_pose', default='0.00')
    y_pose = LaunchConfiguration('y_pose', default='0.00')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    roll = LaunchConfiguration('roll', default='0.00')
    pitch = LaunchConfiguration('pitch', default='0.00')
    yaw = LaunchConfiguration('yaw', default='0.00')

    # ---- robot_description (xacro -> urdf xml string) ----
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('fuzzbot_description'), 'urdf', fuzzbot_urdf_name]
            ),
            ' ',
            'prefix:=', prefix,
            ' ',
            'use_sim:=', use_sim,
        ]
    )

    # ---- Nodes (minimum) ----
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim}],
        output='screen',
    )

    # Optional RViz (debug)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('fuzzbot_bringup'), 'rviz', 'fuzzbot.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'verbose': 'false',
            'world': world,
        }.items(),
    )

    delete_entity = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/delete_entity',
            'gazebo_msgs/srv/DeleteEntity',
            f'{{name: "{entity_name}"}}',
        ],
        output='screen',
    )

    reset_world = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/reset_world',
            'std_srvs/srv/Empty',
            '{}',
        ],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fuzzbot_system',
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
            '-R', roll, '-P', pitch, '-Y', yaw,
        ],
        output='screen',
    )

    delayed_delete = TimerAction(
        period=1.0,
        actions=[delete_entity],
    )

    delayed_reset = TimerAction(
        period=1.5,
        actions=[reset_world],
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_entity],
    )

    # ---- Launch arguments ----
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='false', description='Whether execute rviz2'),
        DeclareLaunchArgument('prefix', default_value='""', description='Prefix of the joint and link names'),
        DeclareLaunchArgument('use_sim', default_value='true', description='Use simulation clock (/clock)'),
        DeclareLaunchArgument('world', default_value=world, description='Directory of gazebo world file'),
        DeclareLaunchArgument('x_pose', default_value=x_pose, description='Spawn position X'),
        DeclareLaunchArgument('y_pose', default_value=y_pose, description='Spawn position Y'),
        DeclareLaunchArgument('z_pose', default_value=z_pose, description='Spawn position Z'),
        DeclareLaunchArgument('roll', default_value=roll, description='Spawn orientation roll'),
        DeclareLaunchArgument('pitch', default_value=pitch, description='Spawn orientation pitch'),
        DeclareLaunchArgument('yaw', default_value=yaw, description='Spawn orientation yaw'),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_launch,
            robot_state_pub_node,
            delayed_delete,
            delayed_reset,
            delayed_spawn,
            rviz_node,
        ]
    )