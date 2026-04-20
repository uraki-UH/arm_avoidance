import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

frame_id = 'livox_frame'

package_dir = get_package_share_directory("ais_gng")

def generate_launch_description():
    declar_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='kimura.yaml',
        description='config'
    )
    declar_use_reverse = DeclareLaunchArgument(
        'use_reverse_z',
        default_value='false',
        description='enable reverse z pointcloud node'
    )
    declar_reverse_input = DeclareLaunchArgument(
        'reverse_input_topic',
        default_value='scan',
        description='input topic for reverse z node'
    )
    declar_reverse_output = DeclareLaunchArgument(
        'reverse_output_topic',
        default_value='scan/reversed_z',
        description='output topic for reverse z node'
    )
    declar_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='scan topic used by ais_gng'
    )
    declar_use_cluster_box_node = DeclareLaunchArgument(
        'use_cluster_box_node',
        default_value='true',
        description='enable cluster summary/box node'
    )
    declar_cluster_summary_topic = DeclareLaunchArgument(
        'cluster_summary_topic',
        default_value='/fuzzy_classifier/cluster_summary',
        description='output topic for cluster summary json'
    )
    declar_cluster_box_topic = DeclareLaunchArgument(
        'cluster_box_topic',
        default_value='/fuzzy_classifier/cluster_boxes',
        description='output MarkerArray topic for cluster boxes'
    )
    declar_cluster_box_alpha = DeclareLaunchArgument(
        'cluster_box_alpha',
        default_value='0.35',
        description='cluster box alpha [0.0-1.0]'
    )
    declar_render_safe_terrain_box = DeclareLaunchArgument(
        'render_safe_terrain_box',
        default_value='false',
        description='render SAFE_TERRAIN clusters as boxes'
    )
    
    gng_config_path = PathJoinSubstitution([package_dir, 'config', 'gng', LaunchConfiguration('lidar')])

    reverse_z = Node(
        package="ais_gng",
        executable="reverse_z_pointcloud",
        parameters=[
            {"input_topic": LaunchConfiguration('reverse_input_topic')},
            {"output_topic": LaunchConfiguration('reverse_output_topic')},
        ],
        condition=IfCondition(LaunchConfiguration('use_reverse_z')),
        output="screen",
    )

    ais_gng_without_reverse = Node(
            package="ais_gng",
            executable="ais_gng_node",
            parameters=[gng_config_path, {'use_sim_time': True}],
            remappings=[('scan', LaunchConfiguration('scan_topic'))],
            condition=UnlessCondition(LaunchConfiguration('use_reverse_z')),
            output='screen',
        )

    ais_gng_with_reverse = Node(
            package="ais_gng",
            executable="ais_gng_node",
            parameters=[gng_config_path, {'use_sim_time': True}],
            remappings=[('scan', LaunchConfiguration('reverse_output_topic'))],
            condition=IfCondition(LaunchConfiguration('use_reverse_z')),
            output='screen',
        )
    
    map2lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=[
            '0.0', '0.0', '0.0',         # x, y, z
            '0.0',' 0.0', '0.0',          # roll, pitch, yaw (in radians)
            #  'base_frame', frame_id  # parent_frame, child_frame
            'camera_depth_optical_frame', frame_id
        ]
    )

    cluster_summary_node = Node(
        package='ais_gng',
        executable='cluster_summary_node',
        parameters=[
            {
                'use_sim_time': True,
                'output_topic': LaunchConfiguration('cluster_summary_topic'),
                'box_topic': LaunchConfiguration('cluster_box_topic'),
                'box_alpha': ParameterValue(LaunchConfiguration('cluster_box_alpha'), value_type=float),
                'publish_cluster_boxes': True,
                'render_safe_terrain_box': ParameterValue(LaunchConfiguration('render_safe_terrain_box'), value_type=bool),
            }
        ],
        condition=IfCondition(LaunchConfiguration('use_cluster_box_node')),
        output='screen',
    )

    return LaunchDescription([
        declar_lidar,
        declar_use_reverse,
        declar_reverse_input,
        declar_reverse_output,
        declar_scan_topic,
        declar_use_cluster_box_node,
        declar_cluster_summary_topic,
        declar_cluster_box_topic,
        declar_cluster_box_alpha,
        declar_render_safe_terrain_box,
        map2lidar_tf_node,
        reverse_z,
        ais_gng_without_reverse,
        ais_gng_with_reverse,
        cluster_summary_node,
    ])
