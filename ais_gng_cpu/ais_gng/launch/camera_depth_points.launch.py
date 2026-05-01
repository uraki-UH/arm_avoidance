from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("ais_gng")
    gng_config_path = PathJoinSubstitution([package_dir, 'config', 'gng', LaunchConfiguration('lidar')])

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/camera/camera/depth/color/points',
        description='camera point cloud topic used by ais_gng',
    )
    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value='kimura.yaml',
        description='GNG parameter yaml under config/gng',
    )
    target_frame_id_arg = DeclareLaunchArgument(
        'target_frame_id',
        default_value='base_link',
        description='target frame for TF transformation (e.g. base_link, map)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use simulation time',
    )

    ais_gng = Node(
        package='ais_gng',
        executable='ais_gng_node',
        name='ais_gng_camera_node',
        output='screen',
        parameters=[
            gng_config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input.base_frame_id': LaunchConfiguration('target_frame_id'),
            },
        ],
        remappings=[('scan', LaunchConfiguration('scan_topic'))],
    )

    return LaunchDescription([
        scan_topic_arg,
        lidar_arg,
        target_frame_id_arg,
        use_sim_time_arg,
        ais_gng,
    ])
