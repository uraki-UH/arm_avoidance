from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

package_dir = get_package_share_directory("ais_gng")

def generate_launch_description():
    declar_backend = DeclareLaunchArgument(
        'backend',
        default_value='cpu',
        description='backend to launch: cpu or gpu'
    )

    ais_gng_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_dir, 'launch', 'ais_gng.launch.py'])
        ),
        launch_arguments={
            'lidar': 'at128.yaml',
            'backend': LaunchConfiguration('backend'),
        }.items(),
    )

    map2lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=[
            '-12.0', '0.0', '6.283220',
            #  '-0.058535', '-0.216544', '0.0',
             '0.0', '0.216544', '0.058535',
            'map', 'hesai_lidar'
        ]
    )

    return LaunchDescription([
        declar_backend,
        ais_gng_launch,
        map2lidar_tf_node
    ])
