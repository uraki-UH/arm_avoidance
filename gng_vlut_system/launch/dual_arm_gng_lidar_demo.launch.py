from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('gng_vlut_system'))
    defaults = {
        'params_file': str(share/'config/topo_dual_arm_max.yaml'),
        'demo_config': str(share/'config/dual_arm_gazebo_demo.yaml'),
        'avoidance_config': str(share/'config/dual_arm_gng_lidar_demo.yaml'),
        'point_cloud_source': 'external_lidar',
        'depth_camera_config': str(share/'config/dual_arm_depth_camera.yaml'),
        'gui': '',
        'enable_external_control': 'false',
        'enable_dynamixel_leader': 'false',
        'dynamixel_input_topic': '/dynamixel/state/present',
        'enable_auto_start': '',
        'gazebo_master_uri': 'http://127.0.0.1:11355',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/dual_arm_gazebo_demo.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
