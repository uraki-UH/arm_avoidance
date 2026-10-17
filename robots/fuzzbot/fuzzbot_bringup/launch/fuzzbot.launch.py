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

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

package_dir = get_package_share_directory("fuzzbot_bringup")
user_config_path = os.path.join(package_dir, "config", "lidar", "MID360_config.json")
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    dynamixel_handler_config_path    = PathJoinSubstitution([FindPackageShare("fuzzbot_bringup"), "config", "node", "config_dynamixel_handler.yaml"])

    FUZZBOT_MODEL = os.environ['FUZZBOT_MODEL']
    if FUZZBOT_MODEL == "fuzzbot_pro_normal":
        fuzzbot_urdf_name = 'fuzzbot_pro_normal.urdf.xacro'
        fuzzbot_urdf_path = PathJoinSubstitution([FindPackageShare('fuzzbot_description'), 'urdf', fuzzbot_urdf_name])
        fuzzbot_config_path = PathJoinSubstitution([FindPackageShare("fuzzbot_bringup"), "config", "node", "config_fuzzbot_pro.yaml"])
    elif FUZZBOT_MODEL == "fuzzbot_pro_tilt":
        fuzzbot_urdf_name = 'fuzzbot_pro_tilt.urdf.xacro'
        fuzzbot_urdf_path = PathJoinSubstitution([FindPackageShare('fuzzbot_description'), 'urdf', fuzzbot_urdf_name])
        fuzzbot_config_path = PathJoinSubstitution([FindPackageShare("fuzzbot_bringup"), "config", "node", "config_fuzzbot_pro.yaml"])

    prefix = ""
    use_sim = "false"
    use_fake_hardware = "false"
    fake_sensor_commands = "false"
    
    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            fuzzbot_urdf_path,
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'fake_sensor_commands:=',
            fake_sensor_commands,
        ]
    )

    node_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': False}],
        output='screen'
    )

    node_dynamixel_handler = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler',
        output='screen',
        parameters=[dynamixel_handler_config_path],
    )
    
    node_wheel_handler = Node(
        package='fuzzbot_hardware',
        executable='wheel_handler_node',
        output='screen',
        parameters=[fuzzbot_config_path],
    )

    node_livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )   

    nodes = [
        node_robot_state_pub,
        node_dynamixel_handler,
        node_wheel_handler,
        node_livox_driver
    ]
    
    return LaunchDescription(nodes)