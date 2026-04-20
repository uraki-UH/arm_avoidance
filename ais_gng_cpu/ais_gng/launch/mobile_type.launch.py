import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

package_dir = get_package_share_directory("ais_gng")
user_config_path = os.path.join(package_dir, "config", "lidar", "MID360_config.json")
glim_config_path = PathJoinSubstitution([FindPackageShare('ais_gng'),'config', 'glim'])
gng_config_path = PathJoinSubstitution([package_dir, 'config', 'gng', 'glim.yaml'])
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
    map2lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=[
            '0.0', '0.0', '0.0', # x, y, z
            '0.0', '0.0', '0.0', # roll, pitch, yaw (in radians)
            'map', 'glim'      # parent_frame, child_frame
        ]
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
    )    

    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        parameters=[{'config_path': glim_config_path}],
        output='screen',
        remappings=[('/glim_ros/points', '/scan')],
    )

    ais_gng = Node(
        package="ais_gng",
        executable="ais_gng",
        parameters=[gng_config_path],
        output='screen',
    )

    return LaunchDescription([
        map2lidar_tf_node,
        livox_driver,
        glim_node,
        ais_gng,
    ])