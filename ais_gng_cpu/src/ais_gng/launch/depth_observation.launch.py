"""既存depth録画からの画素番号付き点群とGNG観測範囲の別系統起動。"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def quaternion_product(first, second):
    x, y, z, w = first
    a, b, c, d = second
    return [w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a, w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c]


def setup(context):
    with Path(LaunchConfiguration('calibration_file').perform(context)).open() as stream:
        calibration = yaml.safe_load(stream)['/**']['ros__parameters']
    origin = [float(calibration[name]) for name in ('x', 'y', 'z')]
    roll, pitch, yaw = [math.radians(float(calibration[name]))/2 for name in ('roll', 'pitch', 'yaw')]
    # pointcloud_transformer_cppと同じRx*Ry*Rzの順序。
    rotation = quaternion_product(quaternion_product(
        [math.sin(roll), 0., 0., math.cos(roll)], [0., math.sin(pitch), 0., math.cos(pitch)]),
        [0., 0., math.sin(yaw), math.cos(yaw)])
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = '/' + namespace if namespace else ''
    target_frame = calibration['target_frame']
    return [
        Node(package='ais_gng', executable='depth_pixel_points.py', name='depth_pixel_points', namespace=namespace,
             output='screen', parameters=[{
                 'depth_topic': LaunchConfiguration('depth_topic').perform(context),
                 'camera_info_topic': LaunchConfiguration('camera_info_topic').perform(context),
                 'output_topic': prefix + '/points', 'output_camera_info_topic': prefix + '/camera_info',
                 'target_frame': target_frame, 'sensor_origin': origin, 'sensor_rotation': rotation,
             }]),
        Node(package='ais_gng', executable='ais_gng_cpu', name='depth_gng', namespace=namespace,
             output='screen', parameters=[{
                 'input.topic_names': [prefix + '/points'], 'input.point_cloud_num': 500000,
                 'input.local_coordinates': True, 'input.base_frame_id': target_frame,
                 'node.num_max': 20000, 'node.learning_num': 4000,
                 'node.enable_observation_support': True,
                 'input.observation_origin': origin, 'input.observation_origin_frame': target_frame,
                 'input.observation_camera_rotation': rotation,
                 'input.observation_camera_info_topic': prefix + '/camera_info',
                 'classify.human': False, 'classify.car': False,
                 'plane_cluster.direct_enabled': False, 'nonplane_component.direct_enabled': False,
             }]),
    ]


def generate_launch_description():
    calibration = str(Path(get_package_share_directory('pointcloud_transformer_cpp')) / 'config/realsense_calibration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=calibration),
        DeclareLaunchArgument('namespace', default_value='observation_depth'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/depth/camera_info'),
        OpaqueFunction(function=setup),
    ])
