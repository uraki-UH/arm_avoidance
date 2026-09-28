"""Gazeboの実レイ点群と既存の自己除去・VLUT配信の構成。"""
from pathlib import Path
import xml.etree.ElementTree as et

from launch_ros.actions import Node


def add_lidar(world, namespace):
    model = et.SubElement(world, 'model', name='avoidance_lidar')
    et.SubElement(model, 'static').text = 'true'
    et.SubElement(model, 'pose').text = '0.85 0 0.75 0 0.4 3.141592653589793'
    link = et.SubElement(model, 'link', name='sensor_link')
    sensor = et.SubElement(link, 'sensor', name='lidar', type='ray')
    et.SubElement(sensor, 'always_on').text = 'true'
    et.SubElement(sensor, 'update_rate').text = '10'
    ray = et.SubElement(sensor, 'ray')
    scan = et.SubElement(ray, 'scan')
    for name, count, angle in [('horizontal', 180, 1.0), ('vertical', 72, 0.6)]:
        axis = et.SubElement(scan, name)
        for key, value in [('samples', count), ('resolution', 1), ('min_angle', -angle), ('max_angle', angle)]:
            et.SubElement(axis, key).text = str(value)
    scan_range = et.SubElement(ray, 'range')
    for key, value in [('min', 0.07), ('max', 2.5), ('resolution', 0.005)]:
        et.SubElement(scan_range, key).text = str(value)
    plugin = et.SubElement(sensor, 'plugin', name='lidar_ros', filename='libgazebo_ros_ray_sensor.so')
    ros = et.SubElement(plugin, 'ros')
    et.SubElement(ros, 'namespace').text = '/'+namespace
    et.SubElement(ros, 'remapping').text = '~/out:=lidar_points'
    et.SubElement(plugin, 'output_type').text = 'sensor_msgs/PointCloud2'
    et.SubElement(plugin, 'frame_name').text = namespace+'/lidar'


def pipeline_nodes(params_path, params, namespace, config):
    base = namespace+'/base_link'
    result_dir = Path(params['gng']['data_directory'])/params['gng']['experiment_id']
    for filename in ('gng.bin', 'vlut.bin'):
        if not (result_dir/filename).is_file():
            raise FileNotFoundError(f'先に対象機種のGNG/VLUT学習が必要です: {result_dir/filename}')
    def node(executable, extra, enable_yaml=False):
        values = [str(params_path)] if enable_yaml else []
        return Node(package='gng_vlut_system', executable=executable, namespace=namespace,
                    output='screen', parameters=values+[{'use_sim_time': True, **extra}])
    return [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '.85', '--z', '.75', '--pitch', '.4', '--yaw', '3.141592653589793',
                        '--frame-id', 'world', '--child-frame-id', namespace+'/lidar'],
             parameters=[{'use_sim_time': True}]),
        node('world_index_to_voxel_node', {
            'input_topic': 'lidar_points', 'output_topic': 'roi_voxels',
            'world_frame_id': base, 'target_frame_id': base,
            'enable_world_index': False, 'enable_roi_query': False, 'enable_world_bucket_publish': False,
            'voxel_size': 0.02, 'enable_reachability_filter': True,
            'min_reachability_x': -0.6, 'max_reachability_x': 0.9,
            'min_reachability_y': -0.9, 'max_reachability_y': 0.9,
            'min_reachability_z': 0.04, 'max_reachability_z': 1.0}),
        node('self_recognition_viz_node', {
            'joint_topic': 'joint_states', 'self_recognition.resolution': 0.02,
            'self_recognition.target_frame_id': 'base_link', 'self_recognition.marker_frame_id': 'base_link',
            'self_recognition.mask_topic': 'self_voxel'}, True),
        node('self_voxel_filter_node', {}, True),
        node('voxel_to_vlut_node', {
            'input_topic': 'self_filter_roi_voxels', 'occupied_voxels_topic': 'occupied_voxels',
            'danger_voxels_topic': 'danger_voxels', 'target_frame_id': base,
            'output_voxel_size': 0.02, 'danger_inflation': float(config['danger_inflation']), 'publish_hz': 10.0}),
        node('topofuzzy_bridge_node', {
            'gng_model_path': str(result_dir/'gng.bin'), 'vlut_path': str(result_dir/'vlut.bin'),
            'frame_id': 'base_link', 'source_frame_id': 'base_link', 'topic_name': 'Tmap_static',
            'node_feature_topic': 'topological_node_features', 'node_state_topic': 'gng_node_states', 'edge_mode': 0, 'publish_hz': 10.0,
            'occupied_voxels_topic': 'occupied_voxels', 'danger_voxels_topic': 'danger_voxels',
            'visualization_gng.enabled': False}, True),
    ]
