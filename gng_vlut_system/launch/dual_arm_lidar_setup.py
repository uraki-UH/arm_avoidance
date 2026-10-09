"""Gazeboの外置きLiDAR／頭部深度点群と自己除去・VLUT配信の構成。"""
from pathlib import Path
import xml.etree.ElementTree as et

from launch_ros.actions import Node


def pipeline_config(config):
    """旧デモ互換の既定値と機体別センサ設定。"""
    return config.get('pipeline', {
        'base_frame': 'base_link', 'points_topic': 'lidar_points', 'enable_lidar': True,
        'voxel_size': 0.02, 'publish_hz': 10.0,
        'roi_min': [-0.6, -0.9, 0.04], 'roi_max': [0.9, 0.9, 1.0],
        'lidar': {'pose': [0.85, 0, 0.75, 0, 0.4, 3.141592653589793], 'update_hz': 10,
                  'horizontal_samples': 180, 'vertical_samples': 72,
                  'horizontal_angle': 1.0, 'vertical_angle': 0.6,
                  'min_range': 0.07, 'max_range': 2.5, 'range_resolution': 0.005}})


def add_lidar(world, namespace, config=None):
    pipeline = pipeline_config(config or {})
    if not pipeline['enable_lidar']:
        return
    settings = pipeline['lidar']
    model = et.SubElement(world, 'model', name='avoidance_lidar')
    et.SubElement(model, 'static').text = 'true'
    et.SubElement(model, 'pose').text = ' '.join(map(str, settings['pose']))
    link = et.SubElement(model, 'link', name='sensor_link')
    sensor = et.SubElement(link, 'sensor', name='lidar', type='ray')
    et.SubElement(sensor, 'always_on').text = 'true'
    et.SubElement(sensor, 'update_rate').text = str(settings['update_hz'])
    ray = et.SubElement(sensor, 'ray')
    scan = et.SubElement(ray, 'scan')
    for name in ('horizontal', 'vertical'):
        count, angle = settings[name + '_samples'], settings[name + '_angle']
        axis = et.SubElement(scan, name)
        for key, value in [('samples', count), ('resolution', 1), ('min_angle', -angle), ('max_angle', angle)]:
            et.SubElement(axis, key).text = str(value)
    scan_range = et.SubElement(ray, 'range')
    for key, value in [('min', settings['min_range']), ('max', settings['max_range']), ('resolution', settings['range_resolution'])]:
        et.SubElement(scan_range, key).text = str(value)
    plugin = et.SubElement(sensor, 'plugin', name='lidar_ros', filename='libgazebo_ros_ray_sensor.so')
    ros = et.SubElement(plugin, 'ros')
    et.SubElement(ros, 'namespace').text = '/'+namespace
    et.SubElement(ros, 'remapping').text = '~/out:=' + pipeline['points_topic']
    et.SubElement(plugin, 'output_type').text = 'sensor_msgs/PointCloud2'
    et.SubElement(plugin, 'frame_name').text = namespace+'/lidar'


def point_cloud_topic(source):
    if source not in ('external_lidar', 'head_depth'):
        raise ValueError('point_cloud_sourceはexternal_lidarまたはhead_depthが必要です')
    return 'camera/depth/points' if source == 'head_depth' else 'lidar_points'



def simulation_self_node(params_path, namespace, pipeline):
    """実点群の除去マスクとは独立した、Gazebo実測姿勢の自己ボクセル。"""
    base_frame = pipeline['base_frame']
    return Node(package='gng_vlut_system', executable='self_recognition_viz_node',
            namespace=namespace, output='screen', parameters=[str(params_path), {
                'use_sim_time': True, 'joint_topic': '/'+namespace+'/joint_states',
                'max_joint_state_age_sec': 0.5,
                'self_recognition.resolution': pipeline['voxel_size'],
                'self_recognition.root_link': base_frame,
                'self_recognition.target_frame_id': base_frame,
                'self_recognition.marker_frame_id': base_frame,
                'self_recognition.mask_topic': '/'+namespace+'/self_voxel'}])


def pipeline_nodes(params_path, params, namespace, config, point_cloud_source='external_lidar'):
    input_topic = point_cloud_topic(point_cloud_source)
    pipeline = pipeline_config(config)
    if 'external_environment' in pipeline:
        return [simulation_self_node(params_path, namespace, pipeline)]
    if point_cloud_source == 'external_lidar':
        input_topic = pipeline['points_topic']
    base_frame = pipeline['base_frame']
    base = namespace+'/'+base_frame
    voxel_size, publish_hz = pipeline['voxel_size'], pipeline['publish_hz']
    result_dir = Path(params['gng']['data_directory'])/params['gng']['experiment_id']
    gng_filename, vlut_filename = (params['gng'].get('gng_model_filename', 'gng.bin'),
                                   params['gng'].get('vlut_filename', 'vlut.bin'))
    for filename in (gng_filename, vlut_filename):
        if not (result_dir/filename).is_file():
            raise FileNotFoundError(f'先に対象機種のGNG/VLUT学習が必要です: {result_dir/filename}')
    def node(executable, extra, enable_yaml=False, node_namespace=None):
        values = [str(params_path)] if enable_yaml else []
        return Node(package='gng_vlut_system', executable=executable, namespace=node_namespace or namespace,
                    output='screen', parameters=values+[{'use_sim_time': True, **extra}])
    actions = []
    if pipeline['enable_lidar'] and point_cloud_source == 'external_lidar':
        pose_args = [item for key, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), pipeline['lidar']['pose'])
                     for item in ('--'+key, str(value))]
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
            arguments=pose_args+['--frame-id', 'world', '--child-frame-id', namespace+'/lidar'],
            parameters=[{'use_sim_time': True}]))
    roi = {direction + '_reachability_' + axis: value
           for direction in ('min', 'max') for axis, value in zip('xyz', pipeline['roi_'+direction])}
    if 'pipeline' in config:
        # 共通構成のROIは指定範囲そのもの。ノード既定の追加余白を無効化
        roi.update({f'reachability_margin_{axis}': 0.0 for axis in 'xyz'})
    if 'external_cloud' in pipeline:
        actions.append(node('external_pointcloud_bridge.py', {
            **pipeline['external_cloud'], 'output_topic': pipeline['points_topic'],
            'target_frame': base, 'use_sim_time': False, 'max_publish_hz': publish_hz,
            'urdf_path': params['urdf_path'], 'real_root_frame': namespace+'/real/'+base_frame}))
    is_external = 'external_cloud' in pipeline
    self_mask_topic = '/'+namespace+('/real/self_voxel' if is_external else '/self_voxel')
    return actions + [
        node('world_index_to_voxel_node', {
            'input_topic': input_topic,
            'allow_latest_transform': point_cloud_source == 'external_lidar',
            'output_topic': 'roi_voxels',
            'world_frame_id': base, 'target_frame_id': base,
            'enable_world_index': False, 'enable_roi_query': False, 'enable_world_bucket_publish': False,
            'voxel_size': voxel_size, 'enable_reachability_filter': True, **roi}),
        node('self_recognition_viz_node', {
            'joint_topic': '/'+namespace+'/real_joint_states' if is_external else 'joint_states',
            'self_recognition.resolution': voxel_size,
            'max_joint_state_age_sec': 0.5 if is_external else 0.0,
            'self_recognition.root_link': base_frame,
            'self_recognition.target_frame_id': '/'+base if is_external else base_frame,
            'self_recognition.marker_frame_id': '/'+base if is_external else base_frame,
            'self_recognition.mask_topic': self_mask_topic}, True,
            node_namespace=namespace+'/real' if is_external else namespace),
        node('self_voxel_filter_node', {'self_recognition.enable_environment_self_filter': True,
            'self_recognition.mask_topic': self_mask_topic,
            'self_recognition.raw_environment_voxel_topic': 'roi_voxels',
            'self_recognition.filtered_environment_voxel_topic': 'self_filter_roi_voxels'}, True),
        node('voxel_to_vlut_node', {
            'input_topic': 'self_filter_roi_voxels',
            'occupied_voxels_topic': 'occupied_voxels',
            'danger_voxels_topic': 'danger_voxels', 'target_frame_id': base,
            'output_voxel_size': voxel_size, 'danger_inflation': float(config['danger_inflation']), 'publish_hz': publish_hz}),
        node('topofuzzy_bridge_node', {
            'gng_model_path': str(result_dir/gng_filename), 'vlut_path': str(result_dir/vlut_filename),
            'frame_id': base_frame, 'source_frame_id': base_frame, 'topic_name': 'Tmap_static',
            'node_feature_topic': 'topological_node_features', 'node_state_topic': 'gng_node_states', 'edge_mode': 0, 'publish_hz': publish_hz,
            'occupied_voxels_topic': 'occupied_voxels', 'danger_voxels_topic': 'danger_voxels',
            'visualization_gng.enabled': False}, True),
    ] + ([simulation_self_node(params_path, namespace, pipeline)] if is_external else [])
