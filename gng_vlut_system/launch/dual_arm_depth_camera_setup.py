"""RealSense取付リンクに追従する仮想深度カメラの生成。"""
import math
from pathlib import Path
import xml.etree.ElementTree as et

import yaml


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict) or not isinstance(config.get('head_depth_camera'), dict):
        raise ValueError('head_depth_camera設定が必要です')
    config = config['head_depth_camera']
    required = {'width', 'height', 'horizontal_fov', 'update_hz', 'min_range', 'max_range', 'xyz', 'rpy'}
    if set(config) != required:
        raise ValueError('深度カメラ設定の不足または未知の項目があります')
    for name in ('width', 'height'):
        if type(config[name]) is not int or not 2 <= config[name] <= 4096:
            raise ValueError(name+'は2〜4096の整数が必要です')
    for name in ('horizontal_fov', 'update_hz', 'min_range', 'max_range'):
        value = config[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(name+'は有限の正数が必要です')
    if config['horizontal_fov'] >= math.pi or config['min_range'] >= config['max_range']:
        raise ValueError('画角または測距範囲が不正です')
    for name in ('xyz', 'rpy'):
        value = config[name]
        if (not isinstance(value, list) or len(value) != 3 or
                any(type(item) not in (int, float) or not math.isfinite(item) for item in value)):
            raise ValueError(name+'は有限数値3個の配列が必要です')
    return config


def add_depth_camera(root, namespace, config):
    """取付補正用リンク・光学TF・RGB/深度/点群の追加。元URDFは不変。"""
    links = {link.get('name') for link in root.findall('link')}
    if 'camera_link' not in links or {'head_depth_link', 'head_depth_optical_frame'} & links:
        raise ValueError('camera_linkの欠落または深度カメラリンクの重複')
    for child, parent, xyz, rpy in (
            ('head_depth_link', 'camera_link', config['xyz'], config['rpy']),
            ('head_depth_optical_frame', 'head_depth_link', [0, 0, 0], [-math.pi/2, 0, -math.pi/2])):
        et.SubElement(root, 'link', name=child)
        joint = et.SubElement(root, 'joint', name=child+'_fixed', type='fixed')
        et.SubElement(joint, 'parent', link=parent)
        et.SubElement(joint, 'child', link=child)
        et.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
    gazebo = et.SubElement(root, 'gazebo', reference='head_depth_link')
    sensor = et.SubElement(gazebo, 'sensor', name='head_depth_camera', type='depth')
    et.SubElement(sensor, 'always_on').text = 'true'
    et.SubElement(sensor, 'visualize').text = 'false'
    et.SubElement(sensor, 'update_rate').text = str(config['update_hz'])
    camera = et.SubElement(sensor, 'camera', name='head_depth_camera')
    et.SubElement(camera, 'horizontal_fov').text = str(config['horizontal_fov'])
    image = et.SubElement(camera, 'image')
    for name in ('width', 'height'):
        et.SubElement(image, name).text = str(config[name])
    et.SubElement(image, 'format').text = 'R8G8B8'
    clip = et.SubElement(camera, 'clip')
    et.SubElement(clip, 'near').text = str(config['min_range'])
    et.SubElement(clip, 'far').text = str(config['max_range'])
    plugin = et.SubElement(sensor, 'plugin', name='head_depth_camera_ros', filename='libgazebo_ros_camera.so')
    ros = et.SubElement(plugin, 'ros')
    et.SubElement(ros, 'namespace').text = '/'+namespace
    for source, target in (('camera/image_raw', 'camera/color/image_raw'),
                           ('camera/camera_info', 'camera/color/camera_info'),
                           ('camera/points', 'camera/depth/points')):
        et.SubElement(ros, 'remapping').text = source+':='+target
    et.SubElement(plugin, 'camera_name').text = 'camera'
    et.SubElement(plugin, 'frame_name').text = namespace+'/head_depth_optical_frame'
    et.SubElement(plugin, 'min_depth').text = str(config['min_range'])
    et.SubElement(plugin, 'max_depth').text = str(config['max_range'])
