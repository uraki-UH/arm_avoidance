"""頭部深度センサーの取付・光学座標・設定拒否・入力選択の回帰。"""
import importlib.util
import math
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as et

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml


package = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, package/'launch'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


depth = load('dual_arm_depth_camera_setup')


@pytest.mark.parametrize('name', ['dual_arm_control.launch', 'dual_arm_gazebo_demo.launch'])
def test_launch_description_without_process_start(name):
    """ノード未起動での引数宣言と深度設定パスの検査。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    launch = load(name)
    with patch.object(launch, 'get_package_share_directory', return_value=str(package)):
        description = launch.generate_launch_description()
    context = LaunchContext()
    declarations = {item.name: item for item in description.entities if isinstance(item, DeclareLaunchArgument)}
    assert context.perform_substitution(declarations['point_cloud_source'].default_value[0]) == 'external_lidar'
    assert context.perform_substitution(declarations['depth_camera_config'].default_value[0]) == str(package/'config/dual_arm_depth_camera.yaml')


@pytest.fixture
def config():
    return depth.load_config(package/'config/dual_arm_depth_camera.yaml')


@pytest.mark.parametrize('model', ['topo_dual_arm_max', 'topo_dual_arm_max_long'])
def test_mount_and_optical_axes(model, config):
    root = et.parse(package.parent/'urdf'/model/'topo_dual_arm_max.urdf').getroot()
    depth.add_depth_camera(root, 'sim_'+model, config)
    joint = root.find("joint[@name='head_depth_link_fixed']")
    assert joint.find('parent').get('link') == 'camera_link'
    assert joint.find('origin').get('xyz') == '0.0 0.0 0.0'
    optical = root.find("joint[@name='head_depth_optical_frame_fixed']/origin")
    rotation = Rotation.from_euler('xyz', list(map(float, optical.get('rpy').split()))).as_matrix()
    np.testing.assert_allclose(rotation@[0, 0, 1], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(rotation@[1, 0, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(rotation@[0, 1, 0], [0, 0, -1], atol=1e-12)
    sensor = root.find("gazebo[@reference='head_depth_link']/sensor")
    assert sensor.get('type') == 'depth'
    assert sensor.findtext('plugin/frame_name') == 'sim_'+model+'/head_depth_optical_frame'
    assert sensor.findtext('plugin/min_depth') == str(config['min_range'])
    assert sensor.findtext('plugin/max_depth') == str(config['max_range'])
    assert sensor.findtext('plugin/ros/namespace') == '/sim_'+model
    assert 'camera/points:=camera/depth/points' in [x.text for x in sensor.findall('plugin/ros/remapping')]


def test_calibration_and_duplicate_rejection(config):
    root = et.fromstring('<robot><link name="camera_link"/></robot>')
    config.update(xyz=[0.01, 0.02, 0.03], rpy=[0.1, 0.2, 0.3])
    depth.add_depth_camera(root, 'sim_test', config)
    origin = root.find("joint[@name='head_depth_link_fixed']/origin")
    assert origin.get('xyz') == '0.01 0.02 0.03'
    assert origin.get('rpy') == '0.1 0.2 0.3'
    with pytest.raises(ValueError):
        depth.add_depth_camera(root, 'sim_test', config)
    with pytest.raises(ValueError):
        depth.add_depth_camera(et.fromstring('<robot/>'), 'sim_test', config)


@pytest.mark.parametrize('name,value', [
    ('width', True), ('height', 1), ('width', 4097), ('height', 12.5),
    ('update_hz', 0), ('update_hz', math.nan), ('min_range', -1), ('max_range', 0.05),
    ('horizontal_fov', math.pi), ('horizontal_fov', math.inf), ('xyz', [0, 0]),
    ('rpy', [0, math.nan, 0]), ('xyz', [False, 0, 0]), ('unknown', 1)])
def test_invalid_config(tmp_path, config, name, value):
    config[name] = value
    path = tmp_path/'camera.yaml'
    path.write_text(yaml.safe_dump({'head_depth_camera': config}))
    with pytest.raises(ValueError):
        depth.load_config(path)


@pytest.mark.parametrize('content', ['', '[]', 'head_depth_camera: []', 'head_depth_camera: {}'])
def test_missing_config(tmp_path, content):
    path = tmp_path/'camera.yaml'
    path.write_text(content)
    with pytest.raises(ValueError):
        depth.load_config(path)


@pytest.mark.parametrize('source', ['external_lidar', 'head_depth'])
def test_pipeline_selection(tmp_path, source):
    pipeline = load('dual_arm_lidar_setup')
    model_dir = tmp_path/'model'
    model_dir.mkdir()
    for name in ('gng.bin', 'vlut.bin'):
        (model_dir/name).touch()
    params = {'gng': {'data_directory': str(tmp_path), 'experiment_id': 'model'}}
    with patch.object(pipeline, 'Node', side_effect=lambda **kwargs: kwargs):
        nodes = pipeline.pipeline_nodes('params.yaml', params, 'sim_test', {'danger_inflation': 0.12}, source)
    assert sum(n['executable'] == 'static_transform_publisher' for n in nodes) == int(source == 'external_lidar')
    voxel = next(n for n in nodes if n['executable'] == 'world_index_to_voxel_node')['parameters'][-1]
    assert voxel['input_topic'] == ('camera/depth/points' if source == 'head_depth' else 'lidar_points')
    assert voxel['allow_latest_transform'] == (source == 'external_lidar')
    with pytest.raises(ValueError):
        pipeline.point_cloud_topic('unknown')
