"""Max・Longの下垂姿勢・左右グリッパー・明示設定優先の検証。"""
import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as et
from unittest.mock import patch

import numpy as np
import pytest
import yaml

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'gng_vlut_system/scripts'))
from joint_command_model import joint_command_model, dynamixel_mapping
from dual_arm_avoidance_geometry import read_joint_geometry, compile_link_operations


def test_follower_gripper_signs_mimic_and_round_trip():
    model = joint_command_model(root/'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf')
    config = yaml.safe_load((root/'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_max_ids_31_52.yaml').read_text())['/**']['ros__parameters']
    mapping = dynamixel_mapping(model, config)
    assert mapping.entries['R_gripper_joint'] == (38, 1., 0.)
    assert mapping.entries['L_gripper_joint'] == (48, -1., 0.)
    ids, angles = mapping.convert({'R_gripper_joint': math.radians(30.), 'L_gripper_joint': math.radians(20.),
                                   'R_joint2': math.pi/2, 'L_joint2': -math.pi/2})
    assert dict(zip(ids, angles)) == pytest.approx({32: 90., 38: 30., 42: -90., 48: -20.})
    assert model.expand({'R_gripper_joint': .3, 'L_gripper_joint': .2})['R_gripper_mimic'] == -.3


@pytest.mark.parametrize('model_path', ['source.urdf', 'models/standard/source.urdf'])
def test_corrected_motor_rest_pose_points_both_arms_down(model_path):
    path = root/'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/app'/model_path
    urdf = et.parse(path).getroot()
    joints, names, _, parents = read_joint_geometry(urdf)
    _, link_indices, operations = compile_link_operations(urdf, joints, names, parents)
    config = yaml.safe_load((root/'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_max_ids_31_52.yaml').read_text())['/**']['ros__parameters']
    values = dict.fromkeys(names, 0.)
    for name, motor_deg in [('R_joint2', 90.), ('L_joint2', -90.)]:
        idx = config['joint_names'].index(name)
        values[name] = math.radians(motor_deg+config['joint_offsets_deg'][idx])*config['joint_scales'][idx]
    transforms = np.empty((len(operations)+1, 4, 4))
    transforms[0] = np.eye(4)
    for parent, child, kind, fixed, axis, idx, multiplier, offset, cross, square in operations:
        motion = np.eye(4)
        if idx >= 0:
            angle = values[names[idx]]*multiplier+offset
            if kind in ('revolute', 'continuous'):
                motion[:3, :3] += np.sin(angle)*cross+(1-np.cos(angle))*square
            elif kind == 'prismatic':
                motion[:3, 3] = axis*angle
        transforms[child] = transforms[parent]@fixed@motion
    for side in ('R', 'L'):
        delta = transforms[link_indices[side+'_tcp'], :3, 3]-transforms[link_indices[side+'_link2'], :3, 3]
        assert delta[2] < -.3
        assert np.linalg.norm(delta[:2]) < .03


@pytest.mark.parametrize('robot_name,expected', [
    ('topo_dual_arm_max_long', 'dynamixel_joint_state_bridge_max_ids_31_52.yaml'),
    ('topo_dual_arm_max', 'dynamixel_joint_state_bridge_max_ids_31_52.yaml'),
    ('ToPoDualArm', 'dynamixel_joint_state_bridge_ids_31_52.yaml')])
def test_current_pose_selects_model_mapping(tmp_path, robot_name, expected):
    launch = pytest.importorskip('launch')
    path = root/'gng_vlut_system/launch/dynamixel_current_pose.launch.py'
    spec = importlib.util.spec_from_file_location('current_pose_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    params = tmp_path/'robot.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_name': robot_name}}}))
    context = launch.LaunchContext()
    context.launch_configurations.update(params_file=str(params), mapping_file='', input_topic='/fixture/present',
        output_topic='', enable_viewer_output='true', enable_reader='false', enable_viewer='false')
    with patch.object(module, 'get_package_share_directory', side_effect=lambda name: str(root/name)), patch.object(module, 'Node') as node:
        module.launch_setup(context)
        assert Path(node.call_args.kwargs['parameters'][0]).name == expected
        context.launch_configurations['mapping_file'] = str(root/'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_ids_31_52.yaml')
        module.launch_setup(context)
        assert node.call_args.kwargs['parameters'][0] == context.launch_configurations['mapping_file']
