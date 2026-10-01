"""機種選択と実URDFの整合、直動グリッパー、左右退避の検証。"""
import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
import numpy as np
import pytest
import yaml

workspace = Path(__file__).resolve().parents[2]
share = workspace / 'gng_vlut_system'
sys.path.insert(0, str(share / 'scripts'))
from dual_arm_avoidance_geometry import robot_geometry
from dual_arm_gazebo_demo import load_motion


@pytest.fixture
def launch_module(monkeypatch):
    spec = importlib.util.spec_from_file_location('control_launch', share / 'launch/dual_arm_control.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(share))
    return module


def make_context(robot, **overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        'robot': robot, 'params_file': '', 'demo_config': '', 'avoidance_config': '',
        'leader_topic': '/leader/joint_states', 'enable_keyboard': 'false',
        'udp_config': '', 'gui': 'false', 'allow_remote_udp': 'false',
        'gazebo_master_uri': 'http://127.0.0.1:11355', 'leader_mapping_file': '',
        **overrides})
    return context


@pytest.mark.parametrize('robot,robot_name,enable_gng', [
    ('topodualarm', 'ToPoDualArm', False),
    ('max', 'topo_dual_arm_max', True),
    ('max_long', 'topo_dual_arm_max_long', True)])
def test_selected_model_and_avoidance(launch_module, robot, robot_name, enable_gng):
    actions = launch_module.launch_setup(make_context(robot))
    include = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    args = dict(include.launch_arguments)
    params = yaml.safe_load(Path(args['params_file']).read_text())['/**']['ros__parameters']
    config = yaml.safe_load(Path(args['avoidance_config']).read_text())['dual_arm_avoidance_demo']
    assert params['robot_name'] == robot_name
    assert config['enable_gng_vlut'] is enable_gng
    assert args['enable_auto_start'] == 'false'
    assert args['udp_config'] == ''


def test_wrong_model_and_unvalidated_udp_rejected(launch_module):
    with pytest.raises(ValueError, match='機種名が不一致'):
        launch_module.launch_setup(make_context('topodualarm', params_file=str(share / 'config/topo_dual_arm_max.yaml')))
    with pytest.raises(ValueError, match='直動関節'):
        launch_module.launch_setup(make_context('topodualarm', udp_config='unused.yaml'))


def test_prismatic_gripper_poses():
    config = yaml.safe_load((share / 'config/topodualarm_gazebo_demo.yaml').read_text())['dual_arm_gazebo_demo']
    names, poses = load_motion(workspace / 'urdf/dual_arm_urdf/dual_arm_robot.urdf', config)
    assert len(names) == 19
    values = dict(poses)['grippers_open']
    assert values[names.index('L_gripper_joint')] == 0.02
    assert values[names.index('R_gripper_joint')] == 0.02


@pytest.mark.parametrize('sign', [1, -1])
def test_topodualarm_retreat_geometry(sign):
    config = yaml.safe_load((share / 'config/topodualarm_avoidance_demo.yaml').read_text())['dual_arm_avoidance_demo']
    geometry = robot_geometry(workspace / 'urdf/dual_arm_urdf/dual_arm_robot.urdf')
    home = np.zeros(len(geometry.joint_names))
    positions = home.copy()
    for x in np.linspace(config['hand_far_x'], config['hand_near_x'], 161):
        hand = np.array([x, sign * config['hand_y'], config['hand_z']])
        elbow = hand + [config['arm_length'], 0, 0]
        positions, has_candidate = geometry.choose_step(
            positions, home, hand, elbow, config['arm_radius'], config['target_clearance'], 0.035,
            config['min_planning_clearance_th'])
        assert has_candidate
        gap, centers, _, _ = geometry.clearance(positions, hand, elbow, config['arm_radius'])
        assert geometry.has_internal_clearance(centers, config['min_planning_clearance_th'])
        assert gap > config['min_clearance_th']
    assert geometry.clearance(home, hand, elbow, config['arm_radius'])[0] < 0
    assert max(abs(positions)) > 0.2
