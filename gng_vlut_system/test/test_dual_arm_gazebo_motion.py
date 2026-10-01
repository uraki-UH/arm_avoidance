"""実URDFに対するデモ姿勢の検査。"""
import copy
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as et

import pytest
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('dual_arm_gazebo_demo', WORKSPACE/'gng_vlut_system/scripts/dual_arm_gazebo_demo.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def gazebo_launch():
    path = WORKSPACE/'gng_vlut_system/launch/dual_arm_gazebo_demo.launch.py'
    spec = importlib.util.spec_from_file_location('gazebo_launch', path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    return launch


@pytest.mark.parametrize('model,filename', [
    ('topo_dual_arm_max', 'topo_dual_arm_max.urdf'),
    ('topo_dual_arm_max_long', 'topo_dual_arm_max.urdf'),
    ('dual_arm_urdf', 'dual_arm_robot.urdf'),
])
def test_control_builder_preserves_urdf_and_joint_order(gazebo_launch, model, filename):
    root = et.parse(WORKSPACE/'urdf'/model/filename).getroot()
    before = et.tostring(root)
    config = {'motor_position_gain': 30.0, 'motor_limit_scale': 0.8}
    positions = gazebo_launch.initial_joint_positions(root, {'L_joint1': 0.2})
    control, names = gazebo_launch.build_ros2_control(root, config, positions)
    joints = [joint for joint in root.findall('joint') if joint.get('type') != 'fixed']
    assert et.tostring(root) == before
    assert names == [joint.get('name') for joint in joints if joint.find('mimic') is None]
    assert control.findtext('hardware/plugin') == 'gng_vlut_system/bounded_gazebo_system'
    assert [joint.get('name') for joint in control.findall('joint')] == [joint.get('name') for joint in joints]
    for source, actual in zip(joints, control.findall('joint')):
        assert actual.find('command_interface').get('name') == 'position'
        assert actual.findtext("param[@name='position_gain']") == '30.0'
        assert actual.findtext("param[@name='motor_limit_scale']") == '0.8'
        assert float(actual.findtext("state_interface[@name='position']/param")) == positions[source.get('name')]
        assert [item.get('name') for item in actual.findall('state_interface')] == ['position', 'velocity', 'effort']
        mimic = source.find('mimic')
        if mimic is not None:
            assert actual.findtext("param[@name='mimic']") == mimic.get('joint')
            assert actual.findtext("param[@name='multiplier']") == mimic.get('multiplier', '1')


@pytest.mark.parametrize('enable_integrated_control,rate', [(False, 50.0), (True, 100.0)])
def test_controller_builder_preserves_closed_loop_settings(gazebo_launch, enable_integrated_control, rate):
    names = ['shoulder', 'gripper']
    result = gazebo_launch.build_controllers('sim_test', names, enable_integrated_control)
    manager = result['/sim_test/controller_manager']['ros__parameters']
    assert manager['use_sim_time'] is True
    assert manager['update_rate'] == 1000
    assert manager['joint_state_broadcaster']['type'] == 'joint_state_broadcaster/JointStateBroadcaster'
    assert manager['dual_arm_controller']['type'] == 'joint_trajectory_controller/JointTrajectoryController'
    controller = result['/sim_test/dual_arm_controller']['ros__parameters']
    assert controller == {
        'joints': names, 'command_interfaces': ['position'], 'state_interfaces': ['position', 'velocity'],
        'state_publish_rate': rate, 'action_monitor_rate': 20.0,
        'allow_partial_joints_goal': False, 'open_loop_control': False,
        'constraints': {'goal_time': 2.0, 'stopped_velocity_tolerance': 0.05}}
    assert controller['joints'] is not names


def test_control_plugin_namespace_and_parameters(gazebo_launch):
    gazebo = gazebo_launch.build_control_plugin('sim_test', Path('/tmp/controllers.yaml'))
    plugin = gazebo.find('plugin')
    assert plugin.get('filename') == 'libgazebo_ros2_control.so'
    assert plugin.findtext('ros/namespace') == '/sim_test'
    assert plugin.findtext('ros/remapping') == '/joint_states:=/sim_test/joint_states'
    assert plugin.findtext('robot_param_node') == '/sim_test/robot_state_publisher'
    assert plugin.findtext('robot_param') == 'robot_description'
    assert plugin.findtext('parameters') == '/tmp/controllers.yaml'


@pytest.mark.parametrize('enable_live_obstacles', [False, True])
@pytest.mark.parametrize('enable_gng_vlut', [False, True])
@pytest.mark.parametrize('physics_solver', ['quick', 'world'])
def test_world_builder_isolates_obstacle_and_sensor(gazebo_launch, enable_live_obstacles, enable_gng_vlut, physics_solver):
    config = yaml.safe_load((WORKSPACE/'gng_vlut_system/config/dual_arm_gng_lidar_demo.yaml').read_text())['dual_arm_avoidance_demo']
    config.update(enable_live_obstacles=enable_live_obstacles, enable_gng_vlut=enable_gng_vlut, physics_solver=physics_solver)
    before = copy.deepcopy(config)
    xml = (WORKSPACE/'gng_vlut_system/worlds/dual_arm_demo.world').read_text()
    sensor = Mock()
    root = gazebo_launch.build_avoidance_world(xml, config, 'sim_test', sensor)
    world = root.find('world')
    assert config == before
    assert world.findtext('physics/ode/solver/type') == physics_solver
    assert world.findtext("plugin[@name='avoidance_state']/ros/namespace") == '/avoidance_demo'
    assert world.findtext("plugin[@name='avoidance_state']/update_rate") == '30.0'
    assert (world.find("model[@name='human_forearm']") is not None) is not enable_live_obstacles
    assert et.tostring(world.find("model[@name='work_table']")) == et.tostring(et.fromstring(xml).find("world/model[@name='work_table']"))
    if enable_gng_vlut:
        sensor.assert_called_once_with(world, 'sim_test', config)
    else:
        sensor.assert_not_called()


def test_world_builder_without_external_sensor_and_invalid_solver(gazebo_launch):
    xml = (WORKSPACE/'gng_vlut_system/worlds/dual_arm_demo.world').read_text()
    config = {'enable_live_obstacles': True, 'enable_gng_vlut': True}
    root = gazebo_launch.build_avoidance_world(xml, config, 'sim_test')
    assert root.find("world/model[@name='avoidance_lidar']") is None
    config['physics_solver'] = 'unknown'
    with pytest.raises(ValueError, match='physics_solver'):
        gazebo_launch.build_avoidance_world(xml, config, 'sim_test')


class MotionConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((WORKSPACE/'gng_vlut_system/config/dual_arm_gazebo_demo.yaml').read_text())['dual_arm_gazebo_demo']
        self.urdf = WORKSPACE/'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf'

    def test_both_models(self):
        for model in ['topo_dual_arm_max','topo_dual_arm_max_long']:
            names, poses = module.load_motion(WORKSPACE/'urdf'/model/'topo_dual_arm_max.urdf', self.config)
            self.assertEqual(len(names), 19)
            self.assertNotIn('L_gripper_mimic', names)
            self.assertEqual(len(poses), 8)
            for _, values in poses:
                self.assertEqual(len(values), len(names))
                for name in ['waist_joint','neck_pan_joint','neck_tilt_joint']:
                    self.assertEqual(values[names.index(name)], 0.0)

    def test_reject_invalid_targets(self):
        for joints in [{'L_joint4':100.0}, {'unknown_joint':0.0}, {'L_joint1':float('nan')}]:
            config = copy.deepcopy(self.config)
            config['poses'] = [{'name':'invalid','joints':joints}]
            with self.assertRaises(ValueError):
                module.load_motion(self.urdf, config)

    def test_reject_empty_motion(self):
        self.config['poses'] = []
        with self.assertRaises(ValueError):
            module.load_motion(self.urdf, self.config)


if __name__ == '__main__':
    unittest.main()
