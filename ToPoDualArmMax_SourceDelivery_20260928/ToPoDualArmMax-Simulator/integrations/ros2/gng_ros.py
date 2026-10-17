"""ROS実測・センシングからのGNG-VLUT回避指令。ブラウザ・MuJoCoへの依存なし。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import threading
import time
from types import MappingProxyType
import xml.etree.ElementTree as et

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gng_inputs import load_models, workspace
from gng_avoidance import gng_filter
from dual_arm_avoidance_geometry import robot_geometry
from joint_command_model import joint_command_model
from ros_plugins import definition, parameters


class measured_robot:
    """関節実測の一括スナップショットと、回避指令だけの出力窓口。"""
    def __init__(self, urdf_path, fixed_names, node, publisher, max_age_sec):
        self.model = joint_command_model(urdf_path)
        self.independent = {joint.get('name'): joint for joint in et.parse(urdf_path).getroot().findall('joint')
                            if joint.get('name') in self.model.independent_names}
        self.locks = tuple(fixed_names)
        self.node, self.publisher, self.max_age_sec = node, publisher, max_age_sec
        self.lock = threading.Lock()
        self.positions, self.velocities, self.efforts, self.received, self.stamps = {}, {}, {}, {}, {}
        self._targets = {}
        self.cycle = None

    def receive(self, message):
        values = self.model.feedback_positions(message.name, message.position)
        if not values or any(len(array) not in (0, len(message.name)) for array in (message.velocity, message.effort)):
            raise ValueError('関節実測の配列長が不正です')
        if any(not math.isfinite(value) for array in (message.velocity, message.effort) for value in array):
            raise ValueError('関節実測の速度・トルクが非有限値です')
        now = time.monotonic()
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if stamp and not 0 <= self.node.get_clock().now().nanoseconds*1e-9-stamp <= self.max_age_sec:
            raise ValueError('制御対象の関節実測が古いか未来時刻です')
        velocity = dict(zip(message.name, message.velocity))
        effort = dict(zip(message.name, message.effort))
        with self.lock:
            for name, value in values.items():
                elapsed = (stamp or now)-self.stamps.get(name, stamp or now)
                if name in velocity:
                    self.velocities[name] = float(velocity[name])
                elif 0 < elapsed <= self.max_age_sec and name in self.positions:
                    delta = value-self.positions[name]
                    if self.model.joints[name].is_continuous:
                        delta = math.remainder(delta, 2*math.pi)
                    self.velocities[name] = delta/elapsed
                else:
                    self.velocities.pop(name, None)
                if name in effort:
                    self.efforts[name] = float(effort[name])
                else:
                    self.efforts.pop(name, None)
                self.positions[name], self.received[name], self.stamps[name] = float(value), now, stamp or now

    def begin_cycle(self):
        with self.lock:
            now = time.monotonic()
            missing = [name for name in self.independent if name not in self.velocities or now-self.received.get(name, -math.inf) > self.max_age_sec]
            if missing:
                raise ValueError('関節実測・速度の入力待機: '+', '.join(missing))
            self.cycle = (dict(self.positions), dict(self.velocities), dict(self.efforts))
        if not self._targets:
            self._targets = dict(self.cycle[0])

    @property
    def targets(self):
        return MappingProxyType(self._targets)

    def state(self):
        return dict(self.cycle[0])

    def motion_state(self):
        return dict(velocity=dict(self.cycle[1]), effort=dict(self.cycle[2]))

    def validate(self, values):
        if not isinstance(values, dict) or set(values)-set(self.independent):
            raise ValueError('名目目標の関節名が不正です')
        result = dict(self._targets)
        for name, value in values.items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('名目目標の角度が非有限値です')
            low, high, _ = self.model.bounds[name]
            result[name] = min(high, max(low, value))
        return result

    def set_targets(self, values, **unused):
        targets = self.validate(values)
        names = [name for name in values if name not in self.locks]
        if not names:
            return
        message = JointTrajectory()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.joint_names = names
        message.points = [JointTrajectoryPoint(positions=[targets[name] for name in names])]
        self.publisher.publish(message)
        self._targets = targets

    def hold_position(self):
        self.set_targets(self.state())


class measured_scene:
    """固定基台と実測関節角によるリンク変換。物理計算・実機指令の所有なし。"""
    def __init__(self, robot, geometry, world_base, is_simulation=False):
        self.robot, self.geometry, self.world_base = robot, geometry, world_base
        self.is_simulation = is_simulation

    def link_transform(self, name):
        if name not in self.geometry.link_indices:
            raise ValueError('URDFのリンクがありません: '+name)
        positions = np.array([self.robot.state()[name] for name in self.geometry.joint_names])
        return self.world_base @ self.geometry.link_transforms(positions)[self.geometry.link_indices[name]]


class avoidance_node(Node):
    """実測入力・名目目標と、単一点の回避指令を接続するROS実行系。"""
    def __init__(self, args):
        super().__init__('gng_avoidance_controller')
        self.params, self.models = load_models(args.params_file)
        config = self.params.get('avoidance', {})
        self.max_state_age_sec = float(config.get('max_state_age_sec', .5))
        publish_hz = float(config.get('publish_hz', 50.))
        if not all(math.isfinite(value) and value > 0 for value in (self.max_state_age_sec, publish_hz)):
            raise ValueError('実測の鮮度・指令周期が不正です')
        self.settings = parameters(definition('gng'), config.get('settings', {}))
        self.enable_nonstop_simulation = getattr(args, 'nonstop_simulation', False)
        if getattr(args, 'gng_strategy', None):
            self.settings['gng_strategy'] = args.gng_strategy
        if self.enable_nonstop_simulation:
            self.settings['enable_nonstop_simulation'] = True
            if getattr(args, 'active_profile', None):
                self.settings['nonstop_profiles'] = [args.active_profile]
        if args.input_kind:
            self.settings['input_kind'] = args.input_kind
        self.settings['runtime_namespace'] = 'avoidance_'+self.params['robot_name']
        if args.source_tf_topic != '/tf' or args.source_static_tf_topic != '/tf_static':
            self.settings.update(recorded_tf_topic=args.source_tf_topic, recorded_static_tf_topic=args.source_static_tf_topic)
        self.settings['params_file'] = str(Path(args.params_file).resolve())
        self.model_id = args.model
        self.geometry = robot_geometry(self.params['urdf_path'], environment_shapes=())
        active = {name for model in self.models.values() for name in model['joint_names']}
        fixed = {name for model in self.models.values() for name in model['fixed_joints']} - active
        command_topic = args.command_topic or ('/sim/nonstop/joint_command' if self.enable_nonstop_simulation else config.get('command_topic', '/avoidance/joint_command'))
        state_topic = args.state_topic or config.get('state_topic', '/sim/joint_states')
        goal_topic = args.goal_topic or ('/sim/nonstop/goal' if self.enable_nonstop_simulation else config.get('goal_topic', '/avoidance/goal'))
        self.state_topic, self.command_topic = state_topic, command_topic
        if self.enable_nonstop_simulation and tuple(self.resolve_topic_name(topic) for topic in (state_topic, command_topic, goal_topic)) != (
                '/sim/joint_states', '/sim/nonstop/joint_command', '/sim/nonstop/goal'):
            raise ValueError('非停止モードはMuJoCo専用の実測・指令・目標トピックだけで使用可能です')
        if len({self.resolve_topic_name(topic) for topic in (command_topic, state_topic, goal_topic)}) != 3:
            raise ValueError('実測・目標・回避指令には別トピックが必要です')
        self.robot = measured_robot(self.params['urdf_path'], fixed, self,
                                    self.create_publisher(JointTrajectory, command_topic, 1), self.max_state_age_sec)
        self.base_pos = config.get('base_pos', [0., 0., 0.])
        self.base_rot_deg = config.get('base_rot_deg', [0., 0., 0.])
        if any(not isinstance(value, list) or len(value) != 3 or not np.all(np.isfinite(value)) for value in (self.base_pos, self.base_rot_deg)):
            raise ValueError('基台の配置にはXYZ・RPYの3値が必要です')
        self.status_pub = self.create_publisher(String, '/avoidance/status', 10)
        self.create_subscription(JointState, state_topic, self.receive_state, qos_profile_sensor_data)
        self.create_subscription(JointState, goal_topic, self.receive_goal, 1)
        self.goal_lock = threading.Lock()
        self.pending_goal = None
        self.filter = None
        self.worker_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ros-avoidance')
        self.future = None
        self.is_closed = False
        self.last_status = None
        self.timer = self.create_timer(1./publish_hz, self.tick)
        self.get_logger().info(f'実測: {state_topic} / 目標: {goal_topic} / 指令: {command_topic}')

    def receive_state(self, message):
        try:
            self.robot.receive(message)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def receive_goal(self, message):
        try:
            goal = self.robot.model.canonical_positions(message.name, message.position)
            if not goal or message.velocity or message.effort:
                raise ValueError('名目目標は関節名と位置だけの指定です')
            with self.goal_lock:
                self.pending_goal = goal
        except ValueError as error:
            self.get_logger().warning(str(error))

    def update(self):
        self.robot.begin_cycle()
        if self.filter is None:
            if self.enable_nonstop_simulation:
                endpoints = (self.get_publishers_info_by_topic(self.state_topic),
                             self.get_subscriptions_info_by_topic(self.command_topic))
                if any(len(group) != 1 or group[0].node_name != 'dual_arm_simulation_cli'
                       or group[0].node_namespace != '/' for group in endpoints):
                    raise ValueError('非停止モードの専用MuJoCo実測・指令接続待機')
            # 制御基台の明示配置。観測用bagの基台TFからの位置・姿勢転用なし。
            world_base = np.eye(4)
            world_base[:3, 3] = self.base_pos
            world_base[:3, :3] = Rotation.from_euler('xyz', self.base_rot_deg, degrees=True).as_matrix()
            scene = measured_scene(self.robot, self.geometry, world_base, self.enable_nonstop_simulation)
            self.filter = gng_filter(scene, self.settings, self, dict(model=self.model_id), (self.params, self.models))
        with self.goal_lock:
            goal, self.pending_goal = self.pending_goal, None
        if goal is not None:
            self.filter.set_targets(goal)
        self.filter.update()
        return dict(self.filter.status)

    def tick(self):
        if self.is_closed or self.future is not None and not self.future.done():
            return
        if self.future is not None:
            has_failed = False
            try:
                status = self.future.result()
            except ValueError as error:
                status = dict(phase='waiting', message=str(error))
            except Exception as error:
                self.get_logger().error(str(error))
                self.timer.cancel()
                has_failed = True
                status = dict(phase='failed', message=str(error))
            message = String(data=json.dumps(status, ensure_ascii=False))
            self.status_pub.publish(message)
            text = (status.get('phase'), status.get('message', ''))
            if text != self.last_status:
                self.get_logger().info(' / '.join(text))
                self.last_status = text
            if has_failed:
                raise RuntimeError(status['message'])
        self.future = self.worker_pool.submit(self.update)

    def close(self):
        self.is_closed = True
        self.timer.cancel()
        self.worker_pool.shutdown(wait=True, cancel_futures=True)
        if self.filter is not None:
            self.filter.close()
        self.destroy_node()


def main():
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params-file', default=str(workspace/'gng_vlut_system/config/topo_dual_arm_max_long.yaml'))
    parser.add_argument('--model', choices=('long', 'standard'), default='long')
    parser.add_argument('--state-topic', default='')
    parser.add_argument('--goal-topic', default='')
    parser.add_argument('--command-topic', default='')
    parser.add_argument('--source-tf-topic', default='/tf')
    parser.add_argument('--source-static-tf-topic', default='/tf_static')
    parser.add_argument('--input-kind', choices=('pointcloud', 'voxel'), default='')
    parser.add_argument('--gng-strategy', choices=('graph', 'avoidance_first', 'vrm_predictive'))
    parser.add_argument('--nonstop-simulation', action='store_true', help='MuJoCo専用の非停止実験。実機使用禁止')
    parser.add_argument('--active-profile', help='非停止実験の操作グループ。省略時は学習済み全グループ')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = None
    try:
        node = avoidance_node(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
