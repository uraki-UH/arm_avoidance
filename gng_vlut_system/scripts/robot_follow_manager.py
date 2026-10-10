#!/usr/bin/env python3
"""s・r・fの追従経路管理。構成の配信・実測監視・世代付き目標の転送。"""
import json
import math
import signal
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

from joint_command_model import joint_command_model
from robot_follow_model import validate_config


class robot_follow_manager(Node):
    def __init__(self):
        super().__init__('manager')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('追従監視にはuse_sim_time=falseが必要です')
        config_file = self.declare_parameter('config_file', '').value
        self.config = validate_config(yaml.safe_load(Path(config_file).read_text()))
        self.model = joint_command_model(self.declare_parameter('urdf_path', '').value)
        self.names = self.declare_parameter('joint_names', ['R_joint1']).value
        if not self.names or any(name not in self.model.independent_names for name in self.names):
            raise ValueError('追従対象の独立関節設定不正')
        self.allow_hardware_output = self.declare_parameter('allow_hardware_output', False).value
        self.profile = self.declare_parameter('profile', 'manual').value
        if self.profile not in self.config['profiles']:
            raise ValueError('未登録の追従構成: '+self.profile)
        self.session_id = uuid.uuid4().hex
        self.samples, self.last_stamps = {}, {}
        self.hardware_status = {}
        self.hardware_status_sec = -math.inf
        self.simulator_status = {}
        self.simulator_status_sec = -math.inf
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_pub = self.create_publisher(String, 'config', latched)
        self.status_pub = self.create_publisher(String, 'status', 1)
        self.simulator_pub = self.create_publisher(JointState, 'simulator_target', qos_profile_sensor_data)
        self.follower_pub = self.create_publisher(JointState, 'follower_target', qos_profile_sensor_data)
        for role, topic in self.config['roles'].items():
            self.create_subscription(JointState, topic, lambda msg, role=role: self.on_state(role, msg), qos_profile_sensor_data)
        self.create_subscription(String, 'follower/status', self.on_hardware_status, 1)
        self.create_subscription(String, 'simulator_status', self.on_simulator_status, 1)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(.5, self.publish_config)
        self.create_timer(.2, self.publish_status)
        self.publish_config()

    def on_parameters(self, parameters):
        if len(parameters) != 1 or parameters[0].name != 'profile' or (not isinstance(parameters[0].value, str) or parameters[0].value not in self.config['profiles']):
            return SetParametersResult(successful=False, reason='登録済みのprofileのみ変更可能')
        if parameters[0].value != self.profile:
            self.profile = parameters[0].value
            self.session_id = uuid.uuid4().hex
            self.samples.clear()
            self.simulator_status = {}
            self.publish_config()
        return SetParametersResult(successful=True)

    def publish_config(self):
        self.config_pub.publish(String(data=json.dumps({**self.config, 'profile': self.profile,
            'session_id': self.session_id, 'robot_model': 'long' if 'long' in self.model_path_name else 'standard',
            'allow_hardware_output': self.allow_hardware_output,
            'simulator_target_topic': '/robot_follow/simulator_target'}, ensure_ascii=False)))

    @property
    def model_path_name(self):
        return self.get_parameter('urdf_path').value

    def on_hardware_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
            self.hardware_status, self.hardware_status_sec = status, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_simulator_status(self, message):
        try:
            status = json.loads(message.data)
            if isinstance(status, dict) and status.get('session_id') == self.session_id:
                self.simulator_status, self.simulator_status_sec = status, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_state(self, role, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (time.time_ns()-stamp)*1e-9
        if (not 0 <= age < self.config['max_state_age_sec'] or
                stamp <= self.last_stamps.get(role, -1) or
                len(message.name) != len(message.position) or len(set(message.name)) != len(message.name) or
                any(not math.isfinite(value) for value in message.position)):
            return
        if role == 'r' and message.header.frame_id != 'dynamixel_leader':
            return
        if role == 's' and message.header.frame_id != 'robot_follow_s:'+self.session_id:
            return
        pose = {name: value for name, value in zip(message.name, message.position) if name in self.model.independent_names}
        if not all(name in pose for name in self.names):
            return
        if (self.count_publishers('/robot_follow/config') != 1 or
                self.count_publishers(self.config['roles'][role]) != 1):
            return
        self.last_stamps[role], self.samples[role] = stamp, message
        profile = self.config['profiles'][self.profile]
        if profile['simulator_source'] == role:
            names = list(pose) if profile['simulator_mode'] == 'display' else self.names
            state = JointState(name=names, position=[pose[name] for name in names])
            state.header = message.header
            self.simulator_pub.publish(state)
        if profile['follower_source'] != role:
            return
        if role == 's':
            status = self.simulator_status
            if (time.monotonic()-self.simulator_status_sec > self.config['max_state_age_sec'] or
                    status.get('session_id') != self.session_id or
                    (profile['simulator_mode'] == 'dynamics' and
                     (status.get('is_running') is not True or status.get('is_input_stopped') is not False))):
                return
        target = JointState(name=self.names, position=[pose[name] for name in self.names])
        target.header.stamp = message.header.stamp
        target.header.frame_id = 'robot_follow_target:'+self.session_id
        self.follower_pub.publish(target)

    def publish_status(self):
        now_ns = time.time_ns()
        ages = {role: (now_ns-self.last_stamps[role])*1e-9 if role in self.last_stamps else None for role in ('s', 'r', 'f')}
        self.status_pub.publish(String(data=json.dumps({'profile': self.profile, 'session_id': self.session_id,
            'state_age_sec': ages, 'has_fresh_input': {role: age is not None and 0 <= age < self.config['max_state_age_sec'] for role, age in ages.items()},
            'hardware': self.hardware_status, 'has_fresh_hardware_status': time.monotonic()-self.hardware_status_sec < .5}, ensure_ascii=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exit_state = {'is_requested': False}
    def request_exit(_kind, _frame):
        exit_state['is_requested'] = True
    handlers = {kind: signal.signal(kind, request_exit) for kind in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = robot_follow_manager()
        while rclpy.ok() and not exit_state['is_requested']:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for kind, handler in handlers.items():
            signal.signal(kind, handler)


if __name__ == '__main__':
    main()
