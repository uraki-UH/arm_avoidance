#!/usr/bin/env python3
"""Dynamixel実測の関節名・rad換算と表示／追従用配信。"""
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml

from joint_command_model import joint_command_model, dynamixel_mapping, gripper_input_mapping


class joint_state_converter:
    def __init__(self, model, mapping_config, config):
        self.model = model
        self.mapping = dynamixel_mapping(model, mapping_config)
        self.role = config['role']
        self.max_state_age_sec = config['max_state_age_sec']
        if self.role not in ('leader', 'follower') or not math.isfinite(self.max_state_age_sec) or self.max_state_age_sec <= 0:
            raise ValueError('実測入力の役割・鮮度設定が不正です')
        self.names = list(config['joint_names']) or [name for name in model.independent_names if name in self.mapping.entries]
        self.gripper_mapping = {}
        if config['enable_gripper_input']:
            if self.role != 'leader':
                raise ValueError('開閉端による換算はリーダー入力専用です')
            for side, label in [('R', 'right'), ('L', 'left')]:
                name = side+'_gripper_joint'
                if name not in model.bounds or name not in self.mapping.entries:
                    raise ValueError('グリッパーのURDF・ID対応が不足しています: '+name)
                self.gripper_mapping[name] = gripper_input_mapping(
                    *model.bounds[name][:2], config[label+'_gripper_open_deg'], config[label+'_gripper_closed_deg'])
                if name not in self.names:
                    self.names.append(name)
        if not self.names or len(set(self.names)) != len(self.names) or any(
                name not in model.independent_names or name not in self.mapping.entries for name in self.names):
            raise ValueError('実測入力にはID対応済みの独立関節が必要です')
        self.motor_names = {str(self.mapping.entries[name][0]): name for name in self.names}
        self.fixed_positions = dict(zip(mapping_config.get('fixed_joint_names', []), mapping_config.get('fixed_joint_positions', [])))
        if (len(mapping_config.get('fixed_joint_names', [])) != len(mapping_config.get('fixed_joint_positions', [])) or
                len(self.fixed_positions) != len(mapping_config.get('fixed_joint_names', []))) or any(
                name in self.names or name not in model.independent_names or not math.isfinite(value)
                for name, value in self.fixed_positions.items()):
            raise ValueError('固定関節の実測表示設定が不正です')
        self.enable_mimic = config['enable_mimic']
        self.positions, self.velocities, self.stamps = {}, {}, {}

    def convert(self, message, now_ns):
        if (message.header.frame_id != 'dynamixel_motor' or
                not len(message.name) == len(message.position) == len(message.velocity) or
                len(set(message.name)) != len(message.name)):
            self.stamps.clear()
            return None
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if not 0 <= (now_ns-stamp)*1e-9 < self.max_state_age_sec:
            return None
        has_sample = False
        for motor_name, position, velocity in zip(message.name, message.position, message.velocity):
            name = self.motor_names.get(motor_name)
            if name is None or stamp <= self.stamps.get(name, -1):
                continue
            if not all(map(math.isfinite, (position, velocity))):
                self.stamps.pop(name, None)
                continue
            _, scale, offset = self.mapping.entries[name]
            value, speed = (position+math.radians(offset))*scale, velocity*scale
            if name in self.gripper_mapping:
                value, speed = self.gripper_mapping[name].convert(position, velocity)
            if not all(map(math.isfinite, (value, speed))):
                self.stamps.pop(name, None)
                continue
            self.positions[name], self.velocities[name], self.stamps[name] = value, speed, stamp
            has_sample = True
        if not has_sample or not all(name in self.stamps and
                0 <= (now_ns-self.stamps[name])*1e-9 < self.max_state_age_sec for name in self.names):
            return None
        positions = {name: self.positions[name] for name in self.names}
        velocities = {name: self.velocities[name] for name in self.names}
        positions.update(self.fixed_positions)
        velocities.update(dict.fromkeys(self.fixed_positions, 0.))
        if self.enable_mimic:
            positions = self.model.expand(positions)
            velocities = {name: multiplier*velocities[parent] for name, (parent, multiplier, _) in self.model.aliases.items()
                          if parent in velocities}
        names = list(positions)
        state = JointState(name=names, position=[positions[name] for name in names], velocity=[velocities[name] for name in names])
        # 最古の構成関節の実測時刻。部分欠測・同時刻再送による鮮度更新の防止
        state.header.stamp.sec, state.header.stamp.nanosec = divmod(min(self.stamps[name] for name in self.names), 1_000_000_000)
        state.header.frame_id = 'dynamixel_'+self.role
        return state


class dynamixel_joint_state_input(Node):
    def __init__(self, **kwargs):
        super().__init__('dynamixel_joint_state_input', **kwargs)
        defaults = {'urdf_path': '', 'mapping_file': '', 'role': 'follower',
                    'driver_namespace': '/dynamixel', 'input_topic': '', 'output_topic': '/follower/joint_states',
                    'joint_names': [''], 'max_state_age_sec': .3, 'enable_gripper_input': False,
                    'enable_mimic': True, 'right_gripper_open_deg': 0., 'right_gripper_closed_deg': 180.,
                    'left_gripper_open_deg': 0., 'left_gripper_closed_deg': -180.}
        config = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        if config['joint_names'] == ['']:
            config['joint_names'] = []
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Dynamixel実測入力にはuse_sim_time=falseが必要です')
        mapping = yaml.safe_load(Path(config['mapping_file']).read_text())['/**']['ros__parameters']
        self.converter = joint_state_converter(joint_command_model(config['urdf_path']), mapping, config)
        self.publisher = self.create_publisher(JointState, config['output_topic'], qos_profile_sensor_data)
        self.create_subscription(JointState, config['input_topic'] or config['driver_namespace'].rstrip('/')+'/fresh_joint_states', self.on_measured, qos_profile_sensor_data)

    def on_measured(self, message):
        state = self.converter.convert(message, time.time_ns())
        if state is not None:
            self.publisher.publish(state)


def main():
    rclpy.init()
    node = None
    try:
        node = dynamixel_joint_state_input()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
