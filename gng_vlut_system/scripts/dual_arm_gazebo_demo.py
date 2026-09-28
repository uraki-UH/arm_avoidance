#!/usr/bin/env python3
"""Gazebo専用の関節軌道デモと停止操作。"""
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from motion_smoothing import make_rest_to_rest_trajectory, quintic_duration
import yaml


def load_motion(urdf_path, config):
    """URDF関節制限とデモ姿勢の整合検査。"""
    root = ET.parse(urdf_path).getroot()
    joints = {joint.get('name'): joint for joint in root.findall('joint')
              if joint.get('type') != 'fixed' and joint.find('mimic') is None}
    names = list(joints)
    poses = []
    for pose in config['poses']:
        if set(pose['joints']) - set(names):
            raise ValueError('デモ姿勢にURDF未定義の関節があります')
        values = [float(pose['joints'].get(name, 0.0)) for name in names]
        for name, value in zip(names, values):
            joint = joints[name]
            limit = joint.find('limit')
            if not math.isfinite(value):
                raise ValueError(f'関節角が有限値ではありません: {name}')
            if joint.get('type') != 'continuous' and limit is not None:
                if not float(limit.get('lower', '0')) <= value <= float(limit.get('upper', '0')):
                    raise ValueError(f'関節角がURDF範囲外です: {name}={value}')
        poses.append((pose['name'], values))
    if not poses:
        raise ValueError('デモ姿勢が空です')
    return names, poses


class DualArmGazeboDemo(Node):
    def __init__(self):
        super().__init__('dual_arm_gazebo_demo')
        if not self.get_namespace().lstrip('/').startswith('sim_'):
            raise ValueError('デモはsim_で始まる名前空間専用です')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('デモはuse_sim_time=true専用です')
        config_path = self.declare_parameter('demo_config', '').value
        urdf_path = self.declare_parameter('urdf_path', '').value
        config = yaml.safe_load(Path(config_path).read_text())['dual_arm_gazebo_demo']
        self.joint_names, self.poses = load_motion(urdf_path, config)
        self.enable_auto_start = self.declare_parameter('enable_auto_start', bool(config['enable_auto_start'])).value
        self.enable_loop = bool(config['enable_loop'])
        self.segment_duration_sec = float(config['segment_duration_sec'])
        self.pause_duration_sec = float(config['pause_duration_sec'])
        self.max_joint_velocity = float(config['max_joint_velocity'])
        self.max_state_age_sec = float(config['max_state_age_sec'])
        self.goal_position_th = float(config['goal_position_th'])
        for value in [self.segment_duration_sec, self.max_joint_velocity, self.max_state_age_sec, self.goal_position_th]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError('時間・速度・位置判定値は正の有限値が必要です')
        if not math.isfinite(self.pause_duration_sec) or self.pause_duration_sec < 0:
            raise ValueError('待機時間は非負の有限値が必要です')
        self.client = ActionClient(self, FollowJointTrajectory, 'dual_arm_controller/follow_joint_trajectory')
        self.state_sub = self.create_subscription(JointState, 'joint_states', self.on_state, 10)
        self.status_pub = self.create_publisher(String, 'demo/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.start_service = self.create_service(Trigger, 'demo/start', self.start)
        self.stop_service = self.create_service(Trigger, 'demo/stop', self.stop)
        self.positions = {}
        self.last_state_sec = 0.0
        self.is_running = False
        self.has_pending_goal = False
        self.goal_handle = None
        self.pose_idx = 0
        self.next_goal_sec = 0.0
        self.state = 'waiting'
        self.last_error = ''
        self.timer = self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def on_state(self, message):
        if len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in self.joint_names):
            return
        self.positions = positions
        self.last_state_sec = time.monotonic()

    def is_ready(self):
        return bool(self.positions) and time.monotonic() - self.last_state_sec <= self.max_state_age_sec and self.client.server_is_ready()

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'pose_idx': self.pose_idx,
            'pose': self.poses[self.pose_idx % len(self.poses)][0],
            'error': self.last_error}, ensure_ascii=False)))

    def start(self, request, response):
        del request
        if self.is_running or self.has_pending_goal or self.goal_handle is not None:
            response.success = False
            response.message = '動作中または停止処理中です'
        elif not self.is_ready():
            response.success = False
            response.message = '関節状態とコントローラの準備待ちです'
        else:
            self.is_running = True
            self.pose_idx = 0
            self.last_error = ''
            self.state = 'running'
            self.next_goal_sec = 0.0
            response.success = True
            response.message = 'デモ開始'
        self.publish_status()
        return response

    def stop(self, request, response):
        del request
        self.enable_auto_start = False
        self.is_running = False
        self.state = 'stopping' if self.goal_handle is not None or self.has_pending_goal else 'idle'
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        response.success = True
        response.message = '軌道取消要求。停止完了はdemo/statusで確認してください'
        self.publish_status()
        return response

    def tick(self):
        if self.enable_auto_start and self.is_ready():
            self.enable_auto_start = False
            self.start(None, Trigger.Response())
        if not self.is_running:
            if self.state == 'waiting' and self.is_ready():
                self.state = 'idle'
                self.publish_status()
            return
        if not self.is_ready():
            self.last_error = '関節状態の失効またはコントローラ切断'
            self.stop(None, Trigger.Response())
            self.state = 'fault'
            self.publish_status()
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.has_pending_goal or self.goal_handle is not None or now_sec < self.next_goal_sec:
            return
        target = self.poses[self.pose_idx][1]
        current = [self.positions[name] for name in self.joint_names]
        duration_sec = quintic_duration(current, target, self.segment_duration_sec, self.max_joint_velocity)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = make_rest_to_rest_trajectory(self.joint_names, current, target, duration_sec)
        goal.goal_time_tolerance.sec = 2
        goal.goal_tolerance = [JointTolerance(name=name, position=self.goal_position_th) for name in self.joint_names]
        self.has_pending_goal = True
        self.client.send_goal_async(goal).add_done_callback(self.on_goal)
        self.publish_status()

    def on_goal(self, future):
        self.has_pending_goal = False
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.goal_handle = None
                self.is_running = False
                self.state = 'fault'
                self.last_error = 'コントローラが軌道を拒否しました'
                self.publish_status()
                return
            self.goal_handle.get_result_async().add_done_callback(self.on_result)
            if not self.is_running:
                self.goal_handle.cancel_goal_async()
        except Exception as error:
            self.goal_handle = None
            self.is_running = False
            self.state = 'fault'
            self.last_error = str(error)
            self.publish_status()

    def on_result(self, future):
        self.goal_handle = None
        try:
            response = future.result()
            result = response.result
            if not self.is_running:
                if self.state != 'fault':
                    self.state = 'idle'
            elif response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.is_running = False
                self.state = 'fault'
                self.last_error = result.error_string or f'軌道終了状態: {response.status}'
            else:
                self.pose_idx += 1
                if self.pose_idx == len(self.poses):
                    self.pose_idx = 0
                    if not self.enable_loop:
                        self.is_running = False
                        self.state = 'completed'
                self.next_goal_sec = self.get_clock().now().nanoseconds * 1e-9 + self.pause_duration_sec
        except Exception as error:
            self.is_running = False
            self.state = 'fault'
            self.last_error = str(error)
        self.publish_status()


def main():
    rclpy.init()
    node = DualArmGazeboDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
