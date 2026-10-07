#!/usr/bin/env python3
"""Gazebo専用タスク実行ノード。標準軌道Actionへの単一出力。"""

import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

from task_program import joint_sample, load_program, read_joint_bounds, run_state, task_runner


class trajectory_backend:
    """受理前の取消要求を保持した、同時指令数1のActionアダプタ。"""

    def __init__(self, node, joint_names, limits):
        self.node, self.joint_names = node, joint_names
        self.limits = limits
        self.client = ActionClient(node, FollowJointTrajectory, 'dual_arm_controller/follow_joint_trajectory')
        self.is_busy = False
        self.result = None
        self.handle = None
        self.has_cancel_request = False

    def begin(self, points):
        if self.is_busy:
            raise RuntimeError('同時軌道指令は禁止です')
        self.is_busy, self.result, self.handle, self.has_cancel_request = True, None, None, False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)
        goal.goal_time_tolerance.sec = 3
        goal.goal_tolerance = [JointTolerance(name=name, position=self.limits['max_position_error_th'],
                              velocity=self.limits['max_stopped_velocity_th']) for name in self.joint_names]
        for point in points:
            item = JointTrajectoryPoint()
            item.positions, item.velocities, item.accelerations = list(point.positions), list(point.velocities), list(point.accelerations)
            stamp = round(point.time_sec * 1_000_000_000)
            item.time_from_start.sec, item.time_from_start.nanosec = divmod(stamp, 1_000_000_000)
            goal.trajectory.points.append(item)
        self.client.send_goal_async(goal).add_done_callback(self._accepted)

    def _accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                self.is_busy, self.result = False, False
                return
            self.handle.get_result_async().add_done_callback(self._finished)
            if self.has_cancel_request:
                self.handle.cancel_goal_async().add_done_callback(self._canceled)
        except Exception as error:
            self.node.get_logger().error(f'軌道受理エラー: {error}')
            self.is_busy, self.result = False, False

    def _finished(self, future):
        try:
            result = future.result()
            self.result = (result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0)
            if result.status == GoalStatus.STATUS_ABORTED:
                self.node.get_logger().error('軌道異常終了: ' + result.result.error_string)
        except Exception as error:
            self.node.get_logger().error(f'軌道結果エラー: {error}')
            self.result = False
        self.is_busy, self.handle = False, None

    def _canceled(self, future):
        try:
            if not future.result().goals_canceling:
                self.node.get_logger().warning('取消応答なし。軌道結果と実測停止の確認待ち')
        except Exception as error:
            self.node.get_logger().error(f'軌道取消エラー: {error}')

    def cancel(self):
        if self.has_cancel_request:
            return
        self.has_cancel_request = True
        if self.handle is not None and self.is_busy:
            self.handle.cancel_goal_async().add_done_callback(self._canceled)


class task_executor(Node):
    """関節実測・操作サービス・監視を実行器へ接続する薄いROS層。"""

    def __init__(self):
        super().__init__('task_executor')
        self.declare_parameter('task_file', '')
        self.declare_parameter('urdf', '')
        self.declare_parameter('enable_autostart', False)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.get_parameter('use_sim_time').value or not self.get_namespace().startswith('/sim_'):
            raise ValueError('use_sim_timeとsim_名前空間を使ったGazebo専用の入口です')
        data = yaml.safe_load(Path(self.get_parameter('task_file').value).read_text())
        bounds = read_joint_bounds(self.get_parameter('urdf').value)
        self.program = load_program(data, bounds)
        self.backend = trajectory_backend(self, self.program.joint_names, self.program.limits)
        self.runner = task_runner(self.program, self.backend)
        self.sample = None
        self.sample_received_sec = None
        self.enable_autostart = self.get_parameter('enable_autostart').value
        self.status_publisher = self.create_publisher(String, 'task_executor/status', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, qos_profile_sensor_data)
        for name in ('start', 'pause', 'resume', 'cancel'):
            self.create_service(Trigger, 'task_executor/' + name,
                                lambda request, response, command=name: self.on_command(command, response))
        self.create_service(SetBool, 'task_executor/obstacle', self.on_obstacle)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, message):
        try:
            if len(set(message.name)) != len(message.name):
                raise ValueError('関節名重複')
            positions = dict(zip(message.name, message.position))
            velocities = dict(zip(message.name, message.velocity))
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            sample = joint_sample(stamp, tuple(positions[name] for name in self.program.joint_names),
                                  tuple(velocities[name] for name in self.program.joint_names))
            if self.sample is not None and stamp <= self.sample.stamp_sec:
                return
            self.sample, self.sample_received_sec = sample, time.monotonic()
        except (KeyError, ValueError):
            self.sample = None

    def fresh_sample(self):
        if self.sample_received_sec is None or time.monotonic() - self.sample_received_sec > self.program.limits['max_feedback_age_sec']:
            return None
        return self.sample

    def check_output(self):
        if not self.backend.client.server_is_ready():
            raise ValueError('軌道Actionサーバーの準備待ち')
        if self.count_publishers('dual_arm_controller/joint_trajectory'):
            raise ValueError('軌道トピックに別の指令元が存在します')

    def on_command(self, command, response):
        try:
            if command in ('start', 'resume'):
                self.check_output()
                getattr(self.runner, command)(self.now_sec(), self.fresh_sample())
            else:
                self.runner.interrupt(self.now_sec(), is_cancel=command == 'cancel')
            self.enable_autostart = False
            response.success, response.message = True, self.runner.state.value
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def on_obstacle(self, request, response):
        self.runner.set_obstacle(request.data, self.now_sec())
        response.success, response.message = True, self.runner.state.value
        return response

    def tick(self):
        try:
            if self.runner.state == run_state.running:
                self.check_output()
            self.runner.tick(self.now_sec(), self.fresh_sample())
            if self.enable_autostart and self.runner.state == run_state.idle:
                self.on_command('start', Trigger.Response())
        except (ValueError, RuntimeError) as error:
            self.runner.fail(str(error))
        status = self.runner.status()
        status.update(is_command_active=self.backend.is_busy, command_result=self.backend.result)
        if self.sample is not None and not self.runner.feedback_error(self.now_sec(), self.sample):
            status['max_actual_velocity'] = max(abs(value) for value in self.sample.velocities)
            if self.runner.target is not None:
                status['position_errors'] = dict(zip(self.program.joint_names,
                    (actual - target for actual, target in zip(self.sample.positions, self.runner.target))))
        self.status_publisher.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = task_executor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.backend.cancel()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
