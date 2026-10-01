#!/usr/bin/env python3
"""Gazebo実測姿勢と前腕カプセルによる局所退避・可視化デモ。"""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from motion_smoothing import make_rest_to_rest_trajectory, quintic_max_step
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from dual_arm_avoidance_geometry import robot_geometry


class avoidance_demo(Node):
    def __init__(self):
        super().__init__('dual_arm_avoidance_demo')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('avoidance_config', '')
        if not self.get_namespace().startswith('/sim_') or not self.get_parameter('use_sim_time').value:
            raise ValueError('Gazeboのsim_名前空間とuse_sim_timeが必要です')
        self.config = yaml.safe_load(Path(self.get_parameter('avoidance_config').value).read_text())['dual_arm_avoidance_demo']
        self.declare_parameter('enable_auto_start', self.config['enable_auto_start'])
        self.enable_auto_start = bool(self.get_parameter('enable_auto_start').value)
        self.enable_stamped_commands = bool(self.declare_parameter('enable_stamped_commands', False).value)
        self.has_safety_state = False
        self.is_stop_latched = False
        for key in ('arm_length', 'arm_radius', 'approach_sec', 'hold_sec', 'withdraw_sec', 'settle_sec',
                    'target_clearance', 'min_clearance_th', 'max_state_age_sec', 'max_joint_velocity', 'control_period_sec'):
            if not np.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key}は有限の正数が必要です')
        for key in ('hand_far_x', 'hand_near_x', 'hand_y', 'hand_z'):
            if not np.isfinite(self.config[key]):
                raise ValueError(f'{key}は有限値が必要です')
        if self.config['min_clearance_th'] >= self.config['target_clearance']:
            raise ValueError('停止距離は目標余裕より小さい値が必要です')
        # 物理追従のずれに対する、計画段階の内部形状余裕 [m]
        self.min_planning_clearance_th = self.config.get('min_planning_clearance_th', 0.005)
        if not np.isfinite(self.min_planning_clearance_th) or self.min_planning_clearance_th < 0.005:
            raise ValueError('計画時の内部形状余裕は停止判定の0.005 mを確保する値が必要です')
        if self.config['hand_far_x'] <= self.config['hand_near_x']:
            raise ValueError('接近開始位置と終点の順序が逆です')
        if not self.config['sides'] or any(side not in ('left', 'right') for side in self.config['sides']):
            raise ValueError('接近側はleft/rightの非空リストが必要です')
        self.geometry = robot_geometry(self.get_parameter('urdf_path').value, self.config.get('planning_groups'))
        self.joint_limits = {joint.get('name'): {key: float(joint.find('limit').get(key))
                             for key in ('velocity', 'effort')}
                             for joint in ET.parse(self.get_parameter('urdf_path').value).getroot().findall('joint')
                             if joint.get('type') != 'fixed'}
        self.has_joint_limit_violation = False
        self.positions = None
        self.home = None
        self.hand = None
        self.joint_time = 0.0
        self.obstacle_time = 0.0
        self.last_ros_time = -1
        self.state = 'waiting'
        self.error = ''
        self.phase = 'waiting'
        self.side_idx = 0
        self.start_sec = 0.0
        self.run_generation = 0
        self.run_start_stamp_sec = 0.0
        self.next_control_sec = 0.0
        self.min_observed_clearance = float('inf')
        self.min_home_clearance = float('inf')
        self.max_excursion = 0.0
        self.trails = {name: [] for name in self.config.get('trail_links', ['L_link7', 'R_link7'])}
        self.last_visual = None
        self.pending_set = None
        self.has_sent_hold = False
        self.command = self.create_publisher(JointTrajectory, 'dual_arm_controller/joint_trajectory', 1)
        self.status = self.create_publisher(String, 'avoidance/status', 1)
        self.markers = self.create_publisher(MarkerArray, 'avoidance/markers', 1)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 1)
        if not self.config.get('enable_live_obstacles', False):
            self.create_subscription(ModelStates, '/avoidance_demo/model_states', self.on_obstacle, qos_profile_sensor_data)
        self.create_subscription(Bool, 'safety/is_stop_latched', self.on_safety_stop,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.set_state = (None if self.config.get('enable_live_obstacles', False) else
                          self.create_client(SetEntityState, '/avoidance_demo/set_entity_state'))
        self.create_service(Trigger, 'avoidance/start', self.on_start)
        self.create_service(Trigger, 'avoidance/stop', self.on_stop)
        self.timer = self.create_timer(self.config['control_period_sec'], self.tick,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if not all(name in values and np.isfinite(values[name]) for name in self.geometry.joint_names):
            return
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if stamp <= self.last_ros_time:
            return
        self.last_ros_time = stamp
        self.positions = np.array([values[name] for name in self.geometry.joint_names])
        self.joint_time = time.monotonic()
        # 接触反力を含む実測上限監視。単位ごとの丸め許容差1e-6
        self.has_joint_limit_violation = False
        for idx, name in enumerate(message.name):
            if name not in self.joint_limits:
                continue
            for field in ('velocity', 'effort'):
                data = getattr(message, field)
                if idx >= len(data) or not np.isfinite(data[idx]) or abs(data[idx]) > self.joint_limits[name][field]+1e-6:
                    self.has_joint_limit_violation = True
                    if self.state == 'running':
                        self.fail(f'関節上限超過または実測欠落: {name}/{field}')
                    return

    def on_obstacle(self, message):
        if 'human_forearm' not in message.name:
            return
        point = message.pose[message.name.index('human_forearm')].position
        hand = np.array([point.x, point.y, point.z])
        if np.all(np.isfinite(hand)):
            self.hand = hand
            self.obstacle_time = time.monotonic()

    def is_fresh(self):
        now = time.monotonic()
        has_obstacle = (self.config.get('enable_live_obstacles', False) or
                        (self.hand is not None and now-self.obstacle_time < self.config['max_state_age_sec']))
        return (self.positions is not None and has_obstacle and
                now-self.joint_time < self.config['max_state_age_sec'] and
                self.command.get_subscription_count() > 0)

    def observe_clearance(self):
        return self.geometry.clearance(self.positions, self.hand,
            self.hand+np.array([self.config['arm_length'], 0, 0]), self.config['arm_radius'])

    def on_start(self, _request, response):
        if not self.has_safety_state or self.is_stop_latched:
            response.success = False
            response.message = 'Gazebo停止状態の確認・ラッチ解除が必要です'
            return response
        if self.state == 'running':
            response.success = False
            response.message = '実行中です'
            return response
        if self.has_joint_limit_violation or not self.is_fresh() or not self.geometry.has_internal_clearance(self.geometry.centers(self.positions)):
            response.success = False
            response.message = '関節・障害物の実測更新と初期姿勢の余裕が必要です'
            return response
        gap = self.observe_clearance()[0]
        if gap <= self.config['min_clearance_th']:
            response.success = False
            response.message = '開始時の障害物距離が不足しています'
            return response
        self.home = self.positions.copy()
        self.state, self.error, self.side_idx = 'running', '', 0
        self.run_generation += 1
        self.run_start_stamp_sec = self.last_ros_time * 1e-9
        self.start_sec = self.get_clock().now().nanoseconds*1e-9
        self.min_observed_clearance = float('inf')
        self.min_home_clearance = float('inf')
        self.max_excursion = 0.0
        self.has_sent_hold = False
        self.trails = {name: [] for name in self.trails}
        response.success, response.message = True, '接近・退避デモ開始'
        return response

    def hold(self):
        if self.positions is not None and not self.has_sent_hold:
            self.publish_target(self.positions)
            self.has_sent_hold = True

    def on_safety_stop(self, message):
        self.has_safety_state = True
        self.is_stop_latched = bool(message.data)
        if self.is_stop_latched:
            # 駆動側保持への委任。解除後も明示開始までのデモ停止維持
            self.enable_auto_start = False
            self.state, self.phase, self.error = 'stopped', 'software_stop', ''

    def on_stop(self, _request, response):
        if self.is_stop_latched:
            response.success, response.message = True, 'Gazebo停止ラッチを維持しています'
            return response
        self.state, self.phase = 'idle', 'stopped'
        self.hold()
        response.success, response.message = True, '関節目標と障害物移動を停止'
        return response

    def fail(self, error):
        if self.state != 'fault':
            self.get_logger().error(error)
        self.state, self.error = 'fault', error
        self.hold()

    def publish_target(self, target):
        if self.is_stop_latched:
            return
        message = make_rest_to_rest_trajectory(
            self.geometry.joint_names, self.positions.tolist(), target.tolist(), self.config['control_period_sec'])
        if self.enable_stamped_commands:
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(self.last_ros_time, 1_000_000_000)
        self.command.publish(message)

    def update_obstacle(self, desired):
        if self.config.get('enable_live_obstacles', False):
            return
        if self.pending_set is not None and self.pending_set.done():
            try:
                if not self.pending_set.result().success:
                    self.fail('Gazebo障害物の移動失敗')
            except Exception as error:
                self.fail(str(error))
            self.pending_set = None
        if (desired is not None and self.state == 'running' and self.pending_set is None
                and self.set_state.service_is_ready()):
            request = SetEntityState.Request()
            request.state.name, request.state.reference_frame = 'human_forearm', 'world'
            request.state.pose.position = Point(x=float(desired[0]), y=float(desired[1]), z=float(desired[2]))
            request.state.pose.orientation.w = 1.0
            self.pending_set = self.set_state.call_async(request)

    def scenario(self):
        elapsed = self.get_clock().now().nanoseconds*1e-9-self.start_sec
        approach, hold, withdraw, settle = [self.config[key] for key in
                                          ('approach_sec', 'hold_sec', 'withdraw_sec', 'settle_sec')]
        duration = approach+hold+withdraw+settle
        if elapsed >= duration:
            if np.max(np.abs(self.positions-self.home)) > 0.08:
                self.fail('復帰時間内に初期姿勢へ戻れませんでした')
                return None
            self.side_idx += 1
            if self.side_idx >= len(self.config['sides']):
                self.state, self.phase = 'completed', 'completed'
                self.hold()
                return None
            self.start_sec += duration
            elapsed -= duration
        if elapsed < approach:
            ratio, self.phase = elapsed/approach, 'approaching'
        elif elapsed < approach+hold:
            ratio, self.phase = 1.0, 'near'
        elif elapsed < approach+hold+withdraw:
            ratio, self.phase = 1-(elapsed-approach-hold)/withdraw, 'withdrawing'
        else:
            ratio, self.phase = 0.0, 'returning'
        sign = 1 if self.config['sides'][self.side_idx] == 'left' else -1
        return np.array([self.config['hand_far_x']*(1-ratio)+self.config['hand_near_x']*ratio,
                         sign*self.config['hand_y'], self.config['hand_z']])

    def select_target(self, hand, elbow, step):
        return self.geometry.choose_step(self.positions, self.home, hand, elbow,
            self.config['arm_radius'], self.config['target_clearance'], step, self.min_planning_clearance_th)

    def tick(self):
        began = time.monotonic()
        self.update_obstacle(None)
        if self.state == 'waiting' and self.has_safety_state and self.is_fresh():
            self.state = 'idle'
            if self.enable_auto_start:
                self.on_start(None, Trigger.Response())
        gap = None
        if self.state == 'running' and not self.is_fresh():
            self.fail('実測関節または障害物情報の失効')
        if self.is_fresh():
            is_live = self.config.get('enable_live_obstacles', False)
            gap, centers, idx, closest = self.observe_clearance()
            if self.state == 'running':
                self.min_observed_clearance = min(self.min_observed_clearance, gap)
                if not is_live:
                    elbow = self.hand+np.array([self.config['arm_length'], 0, 0])
                    self.min_home_clearance = min(self.min_home_clearance,
                        self.geometry.clearance(self.home, self.hand, elbow, self.config['arm_radius'])[0])
                self.max_excursion = max(self.max_excursion, float(np.max(np.abs(self.positions-self.home))))
                if gap < self.config['min_clearance_th']:
                    self.fail('デモ停止距離に到達')
                elif not self.geometry.has_internal_clearance(centers):
                    self.fail('自己干渉・床・作業台の外接形状余裕不足')
                else:
                    desired = None if is_live else self.scenario()
                    if is_live:
                        self.phase = 'live_pointcloud'
                    now_sec = self.get_clock().now().nanoseconds*1e-9
                    if self.state == 'running' and now_sec >= self.next_control_sec:
                        self.next_control_sec = now_sec+self.config['control_period_sec']
                        # 障害物の次更新位置も含む保守的な接近先での評価
                        predicted_hand = None if is_live else (desired if desired[0] < self.hand[0] else self.hand)
                        predicted_elbow = None if is_live else predicted_hand+np.array([self.config['arm_length'], 0, 0])
                        step = quintic_max_step(self.config['max_joint_velocity'], self.config['control_period_sec'])
                        target, has_candidate = self.select_target(predicted_hand, predicted_elbow, step)
                        if not has_candidate:
                            self.fail('退避候補なし')
                        else:
                            self.publish_target(target)
                            self.update_obstacle(desired)
            for name, trail in self.trails.items():
                points = centers[[i for i, shape in enumerate(self.geometry.spheres) if shape[0] == name]]
                if not len(points):
                    continue
                point = points.mean(axis=0)
                if not trail or np.linalg.norm(point-trail[-1]) > .003:
                    trail.append(point)
                    del trail[:-250]
            self.last_visual = (gap, centers[idx], closest, self.geometry.radii[idx])
            self.publish_markers(*self.last_visual)
        elif self.last_visual is not None:
            self.publish_markers(*self.last_visual, is_stale=True)
        def finite(value):
            return float(value) if np.isfinite(value) else None
        self.status.publish(String(data=json.dumps({
            'state': self.state, 'phase': self.phase, 'error': self.error,
            'run_generation': self.run_generation, 'run_start_stamp_sec': self.run_start_stamp_sec,
            'is_stop_latched': self.is_stop_latched,
            'side': self.config['sides'][min(self.side_idx, len(self.config['sides'])-1)],
            'clearance_m': gap, 'min_clearance_m': finite(self.min_observed_clearance),
            'min_home_clearance_m': finite(self.min_home_clearance), 'max_excursion_rad': self.max_excursion,
            'joint_age_sec': time.monotonic()-self.joint_time,
            'obstacle_age_sec': time.monotonic()-self.obstacle_time,
            'compute_ms': (time.monotonic()-began)*1000,
        }, allow_nan=False)))

    def publish_markers(self, gap, robot_point, human_point, robot_radius, is_stale=False):
        message = MarkerArray()
        def marker(idx, kind, position, scale, color):
            item = Marker()
            item.header.frame_id = 'world'
            item.header.stamp = self.get_clock().now().to_msg()
            item.ns, item.id, item.type, item.action = 'human_avoidance', idx, kind, Marker.ADD
            item.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
            item.pose.orientation.w = 1.0
            item.scale.x, item.scale.y, item.scale.z = map(float, scale)
            item.color.r, item.color.g, item.color.b, item.color.a = map(float, color)
            message.markers.append(item)
            return item
        radius, length = self.config['arm_radius'], self.config['arm_length']
        color = [1.0, 0.5, 0.1, 0.9]
        if not self.config.get('enable_live_obstacles', False):
            for idx, point in enumerate((self.hand, self.hand+np.array([length, 0, 0]))):
                marker(idx, Marker.SPHERE, point, [2*radius]*3, color)
            item = marker(2, Marker.CYLINDER, self.hand+np.array([length/2, 0, 0]), [2*radius, 2*radius, length], color)
            item.pose.orientation.y = np.sqrt(0.5)
            item.pose.orientation.w = np.sqrt(0.5)
        else:
            radius = self.cell_radius
        color = [1, 0, 0, 1] if is_stale or gap < self.config['target_clearance'] else [0, 1, 1, 1]
        item = marker(3, Marker.LINE_LIST, [0, 0, 0], [.006, 0, 0], color)
        direction = human_point-robot_point
        direction /= max(float(np.linalg.norm(direction)), 1e-12)
        robot_point = robot_point+direction*robot_radius
        human_point = human_point-direction*radius
        item.points = [Point(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in (robot_point, human_point)]
        item = marker(4, Marker.TEXT_VIEW_FACING, [0, 0, .85], [0, 0, .04], [1, 1, 1, 1])
        item.text = (f'{self.state} / stale state' if is_stale else
                     f'{self.state} / {self.phase} / clearance {gap:.3f} m')
        for idx, trail in enumerate(self.trails.values()):
            item = marker(5+idx, Marker.LINE_STRIP, [0, 0, 0], [.005, 0, 0],
                          [0.2, 1, 0.3, 1] if idx == 0 else [0.6, 0.4, 1, 1])
            item.points = [Point(x=float(point[0]), y=float(point[1]), z=float(point[2])) for point in trail]
        self.markers.publish(message)


def main():
    rclpy.init()
    node = None
    try:
        node = avoidance_demo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
