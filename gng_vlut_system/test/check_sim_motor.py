#!/usr/bin/env python3
"""共通ROSトピックによるシミュレータの速度追従・トルク飽和・停止試験。"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as et

import numpy as np
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from check_gazebo_software_stop import owned_launch, process_snapshot, save_json


package_dir = Path(__file__).resolve().parents[1]


def fixture_urdf(path):
    """慣性0.1 kg m²、駆動上限0.5 N m、重力軸回転の検証モデル。"""
    path.write_text('''<robot name="motor_fixture">
      <link name="base"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="rotor"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual></link>
      <joint name="motor_joint" type="revolute"><parent link="base"/><child link="rotor"/>
        <origin xyz="0 0 1"/><axis xyz="0 0 1"/><limit lower="-100" upper="100" velocity="20" effort="0.5"/>
        <dynamics damping="0" friction="0"/></joint></robot>''')
    return path


class motor_trial:
    def __init__(self, model, output, enable_topic_remap=False, viewer_stream_topic="", backend="harmonic"):
        self.model, self.output = model, output
        self.backend = backend
        self.namespace = '/sim_motor_check'
        self.enable_topic_remap = enable_topic_remap
        self.state_topic = '/robot_io/joint_states' if enable_topic_remap else self.namespace+'/joint_states'
        self.trajectory_topic = '/robot_io/joint_trajectory' if enable_topic_remap else self.namespace+'/dual_arm_controller/joint_trajectory'
        self.description_topic = '/robot_io/robot_description' if enable_topic_remap else self.namespace+'/robot_description'
        self.description, self.transforms, self.static_transforms, self.clocks = [], [], [], []
        self.node = rclpy.create_node('motor_tracking_check')
        self.viewer_poses = []
        self.viewer_stream_topic = viewer_stream_topic
        if viewer_stream_topic:
            self.node.create_subscription(String, viewer_stream_topic,
                lambda msg: self.viewer_poses.append(json.loads(msg.data)["robot"]), qos_profile_sensor_data)
        self.joints = None
        self.states, self.samples = [], []
        self.stage = 'startup'
        self.node.create_subscription(TFMessage, '/tf', self.transforms.append, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, '/tf_static', self.static_transforms.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Clock, '/clock', self.clocks.append, qos_profile_sensor_data)
        self.node.create_subscription(JointState, self.state_topic, self.on_joints, qos_profile_sensor_data)
        self.node.create_subscription(JointTrajectoryControllerState,
            self.namespace+'/dual_arm_controller/controller_state', self.on_state, qos_profile_sensor_data)
        self.command = self.node.create_publisher(JointTrajectory, self.trajectory_topic, 10)
        self.launch = owned_launch(output, process_snapshot())
        self.report = {'backend': backend, 'model': model, 'result': 'failed', 'checks': {}}

    def on_joints(self, msg):
        self.joints = msg
        self.samples.append({'stage': self.stage, 't': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'names': list(msg.name), 'q': list(msg.position), 'v': list(msg.velocity), 'effort': list(msg.effort)})

    def on_state(self, msg):
        ref = msg.reference if hasattr(msg, 'reference') else msg.desired
        actual = msg.feedback if hasattr(msg, 'feedback') else msg.actual
        self.states.append({'stage': self.stage, 't': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'names': list(msg.joint_names), 'q_ref': list(ref.positions), 'v_ref': list(ref.velocities),
            'q': list(actual.positions), 'v': list(actual.velocities),
            'output': list(msg.output.effort) if hasattr(msg, 'output') else []})

    def wait(self, predicate, max_sec=60):
        deadline = time.monotonic()+max_sec
        while not predicate():
            rclpy.spin_once(self.node, timeout_sec=.02)
            if self.launch.process.poll() is not None:
                raise RuntimeError('シミュレータの早期終了')
            if time.monotonic() > deadline:
                raise TimeoutError(self.stage)

    def advance(self, duration):
        begin = self.samples[-1]['t']
        self.wait(lambda: self.samples[-1]['t']-begin >= duration)

    def send(self, points):
        msg = JointTrajectory(joint_names=self.names)
        for t, q, v in points:
            point = JointTrajectoryPoint(positions=q, velocities=v)
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int(round((t-int(t))*1e9))
            msg.points.append(point)
        self.command.publish(msg)

    def positions(self):
        measured = dict(zip(self.joints.name, self.joints.position))
        return [measured[name] for name in self.names]

    def execute(self):
        urdf = (fixture_urdf(self.output/'fixture.urdf') if self.model == 'fixture' else
                package_dir.parent/'urdf'/('topo_dual_arm_max'+('_long' if self.model == 'long' else ''))/'topo_dual_arm_max.urdf')
        root = et.parse(urdf).getroot()
        joints = [j for j in root.findall('joint') if j.get('type') != 'fixed' and j.find('mimic') is None]
        self.names = [j.get('name') for j in joints]
        limits = {j.get('name'): float(j.find('limit').get('effort')) for j in joints}
        if self.backend == 'isaac':
            # Isaac本体は別起動。試験所有のTF・controller接続のみの起動
            command = ['ros2', 'launch', str(package_dir/'launch/dual_arm_isaac.launch.py'),
                       'namespace:=sim_motor_check']
        else:
            command = ['ros2', 'launch', str(package_dir/'launch/dual_arm_gz.launch.py'),
                       'urdf:='+str(urdf), 'namespace:=sim_motor_check', 'output_dir:='+str(self.output/'generated')]
            if self.enable_topic_remap:
                command += ['state_topic:='+self.state_topic, 'trajectory_topic:='+self.trajectory_topic,
                            'description_topic:='+self.description_topic]
        self.launch.start(command)
        self.wait(lambda: self.joints is not None and self.states and self.command.get_subscription_count() > 0, 100)
        self.advance(1.)
        # 起動後の購読者へのURDF再配信と標準状態・TF・時刻の整合確認
        self.node.create_subscription(String, self.description_topic, self.description.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.wait(lambda: self.description and self.transforms and self.static_transforms and self.clocks, 15)
        assert et.fromstring(self.description[-1].data).tag == 'robot'
        assert len(self.joints.name) == len(self.joints.position) == len(self.joints.velocity) == len(self.joints.effort)
        assert self.node.count_publishers(self.state_topic) == 1
        if self.enable_topic_remap:
            assert self.node.count_publishers(self.namespace+'/joint_states') == 0
            assert self.node.count_subscribers(self.namespace+'/dual_arm_controller/joint_trajectory') == 0
        self.report['checks']['standard_topics'] = {
            'state_topic': self.state_topic, 'trajectory_topic': self.trajectory_topic,
            'description_topic': self.description_topic, 'has_late_description': True,
            'has_tf': True, 'has_tf_static': True, 'has_clock': True}

        for speed in (.2, .5):
            self.stage = f'tracking_{speed}'
            home = np.asarray(self.positions())
            active_names = ['motor_joint'] if self.model == 'fixture' else ['L_joint4', 'R_joint4']
            active = [self.names.index(name) for name in active_names]
            amplitude = .15
            omega = speed/amplitude
            duration = 2*math.pi/omega
            points = []
            for t in np.linspace(0, duration, int(duration/.01)+1):
                q, v = home.copy(), np.zeros(len(home))
                q[active] += amplitude*(1-math.cos(omega*t))
                v[active] = amplitude*omega*math.sin(omega*t)
                points.append((float(t), q.tolist(), v.tolist()))
            self.send(points)
            self.advance(duration+.2)
            rows = [r for r in self.states if r['stage'] == self.stage and len(r['v_ref']) == len(self.names)]
            error_v = np.array([np.array(r['v'])-r['v_ref'] for r in rows])[:, active]
            error_q = np.array([np.array(r['q'])-r['q_ref'] for r in rows])[:, active]
            result = {'num_samples': len(rows), 'velocity_rms_rad_sec': float(np.sqrt(np.mean(error_v**2))),
                      'max_position_error_rad': float(np.max(np.abs(error_q)))}
            self.report['checks'][self.stage] = result
            print(self.stage, result, flush=True)
            assert len(rows) > 50 and result['velocity_rms_rad_sec'] < .05 and result['max_position_error_rad'] < .05, result
        if self.model == 'fixture':
            self.stage = 'torque_saturation'
            home = self.positions()
            self.send([(.01, [home[0]+5.], [0.])])
            self.advance(.3)
            rows = [r for r in self.samples if r['stage'] == self.stage]
            t, v = np.array([r['t'] for r in rows]), np.array([r['v'][0] for r in rows])
            valid = np.diff(t) > 1e-5
            alpha = np.diff(v)[valid]/np.diff(t)[valid]
            result = {'max_acceleration_rad_sec2': float(np.max(np.abs(alpha))),
                      'max_inertial_torque_nm': float(.1*np.max(np.abs(alpha))),
                      'median_inertial_torque_nm': float(.1*np.median(np.abs(alpha))),
                      'max_velocity_rad_sec': float(np.max(np.abs(v)))}
            self.report['checks'][self.stage] = result
            print(self.stage, result, flush=True)
            assert .45 < result['median_inertial_torque_nm'] < .51 and result['max_inertial_torque_nm'] < .51, result
        if self.model != 'fixture':
            self.stage = 'moving_before_stop'
            home = np.asarray(self.positions())
            target = home.copy()
            target[active] += .4
            self.send([(0., home.tolist(), [0.]*len(home)), (.8, target.tolist(), [0.]*len(home))])
            self.advance(.4)
            measured = dict(zip(self.joints.name, self.joints.velocity))
            assert min(abs(measured[name]) for name in active_names) > .2, measured
        self.stage = 'stop_hold'
        hold = self.positions()
        self.send([(.01, hold, [0.]*len(hold))])
        self.advance(2.)
        rows = [r for r in self.samples if r['stage'] == self.stage]
        tail = [r for r in rows if r['t'] > rows[-1]['t']-.3]
        max_velocity = max(abs(v) for r in tail for v in r['v'])
        remaining_max = np.maximum.accumulate(np.array([max(abs(v) for v in r['v']) for r in rows])[::-1])[::-1]
        settled = np.flatnonzero(remaining_max < .02)
        stop_time = rows[int(settled[0])]['t']-rows[0]['t'] if len(settled) else None
        self.report['checks'][self.stage] = {'max_velocity_rad_sec': max_velocity,
                                            'settled_after_sec': stop_time}
        assert max_velocity < .02, max_velocity
        outputs = [abs(value)/limits[name] for row in self.states for name, value in zip(row['names'], row['output'])]
        self.report['checks']['controller_effort'] = {'num_samples': len(outputs), 'max_limit_ratio': max(outputs) if outputs else None}
        assert outputs and max(outputs) <= 1.0001, self.report['checks']['controller_effort']
        if self.viewer_stream_topic:
            # 同一シミュレーション時刻の複数実測サンプルを保持した照合
            measured = {}
            for row in self.samples:
                measured.setdefault(round(row['t'], 6), []).append(dict(zip(row['names'], row['q'])))
            errors = []
            for pose in self.viewer_poses:
                candidates = measured.get(round(pose['timestamp'], 6), [])
                if not candidates:
                    continue
                values = dict(zip(pose['jointNames'], pose['jointValues']))
                assert all(name in values for name in active_names)
                errors.append(min(max(abs(value-actual[name]) for name, value in values.items() if name in actual)
                                  for actual in candidates))
            assert len(errors) > 10 and max(errors) < 1e-9, (len(errors), max(errors) if errors else None)
            self.report['checks']['viewer_measured_state'] = {
                'num_pose_comparisons': len(errors), 'max_position_error_rad': max(errors)}
        self.report['result'] = 'passed'

    def close(self):
        self.report['cleanup'] = self.launch.cleanup()
        save_json(self.output/'report.json', self.report)
        save_json(self.output/'joint_samples.json', self.samples)
        save_json(self.output/'controller_samples.json', self.states)
        if self.viewer_stream_topic:
            save_json(self.output/'viewer_samples.json', self.viewer_poses)
        self.node.destroy_node()
        assert self.report['cleanup']['is_success'], self.report['cleanup']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['harmonic', 'isaac'], default='harmonic')
    parser.add_argument('--model', choices=['fixture', 'max', 'long'], required=True)
    parser.add_argument('--viewer-stream-topic', default='')
    parser.add_argument('--enable-topic-remap', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.backend == "isaac" and (args.enable_topic_remap or args.model == "fixture"):
        parser.error("Isaac外部接続の試験はmax／longと既定トピックが対象です")
    if os.environ.get('ROS_DOMAIN_ID') != '96':
        raise RuntimeError('検証専用ROS_DOMAIN_ID=96が必要です')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    trial = motor_trial(args.model, args.output.resolve(), args.enable_topic_remap, args.viewer_stream_topic, args.backend)
    try:
        trial.execute()
    except Exception as error:
        trial.report['error'] = str(error)
        raise
    finally:
        trial.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
