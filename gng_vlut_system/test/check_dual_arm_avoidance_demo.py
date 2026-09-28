#!/usr/bin/env python3
"""実Gazeboでの接近・退避・復帰、可視化、状態失効停止の検証。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml

import rclpy
from gazebo_msgs.srv import SetEntityState
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--params-file', default='/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml')
    parser.add_argument('--namespace', default='sim_topo_dual_arm_max')
    parser.add_argument('--launch-file', default='dual_arm_avoidance_demo.launch.py')
    parser.add_argument('--expected-nodes', type=int, default=10000)
    parser.add_argument('--avoidance-config', default='')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('avoidance_demo_check')
    params = yaml.safe_load(Path(args.params_file).read_text())['/**']['ros__parameters']
    limits = {joint.get('name'): {key: float(joint.find('limit').get(key)) for key in ('effort', 'velocity')}
              for joint in ET.parse(params['urdf_path']).getroot().findall('joint') if joint.get('type') != 'fixed'}
    measured_limits = {}
    limit_events = []
    first_joint_samples = []
    positions, status, phases = {}, {}, set()
    marker_count = [0]
    max_velocity = [0.0]
    max_position_velocity = [0.0]
    last_joint_sample = [None]
    compute_ms = []
    history = []
    gng_history = []

    def on_state(message):
        if len(first_joint_samples) < 8:
            first_joint_samples.append({'stamp': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                                        'names': list(message.name), 'positions': list(message.position),
                                        'velocities': list(message.velocity)})
        for idx, name in enumerate(message.name):
            joint_name = name if name in limits else name.removesuffix('_mimic')
            if joint_name not in limits:
                continue
            values = measured_limits.setdefault(joint_name, {'max_effort_nm': 0.0, 'max_velocity_rad_sec': 0.0,
                                                             'max_position_velocity_rad_sec': 0.0})
            for field, data in (('max_effort_nm', message.effort), ('max_velocity_rad_sec', message.velocity)):
                if idx < len(data):
                    assert math.isfinite(data[idx]), (name, field, data[idx])
                    if abs(data[idx]) > limits[joint_name]['effort' if field == 'max_effort_nm' else 'velocity']*1.02 and len(limit_events) < 40:
                        limit_events.append({'joint': joint_name, 'field': field, 'value': data[idx],
                                             'stamp': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                                             'status': dict(status)})
                    values[field] = max(values[field], abs(data[idx]))
        sample = dict(zip(message.name, message.position))
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        previous = last_joint_sample[0]
        if previous is not None and stamp > previous[0] and status.get('state') == 'running':
            # 位置直接設定時の速度欄と区別した、関節角の時刻差分
            rates = [abs(value-previous[1][name])/(stamp-previous[0])
                     for name, value in sample.items() if name in previous[1]]
            for name, value in sample.items():
                joint_name = name if name in limits else name.removesuffix('_mimic')
                if name in previous[1] and joint_name in measured_limits:
                    values = measured_limits[joint_name]
                    values['max_position_velocity_rad_sec'] = max(values['max_position_velocity_rad_sec'],
                        abs(value-previous[1][name])/(stamp-previous[0]))
            if rates:
                max_position_velocity[0] = max(max_position_velocity[0], max(rates))
        last_joint_sample[0] = (stamp, sample)
        positions.update(sample)
        if message.velocity and status.get('state') == 'running':
            max_velocity[0] = max(max_velocity[0], max(abs(value) for value in message.velocity))

    def on_status(message):
        previous = (status.get('state'), status.get('phase'), status.get('side'))
        status.clear()
        status.update(json.loads(message.data))
        current = (status['state'], status['phase'], status['side'])
        if previous != current:
            print('status', status, flush=True)
        phases.add(current)
        if status['state'] == 'running':
            compute_ms.append(status['compute_ms'])
            history.append(dict(status))

    def on_marker(message):
        assert len(message.markers) == 7
        assert all(item.header.frame_id == 'world' for item in message.markers)
        marker_count[0] += 1

    subscriptions = [
        node.create_subscription(JointState, f'/{args.namespace}/joint_states', on_state, 10),
        node.create_subscription(String, f'/{args.namespace}/avoidance/status', on_status, 10),
        node.create_subscription(MarkerArray, f'/{args.namespace}/avoidance/markers', on_marker, 10),
    ]
    subscriptions.append(node.create_subscription(String, f'/{args.namespace}/avoidance/gng_status',
        lambda message: gng_history.append(json.loads(message.data)), 10))
    process = None

    def wait_until(predicate, timeout_sec=30, allow_fault=False):
        deadline = time.monotonic()+timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if status.get('state') == 'fault' and not allow_fault:
                raise AssertionError(status)
            if predicate():
                return
            if process.poll() is not None:
                raise AssertionError(f'launch終了: {process.returncode}')
        raise AssertionError(f'待機上限: {status}')

    def call(name, service_type=Trigger, allow_fault=False):
        client = node.create_client(service_type, name)
        wait_until(client.service_is_ready, allow_fault=allow_fault)
        future = client.call_async(service_type.Request())
        wait_until(future.done, allow_fault=allow_fault)
        response = future.result()
        if hasattr(response, 'success'):
            assert response.success, response.message
        node.destroy_client(client)

    command = ['ros2', 'launch', 'gng_vlut_system', args.launch_file,
               'gui:=false', 'enable_auto_start:=false',
               'gazebo_master_uri:=http://127.0.0.1:11359', f'params_file:={args.params_file}']
    if args.avoidance_config:
        command.append(f'avoidance_config:={args.avoidance_config}')
    print('launch', command, flush=True)
    try:
        with (args.output/'gazebo.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait_until(lambda: status.get('state') == 'idle' and len(positions) >= 19, 180)
            call(f'/{args.namespace}/avoidance/start')
            wait_until(lambda: status.get('state') == 'completed', 600)
            report = dict(status)
            if 'gng_lidar' in args.launch_file:
                assert gng_history and max(row['num_selected_gng'] for row in gng_history) > 0
                assert max(row['num_danger']+row['num_collision'] for row in gng_history) > 0
                report['gng'] = gng_history[-1]
                assert sum(report['gng'][key] for key in ('num_safe', 'num_danger', 'num_collision')) == args.expected_nodes
            assert report['min_clearance_m'] > 0.035, report
            assert report['min_home_clearance_m'] < 0.0, report
            assert report['max_excursion_rad'] > 0.2, report
            assert marker_count[0] > 100, marker_count
            for side in ('left', 'right'):
                for phase in ('approaching', 'near', 'withdrawing', 'returning'):
                    assert ('running', phase, side) in phases, (side, phase, phases)
            # 全可動関節のURDF上限照合。単位ごとの丸め許容差1e-6
            assert set(measured_limits) == set(limits), (measured_limits.keys(), limits.keys())
            for name, values in measured_limits.items():
                assert values['max_effort_nm'] <= limits[name]['effort']+1e-6, (name, values, limits[name])
                assert values['max_velocity_rad_sec'] <= limits[name]['velocity']+1e-6, (name, values, limits[name])
                assert values['max_position_velocity_rad_sec'] <= limits[name]['velocity']+1e-6, (name, values, limits[name])
            report['joint_limits'] = measured_limits
            # 開始直後のGazebo停止と再開後の指令保持確認
            call(f'/{args.namespace}/avoidance/start')
            wait_until(lambda: status.get('state') == 'running')
            call('/pause_physics', Empty)
            wait_until(lambda: status.get('state') == 'fault', 10, allow_fault=True)
            paused = dict(positions)
            call('/unpause_physics', Empty, allow_fault=True)
            until = time.monotonic()+1.0
            wait_until(lambda: time.monotonic() > until, 5, allow_fault=True)
            assert status['state'] == 'fault', status
            drift = max(abs(positions[name]-value) for name, value in paused.items())
            assert drift < 0.03, drift
            report.update(result='passed', marker_messages=marker_count[0],
                          max_joint_velocity_rad_sec=max_velocity[0],
                          max_position_velocity_rad_sec=max_position_velocity[0], resume_drift_rad=drift,
                          mean_compute_ms=sum(compute_ms)/len(compute_ms), max_compute_ms=max(compute_ms),
                          state_loss_error=status['error'])
            call(f'/{args.namespace}/avoidance/stop', allow_fault=True)
            wait_until(lambda: status.get('state') == 'idle', allow_fault=True)
            if 'gng_lidar' in args.launch_file:
                # ロボット状態の更新を維持したまま、センサ視野を空へ移動した欠測試験
                call(f'/{args.namespace}/avoidance/start')
                client = node.create_client(SetEntityState, '/avoidance_demo/set_entity_state')
                wait_until(client.service_is_ready)
                request = SetEntityState.Request()
                request.state.name = 'avoidance_lidar'
                request.state.reference_frame = 'world'
                request.state.pose.position.z = 10.0
                request.state.pose.orientation.w = 1.0
                future = client.call_async(request)
                wait_until(future.done)
                assert future.result().success
                wait_until(lambda: status.get('state') == 'fault', 10, allow_fault=True)
                assert status['joint_age_sec'] < 1.0, status
                report['lidar_loss_error'] = status['error']
                request.state.pose.position.x = .85
                request.state.pose.position.z = .75
                request.state.pose.orientation.x = -math.sin(.2)
                request.state.pose.orientation.z = math.cos(.2)
                request.state.pose.orientation.w = 0.0
                future = client.call_async(request)
                wait_until(future.done, allow_fault=True)
                assert future.result().success
                until = time.monotonic()+2.0
                wait_until(lambda: time.monotonic() > until, 5, allow_fault=True)
                assert status['state'] == 'fault', status
                node.destroy_client(client)
                call(f'/{args.namespace}/avoidance/stop', allow_fault=True)
            (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            (args.output/'history.json').write_text(json.dumps(history)+'\n')
            (args.output/'gng_history.json').write_text(json.dumps(gng_history)+'\n')
            print(json.dumps(report), flush=True)
    finally:
        (args.output/'joint_limits.json').write_text(json.dumps({'urdf': limits, 'measured': measured_limits, 'last_positions': positions, 'limit_events': limit_events, 'first_joint_samples': first_joint_samples}, indent=2)+'\n')
        (args.output/'history.json').write_text(json.dumps(history)+'\n')
        (args.output/'gng_history.json').write_text(json.dumps(gng_history)+'\n')
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        if process is not None:
            print('stopped_launch', process.pid, flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
