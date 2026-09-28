#!/usr/bin/env python3
"""Gazebo実測関節角による開始・停止・全姿勢復帰の検証。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--params-file', default='/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml')
    parser.add_argument('--namespace', default='sim_topo_dual_arm_max')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    log_path = args.output/'gazebo.log'
    rclpy.init()
    node = rclpy.create_node('dual_arm_demo_check')
    positions = {}
    status = {}
    max_positions = {}
    max_velocity = [0.0]
    max_motion_velocity = [0.0]
    state_count = [0]
    stop_positions = {}

    def on_state(message):
        positions.update(zip(message.name, message.position))
        for name, value in zip(message.name, message.position):
            max_positions[name] = max(max_positions.get(name, 0.0), abs(value))
        if message.velocity:
            max_velocity[0] = max(max_velocity[0], max(abs(value) for value in message.velocity))
        if message.velocity and status.get('state') == 'running':
            max_motion_velocity[0] = max(max_motion_velocity[0], max(abs(value) for value in message.velocity))
        state_count[0] += 1

    def on_status(message):
        status.clear()
        status.update(json.loads(message.data))
        print('status', status, flush=True)

    subscriptions = [node.create_subscription(JointState, f'/{args.namespace}/joint_states', on_state, 10),
        node.create_subscription(String, f'/{args.namespace}/demo/status', on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    start = node.create_client(Trigger, f'/{args.namespace}/demo/start')
    stop = node.create_client(Trigger, f'/{args.namespace}/demo/stop')

    def wait_until(predicate, timeout_sec=120, allow_fault=False):
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if status.get('state') == 'fault' and not allow_fault:
                raise AssertionError(status)
            if predicate():
                return
            if process.poll() is not None:
                raise AssertionError(f'Gazebo launch終了: {process.returncode}')
        raise AssertionError(f'待機上限超過: {status}; joint_states={state_count[0]}')

    def call(client):
        wait_until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        wait_until(future.done, 10)
        response = future.result()
        assert response.success, response.message

    command = ['ros2','launch','gng_vlut_system','dual_arm_gazebo_demo.launch.py',
               'gui:=false', 'enable_auto_start:=false', 'gazebo_master_uri:=http://127.0.0.1:11359',
               f'params_file:={args.params_file}']
    print('launch', command, flush=True)
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                wait_until(lambda: status.get('state') == 'idle' and len(positions) >= 19, 180)
                call(start)
                wait_until(lambda: abs(positions.get('L_joint4', 0.0)) > 0.05, 40)
                call(stop)
                wait_until(lambda: status.get('state') == 'idle', 15)
                stop_positions.update(positions)
                end = time.monotonic()+1.0
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                stop_drift = max(abs(positions[name]-value) for name, value in stop_positions.items())
                assert stop_drift < 0.03, stop_drift
                call(start)
                wait_until(lambda: status.get('state') == 'completed', 180)
                final_error = max(abs(value) for value in positions.values())
                assert final_error < 0.05, final_error
                for name in ['L_joint4','R_joint4','L_gripper_joint','R_gripper_joint']:
                    assert max_positions.get(name,0.0) > 0.15, (name,max_positions)
                assert max_velocity[0] < 0.25, max_velocity
                # Gazebo停止による状態配信失効と、再開時の軌道取消の確認
                call(start)
                wait_until(lambda: abs(positions.get('L_joint4', 0.0)) > 0.05, 40)
                pause = node.create_client(Empty, '/pause_physics')
                resume = node.create_client(Empty, '/unpause_physics')
                wait_until(pause.service_is_ready, 10)
                future = pause.call_async(Empty.Request())
                wait_until(future.done, 10)
                wait_until(lambda: status.get('state') == 'fault', 10, allow_fault=True)
                fault_error = status['error']
                paused_positions = dict(positions)
                future = resume.call_async(Empty.Request())
                wait_until(future.done, 10, allow_fault=True)
                end = time.monotonic()+1.0
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                resume_drift = max(abs(positions[name]-value) for name, value in paused_positions.items())
                assert resume_drift < 0.03, resume_drift
                assert status['state'] == 'fault'
                report = {'result':'passed', 'state_loss_error':fault_error,
                          'resume_drift_rad':resume_drift,
                          'max_motion_velocity_rad_sec':max_motion_velocity[0], 'joint_state_messages':state_count[0],
                          'max_velocity_rad_sec':max_velocity[0], 'stop_drift_rad':stop_drift,
                          'final_position_error_rad':final_error, 'max_positions':max_positions}
                (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
                print(json.dumps(report),flush=True)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                print('stopped_launch',process.pid,flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
