#!/usr/bin/env python3
"""実Gazeboの外部指令モードにおける部分関節追従の検証。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise RuntimeError('検証専用ROS_DOMAIN_ID=88が必要です')
    args.output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(Path('/ros2_ws/src/gng_vlut_system/config/dual_arm_gazebo_demo.yaml').read_text())
    config['dual_arm_gazebo_demo'].update(namespace='sim_joint_control_gazebo', enable_viewer=False, enable_gui=False)
    config_path = args.output/'demo.yaml'
    config_path.write_text(yaml.safe_dump(config))
    rclpy.init()
    node = Node('joint_control_gazebo_check')
    namespace = '/sim_joint_control_gazebo'
    positions, status = {}, {}
    samples = []
    def on_joint(msg):
        positions.update(zip(msg.name,msg.position))
        samples.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        'positions':dict(zip(msg.name,msg.position))})
    subscriptions = [node.create_subscription(JointState,namespace+'/joint_states',on_joint,100),
        node.create_subscription(String,namespace+'/joint_control/status',
                                 lambda msg:(status.clear(),status.update(json.loads(msg.data))),10)]
    gripper_pub = node.create_publisher(JointState,namespace+'/gripper_commands',10)
    command = ['ros2','launch','gng_vlut_system','dual_arm_gng_lidar_demo.launch.py',
               'gui:=false','enable_external_control:=true','enable_auto_start:=false',
               'gazebo_master_uri:=http://127.0.0.1:11488','demo_config:='+str(config_path)]
    process = None
    begin = time.monotonic()
    def wait(predicate,max_sec=60):
        deadline = time.monotonic()+max_sec
        while not predicate():
            rclpy.spin_once(node,timeout_sec=.02)
            if process.poll() is not None or time.monotonic()>deadline:
                raise AssertionError({'status':status,'positions':positions,'process':process.poll()})
    try:
        with (args.output/'gazebo.log').open('w') as log:
            process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (args.output/'command.json').write_text(json.dumps(command))
            (args.output/'ownership.json').write_text(json.dumps({'pid':process.pid,'pgid':process.pid}))
            wait(lambda:len(positions)>=21 and gripper_pub.get_subscription_count()>0 and
                 status.get('state')=='waiting_command',120)
            initial = dict(positions)
            start_idx = len(samples)
            msg = JointState()
            msg.name,msg.position = ['L_gripper_joint'],[0.2]
            gripper_pub.publish(msg)
            wait(lambda:status.get('state')=='tracking' and abs(positions['L_gripper_joint']-.2)<.025)
            stamp = samples[-1]['stamp']
            wait(lambda:samples[-1]['stamp']-stamp>1.0)
            max_other_change = max(abs(row['positions'][name]-value) for row in samples[start_idx:]
                                   for name,value in initial.items() if 'gripper' not in name)
            assert max_other_change < .06, max_other_change
            assert abs(positions['L_gripper_joint']-.2)<.025
            assert abs(positions['L_gripper_mimic']+.2)<.025
            assert abs(positions['R_gripper_joint']-initial['R_gripper_joint'])<.025
            report = {'result':'passed','initial_positions':initial,'final_positions':positions,
                      'max_other_joint_change_rad':max_other_change,'wall_sec':time.monotonic()-begin,
                      'status':status}
            (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
