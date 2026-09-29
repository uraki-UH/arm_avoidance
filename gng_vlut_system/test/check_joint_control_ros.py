#!/usr/bin/env python3
"""隔離ROS環境での部分指令・仲裁・出力変換・失効の実通信検証。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from gng_control_msgs.msg import JointControlClaim
from dynamixel_handler_msgs.msg import DynamixelGoal, DynamixelPresent

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from joint_command_model import joint_command_model, dynamixel_mapping


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['viewer', 'gazebo', 'dynamixel'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--enable-dynamixel-input', action='store_true')
    parser.add_argument('--viewer-stack', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        raise RuntimeError('検証専用ROS_DOMAIN_ID=88が必要です')
    args.output.mkdir(parents=True, exist_ok=False)
    namespace = '/sim_joint_control_check'
    urdf_path = '/ros2_ws/src/urdf/topo_dual_arm_max/topo_dual_arm_max.urdf'
    model = joint_command_model(urdf_path)
    rclpy.init()
    node = Node('joint_control_check')
    active, viewer, status = {}, {}, {}
    trajectory, motors = [], []
    enable_feedback = True
    enable_present = False
    present_pub = node.create_publisher(DynamixelPresent, namespace+'/dynamixel_present', 10)
    mapping_config = yaml.safe_load(Path('/ros2_ws/src/dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge.yaml').read_text())['/**']['ros__parameters']
    mapping = dynamixel_mapping(model, mapping_config)
    leader_pose = {name:0.0 for name in mapping.covered_names}
    leader_pose['L_joint2'] = .2
    leader_pose['L_gripper_joint'] = .15
    present_ids, present_positions = mapping.convert(leader_pose)
    feedback = model.initial_positions()
    feedback['L_joint2'] = 0.45
    feedback['L_gripper_joint'] = 0.1
    feedback_pub = node.create_publisher(JointState, namespace+'/joint_states', 10)
    sources = {name:node.create_publisher(JointState, namespace+'/'+name, 10)
               for name in ('joint_commands', 'gripper_commands', 'leader_joint_states')}
    claim_pub = node.create_publisher(JointControlClaim, namespace+'/control_claims',
                                      rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    def update(target, msg):
        target.clear()
        target.update(zip(msg.name, msg.position))
    subscriptions = [
        node.create_subscription(JointState, namespace+'/active_joint_commands', lambda msg:update(active,msg), 10),
        node.create_subscription(JointState, namespace+'/viewer_joint_states', lambda msg:update(viewer,msg), 10),
        node.create_subscription(String, namespace+'/joint_control/status',
                                 lambda msg:(status.clear(),status.update(json.loads(msg.data))),10),
        node.create_subscription(JointTrajectory, namespace+'/dual_arm_controller/joint_trajectory', trajectory.append,10),
        node.create_subscription(DynamixelGoal, namespace+'/dynamixel_goal', motors.append,10),
    ]
    command = ['ros2','launch','gng_vlut_system','joint_control.launch.py',
               'params_file:=/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml',
               'robot_name:=sim_joint_control_check',f'backend:={args.backend}',
               'enable_dynamixel_input:='+str(args.enable_dynamixel_input).lower(),
               'dynamixel_input_topic:='+namespace+'/dynamixel_present','enable_direct_tracking:=true',
               'max_joint_velocity:=1.0','max_state_age_sec:=0.3',
               'dynamixel_topic:='+namespace+'/dynamixel_goal']
    if args.viewer_stack:
        assert args.backend == 'viewer'
        config = yaml.safe_load(Path('/ros2_ws/src/gng_vlut_system/config/topo_dual_arm_max.yaml').read_text())
        config['/**']['ros__parameters']['robot_name'] = 'sim_joint_control_check'
        params_path = args.output/'params.yaml'
        params_path.write_text(yaml.safe_dump(config,allow_unicode=True))
        command = ['ros2','launch','gng_vlut_system','gng_viewer_bridge.launch.py',
                   'params_file:='+str(params_path),'joint_control_backend:=viewer','direct_joint_tracking:=true',
                   'enable_dynamixel_input:='+str(args.enable_dynamixel_input).lower(),
                   'dynamixel_input_topic:='+namespace+'/dynamixel_present']
    process = None
    checks = []
    def send(topic, values):
        msg = JointState()
        msg.name, msg.position = list(values), [float(v) for v in values.values()]
        sources[topic].publish(msg)
    def spin():
        if enable_present:
            msg = DynamixelPresent()
            msg.id_list, msg.position_deg = present_ids, present_positions
            present_pub.publish(msg)
        if enable_feedback:
            msg = JointState()
            values = model.expand(feedback)
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name, msg.position = list(values), list(values.values())
            feedback_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.01)
        if process.poll() is not None:
            raise AssertionError('launch終了: '+str(process.returncode))
    def wait(predicate, max_sec=10.0):
        deadline = time.monotonic()+max_sec
        while not predicate():
            spin()
            if time.monotonic()>deadline:
                raise AssertionError({'status':status, 'active':active,'viewer':viewer})
    def duration(seconds):
        deadline = time.monotonic()+seconds
        wait(lambda:time.monotonic()>=deadline, seconds+2)
    def latest_positions():
        if args.backend == 'viewer':
            return viewer
        if args.backend == 'gazebo':
            if not trajectory:
                return {}
            msg = trajectory[-1]
            return dict(zip(msg.joint_names,msg.points[0].positions))
        if not motors:
            return {}
        return dict(zip(motors[-1].id_list,motors[-1].position_deg))
    def disable_source(name):
        msg = JointControlClaim()
        msg.command_topic, msg.enabled, msg.mode = name, False, msg.MODE_EXCLUSIVE
        claim_pub.publish(msg)
        duration(.15)
    try:
        with (args.output/'launch.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (args.output/'command.json').write_text(json.dumps(command))
            (args.output/'ownership.json').write_text(json.dumps({'pid':process.pid,'pgid':process.pid}))
            wait(lambda:all(pub.get_subscription_count()>0 for pub in sources.values()),30)
            duration(.3)
            assert not trajectory and not motors
            checks.append('指令前の物理出力なし')
            if args.backend == 'viewer':
                send('joint_commands', {'L_joint2':.45})
                wait(lambda:abs(viewer.get('L_joint2',0)-.45)<1e-6)
            send('gripper_commands', {'L_gripper_joint':.3})
            if args.backend == 'dynamixel':
                wait(lambda:abs(latest_positions().get(18,0)-math.degrees(.3)/-.015)<1e-5)
                assert set(latest_positions()) == {18}
                checks.append('単一グリッパのID18のみ出力')
            else:
                wait(lambda:abs(latest_positions().get('L_gripper_joint',0)-.3)<1e-6)
                assert abs(latest_positions()['L_joint2']-.45)<1e-6
                if args.backend == 'viewer':
                    assert abs(viewer['L_gripper_mimic']+.3)<1e-6
                else:
                    assert set(latest_positions()) == set(model.independent_names)
                checks.append('グリッパ部分指令で腕姿勢を保持')
            send('gripper_commands', {'L_gripper_joint':.35})
            wait(lambda:abs(active.get('L_gripper_joint',0)-.35)<1e-6)
            send('leader_joint_states', {'L_joint2':.6,'L_gripper_joint':.1,'L_gripper_mimic':-.1})
            wait(lambda:abs(active.get('L_joint2',0)-.6)<1e-6)
            assert active['L_gripper_joint'] == .35 and 'L_gripper_mimic' not in active
            checks.append('リーダー腕指令と独立グリッパ指令の合成')
            send('gripper_commands', {'L_gripper_joint':math.nan,'R_gripper_joint':0.7})
            duration(.1)
            assert active['L_gripper_joint'] == .35 and 'R_gripper_joint' not in active
            checks.append('非有限値の原子的拒否')
            if args.backend == 'viewer':
                disable_source('joint_commands')
            wait(lambda:'L_joint2' not in active)
            assert active['L_gripper_joint'] == .35
            checks.append('リーダー失効後も単発グリッパ目標を保持')
            release = node.create_client(Trigger, namespace+'/joint_control/release_gripper')
            wait(release.service_is_ready)
            future = release.call_async(Trigger.Request())
            wait(future.done)
            assert future.result().success
            wait(lambda:not active)
            send('gripper_commands', {'L_gripper_joint':.25})
            wait(lambda:abs(active.get('L_gripper_joint',0)-.25)<1e-6)
            checks.append('単発目標の解除後に新しい目標を受付')
            disable_source('gripper_commands')
            wait(lambda:not active)
            if args.backend != 'viewer':
                enable_feedback = False
                duration(.5)
                count = len(trajectory) if args.backend == 'gazebo' else len(motors)
                duration(.2)
                assert (len(trajectory) if args.backend == 'gazebo' else len(motors)) == count
                assert status['state']=='waiting_state'
                checks.append('実測失効時の物理出力停止')
            if args.enable_dynamixel_input:
                assert args.backend == 'viewer'
                wait(lambda:present_pub.get_subscription_count()>0)
                enable_present = True
                wait(lambda:abs(active.get('L_joint2',0)-.2)<1e-6)
                wait(lambda:abs(viewer.get('L_gripper_joint',0)-.15)<1e-6)
                checks.append('DynamixelPresentからリーダー入力への校正付き変換')
                claim = JointControlClaim()
                claim.command_topic, claim.enabled = 'gripper_commands', True
                claim.mode, claim.priority = claim.MODE_EXCLUSIVE, 200
                claim.joint_names = ['L_gripper_joint']
                claim_pub.publish(claim)
                duration(.2)
                send('gripper_commands', {'L_gripper_joint':.35})
                wait(lambda:abs(viewer.get('L_gripper_joint',0)-.35)<1e-6)
                assert abs(viewer['L_gripper_mimic']+.35)<1e-6
                checks.append('Dynamixel入力中のグリッパ部分上書き')
                enable_present = False
                wait(lambda:'L_joint2' not in active)
                assert active['L_gripper_joint']==.35
                checks.append('Dynamixel入力途絶時の関節単位失効')
            (args.output/'report.json').write_text(json.dumps({'backend':args.backend,'checks':checks,'result':'passed'},ensure_ascii=False,indent=2))
            print(json.dumps({'backend':args.backend,'checks':len(checks),'result':'passed'},ensure_ascii=False),flush=True)
    finally:
        if process is not None:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    if sig == signal.SIGINT and process.poll() is None:
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid,sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=10)
                    break
                except subprocess.TimeoutExpired:
                    pass
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
