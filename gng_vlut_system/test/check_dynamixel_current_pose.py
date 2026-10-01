"""疑似シリアルによる現在姿勢の変換・欠測・読取り専用通信の検証。"""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml


def crc(data):
    value = 0
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x8005 if value & 0x8000 else 0)) & 0xffff
    return value


def response(id_value, data):
    packet = b'\xff\xff\xfd\x00' + bytes([id_value]) + struct.pack('<H', len(data)+4) + b'\x55\x00' + data
    return packet + struct.pack('<H', crc(packet))


def main():
    workspace = Path(__file__).resolve().parents[2]
    mapping = yaml.safe_load((workspace/'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_ids_31_52.yaml').read_text())['/**']['ros__parameters']
    master, slave = pty.openpty()
    state = {'is_running': True, 'has_missing': False, 'pulse_offset': 0, 'instructions': [], 'errors': []}

    def emulate():
        buffer = b''
        while state['is_running']:
            if not select.select([master], [], [], 0.1)[0]:
                continue
            try:
                buffer += os.read(master, 4096)
                while len(buffer) >= 7:
                    length = 7 + int.from_bytes(buffer[5:7], 'little')
                    if len(buffer) < length:
                        break
                    packet, buffer = buffer[:length], buffer[length:]
                    assert packet[:4] == b'\xff\xff\xfd\x00'
                    assert crc(packet[:-2]) == int.from_bytes(packet[-2:], 'little')
                    instruction = packet[7]
                    state['instructions'].append(instruction)
                    if instruction == 1:
                        os.write(master, response(packet[4], struct.pack('<HB', 1020, 50)))
                    elif instruction == 0x82:
                        assert struct.unpack('<HH', packet[8:12]) == (132, 4)
                        for id_value in packet[12:-2]:
                            if state['has_missing'] and id_value == 52:
                                continue
                            pulse = 2048 + id_value + state['pulse_offset']
                            os.write(master, response(id_value, struct.pack('<i', pulse)))
                    else:
                        raise AssertionError('読取り以外の命令: '+str(instruction))
            except Exception as error:
                state['errors'].append(str(error))
                return

    thread = threading.Thread(target=emulate, daemon=True)
    thread.start()
    rclpy.init()
    node = rclpy.create_node('dynamixel_current_pose_check')
    enable_viewer = '--viewer' in sys.argv
    viewer_poses = []
    received = {'joint_states': [], 'viewer_joint_states': []}
    subscriptions = [node.create_subscription(
        JointState, '/ToPoDualArm/'+name,
        lambda msg, topic=name: received[topic].append(dict(zip(msg.name, msg.position))), 10)
        for name in received]
    def receive_viewer(msg):
        robot = json.loads(msg.data)['robot']
        viewer_poses.append(dict(zip(robot['jointNames'], robot['jointValues'])))
    subscriptions.append(node.create_subscription(String, '/viewer/internal/stream/robot/pose',
                                                  receive_viewer, qos_profile_sensor_data))
    command = ['ros2', 'launch', str(workspace/'gng_vlut_system/launch/dynamixel_current_pose.launch.py'),
               'enable_viewer:='+str(enable_viewer).lower(), 'device_name:='+os.ttyname(slave)]
    process = None
    report = {}

    def spin_until(predicate, duration=12):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert not state['errors'], state['errors']
            assert process.poll() is None, '起動プロセスの途中終了'
            if predicate():
                return
        raise AssertionError('受信待ちタイムアウト')

    def expected():
        values = {name: ((id_value+state['pulse_offset'])*360/4096+offset)*math.pi/180*scale
                for id_value, name, scale, offset in zip(mapping['joint_ids'], mapping['joint_names'],
                                                        mapping['joint_scales'], mapping['joint_offsets_deg'])}
        values.update(zip(mapping.get('fixed_joint_names', []), mapping.get('fixed_joint_positions', [])))
        return values

    try:
        with open('/tmp/dynamixel_current_pose_check.log', 'w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            # Viewer初回ゼロ姿勢の直後ではなく、実測変換値の到着後の比較
            spin_until(lambda: all(len(values) >= 3 and set(values[-1]) == set(expected()) and
                                   all(abs(values[-1][name]-value) < 1e-10
                                       for name, value in expected().items())
                                   for values in received.values()))
            for values in received.values():
                assert set(values[-1]) == set(expected())
                assert all(abs(values[-1][name]-value) < 1e-10 for name, value in expected().items())
            if enable_viewer:
                spin_until(lambda: viewer_poses and all(abs(viewer_poses[-1][name]-value) < 1e-10
                                                        for name, value in expected().items()))
                assert 'joint_command_output' not in node.get_node_names()
            state['pulse_offset'] = -4096
            spin_until(lambda: all(values and abs(values[-1]['R_joint1']-expected()['R_joint1']) < 1e-10
                                   for values in received.values()))
            if enable_viewer:
                spin_until(lambda: abs(viewer_poses[-1]['R_joint1']-expected()['R_joint1']) < 1e-10)
            state['has_missing'] = True
            end = time.monotonic()+0.4
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            before = {name:len(values) for name, values in received.items()}
            end = time.monotonic()+0.8
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert before == {name:len(values) for name, values in received.items()}
            state['has_missing'] = False
            spin_until(lambda: all(len(values) > before[name]+2 for name, values in received.items()))
            topics = node.get_publisher_names_and_types_by_node('dynamixel_current_pose', '/ToPoDualArm')
            assert not any('command' in name or 'control_claim' in name for name, _ in topics)
            report = {'joint_values':len(expected()), 'servo_ids':18, 'conversion':'pass', 'signed_position':'pass',
                      'missing_and_recovery':'pass', 'command_publishers':0, 'launch_command':command}
            report['viewer_pose_stream'] = 'pass' if enable_viewer else 'not_tested'
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        state['is_running'] = False
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        os.close(slave)
        os.close(master)
    assert not state['errors'], state['errors']
    assert set(state['instructions']) == {1, 0x82}
    report.update(instructions=sorted(set(state['instructions'])), processes_stopped=True)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
