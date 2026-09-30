"""隔離ROSドメインでの被覆補完モデルのViewer配信の受信検証。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ais_gng_msgs.msg import TopologicalMap
from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray
from std_msgs.msg import String


def has_process_group(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_launch(process):
    for signum, timeout_sec in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        process.poll()
        if not has_process_group(process.pid):
            break
        try:
            # 初回のlaunch自身への通知と、残った所有groupへの段階的な終了通知
            if signum == signal.SIGINT and process.returncode is None:
                process.send_signal(signum)
            else:
                os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            process.poll()
            if not has_process_group(process.pid):
                break
            time.sleep(.05)
    assert process.poll() is not None and not has_process_group(process.pid)


def main():
    def cancel(signum, frame):
        raise KeyboardInterrupt(f'中断シグナル: {signum}')

    signal.signal(signal.SIGTERM, cancel)
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    folder = args.folder
    expected = json.loads((folder / 'expected.json').read_text())
    robot_name = expected['robot_name']
    argv = ['ros2', 'launch', 'gng_vlut_system', 'gng_viewer_bridge.launch.py',
            f'params_file:={folder / "preview.yaml"}',
            'joint_control_backend:=viewer', 'enable_dynamixel_input:=false']
    result = {'argv': argv, 'domain_id': os.environ['ROS_DOMAIN_ID'], 'is_passed': False}
    rclpy.init()
    node = rclpy.create_node('coverage_repair_probe')
    received = {}
    subscriptions = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for suffix, msg_type in [('Tmap_static', TopologicalMap), ('Tmap_static_L0', TopologicalMap),
                             ('Tmap_static_L1', TopologicalMap),
                             ('topological_node_features', TopologicalNodeFeatureArray)]:
        subscriptions.append(node.create_subscription(msg_type, f'/{robot_name}/{suffix}',
                             lambda msg, key=suffix: received.__setitem__(key, msg), qos))

    def on_description(msg):
        data = json.loads(msg.data)
        if data.get('tag') == robot_name:
            received['robot_description'] = data

    subscriptions.append(node.create_subscription(String, '/viewer/internal/stream/robot/description',
                                                  on_description, qos))
    process = None
    start = time.monotonic()
    try:
        with (folder / 'launch.log').open('w') as log:
            process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            result['launch_pid'] = process.pid
            while time.monotonic() - start < 90 and len(received) < 5:
                assert process.poll() is None, 'launchの早期終了'
                rclpy.spin_once(node, timeout_sec=0.2)
            assert len(received) == 5, f'受信不足: {list(received)}'
            states = {int(key): value for key, value in expected['nodes'].items()}
            result['topics'] = {}
            for topic, layer in [('Tmap_static', 0), ('Tmap_static_L0', 0), ('Tmap_static_L1', 1)]:
                msg = received[topic]
                actual = {item.id: item for item in msg.nodes}
                assert len(actual) == len(msg.nodes) and set(actual) == set(states), topic
                max_error = 0.0
                for node_id, value in actual.items():
                    pos = (value.pos.x, value.pos.y, value.pos.z)
                    assert all(math.isfinite(item) for item in pos), (topic, node_id)
                    max_error = max(max_error, max(abs(a-b) for a,b in zip(pos, states[node_id]['tcp'][layer])))
                assert max_error < 1e-6, (topic, max_error)
                assert len(msg.edges) % 2 == 0
                if 'edge_counts' in expected:
                    assert len(msg.edges) // 2 == expected['edge_counts'][layer + 1], topic
                assert msg.header.frame_id == expected.get('frame_id', 'world'), msg.header.frame_id
                assert all(idx < len(msg.nodes) for idx in msg.edges)
                result['topics'][topic] = {'num_nodes': len(msg.nodes), 'num_edges': len(msg.edges)//2,
                                           'max_coord_error_m': max_error, 'frame_id': msg.header.frame_id}
            features = received['topological_node_features'].features
            assert len(features) == len(states)
            assert {item.node_id for item in features} == set(states)
            max_angle_error = 0.0
            for feature in features:
                angles = states[feature.node_id]['angles']
                assert len(feature.weight_angle) == len(angles)
                assert all(math.isfinite(item) for item in feature.weight_angle), feature.node_id
                max_angle_error = max(max_angle_error, max(abs(a-b) for a,b in zip(feature.weight_angle, angles)))
            assert max_angle_error < 1e-6, max_angle_error
            assert received['robot_description']['robot']
            result.update(is_passed=True, num_features=len(features), max_angle_error_rad=max_angle_error,
                          robot_description_tag=received['robot_description']['tag'])
    except BaseException as error:
        result['error'] = repr(error)
        raise
    finally:
        if process is not None:
            stop_launch(process)
            result['launch_returncode'] = process.returncode
            # 所有launchのprocess groupが消滅したことの確認
            try:
                os.killpg(process.pid, 0)
                result['has_remaining_process_group'] = True
            except ProcessLookupError:
                result['has_remaining_process_group'] = False
        node.destroy_node()
        rclpy.shutdown()
        result['elapsed_sec'] = time.monotonic() - start
        (folder / 'ros_verification.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    assert not result.get('has_remaining_process_group', False)
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
