"""独立ROSドメイン227での生成・後発購読・保存再利用の有限起動試験。"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '227'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def stop(process):
    if process.poll() is None:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if sig == signal.SIGINT:
                process.send_signal(sig)
            else:
                os.killpg(process.pid, sig)
            try:
                process.wait(timeout=8)
                break
            except subprocess.TimeoutExpired:
                continue
    assert process.poll() is not None


def main():
    rclpy.init()
    node = rclpy.create_node('mobile_graph_startup_check')
    processes = []
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert set(node.get_node_names()) == {'mobile_graph_startup_check'}, 'ROSドメイン227は使用中です'
        with tempfile.TemporaryDirectory(prefix='mobile_graph_check_') as directory:
            path = Path(directory)/'graph.json'
            command = ['ros2', 'launch', 'gng_vlut_system', 'mobile_state_graph.launch.py',
                       'output_path:='+str(path)]
            process = subprocess.Popen(command+['build_only:=true'], start_new_session=True)
            processes.append(process)
            assert process.wait(timeout=40) == 0
            original = path.read_bytes()
            saved = json.loads(original)
            assert saved['state_fields'] == ['x', 'y', 'yaw', 'v', 'omega']
            assert len(saved['states']) > 100 and saved['collision_checked'] is False
            process = subprocess.Popen(command, start_new_session=True)
            processes.append(process)
            # 配信済みデータを後発購読するためのpublisher検出待ち。
            topic = '/fuzzbot/state_graph/data'
            deadline = time.monotonic()+30
            while node.count_publishers(topic) == 0 and time.monotonic() < deadline:
                assert process.poll() is None
                rclpy.spin_once(node, timeout_sec=.1)
            assert node.count_publishers(topic) == 1
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            received = {}
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, topic, lambda msg: received.update(data=msg), qos)
            node.create_subscription(MarkerArray, '/fuzzbot/state_graph/markers',
                                     lambda msg: received.update(markers=msg), qos)
            deadline = time.monotonic()+20
            while len(received) != 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert len(received) == 2, '後発購読でデータを受信できません'
            data = json.loads(received['data'].data)
            assert data['frame_id'] == 'world' and data['states'] == saved['states']
            markers = received['markers'].markers
            assert len(markers) == 3 and all(m.header.frame_id == 'world' for m in markers)
            assert len(markers[0].points) == len(saved['states'])
            assert len(markers[1].points) == 2*len(saved['states'])
            assert len(markers[2].points) >= 2*len(saved['edges'])
            assert all(math.isfinite(v) for m in markers for p in m.points for v in (p.x, p.y, p.z))
            pubs = node.get_publisher_names_and_types_by_node('mobile_state_graph', '/fuzzbot')
            assert {name for name, _ in pubs} <= {
                topic, '/fuzzbot/state_graph/markers', '/parameter_events', '/rosout'}
            assert path.read_bytes() == original
            print(f'PASS nodes={len(saved["states"])} edges={len(saved["edges"])} '
                  f'build_ms={saved["build_ms"]:.2f} cache_bytes={len(original)}', flush=True)
    finally:
        for process in reversed(processes):
            stop(process)
        node.destroy_node()
        rclpy.shutdown()
        print('試験launchはすべて停止済み', flush=True)


if __name__ == '__main__':
    main()
