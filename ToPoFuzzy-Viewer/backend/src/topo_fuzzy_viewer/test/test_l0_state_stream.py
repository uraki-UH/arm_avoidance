"""L0静的モデルの状態件数と、件数のみのROS→WS更新の隔離検証。"""

import os
import struct
import subprocess
import sys
import tempfile
import time

from test_stream_restart import Client, stop


def main():
    # 通常環境と分離したROSドメイン・WebSocketポート
    os.environ['ROS_DOMAIN_ID'] = '225'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
    from rclpy.qos import DurabilityPolicy, QoSProfile

    rclpy.init()
    node = rclpy.create_node('l0_state_stream_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static_topic, update_topic = '/l0_test/Tmap_static', '/l0_test/Tmap'
    received = []
    node.create_subscription(TopologicalMap, static_topic, received.append, qos)
    publisher = node.create_publisher(TopologicalMap, update_topic, qos)
    port = 19097
    processes = []
    client = None
    with tempfile.TemporaryFile() as log:
        try:
            commands = [
                ['/ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/visualization_gng_static_node',
                 '--ros-args', '-p', f'model_path:={sys.argv[1]}', '-p', f'topic_name:={static_topic}',
                 '-p', 'frame_id:=ToPoDualArm/base_link'],
                ['/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node',
                 '--ros-args', '-p', f'port:={port}'],
            ]
            for command in commands:
                processes.append(subprocess.Popen(command, stdout=log, stderr=log))
            deadline = time.monotonic() + 15
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert received, '静的モデルのROS受信なし'
            static = received[0]
            counts = lambda n: (n.num_safe_states, n.num_danger_states, n.num_collision_states)
            assert all(sum(counts(n)) > 0 for n in static.nodes)
            totals = tuple(sum(counts(n)[idx] for n in static.nodes) for idx in range(3))
            print(f'STATIC: nodes={len(static.nodes)} edges={len(static.edges) // 2} counts={totals}')
            graph = TopologicalMap()
            graph.header.frame_id = 'map'
            graph.nodes = [TopologicalNode(id=1, label=1, num_safe_states=1,
                                           num_danger_states=4, num_collision_states=95)]
            publisher.publish(graph)
            deadline = time.monotonic() + 15
            while client is None and time.monotonic() < deadline:
                try:
                    client = Client(port)
                except ConnectionRefusedError:
                    time.sleep(0.1)
            assert client is not None
            topics = {static_topic, update_topic}
            client.until(lambda v: isinstance(v, dict) and v.get('id') == 'sync_sources'
                         and topics <= {s['id'] for s in v['result']['sources']})
            for topic in topics:
                client.send({'id': topic, 'method': 'sources.setActive',
                             'params': {'sourceId': topic, 'active': True}})

            def decode(value):
                if not isinstance(value, bytes) or value[:4] != b'TMG1':
                    return None
                assert struct.unpack_from('<H', value, 4)[0] == 2
                tag_size, frame_size = struct.unpack_from('<II', value, 8)
                num_nodes = struct.unpack_from('<I', value, 20)[0]
                offset = 36 + tag_size + frame_size
                return value[36:36 + tag_size].decode(), [
                    struct.unpack_from('<III', value, offset + idx * 96 + 84)
                    for idx in range(num_nodes)]

            arrived = set()

            def check_initial(value):
                packet = decode(value)
                if packet:
                    topic, states = packet
                    assert states == ([counts(n) for n in static.nodes] if topic == static_topic
                                      else [(1, 4, 95)])
                    arrived.add(topic)
                    client.send({'type': 'stream.topological_map.applied', 'topic': topic})
                return topics <= arrived

            client.until(check_initial)
            # 座標・label・時刻は不変、状態件数だけの変更
            graph.nodes[0].num_danger_states = 59
            graph.nodes[0].num_collision_states = 40
            publisher.publish(graph)
            client.until(lambda value: decode(value) == (update_topic, [(1, 59, 40)]))
            print('PASS: 静的L0のROS→WS件数保持、label不変の状態件数更新')
        except BaseException:
            log.seek(0)
            print(log.read().decode(errors='replace'), file=sys.stderr)
            raise
        finally:
            if client:
                client.sock.close()
            for process in reversed(processes):
                stop(process)
            node.destroy_node()
            rclpy.shutdown()
            assert all(process.poll() is not None for process in processes)
            print('STOPPED: static node, gateway, test node')


if __name__ == '__main__':
    main()
