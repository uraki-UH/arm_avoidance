"""隔離ROS環境での非平面Graph配信・入力到着順・再接続の検証。"""

import base64
import itertools
import json
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time


class Client:
    def __init__(self, port):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(('GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n'
                           'Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n'
                           f'Sec-WebSocket-Key: {key}\r\n\r\n').encode())
        header = b''
        while not header.endswith(b'\r\n\r\n'):
            header += self.read(1)
        assert b'101 Switching Protocols' in header

    def read(self, num):
        data = bytearray()
        while len(data) < num:
            part = self.sock.recv(num - len(data))
            if not part:
                raise EOFError()
            data.extend(part)
        return bytes(data)

    def send(self, value):
        data = json.dumps(value).encode()
        mask = os.urandom(4)
        header = bytes([0x81, 0x80 | len(data)]) if len(data) < 126 else (
            bytes([0x81, 0xfe]) + struct.pack('!H', len(data)))
        self.sock.sendall(header + mask + bytes(v ^ mask[idx % 4] for idx, v in enumerate(data)))

    def until(self, predicate, sec=12):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            self.sock.settimeout(max(0.001, deadline - time.monotonic()))
            header = self.read(2)
            num = header[1] & 127
            if num == 126:
                num = struct.unpack('!H', self.read(2))[0]
            elif num == 127:
                num = struct.unpack('!Q', self.read(8))[0]
            data = self.read(num)
            value = json.loads(data) if header[0] & 15 == 1 else data
            assert not (isinstance(value, dict) and value.get('type') == 'stream.marker_array'), value
            if predicate(value):
                return value
        raise TimeoutError('配信待機の時間超過')


def main():
    os.environ.update(ROS_DOMAIN_ID='218', ROS_LOCALHOST_ONLY='1')
    import rclpy
    from ais_gng_msgs.msg import PlaneCluster, PlaneClusterArray, TopologicalMap, TopologicalNode
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import UInt32MultiArray

    rclpy.init()
    node = rclpy.create_node('nonplane_stream_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topic = '/nonplane_components'
    pubs = [node.create_publisher(cls, name, qos) for cls, name in [
        (TopologicalMap, '/topological_map'), (PlaneClusterArray, '/plane_clusters'),
        (UInt32MultiArray, topic)]]
    port = 19092
    gateway = client = None
    with tempfile.TemporaryFile() as log:
        try:
            gateway = subprocess.Popen([
                '/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node',
                '--ros-args', '-p', f'port:={port}'], stdout=log, stderr=log)
            deadline = time.monotonic() + 15
            while client is None and time.monotonic() < deadline:
                try:
                    client = Client(port)
                except ConnectionRefusedError:
                    time.sleep(0.1)
            assert client is not None
            client.until(lambda v: isinstance(v, dict) and v.get('id') == 'sync_sources'
                         and topic in [s['id'] for s in v['result']['sources']])
            client.send({'id': 'on', 'method': 'sources.setActive',
                         'params': {'sourceId': topic, 'active': True}})
            client.until(lambda v: isinstance(v, dict) and v.get('id') == 'on')
            deadline = time.monotonic() + 10
            while any(pub.get_subscription_count() == 0 for pub in pubs):
                assert time.monotonic() < deadline
                time.sleep(0.05)

            def receive_graph(frame, num_nodes=3):
                def is_graph(value):
                    return (isinstance(value, bytes) and value[:4] == b'TMG1'
                            and struct.unpack_from('<I', value, 16)[0] == frame)
                packet = client.until(is_graph)
                tag_size, frame_size = struct.unpack_from('<II', packet, 8)
                assert packet[36:36 + tag_size].decode() == topic
                assert packet[36 + tag_size:36 + tag_size + frame_size] == b'sensor'
                assert struct.unpack_from('<III', packet, 20) == (
                    num_nodes, 4 if num_nodes else 0, 1 if num_nodes else 0)
                if num_nodes:
                    offset = 36 + tag_size + frame_size
                    assert [struct.unpack_from('<H', packet, offset + idx * 84)[0]
                            for idx in range(3)] == [50, 12, 80]
                    assert [struct.unpack_from('<I', packet, offset + idx * 84 + 12)[0]
                            for idx in range(3)] == [0xffffffff, 7, 7]
                    assert struct.unpack_from('<4H', packet, offset + 3 * 84) == (1, 2, 2, 0)
                client.send({'type': 'stream.topological_map.applied', 'topic': topic})

            for frame, order in enumerate(itertools.permutations(range(3)), 100):
                graph = TopologicalMap()
                graph.header.frame_id = 'sensor'
                graph.header.stamp.sec = graph.frame_number = frame
                graph.nodes = [TopologicalNode(id=value) for value in [50, 12, 80, 99]]
                for idx, vertex in enumerate(graph.nodes):
                    vertex.pos.x = float(idx)
                graph.edges = [1, 2, 2, 0, 0, 3]
                planes = PlaneClusterArray()
                planes.header = graph.header
                planes.frame_number = frame
                planes.clusters = [PlaneCluster(node_indices=[0, 3])]
                membership = UInt32MultiArray(data=[frame, 1, 7, 2, 1, 2])
                for idx in order:
                    pubs[idx].publish([graph, planes, membership][idx])
                    time.sleep(0.12)
                receive_graph(frame)
            print('PASS: 入力3種の全6到着順、元ID、所属、内部・平面接続エッジ、Marker配信なし')

            client.sock.close()
            client = Client(port)
            client.until(lambda v: isinstance(v, dict) and v.get('id') == 'sync_sources')
            client.send({'id': 'resume', 'method': 'sources.setActive',
                         'params': {'sourceId': topic, 'active': True}})
            receive_graph(frame)
            print('PASS: WebSocket再接続・再購読時のGraph再配信')

            graph.header.stamp.sec = graph.frame_number = frame + 1
            pubs[0].publish(graph)
            pubs[2].publish(UInt32MultiArray(data=[frame + 1, 0]))
            receive_graph(frame + 1, 0)
            print('PASS: 空成分によるGraph消去、古い平面フレームとの混合なし')
            assert node.get_topic_names_and_types().count((topic, ['std_msgs/msg/UInt32MultiArray'])) == 1

            client.send({'id': 'off', 'method': 'sources.setActive',
                         'params': {'sourceId': topic, 'active': False, 'removeLayer': True}})
            client.until(lambda v: isinstance(v, dict) and v.get('type') == 'stream.delete'
                         and v.get('topic') == topic)
            print('PASS: 購読解除によるレイヤー削除、ROS所属メッセージ型の維持')
        except BaseException:
            log.seek(0)
            print(log.read().decode(errors='replace'))
            raise
        finally:
            if client:
                client.sock.close()
            node.destroy_node()
            rclpy.shutdown()
            if gateway and gateway.poll() is None:
                gateway.send_signal(signal.SIGINT)
                try:
                    gateway.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    gateway.kill()
                    gateway.wait(timeout=5)


if __name__ == '__main__':
    main()
