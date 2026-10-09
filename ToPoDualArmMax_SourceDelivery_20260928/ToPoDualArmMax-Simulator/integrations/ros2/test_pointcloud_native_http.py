"""独立ROSドメインでの実HTTP入力・実ROS受信の検証。起動プロセスは試験内で終了。"""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
from time import monotonic, sleep
import unittest
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from test_native_points import sample_packet
from test_fixtures.depth_output_reference import create_point_messages as reference


class NativeBridgeHTTPTest(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '224', '専用ROS_DOMAIN_ID=224が必要')
    def test_http_to_ros(self):
        # 通常ドメインの既存ノードとの混在防止。ROSデーモンの起動なし。
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '224', 'ROS_DOMAIN_ID=224で実行してください')
        import rclpy
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import PointCloud2, Image, CameraInfo
        from depth_output import depth_topics
        from rclpy.executors import SingleThreadedExecutor
        probes = []
        for _ in range(30):
            try:
                first = socket.socket();probes.append(first);first.bind(('127.0.0.1', 0));port = first.getsockname()[1]
                second = socket.socket();probes.append(second);second.bind(('127.0.0.1', port + 1));break
            except OSError:
                for probe in probes:probe.close()
                probes.clear()
        else:
            self.fail('試験用ポートを確保できません')
        for probe in probes:probe.close()
        endpoint = f'http://127.0.0.1:{port}'
        command = [sys.executable, str(Path(__file__).with_name('pointcloud_bridge.py')), '--port', str(port), '--tf-topic', '/sim/tf']
        print('START: ' + ' '.join(command), flush=True)
        rclpy.init()
        node = rclpy.create_node('test_native_pointcloud_receiver')
        executor = SingleThreadedExecutor();executor.add_node(node)
        messages = {}
        subscriptions = []
        for topic, kind in zip(['/sim/rgbd/points', *depth_topics], [PointCloud2, Image, CameraInfo, PointCloud2]):
            subscriptions.append(node.create_subscription(kind, topic, lambda message, topic=topic: messages.__setitem__(topic, message), 10))
        child = None
        with tempfile.TemporaryFile(mode='w+') as log:
            try:
                child = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                deadline = monotonic() + 15
                while monotonic() < deadline:
                    if child.poll() is not None:
                        log.seek(0);self.fail(log.read())
                    try:
                        with urlopen(endpoint + '/api/points/status', timeout=.5) as response:
                            self.assertEqual(json.load(response)['pointcloud_backend'], 'cpp')
                        break
                    except (URLError, TimeoutError):sleep(.1)
                else:self.fail('HTTP起動待機超過')
                deadline = monotonic() + 10
                while not all(node.count_publishers(topic) for topic in ['/sim/rgbd/points', *depth_topics]):
                    self.assertLess(monotonic(), deadline, 'ROS発見待機超過')
                    executor.spin_once(timeout_sec=.1)
                for enable_color in (False, True):
                    meta, data = sample_packet(97, 65, enable_color=enable_color)
                    header = json.dumps(meta).encode()
                    packet = b'TPC1' + struct.pack('<I', len(header)) + header + bytes(-len(header) % 4) + data
                    request = Request(endpoint + '/api/points', data=packet, headers={
                        'Content-Type': 'application/octet-stream', 'X-ToPo-Points': '1', 'Origin': 'http://127.0.0.1:8877'})
                    messages.clear()
                    with urlopen(request, timeout=10) as response:
                        result = json.load(response)
                    self.assertEqual(result['count'], meta['count'])
                    deadline = monotonic() + 10
                    while len(messages) < 4:
                        self.assertLess(monotonic(), deadline, 'ROS受信待機超過')
                        executor.spin_once(timeout_sec=.1)
                    stamp = Time(sec=result['stamp_sec'], nanosec=result['stamp_nanosec'])
                    cloud, images = reference(meta, data, stamp)
                    for topic, expected in zip(['/sim/rgbd/points', *depth_topics], [cloud, *images]):
                        self.assertEqual(messages[topic], expected, topic)
                # 不正データのHTTP拒否と直後の正常受付。
                bad = bytearray(packet)
                struct.pack_into('<f', bad, 8 + (len(header) + 3) // 4 * 4, float('nan'))
                request.data = bad
                with self.assertRaises(HTTPError) as error:
                    urlopen(request, timeout=10)
                self.assertEqual(error.exception.code, 400)
                request.data = packet
                with urlopen(request, timeout=10) as response:self.assertEqual(response.status, 200)
            finally:
                if child is not None and child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL);child.wait(timeout=5)
                executor.shutdown();node.destroy_node();rclpy.shutdown()
                print('STOPPED: ' + ' '.join(command), flush=True)


if __name__ == '__main__':
    unittest.main()
