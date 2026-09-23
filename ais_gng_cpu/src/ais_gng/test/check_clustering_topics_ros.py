"""隔離ROS環境での平面・曲面OFF、再起動、Viewer購読解除の有限検証。"""

import argparse
from collections import Counter
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from ais_gng_msgs.msg import PlaneClusterArray, TopologicalMap
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64, UInt32MultiArray
import yaml


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gateway', default=str(Path(get_package_prefix('topo_fuzzy_viewer')) /
                                                'lib/topo_fuzzy_viewer/viewer_ws_gateway_node'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('検証用の隔離ROS_DOMAIN_IDの指定が必要')
    with socket.socket() as port_socket:
        port_socket.bind(('127.0.0.1', 0))
        port = port_socket.getsockname()[1]
    rclpy.init()
    node = rclpy.create_node('clustering_topics_test_driver')
    map_updates = []
    node.create_subscription(TopologicalMap, '/topological_map',
                             lambda msg: map_updates.append((msg.frame_number, len(msg.nodes))),
                             qos_profile_sensor_data)
    publisher = node.create_publisher(PointCloud2, '/clustering_topics_test/points', 1)
    cloud = PointCloud2()
    cloud.header.frame_id = 'map'
    cloud.height, cloud.width = 1, 64
    cloud.fields = [PointField(name=name, offset=4 * idx, datatype=PointField.FLOAT32, count=1)
                    for idx, name in enumerate(('x', 'y', 'z'))]
    cloud.point_step, cloud.row_step, cloud.is_dense = 12, 64 * 12, True
    cloud.data = b''.join(struct.pack('<fff', 1 + .1 * x_idx, .1 * y_idx, .5)
                          for x_idx in range(8) for y_idx in range(8))
    gateway = launch = None
    subscriptions = []
    with tempfile.TemporaryDirectory(prefix='clustering-topics-test-') as directory, \
            tempfile.TemporaryFile(mode='w+') as log:
        try:
            command = [args.gateway, '--ros-args', '-p', f'port:={port}']
            print('起動コマンド:', ' '.join(command), flush=True)
            gateway = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)

            def wait_until(predicate, description, duration=15):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    assert gateway.poll() is None, 'Viewerの異常終了'
                    if launch is not None:
                        assert launch.poll() is None, 'GNG launchの異常終了'
                    cloud.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(cloud)
                    rclpy.spin_once(node, timeout_sec=.1)
                    if predicate():
                        return
                raise AssertionError(description)

            def cluster_topics():
                return {name for name, _ in node.get_topic_names_and_types() if name.startswith(
                    ('/plane_clusters', '/nonplane_components', '/curved_surface_clusters'))}

            for enable_plane, enable_curve in ((False, False), (True, False), (True, True), (False, False)):
                map_updates.clear()
                params = {
                    'plane_clustering': enable_plane, 'curve_clustering': enable_curve,
                    'input.topic_names': ['/clustering_topics_test/points'],
                    'input.local_coordinates': True, 'input.point_cloud_num': 256,
                    'input.visualize': False, 'node.num_max': 128, 'edge.num_max': 1280,
                    'node.learning_num': 0, 'node.grid': .5,
                    'node.interval': [.04, .04, .04, .04],
                    'classify.human': False, 'classify.car': False,
                    'nonplane_component.direct_enabled': True,
                }
                config = Path(directory) / 'sensor.yaml'
                config.write_text(yaml.safe_dump({'ais_gng_node': {'ros__parameters': params}}),
                                  encoding='utf-8')
                command = ['ros2', 'launch', 'ais_gng', 'ais_gng.launch.py', 'backend:=cpu',
                           f'lidar:={config}']
                print('起動コマンド:', ' '.join(command), flush=True)
                launch = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                wait_until(lambda: {'ais_gng_node', 'object_gng_dataset_exporter_node'} <=
                           set(node.get_node_names()), 'GNG・保存ノードの未起動')
                wait_until(lambda: len({frame for frame, num in map_updates if num > 0}) >= 3,
                           'topological_mapの非空更新なし')
                if not enable_plane:
                    wait_until(lambda: not cluster_topics(), 'OFF時の不要クラスタトピック')
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.1)
                        assert not cluster_topics(), cluster_topics()
                    assert 'plane_cluster_visualization_node' not in node.get_node_names()
                else:
                    wait_until(lambda: node.count_publishers('/plane_clusters') == 1 and
                               node.count_publishers('/nonplane_components') == 1 and
                               node.count_subscribers('/plane_clusters') == 3,
                               'ON時の平面発行・可視化・保存・Viewer購読の未接続')
                    counts = Counter()
                    expected = [('/plane_clusters', PlaneClusterArray),
                                ('/nonplane_components', UInt32MultiArray)]
                    if enable_curve:
                        expected.append(('/curved_surface_clusters/update_ms', Float64))
                    else:
                        assert not any(name.startswith('/curved_surface_clusters')
                                       for name in cluster_topics())
                    for topic, message_type in expected:
                        def receive(message, topic=topic):
                            counts[topic] += 1
                        subscriptions.append(node.create_subscription(
                            message_type, topic, receive, qos_profile_sensor_data))
                    wait_until(lambda: all(counts[topic] >= 2 for topic, _ in expected),
                               'ON時のクラスタ・曲面時間の更新なし')
                    print('更新メッセージ数:', dict(counts), flush=True)
                print(f'PASS: plane={enable_plane}, curve={enable_curve}', flush=True)
                for subscription in subscriptions:
                    node.destroy_subscription(subscription)
                subscriptions.clear()
                stop(launch)
                launch = None
                wait_until(lambda: not cluster_topics(), '停止後のViewer購読・トピックの残留')
        except BaseException:
            log.seek(0)
            print(log.read()[-9000:], flush=True)
            raise
        finally:
            stop(launch)
            stop(gateway)
            node.destroy_node()
            rclpy.shutdown()
            print('検証用GNG・Viewer・購読ノードの終了済み', flush=True)


if __name__ == '__main__':
    main()
