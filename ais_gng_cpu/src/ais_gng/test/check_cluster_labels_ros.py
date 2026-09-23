"""実点群の隔離ドメイン転送による人・車ラベル確定の有限検証。"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ais_gng_msgs.msg import TopologicalMap
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-domain', type=int, default=0)
    parser.add_argument('--input-topic', default='/lidar_points')
    parser.add_argument('--params-file', default=str(
        Path(get_package_share_directory('ais_gng')) / 'config/gng_cpu/at128.yaml'))
    parser.add_argument('--executable', default=str(
        Path(get_package_prefix('ais_gng')) / 'lib/ais_gng/ais_gng_cpu'))
    parser.add_argument('--require-label', choices=('human', 'car'), default='human')
    args = parser.parse_args()
    test_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if test_domain == 0 or test_domain == args.source_domain:
        raise RuntimeError('入力元とは異なる非0の検証用ROS_DOMAIN_IDの指定が必要')
    expected_label = TopologicalMap.HUMAN if args.require_label == 'human' else TopologicalMap.CAR
    command = [args.executable, '--ros-args', '--params-file', args.params_file,
               '-r', 'topological_map:=/cluster_labels_test/map']
    for parameter in (
        'input.topic_names:=[/cluster_labels_test/points]', 'input.visualize:=false',
        'plane_cluster.direct_enabled:=false', 'nonplane_component.direct_enabled:=false',
        'surface_model.enable:=false',
    ):
        command.extend(('-p', parameter))

    contexts, nodes, executors = [], [], []
    process = None
    counts, labels, inferred = Counter(), Counter(), Counter()
    examples = []
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            for domain in (args.source_domain, test_domain):
                context = Context()
                rclpy.init(context=context, domain_id=domain)
                contexts.append(context)
                node = rclpy.create_node('cluster_labels_test_relay' if domain == args.source_domain
                                         else 'cluster_labels_test_driver', context=context,
                                         start_parameter_services=False, enable_rosout=False)
                nodes.append(node)
                executor = SingleThreadedExecutor(context=context)
                executors.append(executor)
                executor.add_node(node)
            source, driver = nodes
            # 入力元ドメインには購読のみ。点群とTFの発行先は隔離ドメイン。
            publisher = driver.create_publisher(PointCloud2, '/cluster_labels_test/points',
                                                qos_profile_sensor_data)

            def relay_cloud(cloud):
                counts['input_clouds'] += 1
                publisher.publish(cloud)

            source.create_subscription(PointCloud2, args.input_topic, relay_cloud,
                                       qos_profile_sensor_data)
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for topic, qos in (('/tf', 100), ('/tf_static', static_qos)):
                tf_pub = driver.create_publisher(TFMessage, topic, qos)
                source.create_subscription(TFMessage, topic, tf_pub.publish, qos)

            def on_map(graph):
                counts['maps'] += 1
                counts['nonempty_maps'] += bool(graph.nodes)
                if counts['maps'] == 1:
                    libraries = {line.split()[-1] for line in
                                 Path(f'/proc/{process.pid}/maps').read_text().splitlines()
                                 if 'libgng_cpu.so' in line or 'libais_gng_component_cpu.so' in line}
                    print('使用ライブラリ:', sorted(libraries), flush=True)
                for cluster in graph.clusters:
                    labels[int(cluster.label)] += 1
                    inferred[int(cluster.label_inferred)] += 1
                    if cluster.label == expected_label and len(examples) < 8:
                        examples.append({'frame': graph.frame_number, 'id': cluster.id,
                                         'age': graph.frame_number - cluster.frame,
                                         'label': cluster.label,
                                         'label_inferred': cluster.label_inferred,
                                         'score': round(cluster.label_reliability, 4)})

            driver.create_subscription(TopologicalMap, '/cluster_labels_test/map', on_map,
                                       qos_profile_sensor_data)
            print('起動コマンド:', ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 50
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'GNGの異常終了: {process.returncode}')
                for executor in executors:
                    executor.spin_once(timeout_sec=0.01)
                if counts['maps'] >= 120 and labels[expected_label] > 0:
                    break
            print(json.dumps({'counts': dict(counts), 'labels': dict(labels),
                              'inferred': dict(inferred), 'examples': examples}), flush=True)
            assert counts['nonempty_maps'] >= 30, '非空マップの継続更新の未確認'
            assert labels[expected_label] > 0, '指定クラスの確定ラベルの未確認'
        except BaseException:
            log.seek(0)
            print(log.read()[-6000:], flush=True)
            raise
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            for executor in executors:
                executor.shutdown()
            for node in nodes:
                node.destroy_node()
            for context in contexts:
                if context.ok():
                    context.shutdown()
            print('検証用GNG・点群/TF転送・購読ノードの終了済み', flush=True)


if __name__ == '__main__':
    main()
