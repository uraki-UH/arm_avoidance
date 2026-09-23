"""CPUクラスタ所属情報からROS出力・分類器までの有限回帰検証。"""

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from ais_gng_msgs.msg import TopologicalMap
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def main():
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('検証用の隔離ROS_DOMAIN_IDの指定が必要')
    executable = Path(get_package_prefix('ais_gng')) / 'lib/ais_gng/ais_gng_cpu'
    command = [str(executable), '--ros-args', '-r', '__node:=cluster_members_test',
               '-r', 'topological_map:=/cluster_members_test/map']
    for parameter in [
        'input.topic_names:=[/cluster_members_test/points]',
        'input.base_frame_id:=cluster_members_world', 'input.local_coordinates:=true',
        'input.point_cloud_num:=1024', 'input.voxel_grid_unit:=0.05',
        'input.visualize:=false', 'node.num_max:=512', 'node.learning_num:=0',
        'node.grid:=0.5', 'node.interval:=[0.04,0.04,0.04,0.04]',
        'edge.num_max:=5120', 'cluster.node_num_min:=3', 'cluster.plane.volume:=1000.0',
        'label.fuzzy.unknown:=0.05', 'label.fuzzy.lpf_time_constant:=0.0',
        'classify.human:=true', 'classify.car:=true', 'classify.device:=cpu',
        'plane_cluster.direct_enabled:=false', 'surface_model.enable:=false',
    ]:
        command.extend(['-p', parameter])
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('cluster_members_test_driver', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    process = None
    maps = []
    node.create_subscription(TopologicalMap, '/cluster_members_test/map', maps.append,
                             qos_profile_sensor_data)
    publisher = node.create_publisher(PointCloud2, '/cluster_members_test/points', 10)
    cloud = PointCloud2()
    cloud.header.frame_id = 'cluster_members_world'
    cloud.height, cloud.width = 1, 560
    cloud.fields = [PointField(name=name, offset=4 * idx, datatype=PointField.FLOAT32, count=1)
                    for idx, name in enumerate(('x', 'y', 'z'))]
    cloud.point_step, cloud.row_step, cloud.is_dense = 12, 560 * 12, True
    cloud.data = b''.join(struct.pack('<fff', offset + 0.12 * x_idx,
                                     -0.18 + 0.12 * y_idx, 0.2 + 0.12 * z_idx)
                          for offset in (2.0, 6.0) for z_idx in range(10)
                          for y_idx in range(4) for x_idx in range(7))
    num_verified = 0
    num_inferred = 0
    max_members = 0
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            print('起動コマンド:', ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'GNGの異常終了: {process.returncode}')
                cloud.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(cloud)
                executor.spin_once(timeout_sec=0.1)
                while maps:
                    graph = maps.pop(0)
                    if not graph.clusters:
                        continue
                    owners = set()
                    for cluster in graph.clusters:
                        assert cluster.nodes, '所属ノード配列が空のROSクラスタ'
                        max_members = max(max_members, len(cluster.nodes))
                        for node_idx in cluster.nodes:
                            assert node_idx < len(graph.nodes), '所属ノード添字の範囲外'
                            assert node_idx not in owners, 'クラスタ間の所属ノード重複'
                            owners.add(node_idx)
                        assert math.isfinite(cluster.label_reliability), '分類信頼度の非有限値'
                        num_inferred += cluster.label_inferred in (
                            TopologicalMap.UNKNOWN_OBJECT, TopologicalMap.HUMAN, TopologicalMap.CAR)
                    num_verified += 1
                if num_verified >= 5 and num_inferred > 0:
                    break
            assert num_verified >= 5 and num_inferred > 0, '分類器までの受け渡し未確認'
            print(f'検証フレーム={num_verified}, 最大所属数={max_members}, 推論結果数={num_inferred}',
                  flush=True)
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
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)
            print('検証用GNG・購読ノードの終了済み', flush=True)


if __name__ == '__main__':
    main()
