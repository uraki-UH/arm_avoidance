"""隔離ROSドメインによるTF時刻後退・同一座標出力・座標変換の回帰確認。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from ais_gng_msgs.msg import TopologicalMap
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int64MultiArray
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', choices=('same_wall', 'same_sim', 'transform_sim'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expect-old-data', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '216':
        raise RuntimeError('実機から隔離したROS_DOMAIN_ID=216の指定が必要です')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    has_transform = args.case == 'transform_sim'
    use_sim_time = args.case != 'same_wall'
    executable = Path(get_package_prefix('gng_vlut_system')) / 'lib/gng_vlut_system/topofuzzy_bridge_node'
    command = [str(executable), '--ros-args', '-r', '__ns:=/tf_restart_probe',
               '--params-file', str(Path(__file__).resolve().parents[1] / 'config/ToPoDualArm.yaml'),
               '-p', 'visualization_gng.enabled:=false', '-p', 'urdf_path:=""',
               '-p', 'source_frame_id:=base_link',
               '-p', 'frame_id:=' + ('world' if has_transform else 'base_link'),
               '-p', 'use_sim_time:=' + str(use_sim_time).lower(),
               '-p', 'occupied_voxels_topic:=/tf_restart_probe/occupied',
               '-p', 'topic_name:=Tmap_static']
    rclpy.init()
    node = rclpy.create_node('tf_restart_test_driver')
    received = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TopologicalMap, '/tf_restart_probe/Tmap_static', received.append, qos)
    tf_pub = node.create_publisher(TFMessage, '/tf', 100)
    static_tf_pub = node.create_publisher(TFMessage, '/tf_static', qos)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    occupied_pub = node.create_publisher(Int64MultiArray, '/tf_restart_probe/occupied', 10)
    process = None
    log_path = args.output / 'bridge.log'

    def wait_for(predicate, max_sec=30.0):
        deadline = time.monotonic() + max_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError(log_path.read_text()[-3000:])
        raise AssertionError('受信待機の時間超過')

    def phase(stamp, offset):
        clock = Clock()
        clock.clock.sec = stamp
        # /clock後退の処理完了後に新しい時刻のTFを入力する再起動相当の順序
        deadline = time.monotonic() + .4
        while time.monotonic() < deadline:
            clock_pub.publish(clock)
            rclpy.spin_once(node, timeout_sec=.03)
        transforms = []
        for child in ('sim_ToPoDualArm/neck_tilt_link', 'tf_restart_probe/base_link'):
            transform = TransformStamped()
            transform.header.frame_id = 'world'
            transform.header.stamp.sec = stamp
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transform.transform.translation.x = offset
            transforms.append(transform)
        begin = len(received)
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            clock_pub.publish(clock)
            tf_pub.publish(TFMessage(transforms=transforms))
            occupied_pub.publish(Int64MultiArray())
            rclpy.spin_once(node, timeout_sec=.05)
        wait_for(lambda: len(received) > begin and bool(received[-1].nodes), 5.)
        return received[-1]

    try:
        with log_path.open('w') as log:
            print('起動: ' + ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for(lambda: occupied_pub.get_subscription_count() > 0 and bool(received))
            # 初回100秒・再起動1秒・再び120秒・再起動2秒の二度の巻き戻り
            snapshots = [phase(stamp, offset) for stamp, offset in ((100, 0.), (1, 1.), (120, 2.), (2, 3.))]
            num_tf_subscribers = tf_pub.get_subscription_count()
            num_static_tf_subscribers = static_tf_pub.get_subscription_count()
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        print('所有TF試験ノード: 終了済み', flush=True)

    num_warnings = log_path.read_text().count('TF_OLD_DATA')
    positions = [[(item.id, item.pos.x, item.pos.y, item.pos.z) for item in msg.nodes] for msg in snapshots]
    expected_frame = 'world' if has_transform else 'tf_restart_probe/base_link'
    assert all(msg.header.frame_id == expected_frame for msg in snapshots)
    assert all(msg.edges == snapshots[0].edges for msg in snapshots)
    assert all(len(items) == len(positions[0]) for items in positions)
    for idx, items in enumerate(positions):
        for before, after in zip(positions[0], items):
            assert before[0] == after[0]
            assert abs(after[1] - before[1] - (idx if has_transform else 0)) < 1e-4
            assert abs(after[2] - before[2]) < 1e-4 and abs(after[3] - before[3]) < 1e-4
    result = {'case': args.case, 'num_nodes': len(positions[0]), 'num_tf_subscribers': num_tf_subscribers,
              'num_static_tf_subscribers': num_static_tf_subscribers,
              'num_old_data_warnings': num_warnings, 'num_restarts': 2}
    print(json.dumps(result), flush=True)
    if args.expect_old_data:
        assert num_warnings > 0 and num_tf_subscribers > 0, result
    else:
        assert num_warnings == 0, result
        assert num_tf_subscribers == (1 if has_transform else 0), result
        assert num_static_tf_subscribers == (1 if has_transform else 0), result


if __name__ == '__main__':
    main()
