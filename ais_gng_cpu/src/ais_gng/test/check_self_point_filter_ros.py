#!/usr/bin/env python3
"""専用ROS domainでの自己点除外・ラベル・マスク失効・TF欠落の有限試験。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from voxel_msgs.msg import Voxel
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ais_gng_msgs.msg import TopologicalMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-executable', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-points', type=int, default=2000)
    parser.add_argument('--enable-labels', action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if args.max_points < 1:
        raise ValueError('抽出上限は正整数が必要')
    if os.environ.get('ROS_DOMAIN_ID') != '194' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('専用domain194・localhost限定が必要')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('self_point_filter_check')
    process = None
    report = {'result': 'failed', 'checks': []}
    log = (args.output / 'node.log').open('w')
    try:
        def spin(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)

        spin(1)
        if any(name != node.get_name() for name in node.get_node_names()):
            raise RuntimeError('専用domainに既存ノードあり')
        command = [args.node_executable, '--ros-args', '-r', '__ns:=/self_filter_check', '-r', '__node:=gng']
        parameters = ['input.topic_names:=[/self_filter_check/points]', 'input.base_frame_id:=robot',
                      f'input.point_cloud_num:={args.max_points}', 'input.local_coordinates:=false',
                      'input.sampling_mode:=random',
                      'node.num_max:=128', 'node.learning_num:=100',
                      'classify.human:=false', 'classify.car:=false', 'plane_clustering:=false',
                      'nonplane_component.direct_enabled:=false',
                      'self_filter.mask_topic:=/self_filter_check/mask',
                      'self_filter.max_mask_age_sec:=0.3',
                      f'self_filter.enable_labelled_cloud:={str(args.enable_labels).lower()}']
        for parameter in parameters:
            command.extend(['-p', parameter])
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        report['command'] = command
        report['owned_pid'] = process.pid
        publisher = node.create_publisher(PointCloud2, '/self_filter_check/points', qos_profile_sensor_data)
        mask_publisher = node.create_publisher(Voxel, '/self_filter_check/mask', 1)
        outputs, labels, graphs = [], [], []
        node.create_subscription(PointCloud2, '/self_filter_check/scan/transformed', outputs.append, 10)
        node.create_subscription(PointCloud2, '/self_filter_check/scan/self_labelled', labels.append, qos_profile_sensor_data)
        node.create_subscription(TopologicalMap, '/self_filter_check/topological_map', graphs.append, 10)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'robot', 'sensor'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x = .5
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        spin(3)

        def phase(kind):
            spin(.3)
            outputs.clear(); labels.clear(); graphs.clear()
            until = time.monotonic() + 2.
            while time.monotonic() < until:
                assert process.poll() is None, 'GNGノードの異常終了'
                stamp = node.get_clock().now().to_msg()
                points = [(.51 + idx * .001, .11, .11) for idx in range(32)]
                if kind != 'all_self':
                    points += [(.91 + idx * .001, .31, .31) for idx in range(32)]
                mask = Voxel(voxel_size=.02, x_shift=42, y_shift=21, z_shift=0, offset=1000000)
                mask.header.frame_id = 'robot'
                mask.header.stamp = type(stamp)(sec=stamp.sec, nanosec=stamp.nanosec)
                mask.data = sorted({((math.floor(x / mask.voxel_size) + mask.offset) << 42) |
                                    ((math.floor(y / mask.voxel_size) + mask.offset) << 21) |
                                    (math.floor(z / mask.voxel_size) + mask.offset)
                                    for x, y, z in points[:32]})
                if kind == 'old_mask':
                    mask.header.stamp.sec -= 5
                if kind == 'invalid_mask':
                    mask.x_shift = 64
                if kind not in ('missing_mask', 'stale_mask'):
                    mask_publisher.publish(mask)
                spin(.015)
                cloud = PointCloud2(height=1, width=len(points), point_step=16, row_step=len(points) * 16, is_dense=True)
                cloud.header.frame_id, cloud.header.stamp = ('missing_sensor' if kind == 'missing_tf' else 'sensor'), stamp
                cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                                for idx, name in enumerate(('x', 'y', 'z'))]
                cloud.fields.append(PointField(name='intensity', offset=12, datatype=PointField.UINT32, count=1))
                cloud.data = b''.join(struct.pack('<fffI', x - .5, y, z, idx) for idx, (x, y, z) in enumerate(points))
                publisher.publish(cloud)
                spin(.06)

        phase('missing_mask')
        assert not outputs and not labels and not graphs, '未受信マスクでの学習'
        report['checks'].append('missing_mask')
        phase('valid')
        assert len(outputs) > 3 and graphs, '正常入力の未配信'
        for message in outputs:
            xyz = [struct.unpack_from('<fff', message.data, idx * message.point_step) for idx in range(message.width * message.height)]
            assert len(xyz) == min(args.max_points, 32), '抽出数の不一致'
            assert xyz and all(.90 < point[0] < 1. for point in xyz), '自己点の学習入力への残留またはTF不整合'
        if args.enable_labels:
            assert labels, 'ラベル出力なし'
            labelled = labels[-1]
            label_offset = next(field.offset for field in labelled.fields if field.name == 'self_candidate')
            assert labelled.width == 64 and labelled.header.frame_id == 'sensor'
            assert [labelled.data[idx * labelled.point_step + label_offset] for idx in range(64)] == [1] * 32 + [0] * 32
            assert [struct.unpack_from('<I', labelled.data, idx * labelled.point_step + 12)[0] for idx in range(64)] == list(range(64))
        else:
            assert not labels, '無効設定でのラベル出力'
        report['checks'].append('filter_labels_tf_field_preservation')
        for kind in ('stale_mask', 'old_mask', 'invalid_mask', 'missing_tf', 'all_self'):
            phase(kind)
            assert not outputs and not graphs, kind + ': 保護条件中の学習'
            if kind == 'all_self' and args.enable_labels:
                assert labels and labels[-1].width == 32, '全点自己候補の診断出力なし'
            report['checks'].append(kind)
        phase('valid')
        assert outputs and graphs, '有効マスクの再受信後に学習未復帰'
        report['checks'].append('recovery')
        report['result'] = 'passed'
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        report['is_process_stopped'] = process is None or process.poll() is not None
        node.destroy_node(); rclpy.shutdown(); log.close()
        (args.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
