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
import yaml

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from composition_interfaces.srv import LoadNode
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
    parser.add_argument('--shared-roi', action='store_true')
    parser.add_argument('--enable-world-index', action='store_true')
    args = parser.parse_args()
    if args.enable_world_index and not args.shared_roi:
        raise ValueError('world索引の共有には--shared-roiが必要')
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
    def terminate_test(signum, frame):
        raise RuntimeError('試験上限による停止')
    signal.signal(signal.SIGTERM, terminate_test)
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
        if args.shared_roi:
            command = [args.node_executable, '--ros-args', '-r', '__node:=shared_roi_test_container']
        else:
            for parameter in parameters:
                command.extend(['-p', parameter])
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        report['command'] = command
        report['owned_pid'] = process.pid
        if args.shared_roi:
            client = node.create_client(LoadNode, '/shared_roi_test_container/_container/load_node')
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('共有container起動失敗')
            def load(package, plugin, name, settings):
                request = LoadNode.Request(package_name=package, plugin_name=plugin,
                                           node_name=name, node_namespace='/self_filter_check')
                request.parameters = [Parameter(key, value=value).to_parameter_msg() for key, value in settings.items()]
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=25)
                if not future.done() or not future.result().success:
                    raise RuntimeError('component読込失敗: ' + (str(future.result()) if future.done() else name))
            load('gng_vlut_system', 'robot_sim::bridge::WorldIndexToVoxelNode', 'roi', {
                'input_topic': '/self_filter_check/points', 'shared_point_store': 'shared_roi_test',
                'world_frame_id': 'world' if args.enable_world_index else 'robot', 'target_frame_id': 'robot',
                'enable_world_index': args.enable_world_index, 'enable_roi_query': args.enable_world_index,
                'enable_world_bucket_publish': False, 'allow_latest_transform': False,
                'voxel_size': .02, 'min_reachability_x': 1.4, 'max_reachability_x': 1.8,
                'reachability_margin_x': 0., 'reachability_margin_y': 0., 'reachability_margin_z': 0.,
                'output_topic': '/self_filter_check/roi',
                'self_filter.mask_topic': '/self_filter_check/mask',
                'self_filter.output_topic': '/self_filter_check/filtered',
                'self_filter.inflation': 0., 'self_filter.max_mask_age_sec': .3,
            })
            settings = {key: yaml.safe_load(value) for key, value in (item.split(':=', 1) for item in parameters)}
            settings.update({'input.shared_point_store': 'shared_roi_test', 'self_filter.mask_topic': ''})
            load('ais_gng', 'fuzzrobo::AiSGNGComponent', 'gng', settings)
            report['shared_components'] = ['robot_sim::bridge::WorldIndexToVoxelNode', 'fuzzrobo::AiSGNGComponent']
        publisher = node.create_publisher(PointCloud2, '/self_filter_check/points', qos_profile_sensor_data)
        mask_publisher = node.create_publisher(Voxel, '/self_filter_check/mask',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        outputs, labels, graphs = [], [], []
        roi_outputs = []
        node.create_subscription(Voxel, '/self_filter_check/filtered', roi_outputs.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
        if args.enable_world_index:
            world_transform = TransformStamped()
            world_transform.header.frame_id, world_transform.child_frame_id = 'world', 'robot'
            world_transform.header.stamp = node.get_clock().now().to_msg()
            world_transform.transform.translation.x = 1.25
            world_transform.transform.rotation.z = math.sin(.25)
            world_transform.transform.rotation.w = math.cos(.25)
            broadcaster.sendTransform([transform, world_transform])
        spin(3)
        if args.shared_roi:
            assert node.count_subscribers('/self_filter_check/points') == 1, 'raw点群の重複subscription'
            report['checks'].append('single_raw_subscription')

        def phase(kind):
            spin(.3)
            outputs.clear(); labels.clear(); graphs.clear(); roi_outputs.clear()
            until = time.monotonic() + 2.
            while time.monotonic() < until:
                assert process.poll() is None, 'GNGノードの異常終了'
                stamp = node.get_clock().now().to_msg()
                points = [(.51 + idx * .001, .11, .11) for idx in range(32)]
                if kind != 'all_self':
                    points += [(.91 + idx * .001, .31, .31) for idx in range(32)]
                    if args.shared_roi:
                        points += [(1.51 + idx * .001, .31, .31) for idx in range(32)]
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
                if kind == 'invalid_origin':
                    mask.origin_x = 1.
                if kind == 'delayed_cloud_old_mask':
                    mask.header.stamp = rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds - 400000000).to_msg()
                if kind not in ('missing_mask', 'stale_mask'):
                    mask_publisher.publish(mask)
                spin(.015)
                cloud = PointCloud2(height=1, width=len(points), point_step=16, row_step=len(points) * 16, is_dense=True)
                cloud.header.frame_id, cloud.header.stamp = ('missing_sensor' if kind == 'missing_tf' else 'sensor'), stamp
                if kind == 'delayed_cloud_old_mask':
                    cloud.header.stamp = rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds - 200000000).to_msg()
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
        expected_num = 64 if args.shared_roi else 32
        for message in outputs:
            xyz = [struct.unpack_from('<fff', message.data, idx * message.point_step) for idx in range(message.width * message.height)]
            assert len(xyz) == min(args.max_points, expected_num), '抽出数の不一致'
            assert xyz and all(.90 < point[0] < 1. or (args.shared_roi and 1.50 < point[0] < 1.55)
                               for point in xyz), '自己点の学習入力への残留またはTF不整合'
        if args.shared_roi:
            assert roi_outputs and all(message.data for message in roi_outputs), 'ROI出力なし'
            for message in roi_outputs:
                for cell in message.data:
                    x = ((cell >> 42) - message.offset) * message.voxel_size
                    assert 1.49 < x < 1.55, 'ROI外セルのVLUT入力への混入'
            report['checks'].append('roi_outside_environment_preserved_and_self_excluded')
        if args.enable_labels:
            assert labels, 'ラベル出力なし'
            labelled = labels[-1]
            label_offset = next(field.offset for field in labelled.fields if field.name == 'self_candidate')
            count = expected_num + 32
            assert labelled.width == count and labelled.header.frame_id == 'sensor'
            assert [labelled.data[idx * labelled.point_step + label_offset] for idx in range(count)] == [1] * 32 + [0] * expected_num
            assert [struct.unpack_from('<I', labelled.data, idx * labelled.point_step + 12)[0] for idx in range(count)] == list(range(count))
        else:
            assert not labels, '無効設定でのラベル出力'
        report['checks'].append('filter_labels_tf_field_preservation')
        if args.shared_roi:
            spin(.1); outputs.clear(); graphs.clear(); spin(.2)
            assert not outputs and not graphs, '同一共有フレームの再学習'
            report['checks'].append('no_frame_replay')
        invalid_cases = ['stale_mask', 'old_mask', 'invalid_mask', 'missing_tf', 'all_self']
        if args.shared_roi:
            invalid_cases.insert(3, 'invalid_origin')
            invalid_cases.insert(4, 'delayed_cloud_old_mask')
        for kind in invalid_cases:
            phase(kind)
            assert not outputs and not graphs, kind + ': 保護条件中の学習'
            if args.shared_roi and kind != 'all_self':
                assert not roi_outputs, kind + ': 保護条件中のVLUT入力'
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
