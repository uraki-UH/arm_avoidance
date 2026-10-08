#!/usr/bin/env python3
"""頭部TFによるベース座標出力・関節姿勢変更・TF欠測抑止の隔離ROS検証。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ais_gng_msgs.msg import TopologicalMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--transform-mode', choices=('stamped', 'latest'), default='stamped')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if os.environ.get('ROS_DOMAIN_ID') != '193' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('隔離domain193・localhost限定が必要')
    rclpy.init()
    node = rclpy.create_node('robot_base_frame_check')
    process = None
    report = {'result': 'failed'}
    log = (args.output/'node.log').open('w')
    try:
        end = time.monotonic()+1.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if any(name != node.get_name() for name in node.get_node_names()):
            raise RuntimeError('専用domainに既存ノードあり')
        command = ['/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu', '--ros-args',
                   '-r', '__ns:=/base_frame_test', '-r', '__node:=gng']
        for parameter in ['input.topic_names:=[/base_frame_test/points]',
                          'input.base_frame_id:=robot_base', 'input.local_coordinates:=false',
                          'input.enable_strict_transform:='+str(args.transform_mode == 'stamped').lower(),
                          'input.point_cloud_num:=2000',
                          'node.num_max:=128', 'node.learning_num:=1000',
                          'classify.human:=false', 'classify.car:=false',
                          'plane_clustering:=false', 'nonplane_component.direct_enabled:=false']:
            command.extend(['-p', parameter])
        report['command'] = command
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        report['owned_pid'] = process.pid
        publisher = node.create_publisher(PointCloud2, '/base_frame_test/points', qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(node)
        clouds, graphs = [], []
        node.create_subscription(PointCloud2, '/base_frame_test/scan/transformed', clouds.append, 10)
        node.create_subscription(TopologicalMap, '/base_frame_test/topological_map', graphs.append, 10)
        expected = np.array([[x, y, .7] for x in np.linspace(.8, 1., 16)
                             for y in np.linspace(.1, .3, 16)], dtype=np.float32)
        translation = np.array([.04, .03, .51])

        def phase(yaw, has_transform, is_future=False, frame_id='head_camera'):
            clouds.clear()
            graphs.clear()
            rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                                 [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
            raw = expected if frame_id == 'robot_base' else (expected-translation)@rotation
            end = time.monotonic()+3.
            while time.monotonic() < end:
                assert process.poll() is None, 'GNGノードの異常終了'
                stamp = node.get_clock().now().to_msg()
                if is_future:
                    stamp.sec += 30
                if has_transform:
                    tf = TransformStamped()
                    tf.header.frame_id, tf.child_frame_id = 'robot_base', 'head_camera'
                    tf.header.stamp = stamp
                    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, translation)
                    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
                    broadcaster.sendTransform(tf)
                cloud = PointCloud2(height=1, width=len(raw), point_step=12, row_step=len(raw)*12, is_dense=True)
                cloud.header.frame_id, cloud.header.stamp = frame_id, stamp
                cloud.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                                for idx, name in enumerate(('x', 'y', 'z'))]
                cloud.data = raw.astype('<f4').tobytes()
                publisher.publish(cloud)
                until = time.monotonic()+.1
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.01)

        phase(0., False)
        assert not clouds and not graphs, 'TF未取得入力の誤配信'
        phase(0., False, frame_id='robot_base')
        assert clouds and graphs, '同一座標系の入力の誤抑止'
        assert all(message.header.frame_id == 'robot_base' for message in clouds+graphs)
        errors = []
        for yaw in (0., .8):
            phase(yaw, True)
            assert len(clouds) > 3 and graphs
            assert all(msg.header.frame_id == 'robot_base' for msg in clouds+graphs)
            values = np.frombuffer(clouds[-1].data, dtype='<f4').reshape(-1, clouds[-1].point_step//4)[:, :3]
            error = float(np.max(np.abs(values.mean(axis=0)-expected.mean(axis=0))))
            assert error < 1e-4, error
            graph_points = np.array([[v.pos.x, v.pos.y, v.pos.z] for v in graphs[-1].nodes])
            assert len(graph_points) > 0 and np.max(np.abs(graph_points.mean(axis=0)-expected.mean(axis=0))) < .15
            errors.append(error)
        # 直前の正常出力の配送完了後の、取得時刻にTFがない入力
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        phase(.8, False, frame_id='missing_sensor')
        assert not clouds and not graphs, 'TF欠測後の古い変換による誤配信'
        if args.transform_mode == 'stamped':
            phase(.8, False, True)
            assert not clouds and not graphs, '過去TFによる未来入力の誤配信'
        report.update(result='passed', max_centroid_errors_m=errors,
                      transform_mode=args.transform_mode,
                      has_same_frame_acceptance=True, has_missing_tf_rejection=True,
                      has_timestamp_rejection=args.transform_mode == 'stamped')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        report['is_process_stopped'] = process is None or process.poll() is not None
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
