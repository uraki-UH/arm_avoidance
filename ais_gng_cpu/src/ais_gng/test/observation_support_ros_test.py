#!/usr/bin/env python3
"""隔離ROSドメインでの観測支持出力・TF欠落・時刻整合の検証。"""

import argparse
import math
import os
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import UInt32MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def require(is_valid, message):
    if not is_valid:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--transform-cloud', action='store_true')
    parser.add_argument('--fixed-origin', action='store_true')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '187'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    command = [args.executable, '--ros-args', '-r', '__node:=observation_test_gng']
    for parameter in [
        'node.num_max:=1024', 'node.learning_num:=4000',
        'input.topic_names:=[/observation_test_points]', 'input.point_cloud_num:=2000',
        f'input.local_coordinates:={str(not args.transform_cloud).lower()}',
        'input.base_frame_id:=observation_test_base', 'node.enable_observation_support:=true',
        'classify.human:=false', 'classify.car:=false',
        'plane_cluster.direct_enabled:=false', 'nonplane_component.direct_enabled:=false',
    ]:
        command.extend(['-p', parameter])
    if args.fixed_origin:
        command.extend(['-p', 'input.observation_origin:=[0.4, 0.0, 0.0]',
                        '-p', 'input.observation_origin_frame:=observation_test_world'])
    else:
        command.extend(['-p', 'input.observation_sensor_frame:=observation_test_sensor'])
    process = None
    node = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            legacy_command = command + ['-p', 'node.observation.half_angle_deg:=0.25']
            print('旧設定検査の起動コマンド:', ' '.join(legacy_command), flush=True)
            try:
                legacy_result = subprocess.run(legacy_command, capture_output=True, text=True, timeout=5)
                require(legacy_result.returncode != 0 and 'Observation legacy parameters removed' in legacy_result.stderr,
                        '旧設定の混在を検出できない状態')
            except subprocess.TimeoutExpired as error:
                # コンポーネント生成失敗後も待機するROSラッパー。subprocess.runによる停止・回収済み。
                output = error.stderr or b''
                if isinstance(output, bytes):
                    output = output.decode(errors='replace')
                require('Observation legacy parameters removed' in output,
                        f'旧設定の検査タイムアウト: {output[-2000:]}')
            print('旧設定検査ノード: 終了済み', flush=True)
            print('起動コマンド:', ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('observation_support_test_driver')
            publisher = node.create_publisher(PointCloud2, '/observation_test_points', qos_profile_sensor_data)
            messages = []
            subscription = node.create_subscription(UInt32MultiArray, '/node_observation_support', messages.append, 10)
            broadcaster = TransformBroadcaster(node)
            static_broadcaster = StaticTransformBroadcaster(node)
            if args.transform_cloud:
                base_transform = TransformStamped()
                base_transform.header.frame_id = 'observation_test_base'
                base_transform.child_frame_id = 'observation_test_world'
                base_transform.header.stamp = node.get_clock().now().to_msg()
                base_transform.transform.translation.x = 0.25
                base_transform.transform.translation.y = 0.125
                base_transform.transform.translation.z = 0.25
                base_transform.transform.rotation.z = 1.0
                base_transform.transform.rotation.w = 0.0
                static_broadcaster.sendTransform(base_transform)
            points = [(1.0 + column * 0.06, -0.9 + row * 0.06, 0.2)
                      for row in range(30) for column in range(30)]
            cloud = PointCloud2()
            cloud.height, cloud.width = 1, len(points)
            cloud.point_step, cloud.row_step = 12, len(points) * 12
            cloud.is_dense = True
            cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                            for idx, name in enumerate(('x', 'y', 'z'))]
            cloud.data = b''.join(struct.pack('<fff', *point) for point in points)

            def receive(frame, stamp, has_origin, transform=None):
                messages.clear()
                cloud.header.frame_id, cloud.header.stamp = frame, stamp
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    require(process.poll() is None, 'GNGノードの異常終了')
                    if transform is not None:
                        broadcaster.sendTransform(transform)
                    publisher.publish(cloud)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    for message in messages:
                        data = message.data
                        if len(data) < 9 or data[2] != stamp.sec or data[3] != stamp.nanosec:
                            continue
                        if bool(data[4]) != has_origin:
                            continue
                        require(data[0] == 5, '出力バージョン不一致')
                        output_frame = 'observation_test_base' if args.transform_cloud else frame
                        require(message.layout.dim[0].label == output_frame, '出力座標系不一致')

                        def decode_point(values):
                            return struct.unpack('<fff', struct.pack('<III', *values))

                        origin = decode_point(data[5:8])
                        expected_origin = (0.0, 0.0, 0.0)
                        if has_origin and frame == 'observation_test_world':
                            expected_origin = (-0.15, 0.125, 0.25) if args.transform_cloud else (0.4, 0.0, 0.0)
                        require(all(abs(a - b) < 1e-6 for a, b in zip(origin, expected_origin)),
                                f'観測時原点の不一致: {origin}')
                        offset, ranges = 9, []
                        for _ in range(data[8]):
                            require(offset + 5 <= len(data), 'ノードレコード欠落')
                            flags, yaw, pitch = data[offset + 2:offset + 5]
                            require(flags in (0, 1, 3), '範囲フラグ不一致')
                            if flags & 1:
                                require(has_origin, '無効原点に対する支持範囲の残留')
                                require(flags & 2, '非極方向のyaw欠落')
                                ranges.append((yaw & 65535, yaw >> 16, pitch & 65535, pitch >> 16))
                            else:
                                require(yaw == 0 and pitch == 0, '未支持ノードの旧端点残留')
                            offset += 5
                        require(offset == len(data), 'レコード長不一致')
                        if has_origin and not ranges:
                            continue
                        return ranges
                raise RuntimeError('期待する観測支持メッセージの受信タイムアウト')

            receive('observation_test_missing', node.get_clock().now().to_msg(), False)
            receive('observation_test_sensor', node.get_clock().now().to_msg(), not args.transform_cloud and not args.fixed_origin)
            transform = TransformStamped()
            transform.header.frame_id = 'observation_test_world'
            transform.child_frame_id = 'observation_test_sensor'
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.transform.translation.x = 0.4
            transform.transform.rotation.w = 1.0
            ranges = receive('observation_test_world', transform.header.stamp, True,
                             None if args.fixed_origin else transform)
            # 実測レイ由来の整数角度端点との照合。
            def float32(value):
                return struct.unpack('<f', struct.pack('<f', value))[0]

            expected = set()
            origin = (float32(-0.15), 0.125, 0.25) if args.transform_cloud else (float32(0.4), 0.0, 0.0)
            for point in points:
                x, y, z = struct.unpack('<fff', struct.pack('<fff', *point))
                if args.transform_cloud:
                    x, y, z = float32(-x + 0.25), float32(-y + 0.125), float32(z + 0.25)
                x, y, z = x - origin[0], y - origin[1], z - origin[2]
                yaw = min(65535, max(0, math.floor((math.atan2(y, x) + math.pi) * (65536 / (2 * math.pi)))))
                pitch = min(65535, max(0, math.floor((math.atan2(z, math.hypot(x, y)) + math.pi / 2) * (65536 / math.pi))))
                expected.add((yaw, pitch))
            expected_yaw = {yaw for yaw, _ in expected}
            expected_pitch = {pitch for _, pitch in expected}
            for min_yaw, max_yaw, min_pitch, max_pitch in ranges:
                require(min_yaw in expected_yaw and max_yaw in expected_yaw, '未観測yaw端点')
                require(min_pitch in expected_pitch and max_pitch in expected_pitch, '未観測pitch端点')
                require(any(min_pitch <= pitch <= max_pitch and
                            ((yaw - min_yaw) & 65535) <= ((max_yaw - min_yaw) & 65535)
                            for yaw, pitch in expected), '実測レイを含まない範囲')
            future = node.get_clock().now().to_msg()
            future.sec += 30
            receive('observation_test_world', future, args.fixed_origin)
            receive('observation_test_missing', future, False)
            node.destroy_subscription(subscription)
            print('observation_support_ros_test=passed', flush=True)
        except Exception:
            log.seek(0)
            print(log.read()[-4000:])
            raise
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                print('テスト用GNGノード: 停止済み', flush=True)
                if process.returncode not in (0, -signal.SIGINT):
                    log.seek(0)
                    print(log.read()[-8000:])


if __name__ == '__main__':
    main()
