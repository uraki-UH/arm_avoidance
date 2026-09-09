#!/usr/bin/env python3
"""配信中depthからの画素保持とGNG表参照の有限時間検証。既存ノードの変更なし。"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import UInt32MultiArray

spec = importlib.util.spec_from_file_location('depth_pixel_points', Path(__file__).resolve().parents[1] / 'scripts/depth_pixel_points.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def require(is_valid, message):
    if not is_valid:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    require(not output.exists(), '出力ファイルの既存状態')
    origin = [0.434, -0.693, 0.279]
    rotation = [-0.7571377603893011, -0.1765111953928945, 0.17855536538575983, 0.6030789261660552]
    namespace = '/observation_pixel_test'
    command = [args.executable, '--ros-args', '-r', '__node:=observation_pixel_gng', '-r', f'__ns:={namespace}']
    for parameter in [
        f'input.topic_names:=[{namespace}/points]', 'input.point_cloud_num:=20000',
        'input.sampling_mode:=uniform', 'input.local_coordinates:=true', 'input.base_frame_id:=base_link',
        'node.num_max:=1024', 'node.learning_num:=4000', 'node.enable_observation_support:=true',
        f'input.observation_origin:={origin}', 'input.observation_origin_frame:=base_link',
        f'input.observation_camera_rotation:={rotation}',
        f'input.observation_camera_info_topic:={namespace}/camera_info',
        'classify.human:=false', 'classify.car:=false',
        'plane_cluster.direct_enabled:=false', 'nonplane_component.direct_enabled:=false',
    ]:
        command.extend(['-p', parameter])
    bridge_args = ['--ros-args', '-r', '__node:=observation_pixel_depth']
    for parameter in [f'output_topic:={namespace}/points', f'output_camera_info_topic:={namespace}/camera_info',
                      'target_frame:=base_link', f'sensor_origin:={origin}', f'sensor_rotation:={rotation}']:
        bridge_args.extend(['-p', parameter])
    process = None
    bridge = None
    monitor = None
    executor = None
    records = []
    verified_frames = []
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            print('GNG起動コマンド:', ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            rclpy.init(args=bridge_args)
            bridge = module.depth_pixel_points()
            original_convert = bridge.convert

            def verified_convert(image, info):
                start = time.perf_counter()
                cloud = original_convert(image, info)
                conversion_ms = (time.perf_counter() - start) * 1000
                data = np.frombuffer(cloud.data, dtype=[('xyz', '<f4', (3,)), ('pixel', '<u4')])
                require(len(data) > 0 and np.all(data['pixel'][1:] > data['pixel'][:-1]), '画素番号の順序・空点群')
                dtype = np.dtype(('>' if image.is_bigendian else '<') + ('u2' if image.encoding == '16UC1' else 'f4'))
                depth = np.ndarray((image.height, image.width), dtype=dtype, buffer=image.data,
                                   strides=(image.step, dtype.itemsize)).astype(np.float64).reshape(-1)
                if image.encoding == '16UC1':
                    depth *= bridge.depth_unit
                sampled = data[::max(1, len(data)//100)]
                ids = sampled['pixel']
                rays = np.stack([(ids % info.width - info.k[2])/info.k[0],
                                 (ids // info.width - info.k[5])/info.k[4], np.ones(len(ids))], axis=1)
                expected = (rays * depth[ids, None]) @ bridge.rotation.T + bridge.origin
                require(np.allclose(sampled['xyz'], expected, atol=1e-6, rtol=1e-6), 'XYZと元画素深度の対応不一致')
                verified_frames.append({'stamp': [image.header.stamp.sec, image.header.stamp.nanosec],
                                        'points': len(data), 'conversion_ms': conversion_ms})
                return cloud

            bridge.convert = verified_convert
            monitor = rclpy.create_node('observation_pixel_monitor', use_global_arguments=False)
            monitor.create_subscription(UInt32MultiArray, f'{namespace}/node_observation_lookup_statistics',
                                        lambda message: records.append(list(message.data)), 100)
            executor = SingleThreadedExecutor()
            executor.add_node(bridge)
            executor.add_node(monitor)
            deadline = time.monotonic() + 40
            while time.monotonic() < deadline:
                require(process.poll() is None, '検証GNGの異常終了')
                executor.spin_once(timeout_sec=0.05)
                if len(records) >= 60:
                    break
            require(len(records) >= 30, 'GNG統計フレーム不足')
            hits = [row for row in records if row[3] > 0]
            require(len(hits) >= 20, '画素表参照フレーム不足')
            require(all(row[4] == 0 for row in hits), '画素表参照フレーム内の逆レイ計算')
            require(max(row[5] for row in records) == 1, '固定カメラでの表再構築')
            result = {'verified_depth_frames': verified_frames, 'lookup_frames': records,
                      'pixel_frames': len(hits), 'fallback_frames': len(records)-len(hits),
                      'gng_command': command, 'bridge_arguments': bridge_args}
            with output.open('x') as stream:
                json.dump(result, stream, indent=2)
            print('observation_pixel_live_test=passed', 'frames=', len(records), 'pixel_frames=', len(hits),
                  'fallback_frames=', len(records)-len(hits), flush=True)
        except Exception:
            log.seek(0)
            print(log.read()[-6000:])
            raise
        finally:
            if executor is not None:
                executor.shutdown()
            if bridge is not None:
                bridge.destroy_node()
            if monitor is not None:
                monitor.destroy_node()
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
                print('検証GNG・depth変換・監視ノード: 停止済み', flush=True)


if __name__ == '__main__':
    main()
