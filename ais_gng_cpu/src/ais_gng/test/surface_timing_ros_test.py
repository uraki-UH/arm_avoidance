#!/usr/bin/env python3
"""隔離ドメインでの一行ログ・別ノードの曲面時間通知・停止後のoff復帰の検査。"""
import argparse
import math
import os
import re
import struct
import subprocess
import tempfile
import time

import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64
from boundary_candidates_ros_test import require, stop_process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '197'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    curve_topic = '/surface_timing_test/curves'
    processes = []
    node = None
    with tempfile.TemporaryFile() as gng_log, tempfile.TemporaryFile() as surface_log:
        def start(executable, parameters, log):
            command = [executable, '--ros-args']
            for parameter in parameters:
                command.extend(['-p', parameter])
            print('起動コマンド:', ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            processes.append(process)
            return process

        def contents(log):
            return os.pread(log.fileno(), os.fstat(log.fileno()).st_size, 0).decode(errors='replace')

        try:
            gng = start(args.executable, [
                'input.topic_names:=[/surface_timing_test/points]', 'input.point_cloud_num:=1024',
                'node.num_max:=512', 'node.learning_num:=100', 'input.local_coordinates:=true',
                'node.interval:=[0.02, 0.02, 0.02, 0.02]',
                'classify.human:=false', 'classify.car:=false',
                'boundary.enable_candidates:=true', 'boundary.enable_evidence:=false',
                'plane_cluster.output_topic:=/surface_timing_test/planes',
                f'surface_model.output_topic:={curve_topic}',
            ], gng_log)
            rclpy.init()
            node = rclpy.create_node('surface_timing_test_driver')
            publisher = node.create_publisher(PointCloud2, '/surface_timing_test/points', 10)
            timings = []
            node.create_subscription(Float64, curve_topic+'/update_ms', lambda msg: timings.append(msg.data), 10)
            cloud = PointCloud2()
            cloud.header.frame_id = 'map'
            cloud.width, cloud.height, cloud.point_step, cloud.row_step = 1024, 1, 12, 12288
            cloud.is_dense = True
            cloud.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                            for idx, name in enumerate(('x', 'y', 'z'))]
            cloud.data = b''.join(struct.pack('<fff', x*0.01, y*0.01, 0.5)
                                  for x in range(32) for y in range(32))

            def wait_for(pattern):
                offset = len(contents(gng_log))
                deadline = time.monotonic()+15
                while time.monotonic() < deadline:
                    require(gng.poll() is None, 'GNG異常終了')
                    cloud.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(cloud)
                    end = time.monotonic()+0.1
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.01)
                    match = re.search(pattern, contents(gng_log)[offset:])
                    if match:
                        print('検証ログ:', match.group(0), flush=True)
                        return match
                raise RuntimeError(f'ログ待機時間切れ: {pattern}')

            prefix = r'I: .*GNG: [0-9.]+ ms, Pl: [0-9.]+ ms, NonPL: [0-9.]+ ms, Bound: [0-9.]+ ms \(\d+\), Curve: '
            wait_for(prefix+'off')
            pending = node.create_publisher(Float64, curve_topic+'/update_ms', 1)
            wait_for(prefix+'--')
            node.destroy_publisher(pending)
            surface = start(os.path.join(os.path.dirname(args.executable), 'plane_cluster_incremental_node'), [
                'input_topic:=/topological_map', 'clusters_input_topic:=/surface_timing_test/planes',
                f'surface_model.output_topic:={curve_topic}', 'surface_model.hz:=20.0',
                'surface_model.enable_markers:=false', 'enable_plane_markers:=false', 'enable_nonplane_markers:=false',
            ], surface_log)
            match = wait_for(prefix+r'([0-9.]+) ms')
            require(surface.poll() is None, '曲面ノード異常終了')
            require(any(math.isfinite(value) and value >= 0 and f'{value:.2f}' == match.group(1)
                        for value in timings), '受信した曲面時間と要約ログの不一致')
            require('Surface: ' not in contents(surface_log), '詳細Surfaceログの通常出力への残留')
            stop_process(surface)
            wait_for(prefix+'off')
            require(not re.search(r'Plane:|Nonplane:|Boundary:', contents(gng_log)), '旧表示名の残留')
            print('surface_timing_ros=passed', flush=True)
        finally:
            for process in reversed(processes):
                stop_process(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print(contents(gng_log)[-1500:], flush=True)
            print(contents(surface_log)[-1500:], flush=True)
            print('検証用GNG・曲面ノード: 停止済み', flush=True)


if __name__ == '__main__':
    main()
