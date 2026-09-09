#!/usr/bin/env python3
"""隔離ドメインでの視野端・深度段差の証拠とメタデータ欠落時の不明復帰の検査。"""
import argparse
import os
import struct
import subprocess
import tempfile
import time

import rclpy
from ais_gng_msgs.msg import TopologicalMap
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from boundary_candidates_ros_test import require, stop_process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '193'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    command = [args.executable, '--ros-args', '-r', '__node:=boundary_evidence_test_gng']
    for parameter in [
        'input.topic_names:=[/boundary_evidence_points]', 'input.point_cloud_num:=2000',
        'node.num_max:=1024', 'node.learning_num:=10000', 'input.local_coordinates:=true',
        'node.interval:=[0.02, 0.02, 0.02, 0.02]',
        'input.x_min:=-5.0', 'input.x_max:=5.0', 'input.y_min:=-5.0', 'input.y_max:=5.0',
        'input.z_min:=-5.0', 'input.z_max:=5.0',
        'boundary.enable_candidates:=true', 'boundary.max_neighbors:=65534', 'boundary.max_anchor_dist:=0.15',
        'node.enable_observation_support:=true', 'input.observation_sensor_frame:=boundary_evidence_sensor',
        'input.observation_camera_info_topic:=/boundary_evidence_camera_info',
        'classify.human:=false', 'classify.car:=false',
        'plane_cluster.direct_enabled:=false', 'nonplane_component.direct_enabled:=false',
    ]:
        command.extend(['-p', parameter])
    process = node = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            print('起動コマンド:', ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('boundary_evidence_driver')
            messages = []
            node.create_subscription(TopologicalMap, '/topological_map', messages.append, 10)
            publisher = node.create_publisher(PointCloud2, '/boundary_evidence_points', 10)
            camera = node.create_publisher(CameraInfo, '/boundary_evidence_camera_info', 10)
            info = CameraInfo()
            info.header.frame_id = 'boundary_evidence_sensor'
            info.width = info.height = 7
            info.k = [7., 0., 3., 0., 7., 3., 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            cloud = PointCloud2()
            cloud.header.frame_id = info.header.frame_id
            cloud.width, cloud.height = 49, 1
            cloud.point_step, cloud.row_step, cloud.is_dense = 12, 49*12, True
            cloud.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                            for idx, name in enumerate(('x', 'y', 'z'))]
            for mode, has_info in (('flat', True), ('step', True), ('missing', False)):
                size = 7 if mode == 'flat' else 31
                center = (size-1)/2
                focal = float(size)
                info.width = info.height = size
                info.k = [focal, 0., center, 0., focal, center, 0., 0., 1.]
                cloud.width, cloud.row_step = size*size, size*size*12
                cloud.data = b''.join(struct.pack('<fff', (x-center)*depth/focal, (y-center)*depth/focal, depth)
                                      for y in range(size) for x in range(size)
                                      for depth in [2. if mode != 'flat' and x > center else 1.])
                deadline = time.monotonic()+20
                checked = 0
                while time.monotonic() < deadline and checked < 5:
                    require(process.poll() is None, 'GNG異常終了')
                    messages.clear()
                    cloud.header.stamp = node.get_clock().now().to_msg()
                    if has_info:
                        info.header.stamp = cloud.header.stamp
                        camera.publish(info)
                        rclpy.spin_once(node, timeout_sec=0.02)
                    publisher.publish(cloud)
                    end = time.monotonic()+0.1
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.01)
                    for graph in messages:
                        if graph.header.stamp != cloud.header.stamp or not graph.nodes:
                            continue
                        flags = [item.boundary_evidence for item in graph.nodes]
                        require(all(0 <= value <= 7 for value in flags), '未知の証拠ビット')
                        if mode == 'flat':
                            require(not any(value & 3 for value in flags), '連続面の自由空間・遮蔽誤判定')
                            if any(value & 4 for value in flags):
                                checked += 1
                        elif mode == 'step':
                            if any(value & 1 for value in flags) and any(value & 2 for value in flags):
                                checked += 1
                        else:
                            require(not any(flags), 'CameraInfo欠落時の旧証拠残留')
                            checked += 1
                if checked < 5 and messages:
                    print('診断:', [(item.pos.x, item.pos.y, item.pos.z, item.normal.x, item.normal.y,
                                   item.normal.z, item.boundary_evidence) for item in messages[-1].nodes[:12]], flush=True)
                require(checked >= 5, f'検証フレーム不足: mode={mode} checked={checked}')
                print(f'検証成功: mode={mode} frames={checked}', flush=True)
            print('boundary_evidence_ros=passed', flush=True)
        finally:
            stop_process(process)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            log.seek(0)
            print('\n'.join(log.read().splitlines()[-5:]), flush=True)
            print('検証用GNG: 停止済み', flush=True)


if __name__ == '__main__':
    main()
