"""ロボット座標のROIボクセルとworld表示位置の隔離ROS回帰検証。"""

import math
import os
from pathlib import Path
import runpy
import socket
import struct
import subprocess
import tempfile
import time


def main():
    os.environ.update(ROS_DOMAIN_ID='223', ROS_LOCALHOST_ONLY='1')
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    workspace = Path(__file__).resolve().parents[2]
    helpers = runpy.run_path(str(workspace / 'ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/test_stream_restart.py'))
    client_type, stop = helpers['Client'], helpers['stop']
    rclpy.init()
    node = rclpy.create_node('roi_frame_test')
    cloud_pub = node.create_publisher(PointCloud2, '/roi_test/points', 1)
    static_tf = StaticTransformBroadcaster(node)
    dynamic_tf = TransformBroadcaster(node)
    gateway = roi = client = None

    def transform(parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = x, y, z
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return msg

    def apply(point, x, y, z, yaw):
        cos, sin = math.cos(yaw), math.sin(yaw)
        return (x + cos * point[0] - sin * point[1], y + sin * point[0] + cos * point[1], z + point[2])

    points = [(0.313, 0.187, 0.257), (0.437, -0.143, 0.329), (-0.127, 0.271, -0.113)]
    cloud = PointCloud2()
    cloud.height, cloud.width = 1, len(points)
    cloud.fields = [PointField(name=axis, offset=idx * 4, datatype=7, count=1)
                    for idx, axis in enumerate('xyz')]
    cloud.point_step, cloud.row_step = 12, 12 * len(points)
    cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
    points = list(struct.iter_unpack('<fff', bytes(cloud.data)))

    with tempfile.TemporaryFile() as log:
        try:
            gateway = subprocess.Popen([
                '/ros2_ws/install/topo_fuzzy_viewer/lib/topo_fuzzy_viewer/viewer_ws_gateway_node',
                '--ros-args', '-p', 'port:=19095'], stdout=log, stderr=log)
            deadline = time.monotonic() + 15
            while client is None and time.monotonic() < deadline:
                try:
                    client = client_type(19095)
                except ConnectionRefusedError:
                    time.sleep(0.1)
            assert client is not None
            client.sock.settimeout(0.05)
            root_tf = transform('ToPoDualArm/base_footprint', 'ToPoDualArm/base_link')
            static_tf.sendTransform(root_tf)

            # 未接続の短い名前、接続済みの別座標、明示的ロボット座標の区別。
            for case_idx, (has_source_tf, enable_roi_query, source_frame) in enumerate([
                    (False, True, 'base_link'),
                    (True, True, 'base_link'),
                    (True, False, 'base_link'),
                    (True, True, 'ToPoDualArm/base_link')]):
                topic = f'/roi_test/voxels_{case_idx}'
                command = ['/ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/world_index_to_voxel_node', '--ros-args']
                params = {'input_topic': '/roi_test/points', 'output_topic': topic,
                          'target_frame_id': 'ToPoDualArm/base_link', 'world_frame_id': 'world',
                          'allow_unconnected_source_as_world': 'true', 'voxel_size': '0.02',
                          'enable_roi_query': str(enable_roi_query).lower(),
                          'enable_world_bucket_publish': 'false', 'enable_reachability_filter': 'false'}
                for axis in 'xyz':
                    params[f'min_reachability_{axis}'], params[f'max_reachability_{axis}'] = '-5.0', '5.0'
                for name, value in params.items():
                    command += ['-p', f'{name}:={value}']
                print('START', command, flush=True)
                roi = subprocess.Popen(command, stdout=log, stderr=log)
                if has_source_tf:
                    static_tf.sendTransform([root_tf, transform('world', 'base_link', 0.4, -0.2, 0.1, -0.4)])
                cloud.header.frame_id = source_frame
                voxels = set()
                frame = layout = None
                for x, yaw in [(0.15, 1.5), (0.3, 3.14)]:
                    robot_pose = transform('world', 'ToPoDualArm/base_footprint', x=x, yaw=yaw)
                    world_points = points if not has_source_tf else [apply(p, 0.4, -0.2, 0.1, -0.4) for p in points]
                    if source_frame == 'ToPoDualArm/base_link':
                        world_points = [apply(p, x, 0, 0, yaw) for p in points]
                    local_points = [apply((p[0] - x, p[1], p[2]), 0, 0, 0, -yaw) for p in world_points]
                    expected = {str(sum((math.floor(p[axis] / 0.02) + 1000000) << shift
                                        for axis, shift in enumerate((42, 21, 0)))) for p in local_points}
                    deadline = time.monotonic() + 12
                    next_snapshot = 0.0
                    has_match = False
                    while time.monotonic() < deadline:
                        assert roi.poll() is None and gateway.poll() is None
                        robot_pose.header.stamp = node.get_clock().now().to_msg()
                        dynamic_tf.sendTransform(robot_pose)
                        cloud_pub.publish(cloud)
                        rclpy.spin_once(node, timeout_sec=0.01)
                        client.send({'id': 'roi_test_active', 'method': 'sources.setActive',
                                     'params': {'sourceId': topic, 'active': True}})
                        # ID不変時の差分配信省略に対応するスナップショット取得。
                        if time.monotonic() >= next_snapshot:
                            client.send({'type': 'request.state'})
                            next_snapshot = time.monotonic() + 0.5
                        try:
                            value = client.receive()
                        except socket.timeout:
                            continue
                        if not isinstance(value, dict) or value.get('tag') != topic:
                            continue
                        if value.get('type') == 'stream.voxel':
                            voxels = set(value['data'])
                            frame, layout = value['frameId'], value['layout']
                        elif value.get('type') == 'stream.voxel.delta':
                            voxels.difference_update(value['removed'])
                            voxels.update(value['added'])
                        if frame == 'ToPoDualArm/base_link' and voxels == expected:
                            has_match = True
                            break
                    assert has_match, (case_idx, x, yaw, frame, voxels, expected)
                    # Viewer同様のセル中心復元とworldへのTF適用。量子化誤差だけの位置差。
                    for point in local_points:
                        center = tuple((math.floor(v / layout['voxelSize']) + 0.5) * layout['voxelSize'] for v in point)
                        center_world = apply(center, x, 0, 0, yaw)
                        point_world = apply(point, x, 0, 0, yaw)
                        assert math.dist(center_world, point_world) <= math.sqrt(3) * 0.01 + 1e-8
                    print(f'PASS case={case_idx} x={x} yaw={yaw}: robot-local IDs / world position', flush=True)
                stop(roi)
                roi = None
        except BaseException:
            print(os.pread(log.fileno(), 24000, max(0, os.fstat(log.fileno()).st_size - 24000)).decode(errors='replace'))
            raise
        finally:
            if client:
                client.sock.close()
            stop(roi)
            stop(gateway)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
