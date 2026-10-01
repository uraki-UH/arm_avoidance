#!/usr/bin/env python3
"""独立ROSドメインでの点群入力・自己除去・Emap集約ログの有限検証。実機接続なし。"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import xml.etree.ElementTree as xml

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray
from voxel_msgs.msg import Voxel
import yaml


def main():
    os.environ.update(ROS_DOMAIN_ID='229', ROS_LOCALHOST_ONLY='1')
    root = Path(__file__).resolve().parents[1]
    directory = Path(tempfile.mkdtemp(prefix='viewer_status_check_'))
    config = yaml.safe_load((root/'config/ToPoDualArm.yaml').read_text())
    params = config['/**']['ros__parameters']
    params['enable_dynamixel_current_pose'] = False
    params['enable_realsense_mount_tf'] = False
    params['gripper_volume_graph'] = {'enabled': False}
    params['environment_voxelization']['input_topic'] = '/fixture/points'
    path = directory/'params.yaml'
    path.write_text(yaml.safe_dump(config))
    names = [joint.attrib['name'] for joint in xml.parse(params['urdf_path']).getroot().findall('joint')
             if joint.attrib.get('type') != 'fixed']
    command = ['ros2', 'launch', str(root/'launch/gng_viewer_bridge.launch.py'),
               'params_file:='+str(path), 'joint_control_backend:=external']
    rclpy.init()
    node = rclpy.create_node('viewer_status_check')
    stats, logs, raw, filtered = {}, [], {}, {}
    for stage in ('pc', 'vxl', 'emap'):
        node.create_subscription(Float64MultiArray, '/ToPoDualArm/viewer_status/'+stage,
            lambda message, stage=stage: stats.update({stage: list(message.data)}), qos_profile_sensor_data)
    node.create_subscription(Log, '/rosout', lambda message: logs.append(message), 100)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def save(target, message):
        target[(message.header.stamp.sec, message.header.stamp.nanosec)] = len(message.data)
    node.create_subscription(Voxel, '/ToPoDualArm/roi_voxels', lambda message: save(raw, message), qos)
    node.create_subscription(Voxel, '/ToPoDualArm/self_filter_roi_voxels', lambda message: save(filtered, message), qos)
    joints = node.create_publisher(JointState, '/ToPoDualArm/viewer_joint_states', 10)
    cloud = node.create_publisher(PointCloud2, '/fixture/points', qos_profile_sensor_data)
    points = [(0.25+0.01*x, -0.3+0.01*y, 0.4) for x in range(10) for y in range(10)]
    message = PointCloud2(height=1, width=len(points), is_dense=True, point_step=12,
                         row_step=12*len(points), data=b''.join(struct.pack('<fff', *p) for p in points))
    message.header.frame_id = 'ToPoDualArm/base_link'
    message.fields = [PointField(name=name, offset=4*idx, datatype=PointField.FLOAT32, count=1)
                      for idx, name in enumerate(('x', 'y', 'z'))]
    process = None
    try:
        with (directory/'launch.log').open('w') as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+45
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                message.header.stamp = stamp
                joints.publish(JointState(header=message.header, name=names, position=[0.0]*len(names)))
                cloud.publish(message)
                rclpy.spin_once(node, timeout_sec=.03)
                if len(stats) == 3 and any('PCl: 100 (' in item.msg and '--' not in item.msg for item in logs):
                    break
                assert process.poll() is None, (directory/'launch.log').read_text()[-4000:]
            assert set(stats) == {'pc', 'vxl', 'emap'}, stats
            assert stats['pc'][0] == len(points), stats
            assert all(all(math.isfinite(value) and value >= 0 for value in values) for values in stats.values()), stats
            pairs = [(value, filtered[stamp]) for stamp, value in raw.items() if stamp in filtered]
            assert pairs and all(before >= after for before, after in pairs), pairs
            assert any(stats['vxl'][0] == before and stats['vxl'][2] == before-after for before, after in pairs), stats
            summaries = [item.msg for item in logs if 'PCl: 100 (' in item.msg and '--' not in item.msg]
            assert summaries, [(item.name, item.msg) for item in logs[-10:]]
            assert not any(item.level == Log.INFO and any(text in item.msg for text in
                ('Topic graph:', '直接ROI voxel化:', '環境自己ボクセル除外:', 'Mask received:', 'VLUT update =')) for item in logs)
            # 点群停止後の未更新表示。古い件数の現在値扱いの防止
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            assert any('PCl: -- (-- ms)' in item.msg and 'Vxl: Env --' in item.msg for item in logs[-30:])
            print(json.dumps({'result': 'pass', 'summary': summaries[-1], 'stats': stats,
                              'launch_command': command, 'log': str(directory/'launch.log')}, ensure_ascii=False))
    finally:
        if process is not None and process.poll() is None:
            # launchから子ノードへの伝播を利用。SIGINTの二重配送の回避
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
        print('所有試験launch: 停止済み')


if __name__ == '__main__':
    main()
