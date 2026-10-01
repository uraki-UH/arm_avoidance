#!/usr/bin/env python3
"""新鮮な実点群の仮想配置と、受信ごとのシミュレーション時刻への変換。"""
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState, PointCloud2, PointField
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException
from dual_arm_avoidance_geometry import robot_geometry


def cloud_xyz(message):
    """行末余白・フィールドオフセット・エンディアンを保持したXYZ抽出。"""
    if message.width <= 0 or message.height <= 0 or message.point_step <= 0:
        raise ValueError('空の点群')
    if message.row_step < message.width*message.point_step or len(message.data) < message.row_step*message.height:
        raise ValueError('点群バッファの欠損')
    fields = {field.name: field for field in message.fields}
    columns = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('XYZ浮動小数点フィールドが必要です')
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset < 0 or field.offset+size > message.point_step:
            raise ValueError('XYZフィールドの範囲不正')
        dtype = ('>' if message.is_bigendian else '<') + ('f4' if size == 4 else 'f8')
        columns.append(np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
            offset=field.offset, strides=(message.row_step, message.point_step)).reshape(-1))
    points = np.column_stack(columns)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('有限のXYZ点がありません')
    return points


def transform_xyz(points, camera_pose, translation, quaternion):
    """光学座標→カメラ本体→仮想空間の順での変換。"""
    camera_points = Rotation.from_quat(quaternion).apply(points) + translation
    return Rotation.from_euler('xyz', camera_pose[3:]).apply(camera_points) + camera_pose[:3]


def has_fresh_input(stamp_ns, last_stamp_ns, wall_sec, max_age_sec):
    age = wall_sec-stamp_ns*1e-9
    return stamp_ns > last_stamp_ns and -0.1 <= age <= max_age_sec


def pose_matrix(pose):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def real_root_transform(camera_pose, camera_mount_pose, real_camera_transform):
    """点群と実機自己形状に共通する、仮想空間での実機ルート配置。"""
    return pose_matrix(camera_pose) @ np.linalg.inv(real_camera_transform @ pose_matrix(camera_mount_pose))


class external_pointcloud_bridge(Node):
    def __init__(self):
        super().__init__('external_pointcloud_bridge')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('実点群の入力監視には実時間が必要です')
        defaults = {'input_topic': '/camera/camera/depth/color/points', 'output_topic': 'external_points',
                    'source_frame': 'camera_depth_optical_frame', 'camera_frame': 'camera_link',
                    'target_frame': '', 'camera_pose': [0.] * 6,
                    'urdf_path': '', 'real_joint_topic': '', 'robot_camera_link': 'camera_link',
                    'real_root_frame': '', 'camera_mount_pose': [0.] * 6,
                    'max_input_age_sec': 1.0, 'max_publish_hz': 10.0}
        self.settings = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        if not self.settings['target_frame'].startswith('sim_'):
            raise ValueError('出力にはsim_名前空間の座標系が必要です')
        pose = np.asarray(self.settings['camera_pose'], dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError('camera_poseには有限のxyz/rpyが必要です')
        for key in ('max_input_age_sec', 'max_publish_hz'):
            if not np.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f'{key}には有限の正数が必要です')
        mount = np.asarray(self.settings['camera_mount_pose'], dtype=float)
        if mount.shape != (6,) or not np.isfinite(mount).all():
            raise ValueError('camera_mount_poseには有限のxyz/rpyが必要です')
        self.geometry = robot_geometry(self.settings['urdf_path'])
        if self.settings['robot_camera_link'] not in self.geometry.link_indices:
            raise ValueError('実カメラ取付リンクがURDFにありません')
        if not self.settings['real_joint_topic'] or not self.settings['real_root_frame'].startswith('sim_'):
            raise ValueError('実機の関節トピックと仮想配置用ルートframeが必要です')
        self.real_positions = None
        self.real_joint_time = 0.
        self.last_real_stamp = self.last_published_real_stamp = -1
        self.real_state_detail = '実機全関節の実測待機'
        self.real_state_pub = self.create_publisher(JointState, 'real_joint_states', 1)
        self.real_tf = TransformBroadcaster(self)
        self.create_subscription(JointState, self.settings['real_joint_topic'], self.on_real_joints, qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sim_stamp = None
        self.clock_time = self.output_time = 0.
        self.last_input_stamp = self.last_output_stamp = -1
        self.num_accepted = self.num_rejected = 0
        self.reason = '入力・clock・TF待機'
        self.publisher = self.create_publisher(PointCloud2, self.settings['output_topic'], qos_profile_sensor_data)
        self.status = self.create_publisher(String, 'external_cloud/status', 1)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.settings['input_topic'], self.on_cloud, qos_profile_sensor_data)
        self.create_timer(.5, self.publish_status)

    def on_real_joints(self, message):
        values = dict(zip(message.name, message.position))
        missing = [name for name in self.geometry.joint_names if name not in values or not np.isfinite(values[name])]
        if missing:
            self.real_state_detail = '実測関節の不足: ' + ', '.join(missing)
            return
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if not has_fresh_input(stamp, self.last_real_stamp, time.time(), self.settings['max_input_age_sec']):
            self.real_state_detail = '実測関節stampの失効・重複・未来時刻'
            return
        positions = np.array([values[name] for name in self.geometry.joint_names])
        if np.any(positions < self.geometry.limits[:, 0]-.001) or np.any(positions > self.geometry.limits[:, 1]+.001):
            self.real_state_detail = '実測関節のURDF範囲逸脱'
            return
        self.real_positions, self.last_real_stamp = positions, stamp
        self.real_joint_time, self.real_state_detail = time.monotonic(), ''

    def publish_real_state(self):
        if self.real_positions is None or time.monotonic()-self.real_joint_time > .5:
            self.real_state_detail = '実機関節の未取得・失効。自己除去後の配信待機'
            return
        if self.last_real_stamp <= self.last_published_real_stamp:
            return
        real_camera = self.geometry.link_transforms(self.real_positions)[self.geometry.link_indices[self.settings['robot_camera_link']]]
        transform = real_root_transform(self.settings['camera_pose'], self.settings['camera_mount_pose'], real_camera)
        item = TransformStamped()
        item.header.stamp, item.header.frame_id = self.sim_stamp, self.settings['target_frame']
        item.child_frame_id = self.settings['real_root_frame']
        item.transform.translation.x, item.transform.translation.y, item.transform.translation.z = map(float, transform[:3, 3])
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
        item.transform.rotation.x, item.transform.rotation.y, item.transform.rotation.z, item.transform.rotation.w = map(float, quaternion)
        self.real_tf.sendTransform(item)
        joints = JointState()
        joints.header.stamp = self.sim_stamp
        joints.name, joints.position = self.geometry.joint_names, self.real_positions.tolist()
        self.real_state_pub.publish(joints)
        self.last_published_real_stamp = self.last_real_stamp

    def on_clock(self, message):
        stamp = message.clock.sec*1_000_000_000+message.clock.nanosec
        previous = -1 if self.sim_stamp is None else self.sim_stamp.sec*1_000_000_000+self.sim_stamp.nanosec
        if stamp > previous:
            self.sim_stamp = message.clock
            self.clock_time = time.monotonic()
        elif stamp < previous:
            self.sim_stamp = None
            self.reason = 'シミュレーション時刻の巻戻り。デモ再起動が必要です'
            self.last_output_stamp = max(self.last_output_stamp, previous)

    def on_cloud(self, message):
        now = time.monotonic()
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        try:
            if message.header.frame_id != self.settings['source_frame']:
                raise ValueError('点群frameがsource_frameと不一致')
            if not has_fresh_input(stamp, self.last_input_stamp, time.time(), self.settings['max_input_age_sec']):
                raise ValueError('入力時刻の失効・重複・未来時刻')
            self.last_input_stamp = stamp
            if self.sim_stamp is None or now-self.clock_time > self.settings['max_input_age_sec']:
                raise ValueError('シミュレーションclockの未受信・停止')
            output_stamp = self.sim_stamp.sec*1_000_000_000+self.sim_stamp.nanosec
            if output_stamp <= self.last_output_stamp or now-self.output_time < 1./self.settings['max_publish_hz']:
                return
            transform = self.buffer.lookup_transform(self.settings['camera_frame'], message.header.frame_id, rclpy.time.Time()).transform
            translation, quaternion = transform.translation, transform.rotation
            points = transform_xyz(cloud_xyz(message), self.settings['camera_pose'],
                [translation.x, translation.y, translation.z], [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            result = PointCloud2()
            result.header.frame_id = self.settings['target_frame']
            result.header.stamp = self.sim_stamp
            result.height, result.width = 1, len(points)
            result.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                             for idx, name in enumerate(('x', 'y', 'z'))]
            result.point_step, result.row_step, result.is_dense = 12, len(points)*12, True
            result.data = np.asarray(points, dtype='<f4').tobytes()
            self.publish_real_state()
            self.publisher.publish(result)
            self.last_output_stamp, self.output_time = output_stamp, now
            self.num_accepted += 1
            self.reason = ''
        except (ValueError, TransformException) as error:
            self.num_rejected += 1
            self.reason = str(error)

    def publish_status(self):
        self.status.publish(String(data=json.dumps({
            'num_accepted': self.num_accepted, 'num_rejected': self.num_rejected,
            'output_age_sec': time.monotonic()-self.output_time,
            'detail': self.reason, 'camera_pose': self.settings['camera_pose'],
            'real_state_detail': self.real_state_detail,
            'has_fresh_real_state': self.real_positions is not None and time.monotonic()-self.real_joint_time <= .5,
            'target_frame': self.settings['target_frame']})))


def main():
    rclpy.init()
    node = None
    try:
        node = external_pointcloud_bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
