#!/usr/bin/env python3
"""実時間stampの試験点群によるGazebo継続回避・欠測停止の有限検証。"""
import argparse
import json
from pathlib import Path
import time
import sys

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from voxel_msgs.msg import Voxel
import yaml

from check_dual_arm_control import control_trial, run
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from dual_arm_avoidance_geometry import robot_geometry, mesh_vertices
from external_pointcloud_bridge import real_root_transform


class external_trial(control_trial):
    def __init__(self, *args):
        super().__init__(*args)
        self.gng, self.bridge = {}, {}
        self.motion_start = None
        self.enable_points = True
        self.enable_real_joints = True
        self.raw_counts = {}
        self.max_removed = 0
        self.camera_pose = [0., .6, .6, 0., 0., 0.]
        urdf = Path('/ros2_ws/src/urdf/dual_arm_urdf/dual_arm_robot.urdf')
        self.real_geometry = robot_geometry(urdf)
        self.real_positions = np.zeros(len(self.real_geometry.joint_names))
        for name, value in [('waist_joint', .1), ('neck_pan_joint', .1), ('neck_tilt_joint', .2)]:
            self.real_positions[self.real_geometry.joint_names.index(name)] = value
        transforms = self.real_geometry.link_transforms(self.real_positions)
        placement = real_root_transform(self.camera_pose, [0.]*6,
            transforms[self.real_geometry.link_indices['camera_link']])
        link_transform = placement @ transforms[self.real_geometry.link_indices['L_link4']]
        vertices = mesh_vertices(urdf.parent/'meshes/L_link4.stl')*.001
        self.self_points = vertices @ link_transform[:3, :3].T + link_transform[:3, 3]
        self.cloud = self.node.create_publisher(PointCloud2, '/fixture/points', qos_profile_sensor_data)
        self.real_joints = self.node.create_publisher(JointState, '/fixture/real_joint_states', 10)
        self.node.create_subscription(String, self.namespace+'/avoidance/gng_status', self.on_gng, 10)
        self.node.create_subscription(String, self.namespace+'/external_cloud/status', self.on_bridge, 10)
        self.node.create_subscription(Voxel, self.namespace+'/roi_voxels', self.on_raw, 10)
        self.node.create_subscription(Voxel, self.namespace+'/self_filter_roi_voxels', self.on_filtered, 10)
        self.node.create_timer(.1, self.publish_points)
        self.tf = StaticTransformBroadcaster(self.node)
        item = TransformStamped()
        item.header.frame_id, item.child_frame_id = 'fixture_camera', 'fixture_optical'
        item.transform.rotation.w = 1.
        self.tf.sendTransform(item)

    def on_gng(self, message):
        self.gng = json.loads(message.data)

    def on_bridge(self, message):
        self.bridge = json.loads(message.data)

    def on_raw(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        self.raw_counts[stamp] = len(message.data)
        if len(self.raw_counts) > 100:
            self.raw_counts.pop(next(iter(self.raw_counts)))

    def on_filtered(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if stamp in self.raw_counts:
            self.max_removed = max(self.max_removed, self.raw_counts[stamp]-len(message.data))

    def prepare_command(self, _command):
        package = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((package/'config/realsense_gazebo_input.yaml').read_text())
        config['pipeline']['external_cloud'].update(input_topic='/fixture/points', source_frame='fixture_optical',
            camera_frame='fixture_camera', camera_pose=self.camera_pose, real_joint_topic='/fixture/real_joint_states')
        path = self.args.output/'input.yaml'
        path.write_text(yaml.safe_dump(config))
        return ['ros2', 'launch', 'gng_vlut_system', 'pointcloud_avoidance.launch.py',
                'input_config:='+str(path), 'gui:=false', 'enable_viewer:=false',
                'gazebo_master_uri:=http://127.0.0.1:11369']

    def publish_points(self):
        if self.enable_real_joints:
            joints = JointState(name=self.real_geometry.joint_names, position=self.real_positions.tolist())
            joints.header.stamp.sec, joints.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
            self.real_joints.publish(joints)
        if not self.enable_points:
            return
        elapsed = 0 if self.motion_start is None else time.monotonic()-self.motion_start
        ratio = min(elapsed/24., 1.) if elapsed < 27. else max(0., 1.-(elapsed-27.)/12.)
        x = .4-.39*ratio
        # 実点群経路だけで与える前腕表面。Gazeboモデル・真値トピックの生成なし
        angle = np.linspace(0, 2*np.pi, 30, endpoint=False)
        points = [[axis, .18+.035*np.cos(a), .25+.035*np.sin(a)]
                  for axis in np.linspace(x, x+.35, 30) for a in angle]
        points.extend([[x+.035*np.cos(b), .18+.035*np.sin(b)*np.cos(a), .25+.035*np.sin(b)*np.sin(a)]
                       for b in np.linspace(0, np.pi, 20) for a in angle])
        points = np.vstack([points, self.self_points]) - self.camera_pose[:3]
        message = PointCloud2(height=1, width=len(points), point_step=12, row_step=len(points)*12, is_dense=True)
        message.header.frame_id = 'fixture_optical'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
        message.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                          for idx, name in enumerate(('x', 'y', 'z'))]
        message.data = np.asarray(points, dtype='<f4').tobytes()
        self.cloud.publish(message)

    def execute(self):
        self.set_stage('external_startup')
        self.wait(lambda: self.is_hold_ready() and self.demo.get('state') == 'idle'
                  and self.bridge.get('num_accepted', 0) > 5 and self.max_removed > 0, 90)
        self.report['checks']['input_ready'] = dict(self.bridge)
        self.report['checks']['self_removal'] = {'max_removed_voxels': self.max_removed,
            'real_joint_names': self.real_geometry.joint_names, 'real_positions': self.real_positions.tolist()}
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running', 15)
        self.motion_start = time.monotonic()
        self.set_stage('external_avoidance')

        def has_returned():
            if self.control.get('mode') == 'stopped' or self.demo.get('state') == 'fault':
                raise AssertionError({'demo': self.demo, 'control': self.control, 'gng': self.gng})
            return time.monotonic()-self.motion_start > 56.

        self.wait(has_returned, 65)
        assert self.demo['min_clearance_m'] > .035
        assert self.demo['max_excursion_rad'] > .1
        assert self.gng['num_selected_gng'] > 0
        assert max(abs(self.positions[f'L_joint{idx}']) for idx in range(1, 8)) < .08
        assert self.demo['state'] == 'running'
        self.report['checks']['retreat_return'] = {'demo': dict(self.demo), 'gng': dict(self.gng)}
        self.set_stage('real_joint_loss')
        self.enable_real_joints = False
        accepted = self.bridge['num_accepted']
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 15)
        assert self.bridge['num_accepted'] > accepted
        self.report['checks']['real_joint_loss_stop'] = {'bridge': dict(self.bridge), 'control': dict(self.control)}
        self.enable_real_joints = True
        self.wait(lambda: self.bridge.get('has_fresh_real_state', False), 10)
        self.key(b'l')
        self.wait(self.is_hold_ready, 15)
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running', 15)
        self.set_stage('external_loss')
        self.enable_points = False
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 15)
        assert self.is_fresh()
        self.report['checks']['loss_stop'] = {'control': dict(self.control), 'safety': dict(self.safety)}
        self.set_stage('completed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.robot, args.params_file, args.timeout_sec, args.check_avoidance = 'topodualarm', None, 180, True
    args.output = args.output.resolve()
    return run(args, external_trial)


if __name__ == '__main__':
    raise SystemExit(main())
