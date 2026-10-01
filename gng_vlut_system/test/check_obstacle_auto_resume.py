#!/usr/bin/env python3
"""隔離Gazeboでの点群保持・自動再開・手動停止・欠測停止の有限試験。"""
import argparse
from pathlib import Path
import time

import numpy as np
from sensor_msgs.msg import JointState, PointCloud2, PointField

from check_viewer_environment_gazebo import environment_trial
from check_dual_arm_control import run


class obstacle_resume_trial(environment_trial):
    def __init__(self, *args):
        self.near_point = None
        super().__init__(*args)

    def publish_input(self):
        if self.enable_real_joints:
            joints = JointState(name=self.real_geometry.joint_names, position=self.real_positions.tolist())
            joints.header.stamp = self.node.get_clock().now().to_msg()
            for publisher in self.real_joints:
                publisher.publish(joints)
        if not self.enable_points:
            return
        points = [[x, y, .04] for x in np.linspace(.20, .30, 8) for y in np.linspace(.20, .30, 8)]
        if self.near_point is not None:
            points.extend(self.near_point+np.array([x,y,z]) for x in (-.005,0.,.005)
                          for y in (-.005,0.,.005) for z in (-.005,0.,.005))
        points = np.vstack([points, self.self_points])
        message = PointCloud2(height=1, width=len(points), point_step=12, row_step=len(points)*12, is_dense=True)
        message.header.frame_id = 'ToPoDualArm/base_link'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                          for idx,name in enumerate(('x','y','z'))]
        message.data = np.asarray(points, dtype='<f4').tobytes()
        self.cloud.publish(message)

    def approach(self):
        pose = np.array([self.positions[name] for name in self.real_geometry.joint_names])
        centers = self.real_geometry.centers(pose)
        idx = next(idx for idx, shape in enumerate(self.real_geometry.spheres) if shape[0] == 'L_gripper_base')
        self.near_point = centers[idx]+np.array([.055,0.,0.])
        self.wait(lambda: self.demo.get('phase') == 'obstacle_wait', 12)

    def execute(self):
        self.set_stage('input_ready')
        self.wait(lambda: self.is_hold_ready() and self.demo.get('state') == 'idle'
                  and self.gng.get('num_cloud',0)>5 and self.gng.get('graph_age_sec',9.)<.4, 70)
        self.key(b'a')
        self.wait(lambda: self.control.get('mode') == 'avoidance' and self.demo.get('state') == 'running', 12)
        generation = self.demo['run_generation']
        self.set_stage('obstacle_wait')
        self.approach()
        at = time.monotonic()
        self.wait(lambda: time.monotonic()-at>1., 3)
        initial = dict(self.positions)
        at = time.monotonic()
        self.wait(lambda: time.monotonic()-at>1., 3)
        assert self.demo['phase'] == 'obstacle_wait' and self.control['mode'] == 'avoidance'
        assert self.safety['is_stop_latched'] is False
        drift = max(abs(self.positions[name]-initial[name]) for name in initial)
        assert drift < .01, drift
        self.report['checks']['obstacle_wait'] = {'demo':dict(self.demo),'max_hold_drift_rad':drift}
        self.set_stage('automatic_resume')
        self.near_point = None
        self.wait(lambda: self.demo.get('phase') in ('monitoring','returning','avoiding')
                  and self.demo.get('state') == 'running', 15)
        assert self.demo['run_generation'] == generation and self.control['mode'] == 'avoidance'
        assert self.safety['is_stop_latched'] is False
        self.report['checks']['automatic_resume'] = dict(self.demo)
        self.set_stage('space_stop')
        self.approach()
        self.key(b' ')
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 12)
        self.near_point = None
        at = time.monotonic()
        self.wait(lambda: time.monotonic()-at>1., 3)
        assert self.demo['state'] == 'stopped' and self.safety['is_stop_latched'] is True
        self.report['checks']['space_remains_latched'] = dict(self.demo)
        self.key(b'l')
        self.wait(self.is_hold_ready, 15)
        self.key(b'a')
        self.wait(lambda: self.control.get('mode') == 'avoidance', 12)
        self.approach()
        self.set_stage('cloud_loss_during_wait')
        self.enable_points = False
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 15)
        assert '失効' in self.control.get('detail',''), self.control
        self.report['checks']['cloud_loss_remains_fault'] = dict(self.control)
        self.set_stage('completed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.robot, args.params_file, args.timeout_sec, args.check_avoidance = 'topodualarm', None, 150, True
    args.enable_left_forward = True
    args.max_joint_velocity = args.control_period_sec = args.enable_local_qp = None
    args.output = args.output.resolve()
    return run(args, obstacle_resume_trial)


if __name__ == '__main__':
    raise SystemExit(main())
