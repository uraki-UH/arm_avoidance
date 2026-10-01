#!/usr/bin/env python3
"""通常Viewerの自己除去・Tmap更新からGazebo回避・実測描画までの有限検証。"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from voxel_msgs.msg import Voxel
import yaml

from check_dual_arm_control import control_trial, run
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_avoidance_geometry import robot_geometry, mesh_vertices


class environment_trial(control_trial):
    def __init__(self, *args):
        super().__init__(*args)
        self.gng = {}
        self.phases = set()
        self.command_samples = []
        self.qp_samples = []
        self.motion_start = None
        self.enable_points = self.enable_real_joints = True
        self.raw_counts = {}
        self.max_removed = self.num_viewer_matches = 0
        self.max_viewer_excursion = 0.
        self.measured = {}
        self.sim_self = self.initial_sim_self = None
        self.real_self = self.initial_real_self = None
        self.max_sim_self_change = self.max_real_self_change = 0
        self.num_sim_self = 0
        urdf = Path('/ros2_ws/src/urdf/dual_arm_urdf/dual_arm_robot.urdf')
        self.real_geometry = robot_geometry(urdf)
        self.real_positions = np.zeros(len(self.real_geometry.joint_names))
        self.home_positions = dict.fromkeys(self.real_geometry.joint_names, 0.)
        if self.args.enable_left_forward:
            self.home_positions['L_joint1'] = -np.pi/2
        self.max_sim_joint_velocity = 0.
        self.real_positions[self.real_geometry.joint_names.index('neck_tilt_joint')] = 1.24
        transform = self.real_geometry.link_transforms(self.real_positions)[self.real_geometry.link_indices['L_link4']]
        self.self_points = mesh_vertices(urdf.parent/'meshes/L_link4.stl')*.001 @ transform[:3, :3].T + transform[:3, 3]
        self.cloud = self.node.create_publisher(PointCloud2, '/fixture/points', qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.real_joints = [self.node.create_publisher(JointState, '/ToPoDualArm/'+name, qos)
                            for name in ('joint_states', 'viewer_joint_states')]
        self.node.create_subscription(String, self.namespace+'/avoidance/gng_status', self.on_gng, 10)
        self.node.create_subscription(Voxel, '/ToPoDualArm/roi_voxels', self.on_raw, 10)
        self.node.create_subscription(Voxel, '/ToPoDualArm/self_filter_roi_voxels', self.on_filtered, 10)
        self.node.create_subscription(Voxel, self.namespace+'/self_voxel', self.on_sim_self, qos)
        self.node.create_subscription(Voxel, '/ToPoDualArm/self_voxel', self.on_real_self, qos)
        self.node.create_subscription(String, '/viewer/internal/stream/robot/pose', self.on_viewer, qos_profile_sensor_data)
        self.node.create_timer(.1, self.publish_input)
        self.node.create_subscription(JointTrajectory, self.namespace+'/control/avoidance_trajectory',
                                      self.on_avoidance_command, qos_profile_sensor_data)

    def on_avoidance_command(self, message):
        if self.motion_start is None or len(message.points) < 2:
            return
        first, last = message.points[0], message.points[-1]
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.command_samples.append((time.monotonic(), stamp,
            max(abs(a-b) for a, b in zip(first.positions, last.positions))))
        rows = np.array(self.command_samples)
        summary = {'num_commands': len(rows), 'max_step_rad': float(np.max(rows[:, 2])),
                   'num_moving_commands': int(np.count_nonzero(rows[:, 2] > 1e-4))}
        if len(rows) > 1:
            summary.update(wall_interval_p50_sec=float(np.median(np.diff(rows[:, 0]))),
                           sim_interval_p50_sec=float(np.median(np.diff(rows[:, 1]))))
        self.report['checks']['avoidance_commands'] = summary

    def on_demo(self, message):
        super().on_demo(message)
        self.phases.add(self.demo.get('phase'))

    def on_sim_self(self, message):
        if message.header.frame_id != 'sim_ToPoDualArm/base_footprint':
            self.callback_error = 'Gazebo自己ボクセルの座標系不一致'
        self.sim_self = set(message.data)
        self.num_sim_self += 1
        if self.initial_sim_self is not None:
            self.max_sim_self_change = max(self.max_sim_self_change, len(self.sim_self ^ self.initial_sim_self))

    def on_real_self(self, message):
        self.real_self = set(message.data)
        if self.initial_real_self is not None:
            self.max_real_self_change = max(self.max_real_self_change, len(self.real_self ^ self.initial_real_self))

    def on_joint(self, message):
        super().on_joint(message)
        self.max_sim_joint_velocity = max(self.max_sim_joint_velocity,
            max((abs(value) for name, value in zip(message.name, message.velocity) if name.startswith('L_joint')), default=0.))
        self.report['checks']['max_sim_joint_velocity_rad_sec'] = self.max_sim_joint_velocity
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.measured[stamp] = dict(zip(message.name, message.position))
        if len(self.measured) > 1000:
            self.measured.pop(next(iter(self.measured)))

    def on_viewer(self, message):
        data = json.loads(message.data)
        if data.get('tag') != 'sim_ToPoDualArm':
            return
        robot = data['robot']
        measured = self.measured.get(robot['timestamp'])
        if measured is None:
            return
        values = dict(zip(robot['jointNames'], robot['jointValues']))
        error = max(abs(values[name]-measured[name]) for name in measured if name in values)
        if error > 1e-6:
            self.callback_error = f'Gazebo実測とViewer姿勢の不一致: {error}'
        self.num_viewer_matches += 1
        self.max_viewer_excursion = max(self.max_viewer_excursion,
            max(abs(values[f'L_joint{idx}']-self.home_positions[f'L_joint{idx}']) for idx in range(1, 8)))

    def on_gng(self, message):
        self.gng = json.loads(message.data)
        report = self.gng.get('local_qp')
        if report and report.get('num_calls', 0) and (
                not self.qp_samples or report['num_calls'] != self.qp_samples[-1]['num_calls']):
            self.qp_samples.append(report)
            self.report['checks']['local_qp'] = {
                'num_samples': len(self.qp_samples),
                'num_solved': sum(row['status'] == 'solved' for row in self.qp_samples),
                'total_p50_ms': float(np.percentile([row['total_ms'] for row in self.qp_samples], 50)),
                'total_p95_ms': float(np.percentile([row['total_ms'] for row in self.qp_samples], 95)),
                'max_total_ms': max(row['total_ms'] for row in self.qp_samples),
                'last': report}

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
        params = yaml.safe_load((package/'config/ToPoDualArm.yaml').read_text())
        params['/**']['ros__parameters']['environment_voxelization']['input_topic'] = '/fixture/points'
        params_path = self.args.output/'params.yaml'
        params_path.write_text(yaml.safe_dump(params))
        overlay = yaml.safe_load((package/'config/viewer_environment_gazebo_input.yaml').read_text())
        overlay['pipeline']['external_environment']['points_topic'] = '/fixture/points'
        if self.args.enable_left_forward:
            # 合成障害物の高さに対応する比較試験専用の水平姿勢
            overlay['initial_joint_positions'] = {'L_joint1': -np.pi/2}
        if self.args.max_joint_velocity is not None:
            overlay['max_joint_velocity'] = self.args.max_joint_velocity
        if self.args.control_period_sec is not None:
            overlay['control_period_sec'] = self.args.control_period_sec
        if self.args.enable_local_qp is not None:
            overlay['local_qp'] = {'enable_qp': self.args.enable_local_qp}
        overlay_path = self.args.output/'input.yaml'
        overlay_path.write_text(yaml.safe_dump(overlay))
        launch_path = self.args.output/'trial.launch.py'
        launch_path.write_text('''from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(%r), launch_arguments=%r.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(%r), launch_arguments=%r.items()),
    ])
''' % (str(package/'launch/gng_viewer_bridge.launch.py'),
            {'params_file': str(params_path), 'joint_control_backend': 'external', 'enable_realsense_mount_tf': 'false'},
            str(package/'launch/pointcloud_avoidance.launch.py'),
            {'input_config': str(overlay_path), 'gui': 'false', 'enable_viewer': 'true',
             'robot_config': str(package/'config'/('pointcloud_avoidance_topodualarm_left_forward.yaml'
                if self.args.enable_left_forward else 'pointcloud_avoidance_topodualarm.yaml')),
             'gazebo_master_uri': 'http://127.0.0.1:11369'}))
        return ['ros2', 'launch', str(launch_path)]

    def publish_input(self):
        if self.enable_real_joints:
            joints = JointState(name=self.real_geometry.joint_names, position=self.real_positions.tolist())
            joints.header.stamp = self.node.get_clock().now().to_msg()
            for publisher in self.real_joints:
                publisher.publish(joints)
        if not self.enable_points:
            return
        elapsed = 0 if self.motion_start is None else time.monotonic()-self.motion_start
        withdraw_start = self.args.approach_sec + 3.
        ratio = min(elapsed/self.args.approach_sec, 1.) if elapsed < withdraw_start else max(0., 1.-(elapsed-withdraw_start)/self.args.withdraw_sec)
        x = .85-.53*ratio if self.args.enable_left_forward else .4-.39*ratio
        hand_y, hand_z = (.139, .487) if self.args.enable_left_forward else (.18, .25)
        angle = np.linspace(0, 2*np.pi, 30, endpoint=False)
        points = [[axis, hand_y+.035*np.cos(a), hand_z+.035*np.sin(a)]
                  for axis in np.linspace(x, x+.35, 30) for a in angle]
        points.extend([[x+.035*np.cos(b), hand_y+.035*np.sin(b)*np.cos(a), hand_z+.035*np.sin(b)*np.sin(a)]
                       for b in np.linspace(0, np.pi, 20) for a in angle])
        if self.args.enable_left_forward:
            # 接近物がROI外の区間にも観測可能な床面。空ROIによる開始拒否とは別の速度比較
            points.extend([[floor_x, floor_y, .04] for floor_x in np.linspace(.20, .30, 8)
                           for floor_y in np.linspace(.20, .30, 8)])
        points = np.vstack([points, self.self_points])
        message = PointCloud2(height=1, width=len(points), point_step=12, row_step=len(points)*12, is_dense=True)
        message.header.frame_id = 'ToPoDualArm/base_link'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                          for idx, name in enumerate(('x', 'y', 'z'))]
        message.data = np.asarray(points, dtype='<f4').tobytes()
        self.cloud.publish(message)

    def execute(self):
        self.set_stage('environment_startup')
        self.wait(lambda: self.is_hold_ready() and self.demo.get('state') == 'idle'
                  and self.max_removed > 0 and self.num_viewer_matches > 3
                  and self.sim_self and self.real_self
                  and self.gng.get('num_cloud', 0) > 5 and self.gng.get('graph_age_sec', 9.) < .4, 90)
        self.report['checks']['input_ready'] = dict(self.gng)
        self.report['checks']['self_removal'] = {'max_removed_voxels': self.max_removed}
        self.initial_sim_self, self.initial_real_self = self.sim_self.copy(), self.real_self.copy()
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running', 15)
        self.motion_start = time.monotonic()
        self.set_stage('environment_avoidance')

        def has_returned():
            if self.control.get('mode') == 'stopped' or self.demo.get('state') == 'fault':
                raise AssertionError({'demo': self.demo, 'control': self.control, 'gng': self.gng})
            return (time.monotonic()-self.motion_start > self.args.approach_sec+3.+self.args.withdraw_sec+1.
                    and max(abs(self.positions[f'L_joint{idx}']-self.home_positions[f'L_joint{idx}']) for idx in range(1, 8)) < .08
                    and (not self.args.enable_left_forward or self.demo.get('phase') == 'monitoring'))

        self.wait(has_returned, 80)
        assert self.demo['min_clearance_m'] > .035
        assert self.demo['max_excursion_rad'] > .1
        assert self.gng['num_selected_gng']+self.gng['num_local_steps'] > 0
        assert max(abs(self.positions[f'L_joint{idx}']-self.home_positions[f'L_joint{idx}']) for idx in range(1, 8)) < .08
        assert self.max_viewer_excursion > .1 and self.num_viewer_matches > 20
        self.report['checks']['retreat_return'] = {'demo': dict(self.demo), 'gng': dict(self.gng)}
        if self.args.enable_left_forward:
            assert {'avoiding', 'returning', 'monitoring'} <= self.phases
        self.report['checks']['phases'] = sorted(value for value in self.phases if value is not None)
        if self.args.enable_local_qp:
            assert any(row['status'] == 'solved' for row in self.qp_samples)
        self.report['checks']['viewer_measured_pose'] = {'num_matches': self.num_viewer_matches,
            'max_excursion_rad': self.max_viewer_excursion}
        self.report['checks']['max_sim_joint_velocity_rad_sec'] = self.max_sim_joint_velocity
        assert self.max_sim_self_change > 20 and self.max_real_self_change == 0
        self.report['checks']['separate_self_voxels'] = {'num_sim_messages': self.num_sim_self,
            'max_sim_changed_voxels': self.max_sim_self_change, 'max_real_changed_voxels': self.max_real_self_change}
        self.set_stage('real_joint_loss')
        self.enable_real_joints = False
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 15)
        assert '失効' in self.control.get('detail', ''), self.control
        self.report['checks']['real_joint_loss_stop'] = {'gng': dict(self.gng), 'control': dict(self.control)}
        self.enable_real_joints = True
        self.wait(lambda: self.gng.get('real_joint_age_sec', 9.) < .2, 10)
        self.key(b'l')
        self.wait(self.is_hold_ready, 15)
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running', 15)
        self.set_stage('cloud_loss')
        self.enable_points = False
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 15)
        assert '失効' in self.control.get('detail', ''), self.control
        self.report['checks']['cloud_loss_stop'] = {'gng': dict(self.gng), 'control': dict(self.control)}
        self.set_stage('completed')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--enable-left-forward', action='store_true')
    parser.add_argument('--approach-sec', type=float, default=24.)
    parser.add_argument('--withdraw-sec', type=float, default=12.)
    parser.add_argument('--max-joint-velocity', type=float)
    parser.add_argument('--control-period-sec', type=float)
    parser.add_argument('--enable-local-qp', action=argparse.BooleanOptionalAction, default=None)
    args = parser.parse_args()
    args.robot, args.params_file, args.timeout_sec, args.check_avoidance = 'topodualarm', None, 180, True
    args.output = args.output.resolve()
    return run(args, environment_trial)


if __name__ == '__main__':
    raise SystemExit(main())
