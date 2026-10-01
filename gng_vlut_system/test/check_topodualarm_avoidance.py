#!/usr/bin/env python3
"""ToPoDualArm統合launchの左右退避・A保持・Space停止・所有プロセス回収。"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as et

from check_dual_arm_control import control_trial, run


class avoidance_trial(control_trial):
    def __init__(self, *args):
        super().__init__(*args)
        self.phases = set()
        self.history = []
        self.has_checked_limits = False
        self.num_viewer_poses = 0
        if getattr(self.args, 'check_viewer_startup', False):
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import String
            self.node.create_subscription(String, '/viewer/internal/stream/robot/pose',
                                          self.on_viewer_pose, qos_profile_sensor_data)
        root = et.parse('/ros2_ws/src/urdf/dual_arm_urdf/dual_arm_robot.urdf').getroot()
        self.limits = {item.get('name'): {
            key: float(item.find('limit').get(key)) for key in ('velocity', 'effort')}
            for item in root.findall('joint') if item.get('type') != 'fixed'}

    def prepare_command(self, command):
        if getattr(self.args, 'check_viewer_startup', False):
            # Viewer有効の機種別既定設定による起動確認
            return [arg for arg in command if not arg.startswith('demo_config:=')]
        return command

    def on_viewer_pose(self, message):
        value = json.loads(message.data)
        if value.get('tag') == 'sim_ToPoDualArm':
            self.num_viewer_poses += 1

    def on_joint(self, message):
        super().on_joint(message)
        if self.demo.get('state') != 'running':
            return
        for idx, name in enumerate(message.name):
            if name not in self.limits:
                continue
            for field in ('velocity', 'effort'):
                values = getattr(message, field)
                if idx >= len(values) or not math.isfinite(values[idx]) or abs(values[idx]) > self.limits[name][field] + 1e-6:
                    self.callback_error = f'URDF上限超過または欠測: {name}/{field}'
        self.has_checked_limits = True

    def on_demo(self, message):
        super().on_demo(message)
        self.history.append(dict(self.demo))
        self.phases.add((self.demo.get('side'), self.demo.get('phase')))

    def execute(self):
        self.set_stage('startup')
        self.wait(lambda: self.is_hold_ready() and self.demo.get('state') == 'idle'
                  and 'Enter不要' in self.launch.transcript, 90)
        assert self.node.count_publishers(self.namespace + '/dual_arm_controller/joint_trajectory') == 1
        assert self.control.get('enable_hardware_output') is False
        self.report['checks']['startup_hold'] = dict(self.control)
        if getattr(self.args, 'check_viewer_startup', False):
            self.wait(lambda: self.num_viewer_poses >= 10, 15)
            self.report['checks']['viewer_pose_messages'] = self.num_viewer_poses
            self.set_stage('completed')
            return
        self.set_stage('avoidance')
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running'
                  and self.control.get('mode') == 'avoidance' and self.control.get('phase') == 'idle'
                  and 'モード: 回避 /' in self.launch.transcript, 15)
        self.check_cross_mode_rejection('avoidance', b'l', 'リーダーフォロワーON')
        self.set_stage('both_sides')

        def is_completed():
            if self.demo.get('state') == 'fault' or self.control.get('mode') == 'stopped':
                raise AssertionError({'control': self.control, 'demo': self.demo})
            return self.demo.get('state') == 'completed'

        self.wait(is_completed, 150)
        assert self.has_checked_limits
        assert self.demo['min_clearance_m'] > 0.035
        assert self.demo['min_home_clearance_m'] < 0
        assert self.demo['max_excursion_rad'] > 0.2
        for side in ('left', 'right'):
            for phase in ('approaching', 'near', 'withdrawing', 'returning'):
                assert (side, phase) in self.phases, (side, phase)
        self.report['checks']['both_sides'] = dict(self.demo)
        self.key(b'a')
        self.wait(self.is_hold_ready, 15)
        self.report['checks']['a_hold'] = True
        self.set_stage('space_stop')
        self.key(b' ')
        self.wait(self.is_stopped, 15)
        self.observe_hold('space_hold', True)
        self.report['checks']['space_stop'] = dict(self.safety)
        self.set_stage('completed')

    def close(self):
        (self.args.output / 'avoidance_history.json').write_text(json.dumps(self.history, indent=2) + '\n')
        super().close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check-viewer-startup', action='store_true')
    args = parser.parse_args()
    args.robot = 'topodualarm'
    args.params_file = None
    args.timeout_sec = 180
    args.check_avoidance = True
    args.output = args.output.resolve()
    return run(args, avoidance_trial)


if __name__ == '__main__':
    raise SystemExit(main())
