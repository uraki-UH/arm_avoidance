#!/usr/bin/env python3
"""共通点群回避の実センサー入力・GNG利用・欠測停止・所有プロセス回収。"""
import argparse
import json
from pathlib import Path

from check_dual_arm_control import control_trial, run


class pointcloud_trial(control_trial):
    def __init__(self, *args):
        super().__init__(*args)
        from std_msgs.msg import String
        self.gng = {}
        self.history = []
        self.node.create_subscription(String, self.namespace + '/avoidance/gng_status', self.on_gng, 10)

    def on_gng(self, message):
        self.gng = json.loads(message.data)
        self.history.append(dict(self.gng))

    def prepare_command(self, _command):
        return ['ros2', 'launch', 'gng_vlut_system', 'pointcloud_avoidance.launch.py',
                'robot_config:=' + str(self.args.robot_config), 'gui:=false', 'enable_viewer:=false',
                'gazebo_master_uri:=http://127.0.0.1:11369']

    def execute(self):
        self.set_stage('pointcloud_startup')
        self.wait(lambda: self.is_hold_ready() and self.demo.get('state') == 'idle'
                  and self.gng.get('num_cloud', 0) > 5 and self.gng.get('num_voxels', 0) > 0, 90)
        self.report['checks']['input_ready'] = dict(self.gng)
        assert self.control.get('enable_hardware_output') is False
        self.set_stage('pointcloud_avoidance')
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running', 15)

        def is_completed():
            if self.demo.get('state') == 'fault' or self.control.get('mode') == 'stopped':
                raise AssertionError({'demo': self.demo, 'control': self.control, 'gng': self.gng})
            return self.demo.get('state') == 'completed'

        self.wait(is_completed, 110)
        assert max(row['num_selected_gng'] for row in self.history) > 0
        assert max(row['num_danger'] + row['num_collision'] for row in self.history) > 0
        assert self.demo['min_clearance_m'] > 0.035
        assert self.demo['max_excursion_rad'] > 0.1
        self.report['checks']['pointcloud_retreat'] = {'demo': dict(self.demo), 'gng': dict(self.gng)}
        self.key(b'a')
        self.wait(self.is_hold_ready, 15)
        self.set_stage('lidar_loss')
        self.key(b'a')
        self.wait(lambda: self.demo.get('state') == 'running' and self.control.get('phase') == 'idle', 15)
        from gazebo_msgs.srv import SetEntityState
        client = self.node.create_client(SetEntityState, '/avoidance_demo/set_entity_state')
        self.wait(client.service_is_ready, 5)
        request = SetEntityState.Request()
        request.state.name = 'avoidance_lidar'
        request.state.reference_frame = 'world'
        request.state.pose.position.z = 10.0
        request.state.pose.orientation.w = 1.0
        future = client.call_async(request)
        self.wait(future.done, 5)
        assert future.result().success
        self.wait(lambda: self.control.get('mode') == 'stopped' and self.is_stopped(), 15)
        assert self.is_fresh()
        self.report['checks']['lidar_loss_stop'] = {'control': dict(self.control), 'gng': dict(self.gng), 'safety': dict(self.safety)}
        self.node.destroy_client(client)
        self.set_stage('completed')

    def close(self):
        self.report['last_gng'] = dict(self.gng)
        (self.args.output / 'gng_history.json').write_text(json.dumps(self.history, indent=2) + '\n')
        super().close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--robot-config', type=Path,
                        default=Path('/ros2_ws/src/gng_vlut_system/config/pointcloud_avoidance_topodualarm.yaml'))
    args = parser.parse_args()
    args.robot, args.params_file, args.timeout_sec, args.check_avoidance = 'topodualarm', None, 180, True
    args.output = args.output.resolve()
    return run(args, pointcloud_trial)


if __name__ == '__main__':
    raise SystemExit(main())
