"""汎用bag再生の許可リスト・引数転送・失敗時の再生禁止の検証。ROS起動なし。"""
import json
import os
from pathlib import Path
import subprocess
import signal
import tempfile
import time
import unittest

import yaml

scripts_dir = Path(__file__).resolve().parent


class PlayBagTest(unittest.TestCase):
    def setUp(self):
        self.temporary_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_dir.cleanup)
        self.folder = Path(self.temporary_dir.name)
        self.bag = self.folder/'bag with spaces;literal'
        self.bag.mkdir()
        self.capture = self.folder/'arguments.json'
        binary = self.folder/'bin'
        binary.mkdir()
        ros2 = binary/'ros2'
        ros2.write_text('''#!/usr/bin/env python3
import json,os,signal,sys,time
from pathlib import Path
capture = Path(os.environ["BAG_TEST_CAPTURE"])
signal.signal(signal.SIGINT, lambda unused_signal, unused_frame: sys.exit(0))
signal.signal(signal.SIGTERM, lambda unused_signal, unused_frame: sys.exit(0))
if sys.argv[1] == "launch":
    capture.with_suffix(".mount.json").write_text(json.dumps({"pid": os.getpid(), "args": sys.argv[1:]}))
    if os.environ.get("BAG_TEST_MOUNT_FAIL"): sys.exit(5)
    while True: time.sleep(0.1)
else:
    capture.write_text(json.dumps(sys.argv[1:]))
    if os.environ.get("BAG_TEST_EXPECT_MOUNT"):
        deadline = time.monotonic()+2
        while not capture.with_suffix(".mount.json").exists() and time.monotonic()<deadline: time.sleep(0.01)
    if os.environ.get("BAG_TEST_KEEP_PLAYING"):
        capture.with_suffix(".bag.pid").write_text(str(os.getpid()))
        while True: time.sleep(0.1)
''')
        ros2.chmod(0o755)
        self.environment = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'],
                                BAG_TEST_CAPTURE=str(self.capture))
        self.write_metadata(['/camera/camera/depth/color/points', '/livox/lidar', '/tf', '/tf_static',
                             '/robot/joint_states', '/rosout', '/control/command',
                             '/topo_dual_arm_max_long/joint_states',
                             '/topo_dual_arm_max_long/roi_voxels', '/topo_dual_arm_max_long/self_voxel',
                             '/topo_dual_arm_max_long/self_filter_roi_voxels',
                             '/topo_dual_arm_max_long/occupied_voxels'])

    def write_metadata(self, topics):
        entries = [{'topic_metadata': {'name': name, 'type': 'std_msgs/msg/String'}, 'message_count': 1}
                   for name in topics]
        entries.append({'topic_metadata': {'name': '/empty'}, 'message_count': 0})
        (self.bag/'metadata.yaml').write_text(yaml.safe_dump(
            {'rosbag2_bagfile_information': {'topics_with_message_count': entries}}))

    def run_player(self, *arguments, wrapper=False):
        script = scripts_dir/('play_arm_avoidance_bag.sh' if wrapper else 'play_bag.sh')
        environment = dict(self.environment)
        if wrapper and '--no-camera-tf' not in arguments:
            environment['BAG_TEST_EXPECT_MOUNT'] = '1'
        return subprocess.run(['bash', str(script), str(self.bag), *arguments],
                              env=environment, text=True, capture_output=True, timeout=15)

    def assert_stopped(self, pid):
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_mount_start_and_normal_cleanup(self):
        result = self.run_player('--loop', wrapper=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        mount = json.loads(self.capture.with_suffix('.mount.json').read_text())
        self.assertEqual(mount['args'], ['launch', 'gng_vlut_system', 'realsense_mount_tf.launch.py',
                                        'parent_frame:=topo_dual_arm_max_long/camera_link', 'child_frame:=camera_link'])
        self.assert_stopped(mount['pid'])

    def test_mount_opt_out_and_invalid_bag(self):
        result = self.run_player('--no-camera-tf', wrapper=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.capture.with_suffix('.mount.json').exists())
        self.write_metadata(['/rosout'])
        self.assertNotEqual(self.run_player(wrapper=True).returncode, 0)
        self.assertFalse(self.capture.with_suffix('.mount.json').exists())

    def test_mount_failure_stops_bag(self):
        self.environment.update(BAG_TEST_MOUNT_FAIL='1', BAG_TEST_KEEP_PLAYING='1')
        result = self.run_player(wrapper=True)
        self.assertEqual(result.returncode, 5, result.stderr)
        self.assert_stopped(json.loads(self.capture.with_suffix('.mount.json').read_text())['pid'])
        pid_file = self.capture.with_suffix('.bag.pid')
        if pid_file.exists(): self.assert_stopped(int(pid_file.read_text()))

    def test_ctrl_c_stops_both_children(self):
        self.environment.update(BAG_TEST_KEEP_PLAYING='1', BAG_TEST_EXPECT_MOUNT='1')
        process = subprocess.Popen(['bash', str(scripts_dir/'play_arm_avoidance_bag.sh'), str(self.bag), '--loop'],
                                   env=self.environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic()+5
            while not self.capture.with_suffix('.bag.pid').exists() and time.monotonic()<deadline:
                time.sleep(0.02)
            self.assertTrue(self.capture.with_suffix('.bag.pid').exists())
            process.send_signal(signal.SIGINT)
            process.communicate(timeout=12)
            self.assertEqual(process.returncode, 130)
            self.assert_stopped(json.loads(self.capture.with_suffix('.mount.json').read_text())['pid'])
            self.assert_stopped(int(self.capture.with_suffix('.bag.pid').read_text()))
        finally:
            if process.poll() is None:
                process.terminate()
                process.communicate(timeout=12)

    def selected(self):
        arguments = json.loads(self.capture.read_text())
        return arguments[arguments.index('--topics')+1:]

    def test_defaults_and_argument_forwarding(self):
        result = self.run_player('--', '--loop', '--rate', '0.5', '--remap', '/livox/lidar:=/dataset/points')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.selected(), ['/camera/camera/depth/color/points', '/livox/lidar', '/tf', '/tf_static',
                                           '/topo_dual_arm_max_long/roi_voxels'])
        self.assertEqual(json.loads(self.capture.read_text())[:10],
                         ['bag', 'play', str(self.bag), '--read-ahead-queue-size', '50',
                          '--loop', '--rate', '0.5', '--remap', '/livox/lidar:=/dataset/points'])

    def test_dry_run_without_ros_process(self):
        result = self.run_player('--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('5トピック', result.stdout)
        self.assertFalse(self.capture.exists())

    def test_default_roi_is_before_self_filter(self):
        for wrapper in [False, True]:
            with self.subTest(wrapper=wrapper):
                result = self.run_player(wrapper=wrapper)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('/topo_dual_arm_max_long/roi_voxels', self.selected())
                self.assertEqual('/topo_dual_arm_max_long/self_voxel' in self.selected(), wrapper)
                for suffix in ('self_filter_roi_voxels', 'occupied_voxels'):
                    self.assertNotIn('/topo_dual_arm_max_long/'+suffix, self.selected())

    def test_direct_play_options(self):
        for wrapper in [False, True]:
            with self.subTest(wrapper=wrapper):
                result = self.run_player('--loop', '--rate', '0.5', '--topic', '/robot/joint_states',
                                         '--clock', '50', '--remap', '/livox/lidar:=/dataset/points',
                                         '/tf:=/recorded_tf', wrapper=wrapper)
                self.assertEqual(result.returncode, 0, result.stderr)
                arguments = json.loads(self.capture.read_text())
                self.assertEqual(arguments[5:arguments.index('--topics')],
                                 ['--loop', '--rate', '0.5', '--clock', '50', '--remap',
                                  '/livox/lidar:=/dataset/points', '/tf:=/recorded_tf'])
                self.assertIn('/robot/joint_states', self.selected())

    def test_dry_run_after_direct_loop(self):
        for wrapper in [False, True]:
            with self.subTest(wrapper=wrapper):
                result = self.run_player('--loop', '--dry-run', wrapper=wrapper)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('--loop', result.stdout)
                self.assertFalse(self.capture.exists())
                self.assertFalse(self.capture.with_suffix('.mount.json').exists())

    def test_legacy_flags_after_direct_loop(self):
        result = self.run_player('--loop', '--roi-voxels', '--joint-states', wrapper=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.selected()), 7)
        self.assertIn('--loop', json.loads(self.capture.read_text()))

    def test_mixed_direct_and_separator_options(self):
        result = self.run_player('--loop', '--', '--rate', '0.5')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(self.capture.read_text())[5:8], ['--loop', '--rate', '0.5'])

    def test_profile_patterns_and_deduplication(self):
        profile = self.folder/'topics.txt'
        profile.write_text('# 記録済みトピック限定\n/robot/*\n/tf # 共通TF\n/tf\n/absent\n/empty\n')
        result = self.run_player('--topics-file', str(profile), '--topic', '/livox/lidar')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.selected(), ['/robot/joint_states', '/tf', '/livox/lidar'])
        self.assertIn('/absent', result.stderr)
        self.assertIn('/empty', result.stderr)

    def test_zero_selection_never_plays_everything(self):
        self.write_metadata(['/rosout'])
        result = self.run_player()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('再生対象が0件', result.stderr)
        self.assertFalse(self.capture.exists())

    def test_invalid_inputs_never_play(self):
        for arguments in [('--topic',), ('--topics-file', '/missing/profile'), ('--', '--topics', '/rosout'),
                          ('--topics', '/rosout'), ('another_bag',)]:
            with self.subTest(arguments=arguments):
                self.assertNotEqual(self.run_player(*arguments).returncode, 0)
                self.assertFalse(self.capture.exists())
        (self.bag/'metadata.yaml').write_text('invalid: [')
        self.assertNotEqual(self.run_player().returncode, 0)
        self.assertFalse(self.capture.exists())

    def test_legacy_optional_topics(self):
        result = self.run_player('--joint-states', '--roi-voxels', wrapper=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.selected()[-3:], ['/topo_dual_arm_max_long/roi_voxels',
                                               '/topo_dual_arm_max_long/self_voxel',
                                               '/topo_dual_arm_max_long/joint_states'])


if __name__ == '__main__':
    unittest.main()
