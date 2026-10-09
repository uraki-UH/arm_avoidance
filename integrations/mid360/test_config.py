"""実機接続前の設定・変換・launch構成検証。"""

import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.events import Shutdown

spec = importlib.util.spec_from_file_location("mid360_launch", Path(__file__).with_name("mid360.launch.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class config_test(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load(Path(__file__).with_name("mid360.yaml").read_text())
        self.config.update(host_ip="192.168.1.5", lidar_ip="192.168.1.198")

    def read(self, config):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
            yaml.safe_dump(config, stream)
            stream.flush()
            return module.read_config(stream.name)

    def test_sdk_network_and_no_double_transform(self):
        config = self.read(self.config)
        packet = module.sdk_config(config)
        host = packet["MID360"]["host_net_info"]
        self.assertEqual(host["point_data_ip"], "192.168.1.5")
        self.assertEqual(host["point_data_port"], 56301)
        self.assertEqual(host["imu_data_port"], 56401)
        self.assertEqual(packet["lidar_configs"][0]["ip"], "192.168.1.198")
        self.assertTrue(all(v == 0 for v in packet["lidar_configs"][0]["extrinsic_parameter"].values()))

    def test_invalid_config(self):
        for change in (
            {"host_ip": ""}, {"host_ip": "0.0.0.0"}, {"lidar_ip": "127.0.0.1"},
            {"lidar_ip": "192.168.1.5"}, {"publish_freq": float("nan")},
            {"publish_freq": 101}, {"enable_mount_tf": "false"},
            {"rot_deg": [0, float("inf"), 0]}, {"pos": [0, 0]},
            {"points_topic": "relative"}, {"frame_id": "/mid360"},
            {"imu_topic": self.config["points_topic"]},
            {"enable_mount_tf": True, "parent_frame_id": "mid360_link"},
        ):
            with self.subTest(change=change), self.assertRaises(Exception):
                self.read(dict(self.config, **change))

    def test_optional_tf_and_cleanup(self):
        for enable_tf in (False, True):
            config = copy.deepcopy(self.config)
            config.update(enable_mount_tf=enable_tf, rot_deg=[0, 45, 0])
            with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
                yaml.safe_dump(config, stream)
                stream.flush()
                context = LaunchContext()
                context.launch_configurations["config"] = stream.name
                with patch.object(module.socket, "socket"), patch.object(module, "Node", wraps=module.Node) as node:
                    actions = module.start(context)
                driver = node.call_args_list[0].kwargs
                self.assertEqual(driver["parameters"][0]["xfer_format"], 0)
                self.assertIn(("/livox/lidar", config["points_topic"]), driver["remappings"])
                packet_path = Path(driver["parameters"][0]["user_config_path"])
                self.assertTrue(packet_path.exists())
                if enable_tf:
                    args = node.call_args_list[1].kwargs["arguments"]
                    self.assertAlmostEqual(float(args[args.index("--pitch") + 1]), 0.7853981633974483)
                self.assertEqual(len(actions), 3 if enable_tf else 2)
                # 終了ハンドラによる一時JSONの解放
                cleanup = actions[0].event_handler.handle(Shutdown(reason="test"), context)[0]
                cleanup.execute(context)
                self.assertFalse(packet_path.exists())


if __name__ == "__main__":
    unittest.main()
