"""longのURDF・MID-360取付設定によるTF接続の隔離検証。"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    args = parser.parse_args()
    config = yaml.safe_load(Path(__file__).with_name("mid360.yaml").read_text())
    assert config["parent_frame_id"] == "chest_lidar_link"
    assert config["pos"] == [0, 0, 0] and config["rot_deg"] == [0, 0, 0]
    frame_prefix = "topo_dual_arm_max_long/"
    processes = []
    rclpy.init()
    node = rclpy.create_node("mid360_mount_check")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    try:
        with tempfile.TemporaryDirectory() as folder:
            params = Path(folder) / "robot.yaml"
            params.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
                "robot_description": Path(args.urdf).read_text(),
                "frame_prefix": frame_prefix,
            }}}))
            commands = [
                ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
                 "--ros-args", "--params-file", str(params)],
                ["ros2", "run", "tf2_ros", "static_transform_publisher",
                 "--frame-id", frame_prefix + config["parent_frame_id"], "--child-frame-id", config["frame_id"],
                 "--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0"],
            ]
            for command in commands:
                processes.append(subprocess.Popen(command, start_new_session=True,
                                                   stdout=subprocess.DEVNULL))
            for yaw in (0.0, math.pi / 2):
                deadline = time.monotonic() + 15
                has_match = False
                while time.monotonic() < deadline:
                    if any(p.poll() is not None for p in processes):
                        raise RuntimeError("TF配信プロセスの早期終了")
                    message = JointState()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.name = ["waist_joint"]
                    message.position = [yaw]
                    publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if not buffer.can_transform(frame_prefix + "base_link", config["frame_id"], Time()):
                        continue
                    transform = buffer.lookup_transform(frame_prefix + "base_link", config["frame_id"], Time()).transform
                    pos = transform.translation
                    q = transform.rotation
                    # センサ+X軸のbase_link座標成分
                    axis = (1 - 2 * (q.y*q.y + q.z*q.z),
                            2 * (q.x*q.y + q.w*q.z),
                            2 * (q.x*q.z - q.w*q.y))
                    expected = (0.069326157159 * math.cos(yaw),
                                0.069326157159 * math.sin(yaw), 0.352347255132)
                    expected_axis = (math.cos(yaw)/math.sqrt(2),
                                     math.sin(yaw)/math.sqrt(2), -1/math.sqrt(2))
                    if all(abs(a-b) < 1e-6 for a,b in zip((pos.x,pos.y,pos.z), expected)) and all(
                        abs(a-b) < 1e-6 for a,b in zip(axis, expected_axis)
                    ):
                        has_match = True
                        print(f"waist={math.degrees(yaw):.0f} deg: base_link→mid360_link OK", flush=True)
                        break
                if not has_match:
                    raise AssertionError("URDF45度取付または腰回転のTF不一致")
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
