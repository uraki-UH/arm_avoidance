"""隔離ドメインのPoseArray入力とViewer gatewayの有限時間起動。"""

import json
import os
import select
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "217":
        raise RuntimeError("この検証にはROS_DOMAIN_ID=217が必要")
    rclpy.init()
    node = rclpy.create_node("pose_array_stream_fixture")
    publisher = node.create_publisher(PoseArray, "/grasp_pose_cands", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    command = ["/ros2_ws/install/topo_fuzzy_viewer/lib/topo_fuzzy_viewer/viewer_ws_gateway_node",
               "--ros-args", "-p", "port:=19091", "-r", "__node:=pose_array_stream_test_gateway"]
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            print("READY", flush=True)
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("gatewayの予期しない終了")
                rclpy.spin_once(node, timeout_sec=0.02)
                if not select.select([sys.stdin], [], [], 0)[0]:
                    continue
                line = sys.stdin.readline()
                if not line or line.strip() == "STOP":
                    break
                data = json.loads(line)
                msg = PoseArray()
                msg.header.frame_id = "graspnet_table"
                msg.header.stamp.sec = data["stamp"]
                for position, quaternion in data["poses"]:
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = map(float, position)
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                     pose.orientation.w) = map(float, quaternion)
                    msg.poses.append(pose)
                publisher.publish(msg)
                print("PUBLISHED", flush=True)
        finally:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
            print("STOPPED", flush=True)
            if process.returncode not in (0, -signal.SIGINT):
                log.seek(0)
                print(log.read(), file=sys.stderr)


if __name__ == "__main__":
    main()
