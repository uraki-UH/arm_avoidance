"""隔離ドメインの候補・PoseArray入力とViewer gatewayの有限時間起動。"""

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
from gng_control_msgs.msg import GraspCandidate, GraspCandidateArray
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "217":
        raise RuntimeError("この検証にはROS_DOMAIN_ID=217が必要")
    rclpy.init()
    node = rclpy.create_node("pose_array_stream_fixture")
    publisher = node.create_publisher(GraspCandidateArray, "/grasp_pose_cands", QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pose_publisher = node.create_publisher(PoseArray, "/test/poses", QoSProfile(
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
                msg = GraspCandidateArray(update_id=data.get("update_id", 1))
                msg.header.frame_id = "graspnet_table"
                msg.header.stamp.sec = data["stamp"]
                for idx, (position, quaternion) in enumerate(data["poses"]):
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = map(float, position)
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                     pose.orientation.w) = map(float, quaternion)
                    msg.candidates.append(GraspCandidate(
                        id=data.get("ids", list(range(len(data["poses"]))))[idx],
                        pose=pose, state=data.get("state", GraspCandidate.UNKNOWN)))
                if data.get("kind") == "poses":
                    pose_publisher.publish(PoseArray(header=msg.header, poses=[c.pose for c in msg.candidates]))
                else:
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
