#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class OneShotTFPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("test_tf_once_publisher")

        self.frame_id = args.frame_id
        self.world_frame = args.world_frame
        self.x = args.x
        self.y = args.y
        self.z = args.z
        self.roll = args.roll
        self.pitch = args.pitch
        self.yaw = args.yaw
        self.child_is_prefixed = args.child_is_prefixed
        self.hold_seconds = args.hold_seconds
        self.publish_hz = args.publish_hz

        self.br = TransformBroadcaster(self)

    def publish_once(self) -> None:
        child_frame_id = self.frame_id
        if self.child_is_prefixed and "/" not in child_frame_id:
            child_frame_id = f"topoarm/{child_frame_id}"

        qx, qy, qz, qw = quaternion_from_rpy(self.roll, self.pitch, self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        end_time = time.time() + max(0.0, self.hold_seconds)
        period = 1.0 / max(1.0, self.publish_hz)
        while True:
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            self.br.sendTransform(tf_msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            if time.time() >= end_time:
                break
            time.sleep(period)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Publish a TF update on /tf once or for a short duration."
    )
    parser.add_argument("--world-frame", default="world", help="Parent frame id")
    parser.add_argument("--frame-id", default="base_footprint", help="Child frame id")
    parser.add_argument("--x", type=float, default=0.0, help="Translation x")
    parser.add_argument("--y", type=float, default=0.0, help="Translation y")
    parser.add_argument("--z", type=float, default=0.0, help="Translation z")
    parser.add_argument("--roll", type=float, default=0.0, help="Rotation roll (rad)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Rotation pitch (rad)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Rotation yaw (rad)")
    parser.add_argument("--hold-seconds", type=float, default=1.0, help="Keep publishing for this long")
    parser.add_argument("--publish-hz", type=float, default=20.0, help="Repeat rate while holding")
    parser.add_argument(
        "--child-is-prefixed",
        action="store_true",
        help="Prefix child frame with topoarm/ unless it already contains a slash",
    )
    args = parser.parse_args()

    rclpy.init()
    node = OneShotTFPublisher(args)

    try:
        node.publish_once()
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
