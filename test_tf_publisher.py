import argparse
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster



def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5
    roll_cos = math.cos(roll_half)
    roll_sin = math.sin(roll_half)
    pitch_cos = math.cos(pitch_half)
    pitch_sin = math.sin(pitch_half)
    yaw_cos = math.cos(yaw_half)
    yaw_sin = math.sin(yaw_half)
    return (
        roll_sin * pitch_cos * yaw_cos - roll_cos * pitch_sin * yaw_sin,
        roll_cos * pitch_sin * yaw_cos + roll_sin * pitch_cos * yaw_sin,
        roll_cos * pitch_cos * yaw_sin - roll_sin * pitch_sin * yaw_cos,
        roll_cos * pitch_cos * yaw_cos + roll_sin * pitch_sin * yaw_sin,
    )


class TestTFPublisher(Node):
    def __init__(self, args: argparse.Namespace | None = None):
        super().__init__("test_tf_publisher")

        args = args or argparse.Namespace()

        self.frame_id = self.declare_parameter(
            "frame_id", getattr(args, "frame_id", "ToPoDualArm/base_link")
        ).value
        self.world_frame = self.declare_parameter(
            "world_frame", getattr(args, "world_frame", "world")
        ).value
        self.publish_hz = float(
            self.declare_parameter("publish_hz", getattr(args, "publish_hz", 20.0)).value
        )
        self.is_static = bool(
            self.declare_parameter("is_static", getattr(args, "is_static", False)).value
        )
        self.child_is_prefixed = bool(
            self.declare_parameter(
                "child_is_prefixed", getattr(args, "child_is_prefixed", False)
            ).value
        )
        self.x = float(self.declare_parameter("x", getattr(args, "x", 0.35)).value)
        self.y = float(self.declare_parameter("y", getattr(args, "y", 0.15)).value)
        self.z = float(self.declare_parameter("z", getattr(args, "z", -0.3)).value)
        self.roll = float(self.declare_parameter("roll", getattr(args, "roll", 0.0)).value)
        self.pitch = float(self.declare_parameter("pitch", getattr(args, "pitch", 0.0)).value)
        self.yaw = float(self.declare_parameter("yaw", getattr(args, "yaw", 3.2)).value)

        if self.is_static:
            self.br = StaticTransformBroadcaster(self)
            self.broadcast_transform()
            self.timer = None
        else:
            if self.publish_hz <= 0.0:
                raise ValueError("publish_hzには正の値が必要")
            self.br = TransformBroadcaster(self)
            self.timer = self.create_timer(1.0 / self.publish_hz, self.broadcast_transform)

    def broadcast_transform(self) -> None:
        child_frame_id = self.frame_id
        if self.child_is_prefixed and "/" not in child_frame_id:
            child_frame_id = f"topoarm/{child_frame_id}"

        now = self.get_clock().now()
        qx, qy, qz, qw = quaternion_from_rpy(self.roll, self.pitch, self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.br.sendTransform(tf_msg)

        if self.is_static:
            self.get_logger().info(
                f"static TF送信: {self.world_frame} -> {child_frame_id}"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--frame-id", default="ToPoDualArm/base_link")
    parser.add_argument("--publish-hz", type=float, default=20.0)
    parser.add_argument("--static", dest="is_static", action="store_true")
    parser.add_argument("--hold-seconds", type=float, default=0.0)
    parser.add_argument("--child-is-prefixed", action="store_true")
    parser.add_argument("--x", type=float, default=0.35)
    parser.add_argument("--y", type=float, default=0.15)
    parser.add_argument("--z", type=float, default=-0.3)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=3.2)
    args, ros_args = parser.parse_known_args()
    if args.hold_seconds < 0.0:
        parser.error("--hold-secondsには0以上の値が必要")

    rclpy.init(args=ros_args)
    node = TestTFPublisher(args)
    try:
        if args.hold_seconds == 0.0:
            rclpy.spin(node)
        else:
            end_time = time.monotonic() + args.hold_seconds
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(node, timeout_sec=min(0.1, end_time - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
