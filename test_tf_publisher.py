import argparse
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import math


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


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
        self.x = float(self.declare_parameter("x", getattr(args, "x", 0.35)).value)
        self.y = float(self.declare_parameter("y", getattr(args, "y", 0.15)).value)
        self.z = float(self.declare_parameter("z", getattr(args, "z", -0.3)).value)
        self.yaw = float(self.declare_parameter("yaw", getattr(args, "yaw", 3.2)).value)

        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.publish_hz, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        now = self.get_clock().now()
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = self.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.br.sendTransform(tf_msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--frame-id", default="ToPoDualArm/base_link")
    parser.add_argument("--publish-hz", type=float, default=20.0)
    parser.add_argument("--x", type=float, default=0.35)
    parser.add_argument("--y", type=float, default=0.15)
    parser.add_argument("--z", type=float, default=-0.3)
    parser.add_argument("--yaw", type=float, default=3.2)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TestTFPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
