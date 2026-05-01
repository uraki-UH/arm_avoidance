#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:
    pass


def build_joint_names(prefix: str) -> list[str]:
    return [
        f"{prefix}joint1",
        f"{prefix}joint2",
        f"{prefix}joint3",
        f"{prefix}joint4",
        f"{prefix}joint5",
        f"{prefix}joint6",
        f"{prefix}gripper_left_joint",
        f"{prefix}gripper_right_joint",
    ]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class JointStateTestPublisher(Node):
    def __init__(self, topic: str, hz: float, prefix: str, debug: bool) -> None:
        super().__init__("test_joint_state_publisher")

        self.topic = topic
        self.hz = hz
        self.prefix = prefix
        self.debug = debug

        self.pub = self.create_publisher(JointState, self.topic, 10)
        self.timer = self.create_timer(1.0 / max(1.0, self.hz), self.on_timer)

        self.names = build_joint_names(self.prefix)
        self.start_time = time.time()
        self.last_debug_print = 0.0

        # A smooth, repeating posture family for viewer tests.
        self.home = [0.0, 0.18, -0.32, 0.08, 0.22, 0.00, 0.012, -0.012]
        self.amplitude = [0.55, 0.35, 0.28, 0.30, 0.24, 0.20, 0.010, -0.010]
        self.phase = [0.0, 0.8, 1.4, 2.1, 2.8, 3.5, 0.0, math.pi]
        self.speed = [0.6, 0.45, 0.5, 0.35, 0.4, 0.3, 0.8, 0.8]
        self.limits = [
            (-1.6, 1.6),
            (-1.2, 1.2),
            (-1.4, 1.4),
            (-1.4, 1.4),
            (-1.3, 1.3),
            (-1.6, 1.6),
            (0.0, 0.03),
            (-0.03, 0.0),
        ]

        print(f"publishing test joint_states on {self.topic}", flush=True)
        print(f"joint names: {', '.join(self.names)}", flush=True)

    def compute_positions(self, elapsed: float) -> list[float]:
        positions: list[float] = []
        for i, base in enumerate(self.home):
            wave = math.sin(elapsed * self.speed[i] + self.phase[i])
            value = base + self.amplitude[i] * wave
            lo, hi = self.limits[i]
            positions.append(clamp(value, lo, hi))
        return positions

    def on_timer(self) -> None:
        now = self.get_clock().now().to_msg()
        elapsed = time.time() - self.start_time
        positions = self.compute_positions(elapsed)

        msg = JointState()
        msg.header.stamp = now
        msg.name = self.names
        msg.position = positions
        self.pub.publish(msg)

        if self.last_debug_print == 0.0:
            formatted = ", ".join(f"{value:+.3f}" for value in positions)
            print(f"initial positions: [{formatted}]", flush=True)
            self.last_debug_print = elapsed
        elif self.debug and elapsed - self.last_debug_print >= 1.0:
            formatted = ", ".join(f"{value:+.3f}" for value in positions)
            print(f"t={elapsed:6.2f}s pos=[{formatted}]", flush=True)
            self.last_debug_print = elapsed


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously publish joint_states for viewer-side motion tests."
    )
    parser.add_argument("--topic", default="/joint_states", help="JointState topic to publish")
    parser.add_argument("--hz", type=float, default=30.0, help="Publish rate in Hz")
    parser.add_argument("--prefix", default="", help="Joint name prefix, if the URDF uses one")
    parser.add_argument("--debug", action="store_true", help="Print computed positions while publishing")
    parser.add_argument(
        "--list-joints",
        action="store_true",
        help="Print the joint names that will be published and exit",
    )
    args = parser.parse_args()

    names = build_joint_names(args.prefix)
    if args.list_joints:
        print("\n".join(names))
        return 0

    rclpy.init()
    node = JointStateTestPublisher(args.topic, args.hz, args.prefix, args.debug)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
