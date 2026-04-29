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


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish waving JointState messages for topoarm testing.")
    parser.add_argument("--topic", default="/joint_states_test", help="JointState topic to publish")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish rate in Hz")
    parser.add_argument("--prefix", default="", help="Joint name prefix, if the URDF uses one")
    parser.add_argument("--period-sec", type=float, default=24.0, help="Seconds per full wave cycle")
    parser.add_argument("--amplitude", type=float, default=1.0, help="Multiply the wave amplitude")
    parser.add_argument("--debug", action="store_true", help="Print computed poses while publishing")
    args = parser.parse_args()

    rclpy.init()
    node = Node("wave_joint_state_publisher")
    pub = node.create_publisher(JointState, args.topic, 10)
    rate = node.create_rate(args.hz)
    names = build_joint_names(args.prefix)
    t0 = time.time()

    # Each joint gets its own phase and amplitude so the motion reads clearly in the viewer.
    base_positions = [0.00, 0.20, -0.45, 0.10, 0.18, -0.08, 0.020, -0.020]
    joint_amplitudes = [0.18, 0.55, 0.80, 0.60, 0.50, 0.45, 0.030, 0.030]
    phase_offsets = [0.0, 0.7, 1.5, 2.2, 2.9, 3.6, 0.0, math.pi]
    period_sec = max(2.0, args.period_sec)
    omega = 2.0 * math.pi / period_sec

    print(f"publishing wave joint_states on {args.topic}", flush=True)
    print(f"joint names: {', '.join(names)}", flush=True)

    last_debug_print = 0.0

    try:
        while rclpy.ok():
            t = time.time() - t0
            positions = [
                base_positions[i] + joint_amplitudes[i] * args.amplitude * math.sin(omega * t + phase_offsets[i])
                for i in range(len(base_positions))
            ]

            # Keep the gripper within a sane visible range even if amplitude is increased.
            positions[6] = max(-0.08, min(0.08, positions[6]))
            positions[7] = max(-0.08, min(0.08, positions[7]))

            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            pub.publish(msg)

            if last_debug_print == 0.0:
                formatted = ", ".join(f"{value:+.3f}" for value in positions)
                print(
                    f"initial wave t={t:6.2f}s period={period_sec:4.1f}s "
                    f"pos=[{formatted}]",
                    flush=True,
                )
                last_debug_print = t

            if args.debug and (t - last_debug_print >= 1.0):
                formatted = ", ".join(f"{value:+.3f}" for value in positions)
                print(
                    f"t={t:6.2f}s omega={omega:5.2f} pos=[{formatted}]"
                , flush=True)
                last_debug_print = t

            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
