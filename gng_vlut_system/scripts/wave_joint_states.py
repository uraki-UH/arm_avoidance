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
    parser.add_argument("--topic", default="/joint_states", help="JointState topic to publish")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish rate in Hz")
    parser.add_argument("--prefix", default="", help="Joint name prefix, if the URDF uses one")
    parser.add_argument("--segment-sec", type=float, default=4.0, help="Seconds spent between poses")
    parser.add_argument("--debug", action="store_true", help="Print computed poses while publishing")
    args = parser.parse_args()

    rclpy.init()
    node = Node("wave_joint_state_publisher")
    pub = node.create_publisher(JointState, args.topic, 10)
    rate = node.create_rate(args.hz)
    names = build_joint_names(args.prefix)
    t0 = time.time()

    # A small looping set of poses. The robot transitions smoothly between them.
    poses = [
        [0.00, 0.10, -0.25, 0.05, 0.15, 0.00, 0.010, -0.010],
        [0.45, 0.25, -0.05, 0.20, 0.05, 0.18, 0.020, -0.020],
        [0.85, -0.10, 0.20, -0.15, 0.10, -0.08, 0.006, -0.006],
        [0.20, -0.30, 0.35, 0.12, -0.18, 0.22, 0.018, -0.018],
    ]
    segment_count = len(poses)
    segment_sec = max(0.25, args.segment_sec)
    total_sec = segment_count * segment_sec

    print(f"publishing wave joint_states on {args.topic}", flush=True)
    print(f"joint names: {', '.join(names)}", flush=True)

    last_debug_print = 0.0

    try:
        while rclpy.ok():
            t = time.time() - t0
            phase = (t % total_sec) / segment_sec
            seg_idx = int(phase) % segment_count
            next_idx = (seg_idx + 1) % segment_count
            local_t = phase - int(phase)
            smooth_t = local_t * local_t * (3.0 - 2.0 * local_t)

            current_pose = poses[seg_idx]
            next_pose = poses[next_idx]
            positions = [
                current_pose[i] + (next_pose[i] - current_pose[i]) * smooth_t
                for i in range(len(current_pose))
            ]

            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            pub.publish(msg)

            if last_debug_print == 0.0:
                formatted = ", ".join(f"{value:+.3f}" for value in positions)
                print(
                    f"initial pose seg={seg_idx}->{next_idx} smooth={smooth_t:4.2f} "
                    f"pos=[{formatted}]",
                    flush=True,
                )
                last_debug_print = t

            if args.debug and (t - last_debug_print >= 1.0):
                formatted = ", ".join(f"{value:+.3f}" for value in positions)
                print(
                    f"t={t:6.2f}s phase={phase:5.2f} seg={seg_idx}->{next_idx} "
                    f"smooth={smooth_t:4.2f} pos=[{formatted}]"
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
