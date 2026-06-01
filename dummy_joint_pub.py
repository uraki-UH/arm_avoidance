#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass(frozen=True)
class RobotPreset:
    namespace: str
    joint_names: Sequence[str]
    position_fn: Callable[[float], Sequence[float]]


def clamp(v: float, low: float, high: float) -> float:
    return max(low, min(high, v))


def make_topo_dual_arm_preset() -> RobotPreset:
    joint_names = [
        "left_joint1", "left_joint2", "left_joint3", "left_joint4",
        "left_joint5", "left_joint6", "left_joint7",
        "left_gripper_left_joint", "left_gripper_right_joint",
        "right_joint1", "right_joint2", "right_joint3", "right_joint4",
        "right_joint5", "right_joint6", "right_joint7",
        "right_gripper_left_joint", "right_gripper_right_joint",
    ]

    def positions(t: float) -> Sequence[float]:
        # ゆっくりした安全寄りの動き。必要ならこの関数だけ差し替えればよい。
        a = math.sin(t) * 0.8
        g = clamp(0.02 + 0.01 * math.sin(t * 0.8), 0.0, 0.0775)

        left = [a, a,  a, a,  a, a, a]
        right = list(left)

        # gripper_left_joint が主関節、gripper_right_joint は mimic だが
        # JointState は両方同じ値
        left_gripper = [g, g]
        right_gripper = [g, g]

        return [
            *left,
            *left_gripper,
            *right,
            *right_gripper,
        ]

    return RobotPreset(namespace="ToPoDualArm", joint_names=joint_names, position_fn=positions)


def make_topoarm_preset() -> RobotPreset:
    joint_names = [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
        "gripper_left_joint", "gripper_right_joint",
    ]

    def positions(t: float) -> Sequence[float]:
        a = math.sin(t) *0.8
        g = clamp(0.02 + 0.01 * math.sin(t * 0.8), 0.0, 0.0775)
        return [a, a, a, a, a, a, a, g, g]

    return RobotPreset(namespace="topoarm", joint_names=joint_names, position_fn=positions)


PRESETS: Dict[str, RobotPreset] = {
    "topo_dual_arm": make_topo_dual_arm_preset(),
    "ToPoDualArm": make_topo_dual_arm_preset(),
    "topoarm": make_topoarm_preset(),
}


class DummyJointPublisher(Node):
    def __init__(self, preset: RobotPreset, rate_hz: float):
        super().__init__("dummy_joint_publisher", namespace=preset.namespace)
        self._preset = preset
        # robot namespace 配下だけに publish する。
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._timer = self.create_timer(max(0.01, 1.0 / rate_hz), self._timer_callback)
        self._t = 0.0

        self.get_logger().info(
            f"dummy_joint_pub started: namespace={preset.namespace} "
            f"joints={len(preset.joint_names)} rate_hz={rate_hz:.2f}"
        )

    def _timer_callback(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._preset.joint_names)
        msg.position = list(self._preset.position_fn(self._t))

        if len(msg.position) != len(msg.name):
            self.get_logger().error(
                f"position count mismatch: names={len(msg.name)} positions={len(msg.position)}"
            )
            return

        self._publisher.publish(msg)
        self._t += 0.1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish dummy JointState messages")
    parser.add_argument("--robot", default="topo_dual_arm", help="Robot preset name (topo_dual_arm, topoarm)")
    parser.add_argument("--namespace", default="", help="Override ROS namespace")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Publish rate in Hz")
    args, _ = parser.parse_known_args()
    return args


def main(args=None):
    cli = parse_args()
    preset = PRESETS.get(cli.robot, PRESETS["topo_dual_arm"])
    if cli.namespace:
        preset = RobotPreset(namespace=cli.namespace, joint_names=preset.joint_names, position_fn=preset.position_fn)

    rclpy.init(args=args)
    node = DummyJointPublisher(preset, cli.rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
