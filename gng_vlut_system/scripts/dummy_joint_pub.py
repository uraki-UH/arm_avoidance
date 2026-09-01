#!/usr/bin/env python3

"""URDFの可動関節を安全な範囲で往復させるJointState publisher。"""

from __future__ import annotations

import argparse
import math
import os
import xml.etree.ElementTree as element_tree
from dataclasses import dataclass
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

try:
    from gng_control_msgs.msg import JointControlClaim
except ImportError:
    JointControlClaim = None


@dataclass(frozen=True)
class JointMotion:
    """ダミー更新対象の関節情報。"""

    name: str
    min_position: float
    max_position: float
    center_position: float
    amplitude: float
    phase: float


def read_joint_motions(urdf_path: str, motion_ratio: float) -> list[JointMotion]:
    """URDFからmimic以外の可動関節と可動範囲を取得。"""
    try:
        root = element_tree.parse(urdf_path).getroot()
    except (element_tree.ParseError, OSError) as exc:
        raise RuntimeError(f"URDFを読み込めません: {urdf_path}: {exc}") from exc

    motions: list[JointMotion] = []
    for joint in root.findall("joint"):
        joint_type = joint.get("type", "")
        if joint_type not in {"revolute", "continuous", "prismatic"}:
            continue
        if joint.find("mimic") is not None:
            continue

        joint_name = joint.get("name", "")
        if not joint_name:
            continue

        limit = joint.find("limit")
        if joint_type == "continuous":
            min_position = -math.pi
            max_position = math.pi
        elif limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            min_position = float(limit.get("lower", "0.0"))
            max_position = float(limit.get("upper", "0.0"))
        else:
            min_position = -1.0
            max_position = 1.0

        if max_position < min_position:
            min_position, max_position = max_position, min_position
        center_position = (min_position + max_position) * 0.5
        movable_range = max_position - min_position
        amplitude = movable_range * 0.5 * motion_ratio
        motions.append(JointMotion(
            name=joint_name,
            min_position=min_position,
            max_position=max_position,
            center_position=center_position,
            amplitude=amplitude,
            phase=len(motions) * 0.61,
        ))

    if not motions:
        raise RuntimeError("URDF内に更新対象の可動関節がありません")
    return motions


class DummyJointPublisher(Node):
    """URDF準拠のダミー関節状態publisher。"""

    def __init__(self, topic: str, motions: Sequence[JointMotion], rate_hz: float,
                 motion_speed: float,
                 claim_topic: str, claim_priority: int) -> None:
        super().__init__("dummy_joint_publisher")
        self._motions = list(motions)
        self._motion_speed = motion_speed
        self._elapsed_sec = 0.0
        self._topic = topic
        self._claim_pub = None
        self._claim_priority = claim_priority
        self._publisher = self.create_publisher(JointState, topic, 10)
        if claim_topic:
            if JointControlClaim is None:
                raise RuntimeError(
                    "--claim-topic には gng_control_msgs が必要です")
            claim_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._claim_pub = self.create_publisher(
                JointControlClaim, claim_topic, claim_qos)
            self._publish_claim(True)
        self._timer = self.create_timer(
            max(0.01, 1.0 / rate_hz), self._timer_callback)
        self._step_sec = max(0.01, 1.0 / rate_hz)

        self.get_logger().info(
            f"ダミー関節状態を開始: topic={topic} "
            f"joints={len(self._motions)} rate_hz={rate_hz:.2f} "
            f"claim={'on' if self._claim_pub else 'off'}")

    def _publish_claim(self, enabled: bool) -> None:
        if self._claim_pub is None:
            return
        claim = JointControlClaim()
        claim.command_topic = self._topic
        claim.joint_names = [motion.name for motion in self._motions]
        claim.priority = self._claim_priority
        claim.mode = JointControlClaim.MODE_EXCLUSIVE
        claim.enabled = enabled
        self._claim_pub.publish(claim)

    def _timer_callback(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [motion.name for motion in self._motions]
        msg.position = [
            motion.center_position + motion.amplitude * math.sin(
                self._elapsed_sec * self._motion_speed + motion.phase)
            for motion in self._motions
        ]
        self._publisher.publish(msg)
        self._publish_claim(True)
        self._elapsed_sec += self._step_sec

    def disable_claim(self) -> None:
        """joint_state_muxにダミー制御の終了を通知。"""
        self._publish_claim(False)
        if self._claim_pub is not None:
            self.get_logger().info("ダミー制御claimを無効化")


def parse_args(args: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="URDFの関節名とリミットを用いるダミーJointState publisher")
    parser.add_argument("--urdf", required=True, help="対象URDFの絶対パスまたは相対パス")
    parser.add_argument("--topic", required=True, help="出力JointState topic")
    parser.add_argument("--claim-topic", default="",
                        help="joint_state_mux用のJointControlClaim topic")
    parser.add_argument("--claim-priority", type=int, default=100,
                        help="joint_state_muxで使用する優先度")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="publish周波数")
    parser.add_argument("--motion-ratio", type=float, default=0.35,
                        help="関節リミット幅に対する片振幅の比率")
    parser.add_argument("--motion-speed", type=float, default=0.55,
                        help="往復運動の角速度")
    return parser.parse_args(args)


def main(args: Sequence[str] | None = None) -> int:
    cli = parse_args(remove_ros_args(args=args)[1:])
    urdf_path = os.path.abspath(cli.urdf)
    if not os.path.isfile(urdf_path):
        raise RuntimeError(f"URDFが見つかりません: {urdf_path}")
    if cli.rate_hz <= 0.0:
        raise RuntimeError("--rate-hz は正の値が必要です")

    motion_ratio = min(1.0, max(0.0, cli.motion_ratio))
    motions = read_joint_motions(urdf_path, motion_ratio)
    topic = cli.topic

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DummyJointPublisher(
        topic, motions, cli.rate_hz, cli.motion_speed,
        cli.claim_topic, cli.claim_priority)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.disable_claim()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
