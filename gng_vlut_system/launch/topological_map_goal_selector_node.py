#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Int32MultiArray

from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray
from gng_control_msgs.msg import GraspCandidate, GraspCandidateArray

from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


COLLISION_LABEL = int(getattr(TopologicalNode, "WALL", 2))


@dataclass
class TargetPose:
    position: Tuple[float, float, float]
    orientation: Optional[Tuple[float, float, float, float]]
    frame_id: str


def _point_xyz(point: Point) -> Tuple[float, float, float]:
    return float(point.x), float(point.y), float(point.z)


def _normalize(vec: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z = float(vec[0]), float(vec[1]), float(vec[2])
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0
    return x / norm, y / norm, z / norm


def _quat_to_axis_z(quat: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z, w = map(float, quat)
    # Rotation matrix third column for the local +Z axis.
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


def _safe_label(label: int) -> bool:
    return int(label) != COLLISION_LABEL


def _quat_multiply(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class TopologicalMapGoalSelector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("topological_map_goal_selector_node")
        self.topological_map_topic = args.topological_map_topic
        self.output_topic = args.output_topic
        self.candidate_count = max(1, int(args.candidate_count))
        self.non_collision_only = bool(args.non_collision_only)
        self.orientation_weight = float(args.orientation_weight)
        self.manipulability_weight = float(args.manipulability_weight)
        self.map_msg = None
        self.latest_candidates = None
        self.latest_node_features = {}
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.output_pub = self.create_publisher(TopologicalMap, self.output_topic, 10)
        self.goal_candidate_ids_pub = self.create_publisher(
            Int32MultiArray, args.goal_candidate_ids_topic, qos)
        self.goal_candidate_ids_pub.publish(Int32MultiArray(data=[]))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_sub = self.create_subscription(
            TopologicalMap, self.topological_map_topic, self._on_map, qos)
        self.candidate_sub = self.create_subscription(
            GraspCandidateArray, args.candidate_topic, self._on_candidates, qos)
        self.node_feature_sub = None
        if args.node_feature_topic:
            self.node_feature_sub = self.create_subscription(
                TopologicalNodeFeatureArray, args.node_feature_topic,
                self._on_node_feature_array, 10)
        # 静止候補のTF変化にも追従する計画目標の更新
        if not math.isfinite(args.goal_update_hz) or args.goal_update_hz <= 0.0:
            raise ValueError("goal_update_hzには正の有限値が必要です")
        self.timer = self.create_timer(1.0 / args.goal_update_hz, self._maybe_publish)

    def _on_map(self, msg: TopologicalMap) -> None:
        self.map_msg = msg
        self._maybe_publish()

    def _on_candidates(self, msg: GraspCandidateArray) -> None:
        self.latest_candidates = msg
        self._maybe_publish()

    def _on_node_feature_array(self, msg: TopologicalNodeFeatureArray) -> None:
        self.latest_node_features = {int(feature.node_id): feature for feature in msg.features}
        self._maybe_publish()

    def _transform_target(self, target: TargetPose, source_frame: str,
                          map_frame: str) -> Optional[TargetPose]:
        if not source_frame or not map_frame:
            return None
        if source_frame == map_frame:
            return target

        if self.tf_buffer is None:
            return None

        key = (map_frame, source_frame)
        try:
            if key not in self.transform_cache:
                self.transform_cache[key] = self.tf_buffer.lookup_transform(
                    map_frame, source_frame, rclpy.time.Time())
            transform = self.transform_cache[key]
        except TransformException as ex:
            # 同一評価周期内のTF失敗の共有
            self.transform_cache[key] = None
            self.get_logger().warn(
                f"failed to transform target from {source_frame} to {map_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return None
        if transform is None:
            return None

        if do_transform_point is not None:
            point_msg = PointStamped()
            point_msg.header.frame_id = source_frame
            point_msg.point.x, point_msg.point.y, point_msg.point.z = target.position
            try:
                transformed = do_transform_point(point_msg, transform)
                orientation = target.orientation
                if orientation is not None:
                    tf_rot = (
                        float(transform.transform.rotation.x),
                        float(transform.transform.rotation.y),
                        float(transform.transform.rotation.z),
                        float(transform.transform.rotation.w),
                    )
                    orientation = _quat_multiply(tf_rot, orientation)
                return TargetPose(
                    position=_point_xyz(transformed.point),
                    orientation=orientation,
                    frame_id=map_frame,
                )
            except Exception as ex:
                self.get_logger().warn(f"failed to transform point target: {ex}")
                return None

        return None

    def _maybe_publish(self) -> None:
        source = self.latest_candidates
        if source is None:
            return
        self.transform_cache = {}
        selected_ids = set()
        reach_frame = source.evaluation_header.frame_id
        origin = _point_xyz(source.voxel_origin)
        size = source.voxel_size
        can_match = bool(self.map_msg is not None and reach_frame and
                         math.isfinite(size) and size > 0.0 and
                         all(math.isfinite(value) for value in origin))

        def voxel_cell(position):
            return tuple(math.floor((value - offset) / size)
                         for value, offset in zip(position, origin))

        # 到達mapのIDではなく、計画GNGのIDのみをセルへ登録
        goal_cells = {}
        if can_match:
            for node in self.map_msg.nodes:
                if not all(math.isfinite(value) for value in _point_xyz(node.pos)):
                    continue
                target = TargetPose(_point_xyz(node.pos), None, self.map_msg.header.frame_id)
                resolved = self._transform_target(target, target.frame_id, reach_frame)
                if resolved is not None:
                    goal_cells.setdefault(voxel_cell(resolved.position), set()).add(int(node.id))
            for candidate in source.candidates:
                if candidate.state != GraspCandidate.INSIDE:
                    continue
                pose = candidate.pose
                quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                target = TargetPose(_point_xyz(pose.position), quat, source.header.frame_id)
                if not all(math.isfinite(value) for value in (*target.position, *quat)) or math.hypot(*quat) <= 1e-12:
                    continue
                resolved = self._transform_target(target, target.frame_id, reach_frame)
                planning_target = self._transform_target(target, target.frame_id, self.map_msg.header.frame_id)
                if resolved is not None and planning_target is not None:
                    _, _, goal_ids = self._build_selected_map(
                        self.map_msg, planning_target, goal_cells.get(voxel_cell(resolved.position), set()))
                    selected_ids.update(goal_ids)

        selected_map = TopologicalMap()
        if self.map_msg is not None:
            selected_map.header = copy.deepcopy(self.map_msg.header)
            selected_map.frame_number = self.map_msg.frame_number
            selected_map.nodes = [copy.deepcopy(node) for node in self.map_msg.nodes if int(node.id) in selected_ids]
        self.output_pub.publish(selected_map)
        self.goal_candidate_ids_pub.publish(Int32MultiArray(data=sorted(selected_ids)))

    def _build_selected_map(
        self, map_msg: TopologicalMap, target: TargetPose, allowed_node_ids=None
    ) -> Tuple[TopologicalMap, Dict[int, int], List[int]]:
        scored: List[Tuple[float, int]] = []
        target_dir = _normalize(_quat_to_axis_z(target.orientation)) if target.orientation else None

        for idx, node in enumerate(map_msg.nodes):
            if allowed_node_ids is not None and int(node.id) not in allowed_node_ids:
                continue
            if self.non_collision_only and not _safe_label(node.label):
                continue
            node_pos = (float(node.pos.x), float(node.pos.y), float(node.pos.z))
            dx = node_pos[0] - target.position[0]
            dy = node_pos[1] - target.position[1]
            dz = node_pos[2] - target.position[2]
            score = math.sqrt(dx * dx + dy * dy + dz * dz)

            if target_dir is not None:
                node_dir = _normalize((node.normal.x, node.normal.y, node.normal.z))
                if node_dir != (0.0, 0.0, 0.0):
                    alignment = abs(
                        node_dir[0] * target_dir[0]
                        + node_dir[1] * target_dir[1]
                        + node_dir[2] * target_dir[2]
                    )
                    score += self.orientation_weight * (1.0 - alignment)

            if self.manipulability_weight > 0.0:
                feature = self.latest_node_features.get(int(node.id))
                if feature is not None:
                    if getattr(feature, "manip_valid", False):
                        cond_raw = float(getattr(feature, "manip_condition_number", 0.0))
                        cond = max(1.0, cond_raw) if math.isfinite(cond_raw) else 100.0
                        score += self.manipulability_weight * math.log(cond)
                    else:
                        score += self.manipulability_weight * math.log(100.0)

            scored.append((score, idx))

        scored.sort(key=lambda item: item[0])
        selected_old_indices = [idx for _, idx in scored[: self.candidate_count]]
        selected_lookup = {old_idx: new_idx for new_idx, old_idx in enumerate(selected_old_indices)}
        selected_ids = [int(map_msg.nodes[idx].id) for idx in selected_old_indices]

        out = TopologicalMap()
        out.header = copy.deepcopy(map_msg.header)
        out.frame_number = map_msg.frame_number
        out.nodes = [copy.deepcopy(map_msg.nodes[idx]) for idx in selected_old_indices]
        out.edges = []
        out.clusters = []

        return out, selected_lookup, selected_ids



def main() -> None:
    parser = argparse.ArgumentParser(description="Select topological map nodes near a grasp target.")
    parser.add_argument("--topological-map-topic", default="/ToPoDualArm/topological_map_static")
    parser.add_argument("--output-topic", default="/selected_topological_map")
    parser.add_argument("--candidate-count", type=int, default=8)
    parser.add_argument(
        "--non-collision-only",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--orientation-weight", type=float, default=0.25)
    parser.add_argument("--candidate-topic", default="/grasp_pose_cands")
    parser.add_argument("--goal-update-hz", type=float, default=5.0)
    parser.add_argument("--goal-candidate-ids-topic", default="/selected_goal_candidate_ids")
    parser.add_argument("--node-feature-topic", default="/ToPoDualArm/topological_node_features")
    parser.add_argument("--manipulability-weight", type=float, default=0.25)
    args = parser.parse_args()

    rclpy.init()
    node = TopologicalMapGoalSelector(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
