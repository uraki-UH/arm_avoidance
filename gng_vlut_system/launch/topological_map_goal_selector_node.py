#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseArray, PoseStamped
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from ais_gng_feature_msgs.msg import TopologicalNodeFeature, TopologicalNodeFeatureArray
from gng_control_msgs.msg import GraspCandidateReachability, GraspCandidateReachabilityArray

try:
    from tf2_ros import Buffer, TransformException, TransformListener
    from tf2_geometry_msgs import do_transform_point, do_transform_pose
except Exception:  # pragma: no cover - optional at runtime
    Buffer = None  # type: ignore[assignment]
    TransformException = Exception  # type: ignore[assignment]
    TransformListener = None  # type: ignore[assignment]
    do_transform_point = None  # type: ignore[assignment]
    do_transform_pose = None  # type: ignore[assignment]

try:
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
except Exception:  # pragma: no cover - runtime fallback
    QoSDurabilityPolicy = None  # type: ignore[assignment]
    QoSHistoryPolicy = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    QoSReliabilityPolicy = None  # type: ignore[assignment]


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


def _copy_point(point: Point) -> Point:
    out = Point()
    out.x = float(point.x)
    out.y = float(point.y)
    out.z = float(point.z)
    return out


def _copy_pose(point: Point, quat: Optional[Tuple[float, float, float, float]]) -> PoseStamped:
    msg = PoseStamped()
    msg.pose.position = _copy_point(point)
    if quat is not None:
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
    else:
        msg.pose.orientation.w = 1.0
    return msg


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
        self.marker_topic = args.marker_topic
        self.candidate_count = max(1, int(args.candidate_count))
        self.non_collision_only = bool(args.non_collision_only)
        self.orientation_weight = float(args.orientation_weight)

        self.target_pose_topic = args.target_pose_topic
        self.target_point_topic = args.target_point_topic
        self.target_pose_array_topic = args.target_pose_array_topic
        self.target_score_topic = args.target_score_topic
        self.goal_candidate_ids_topic = args.goal_candidate_ids_topic
        self.node_feature_topic = args.node_feature_topic
        self.manipulability_weight = float(args.manipulability_weight)
        self.allow_untransformed_target = bool(args.allow_untransformed_target)
        self.reachability_map_topic = args.reachability_map_topic or self.topological_map_topic
        self.reachability_voxel_size = args.reachability_voxel_size
        self.reachability_origin = tuple(args.reachability_voxel_origin)
        if not math.isfinite(self.reachability_voxel_size) or self.reachability_voxel_size <= 0.0:
            raise ValueError("reachability_voxel_sizeには正の有限値が必要です")
        if not all(math.isfinite(value) for value in self.reachability_origin):
            raise ValueError("reachability_voxel_originには有限値が必要です")
        if not math.isfinite(args.reachability_publish_hz) or args.reachability_publish_hz <= 0.0:
            raise ValueError("reachability_publish_hzには正の有限値が必要です")
        self.reachability_map = None
        self.reachability_cells = set()

        self.map_msg: Optional[TopologicalMap] = None
        self.map_revision: int = -1
        self.latest_pose_array: Optional[PoseArray] = None
        self.latest_pose_scores: Optional[Float32MultiArray] = None
        self.latest_node_features: Dict[int, TopologicalNodeFeature] = {}

        self.output_pub = self.create_publisher(TopologicalMap, self.output_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        if (
            QoSProfile is not None
            and QoSHistoryPolicy is not None
            and QoSReliabilityPolicy is not None
            and QoSDurabilityPolicy is not None
        ):
            goal_ids_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        else:
            goal_ids_qos = 10

        self.goal_candidate_ids_pub = self.create_publisher(
            Int32MultiArray, self.goal_candidate_ids_topic, goal_ids_qos
        )
        self.goal_candidate_ids_pub.publish(Int32MultiArray(data=[]))
        self.reachability_pub = self.create_publisher(
            GraspCandidateReachabilityArray, args.reachability_topic, goal_ids_qos
        )
        self.reachability_marker_pub = self.create_publisher(
            MarkerArray, args.reachability_marker_topic, goal_ids_qos
        )

        if (
            QoSProfile is not None
            and QoSHistoryPolicy is not None
            and QoSReliabilityPolicy is not None
            and QoSDurabilityPolicy is not None
        ):
            snapshot_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        else:
            snapshot_qos = 10

        self.map_sub = self.create_subscription(TopologicalMap, self.topological_map_topic, self._on_map, snapshot_qos)
        self.reachability_sub = None
        if self.reachability_map_topic != self.topological_map_topic:
            self.reachability_sub = self.create_subscription(
                TopologicalMap, self.reachability_map_topic, self._on_reachability_map, snapshot_qos
            )

        self.pose_sub = None
        if self.target_pose_topic:
            self.pose_sub = self.create_subscription(
                PoseStamped,
                self.target_pose_topic,
                self._on_pose,
                10,
            )

        self.point_sub = None
        if self.target_point_topic:
            self.point_sub = self.create_subscription(
                PointStamped,
                self.target_point_topic,
                self._on_point,
                10,
            )

        self.pose_array_sub = None
        if self.target_pose_array_topic:
            self.pose_array_sub = self.create_subscription(
                PoseArray,
                self.target_pose_array_topic,
                self._on_pose_array,
                snapshot_qos,
            )

        self.score_sub = None
        if self.target_score_topic:
            self.score_sub = self.create_subscription(
                Float32MultiArray,
                self.target_score_topic,
                self._on_pose_scores,
                snapshot_qos,
            )

        self.node_feature_sub = None
        if self.node_feature_topic:
            self.node_feature_sub = self.create_subscription(
                TopologicalNodeFeatureArray,
                self.node_feature_topic,
                self._on_node_feature_array,
                10,
            )

        self.tf_buffer = None
        self.tf_listener = None
        if Buffer is not None and TransformListener is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        # 候補の再受信がない台車移動時にも現在配置での再評価
        self.reachability_timer = self.create_timer(
            1.0 / args.reachability_publish_hz, self._maybe_publish
        )

    def _on_map(self, msg: TopologicalMap) -> None:
        self.map_msg = msg
        self.map_revision = int(msg.frame_number)
        if self.reachability_map_topic == self.topological_map_topic:
            self._set_reachability_map(msg)
        self._maybe_publish()

    def _voxel_cell(self, position):
        return tuple(math.floor((value - origin) / self.reachability_voxel_size)
                     for value, origin in zip(position, self.reachability_origin))

    def _set_reachability_map(self, msg):
        self.reachability_map = msg
        # 動的衝突ラベルと位置到達性を分離した登録TCPセル集合
        self.reachability_cells = {
            self._voxel_cell(_point_xyz(node.pos)) for node in msg.nodes
            if all(math.isfinite(value) for value in _point_xyz(node.pos))
        }

    def _on_reachability_map(self, msg):
        self._set_reachability_map(msg)
        self._maybe_publish()

    def _on_pose(self, msg: PoseStamped) -> None:
        self._on_pose_array(PoseArray(header=msg.header, poses=[msg.pose]))

    def _on_point(self, msg: PointStamped) -> None:
        pose = _copy_pose(msg.point, None).pose
        self._on_pose_array(PoseArray(header=msg.header, poses=[pose]))

    def _on_pose_array(self, msg: PoseArray) -> None:
        self.latest_pose_array = msg
        self._maybe_publish()

    def _on_pose_scores(self, msg: Float32MultiArray) -> None:
        self.latest_pose_scores = msg
        self._maybe_publish()

    def _on_node_feature_array(self, msg: TopologicalNodeFeatureArray) -> None:
        self.latest_node_features = {
            int(feature.node_id): feature for feature in msg.features
        }
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
        if self.latest_pose_array is None:
            return
        self.transform_cache = {}
        source = self.latest_pose_array
        out = GraspCandidateReachabilityArray()
        out.header = copy.deepcopy(source.header)
        out.evaluation_header.stamp = self.get_clock().now().to_msg()
        reach_frame = self.reachability_map.header.frame_id if self.reachability_map else ""
        out.evaluation_header.frame_id = reach_frame
        out.voxel_size = self.reachability_voxel_size
        out.voxel_origin.x, out.voxel_origin.y, out.voxel_origin.z = self.reachability_origin
        out.reachability_map_topic = self.reachability_map_topic
        scores = list(self.latest_pose_scores.data) if self.latest_pose_scores else []

        # 到達mapのセルと計画用GNGノードIDの対応。可視化用mapのIDは計画に不使用
        goal_cells = {}
        if self.map_msg is not None and reach_frame:
            for node in self.map_msg.nodes:
                if not all(math.isfinite(value) for value in _point_xyz(node.pos)):
                    continue
                target = TargetPose(_point_xyz(node.pos), None, self.map_msg.header.frame_id)
                resolved = self._transform_target(target, target.frame_id, reach_frame)
                if resolved is not None:
                    goal_cells.setdefault(self._voxel_cell(resolved.position), set()).add(int(node.id))

        selected_ids = set()
        for candidate_idx, pose in enumerate(source.poses):
            result = GraspCandidateReachability()
            result.candidate_idx = candidate_idx
            result.pose = copy.deepcopy(pose)
            result.shape_score = float(scores[candidate_idx]) if len(scores) == len(source.poses) else float("nan")
            result.reason = "unknown_map"
            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            target = TargetPose(_point_xyz(pose.position), quat, source.header.frame_id)
            if not all(math.isfinite(value) for value in (*target.position, *quat)) or sum(value * value for value in quat) <= 1e-12:
                result.reason = "invalid_pose"
            elif self.reachability_map is not None:
                resolved = self._transform_target(target, target.frame_id, reach_frame)
                if resolved is None:
                    result.reason = "unknown_transform"
                else:
                    result.has_evaluation = True
                    result.evaluated_pose.position.x, result.evaluated_pose.position.y, result.evaluated_pose.position.z = resolved.position
                    (result.evaluated_pose.orientation.x, result.evaluated_pose.orientation.y,
                     result.evaluated_pose.orientation.z, result.evaluated_pose.orientation.w) = resolved.orientation
                    cell = self._voxel_cell(resolved.position)
                    result.is_in_reachability = cell in self.reachability_cells
                    result.reason = "outside_skip_planning"
                    if result.is_in_reachability:
                        result.reason = "inside_no_goal_nodes"
                        if self.map_msg is not None:
                            planning_target = self._transform_target(target, target.frame_id, self.map_msg.header.frame_id)
                            if planning_target is None:
                                result.reason = "inside_unknown_planning_transform"
                            else:
                                _, _, goal_ids = self._build_selected_map(
                                    self.map_msg, planning_target, goal_cells.get(cell, set()))
                                result.goal_node_ids = goal_ids
                                result.can_plan = bool(goal_ids)
                                if result.can_plan:
                                    result.reason = "inside_plan_eligible"
                                    selected_ids.update(goal_ids)
            out.candidates.append(result)

        selected_map = TopologicalMap()
        if self.map_msg is not None:
            selected_map.header = copy.deepcopy(self.map_msg.header)
            selected_map.frame_number = self.map_msg.frame_number
            selected_map.nodes = [copy.deepcopy(node) for node in self.map_msg.nodes if int(node.id) in selected_ids]
        self.reachability_pub.publish(out)
        self.reachability_marker_pub.publish(self._build_reachability_markers(out))
        self.output_pub.publish(selected_map)
        markers = self._build_markers(selected_map, {})
        markers.markers.insert(0, Marker(action=Marker.DELETEALL))
        self.marker_pub.publish(markers)
        self.goal_candidate_ids_pub.publish(Int32MultiArray(data=sorted(selected_ids)))

    def _build_reachability_markers(self, result):
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for candidate in result.candidates:
            if candidate.reason == "invalid_pose":
                continue
            marker = Marker()
            marker.header = copy.deepcopy(result.header)
            marker.ns = "grasp_reachability"
            marker.id = candidate.candidate_idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            # 把持姿勢のローカルZ軸を表す候補ベクトル
            start = copy.deepcopy(candidate.pose.position)
            axis = _quat_to_axis_z((candidate.pose.orientation.x, candidate.pose.orientation.y,
                                   candidate.pose.orientation.z, candidate.pose.orientation.w))
            axis = _normalize(axis)
            end = Point(x=start.x + 0.08 * axis[0], y=start.y + 0.08 * axis[1], z=start.z + 0.08 * axis[2])
            marker.points = [start, end]
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = 0.008, 0.016, 0.02
            if not candidate.has_evaluation:
                color = (0.9, 0.7, 0.1)
            elif candidate.is_in_reachability:
                color = (0.2, 0.85, 0.25)
            else:
                color = (0.55, 0.55, 0.55)
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            markers.markers.append(marker)
        return markers

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

    def _build_markers(self, map_msg: TopologicalMap, selected_indices: Dict[int, int]) -> MarkerArray:
        _ = selected_indices
        markers = MarkerArray()
        frame_id = map_msg.header.frame_id

        sphere_ns = "selected_nodes"
        node_color_table = {
            1: (0.20, 0.85, 0.25, 1.0),
            2: (0.95, 0.70, 0.10, 1.0),
            3: (0.90, 0.25, 0.20, 1.0),
            4: (0.55, 0.70, 1.00, 1.0),
            5: (0.65, 0.35, 0.95, 1.0),
        }

        for idx, node in enumerate(map_msg.nodes):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = sphere_ns
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = _copy_point(node.pos)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.08
            r, g, b, a = node_color_table.get(int(node.label), (0.7, 0.7, 0.7, 1.0))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            markers.markers.append(marker)
        return markers


def main() -> None:
    parser = argparse.ArgumentParser(description="Select topological map nodes near a grasp target.")
    parser.add_argument("--topological-map-topic", default="/ToPoDualArm/topological_map_static")
    parser.add_argument("--output-topic", default="/selected_topological_map")
    parser.add_argument("--marker-topic", default="/selected_topological_map_markers")
    parser.add_argument("--candidate-count", type=int, default=8)
    parser.add_argument(
        "--non-collision-only",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--orientation-weight", type=float, default=0.25)
    parser.add_argument("--target-pose-topic", default="")
    parser.add_argument("--target-point-topic", default="")
    parser.add_argument("--target-pose-array-topic", default="/grasp_pose_cands")
    parser.add_argument("--target-score-topic", default="/grasp_pose_cand_scores")
    parser.add_argument("--goal-candidate-ids-topic", default="/selected_goal_candidate_ids")
    parser.add_argument("--node-feature-topic", default="/ToPoDualArm/topological_node_features")
    parser.add_argument("--manipulability-weight", type=float, default=0.25)
    parser.add_argument(
        "--allow-untransformed-target",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--reachability-map-topic", default="")
    parser.add_argument("--reachability-topic", default="/grasp_pose_cands/reachability")
    parser.add_argument("--reachability-marker-topic", default="/grasp_pose_cands/reachability_markers")
    parser.add_argument("--reachability-voxel-size", type=float, default=0.05)
    parser.add_argument("--reachability-voxel-origin", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--reachability-publish-hz", type=float, default=5.0)
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
