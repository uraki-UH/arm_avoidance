from __future__ import annotations

import argparse
import fnmatch
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, Info, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from sensor_msgs_py import point_cloud2 as pc2


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping: {path}")
    return data


def _infer_storage_id(bag_path: Path) -> str:
    if bag_path.is_file():
        suffix = bag_path.suffix.lower()
        if suffix == ".mcap":
            return "mcap"
        if suffix in {".db3", ".sqlite", ".sqlite3"}:
            return "sqlite3"
        return ""

    metadata = bag_path / "metadata.yaml"
    if metadata.exists():
        try:
            info = Info()
            bag_meta = info.read_metadata(str(bag_path), "")
            storage = getattr(bag_meta, "storage_identifier", "")
            if isinstance(storage, str) and storage:
                return storage
        except Exception:
            try:
                doc = _load_yaml(metadata)
                info = doc.get("rosbag2_bagfile_information", {})
                if isinstance(info, dict):
                    storage = info.get("storage_identifier", "")
                    if isinstance(storage, str) and storage:
                        return storage
            except Exception:
                pass
    return "sqlite3"


def _match_topic(pattern: str, topic_name: str) -> bool:
    if pattern == topic_name:
        return True
    return fnmatch.fnmatchcase(topic_name, pattern)


@dataclass
class TopicSpec:
    topic: str
    alias: str
    kind: str = "generic"
    role: str = "generic"
    required: bool = False
    sample_every: int = 1
    max_messages: Optional[int] = None
    compact: bool = False

    @staticmethod
    def from_dict(data: Dict[str, Any], index: int) -> "TopicSpec":
        topic = str(data.get("topic", "")).strip()
        if not topic:
            raise ValueError(f"topics[{index}].topic is required")
        alias = str(data.get("alias", "")).strip() or topic.strip("/").replace("/", "__") or f"topic_{index}"
        kind = str(data.get("kind", "generic")).strip() or "generic"
        role = str(data.get("role", "generic")).strip() or "generic"
        required = bool(data.get("required", False))
        sample_every = max(1, int(data.get("sample_every", 1) or 1))
        max_messages_raw = data.get("max_messages")
        max_messages = None if max_messages_raw in (None, "", 0) else max(1, int(max_messages_raw))
        compact = bool(data.get("compact", False))
        return TopicSpec(
            topic=topic,
            alias=alias,
            kind=kind,
            role=role,
            required=required,
            sample_every=sample_every,
            max_messages=max_messages,
            compact=compact,
        )


@dataclass
class TopicCapture:
    spec: TopicSpec
    resolved_topics: List[str] = field(default_factory=list)
    messages: List[Dict[str, Any]] = field(default_factory=list)
    message_count: int = 0

    def add(self, topic_name: str, payload: Dict[str, Any], stamp_ns: int) -> None:
        if topic_name not in self.resolved_topics:
            self.resolved_topics.append(topic_name)
        self.message_count += 1
        if (self.message_count - 1) % self.spec.sample_every != 0:
            return
        if self.spec.max_messages is not None and len(self.messages) >= self.spec.max_messages:
            return
        self.messages.append({"stamp_ns": stamp_ns, "topic": topic_name, "data": payload})


def _serialize_pointcloud2(msg: Any) -> List[List[Dict[str, Any]]]:
    field_names = [f.name for f in msg.fields]
    has_rgb = any(name in {"rgb", "rgba"} for name in field_names)
    has_intensity = "intensity" in field_names
    points: List[Dict[str, Any]] = []
    for item in pc2.read_points(msg, field_names=[name for name in ("x", "y", "z", "rgb", "rgba", "intensity") if name in field_names], skip_nans=True):
        point: Dict[str, Any] = {"x": float(item[0]), "y": float(item[1]), "z": float(item[2])}
        offset = 3
        if has_rgb and len(item) > offset:
            point["rgb"] = float(item[offset])
            offset += 1
        if has_intensity and len(item) > offset:
            point["intensity"] = float(item[offset])
        points.append(point)
    return [points]


def _serialize_topological_node_core(node: Any) -> Dict[str, Any]:
    return {
        "id": int(node.id),
        "x": float(node.pos.x),
        "y": float(node.pos.y),
        "z": float(node.pos.z),
        "nx": float(node.normal.x),
        "ny": float(node.normal.y),
        "nz": float(node.normal.z),
        "rho": float(node.rho),
        "label": int(node.label),
        "semantic_label": int(getattr(node, "semantic_label", 0)),
        "semantic_reliability": float(getattr(node, "semantic_reliability", 0.0)),
        "age": int(getattr(node, "age", getattr(node, "frame", 0))),
        "frame": int(getattr(node, "frame", 0)),
        "inpcl_ids": [int(v) for v in getattr(node, "inpcl_ids", [])],
        "winner_point_count": int(getattr(node, "winner_point_count", 0)),
        "winner_point_covariance": [float(v) for v in getattr(node, "winner_point_covariance", [])],
    }


def _serialize_topological_node_features(node: Any) -> Dict[str, Any]:
    ee_pose = getattr(node, "ee_pose", None)
    joint_positions = []
    for p in getattr(node, "joint_positions", []) or []:
        joint_positions.append({
            "x": float(getattr(p, "x", 0.0)),
            "y": float(getattr(p, "y", 0.0)),
            "z": float(getattr(p, "z", 0.0)),
        })
    node_id = int(getattr(node, "node_id", getattr(node, "id", 0)))
    return {
        "node_id": node_id,
        "is_goal": bool(getattr(node, "is_goal", False)),
        "manip_valid": bool(getattr(node, "manip_valid", False)),
        "manip_value": float(getattr(node, "manip_value", 0.0)),
        "manip_condition_number": float(getattr(node, "manip_condition_number", 0.0)),
        "manip_center": {
            "x": float(getattr(getattr(node, "manip_center", None), "x", 0.0)),
            "y": float(getattr(getattr(node, "manip_center", None), "y", 0.0)),
            "z": float(getattr(getattr(node, "manip_center", None), "z", 0.0)),
        },
        "manip_scale": {
            "x": float(getattr(getattr(node, "manip_scale", None), "x", 0.0)),
            "y": float(getattr(getattr(node, "manip_scale", None), "y", 0.0)),
            "z": float(getattr(getattr(node, "manip_scale", None), "z", 0.0)),
        },
        "manip_orientation": {
            "x": float(getattr(getattr(node, "manip_orientation", None), "x", 0.0)),
            "y": float(getattr(getattr(node, "manip_orientation", None), "y", 0.0)),
            "z": float(getattr(getattr(node, "manip_orientation", None), "z", 0.0)),
            "w": float(getattr(getattr(node, "manip_orientation", None), "w", 1.0)),
        },
        "weight_angle": [float(v) for v in getattr(node, "weight_angle", [])],
        "ee_pose": {
            "position": {
                "x": float(getattr(getattr(ee_pose, "position", None), "x", 0.0)),
                "y": float(getattr(getattr(ee_pose, "position", None), "y", 0.0)),
                "z": float(getattr(getattr(ee_pose, "position", None), "z", 0.0)),
            },
            "orientation": {
                "x": float(getattr(getattr(ee_pose, "orientation", None), "x", 0.0)),
                "y": float(getattr(getattr(ee_pose, "orientation", None), "y", 0.0)),
                "z": float(getattr(getattr(ee_pose, "orientation", None), "z", 0.0)),
                "w": float(getattr(getattr(ee_pose, "orientation", None), "w", 1.0)),
            },
        },
        "joint_positions": joint_positions,
    }


def _serialize_topological_map(msg: Any, compact: bool = False) -> Dict[str, Any]:
    nodes = []
    for node in msg.nodes:
        nodes.append(_serialize_topological_node_core(node))

    edges: List[Dict[str, int]] = []
    edge_vals = list(msg.edges)
    for i in range(0, len(edge_vals) - 1, 2):
        a = int(edge_vals[i])
        b = int(edge_vals[i + 1])
        if a == b:
            continue
        edges.append({"a": min(a, b), "b": max(a, b)})

    clusters = []
    cluster_features = [] if not compact else None
    for cluster in msg.clusters:
        clusters.append(
            {
                "id": int(cluster.id),
                "label": int(cluster.label),
                "label_inferred": int(getattr(cluster, "label_inferred", 0)),
                "x": float(cluster.pos.x),
                "y": float(cluster.pos.y),
                "z": float(cluster.pos.z),
                "sx": float(cluster.scale.x),
                "sy": float(cluster.scale.y),
                "sz": float(cluster.scale.z),
                "qx": float(cluster.quat.x),
                "qy": float(cluster.quat.y),
                "qz": float(cluster.quat.z),
                "qw": float(cluster.quat.w),
                "age": int(getattr(cluster, "age", getattr(cluster, "frame", 0))),
                "match": float(cluster.match),
                "vx": float(cluster.velocity.x),
                "vy": float(cluster.velocity.y),
                "vz": float(cluster.velocity.z),
                "reliability": float(getattr(cluster, "reliability", getattr(cluster, "label_reliability", 0.0))),
                "nodes": [int(v) for v in getattr(cluster, "nodes", [])],
            }
        )
        if cluster_features is not None:
            cluster_features.append(
                {
                    "cluster_id": int(cluster.id),
                    "has_velocity_observation": bool(getattr(cluster, "has_velocity_observation", False)),
                    "vel_cov_xx": float(getattr(cluster, "vel_cov_xx", 0.0)),
                    "vel_cov_xy": float(getattr(cluster, "vel_cov_xy", 0.0)),
                    "vel_cov_yy": float(getattr(cluster, "vel_cov_yy", 0.0)),
                    "frame": int(getattr(cluster, "frame", getattr(cluster, "age", 0))),
                }
            )

    node_features = None
    if not compact:
        if hasattr(msg, "node_features") and getattr(msg, "node_features", None):
            node_features = [_serialize_topological_node_features(node) for node in msg.node_features]
        else:
            node_features = [_serialize_topological_node_features(node) for node in msg.nodes]

    return {
        "topic_type": "ais_gng_msgs/msg/TopologicalMap",
        "frame_id": getattr(msg.header, "frame_id", ""),
        "frame_number": int(msg.frame_number),
        "nodes": nodes,
        **({} if node_features is None else {"node_features": node_features}),
        "edges": edges,
        "clusters": clusters,
        **({} if cluster_features is None else {"cluster_features": cluster_features}),
    }


def _serialize_tf_message(msg: Any) -> List[Dict[str, Any]]:
    transforms = []
    for ts in msg.transforms:
        transforms.append(
            {
                "frameId": ts.header.frame_id,
                "childFrameId": ts.child_frame_id,
                "x": float(ts.transform.translation.x),
                "y": float(ts.transform.translation.y),
                "z": float(ts.transform.translation.z),
                "qx": float(ts.transform.rotation.x),
                "qy": float(ts.transform.rotation.y),
                "qz": float(ts.transform.rotation.z),
                "qw": float(ts.transform.rotation.w),
            }
        )
    return transforms


def _serialize_generic(msg: Any) -> Dict[str, Any]:
    data = message_to_ordereddict(msg)
    if isinstance(data, dict):
        return dict(data)
    return {"value": data}


def _serialize_message(msg: Any, msg_type: str, kind: str) -> Any:
    if kind == "topological_map" or msg_type == "ais_gng_msgs/msg/TopologicalMap":
        return _serialize_topological_map(msg)
    if kind == "pointcloud2" or msg_type == "sensor_msgs/msg/PointCloud2":
        return _serialize_pointcloud2(msg)
    if kind == "tf" or msg_type == "tf2_msgs/msg/TFMessage":
        return _serialize_tf_message(msg)
    return _serialize_generic(msg)


def _resolve_specs(available_topics: Dict[str, str], specs: Sequence[TopicSpec]) -> List[Tuple[TopicSpec, List[str]]]:
    resolved: List[Tuple[TopicSpec, List[str]]] = []
    for spec in specs:
        matches = [topic for topic in available_topics if _match_topic(spec.topic, topic)]
        resolved.append((spec, matches))
    return resolved


def _open_reader(bag_path: Path, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    return reader


def _export_bundle(bag_path: Path, spec_dicts: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    storage_id = _infer_storage_id(bag_path)
    reader = _open_reader(bag_path, storage_id)
    available_topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    specs = []
    for i, spec in enumerate(spec_dicts):
        spec = dict(spec)
        topic_spec = TopicSpec.from_dict(spec, i)
        specs.append(topic_spec)
    resolved = _resolve_specs(available_topics, specs)
    captures = {spec.alias: TopicCapture(spec=spec) for spec in specs}
    topic_to_specs: Dict[str, List[TopicCapture]] = {}
    for spec, matches in resolved:
        if not matches and spec.required:
            raise RuntimeError(f"Required topic not found: {spec.topic}")
        for topic in matches:
            topic_to_specs.setdefault(topic, []).append(captures[spec.alias])

    if topic_to_specs:
        topic_filter = StorageFilter()
        topic_filter.topics = sorted(topic_to_specs.keys())
        reader.set_filter(topic_filter)

    while reader.has_next():
        topic_name, serialized, stamp_ns = reader.read_next()
        topic_type = available_topics.get(topic_name)
        if not topic_type:
            continue
        if topic_name not in topic_to_specs:
            continue
        msg_cls = get_message(topic_type)
        msg = deserialize_message(serialized, msg_cls)
        for capture in topic_to_specs[topic_name]:
            if capture.spec.kind == "topological_map":
                payload = _serialize_topological_map(msg, compact=capture.spec.compact)
            else:
                payload = _serialize_message(msg, topic_type, capture.spec.kind)
            capture.add(topic_name, payload, int(stamp_ns))

    topics_output: Dict[str, Any] = {}
    graph_bundle: Optional[Dict[str, Any]] = None
    candidate_graph_bundle: Optional[Dict[str, Any]] = None
    pointcloud_frames: Optional[List[List[Dict[str, Any]]]] = None
    metrics_bundle: Dict[str, Any] = {}

    for alias, capture in captures.items():
        topics_output[alias] = {
            "topic": capture.spec.topic,
            "resolved_topics": capture.resolved_topics,
            "kind": capture.spec.kind,
            "role": capture.spec.role,
            "message_count": capture.message_count,
            "messages": capture.messages,
        }
        if capture.spec.role == "graph" and capture.messages:
            latest = capture.messages[-1]["data"]
            if isinstance(latest, dict) and "nodes" in latest and "edges" in latest:
                graph_bundle = latest
                graph_bundle = dict(graph_bundle)
                graph_bundle["topic"] = capture.spec.topic
                graph_bundle["alias"] = alias
        elif capture.spec.role == "candidate_graph" and capture.messages:
            latest = capture.messages[-1]["data"]
            if isinstance(latest, dict) and "nodes" in latest and "edges" in latest:
                candidate_graph_bundle = latest
                candidate_graph_bundle = dict(candidate_graph_bundle)
                candidate_graph_bundle["topic"] = capture.spec.topic
                candidate_graph_bundle["alias"] = alias
        elif capture.spec.role == "pointcloud" and capture.messages:
            frames = []
            for item in capture.messages:
                data = item["data"]
                if isinstance(data, list):
                    frames.extend(data)
            pointcloud_frames = frames
        elif capture.spec.role == "metrics":
            metrics_bundle[alias] = capture.messages

    out: Dict[str, Any] = {
        "version": "gng_html_bundle_v1",
        "generated_at": _now_iso(),
        "source": {
            "bag_path": str(bag_path),
            "storage_id": storage_id,
        },
        "topics": topics_output,
    }
    if graph_bundle:
        out["graph"] = graph_bundle
        out["nodes"] = graph_bundle.get("nodes", [])
        if graph_bundle.get("node_features") is not None:
            out["node_features"] = graph_bundle.get("node_features", [])
        out["edges"] = graph_bundle.get("edges", [])
        if graph_bundle.get("clusters") is not None:
            out["clusters"] = graph_bundle.get("clusters", [])
        if graph_bundle.get("cluster_features") is not None:
            out["cluster_features"] = graph_bundle.get("cluster_features", [])
    if candidate_graph_bundle:
        out["candidate_graph"] = candidate_graph_bundle
        out["candidate_nodes"] = candidate_graph_bundle.get("nodes", [])
        if candidate_graph_bundle.get("node_features") is not None:
            out["candidate_node_features"] = candidate_graph_bundle.get("node_features", [])
        out["candidate_edges"] = candidate_graph_bundle.get("edges", [])
    if pointcloud_frames is not None:
        out["frames"] = pointcloud_frames
    if metrics_bundle:
        out["metrics"] = metrics_bundle
    return out


def _load_topic_specs(config: Dict[str, Any]) -> List[Dict[str, Any]]:
    topics = config.get("topics", [])
    if not isinstance(topics, list) or not topics:
        raise ValueError("config.topics must be a non-empty list")
    return topics


def cmd_list(args: argparse.Namespace) -> int:
    bag_path = Path(args.bag).expanduser()
    storage_id = _infer_storage_id(bag_path)
    reader = _open_reader(bag_path, storage_id)
    topics = reader.get_all_topics_and_types()
    for item in topics:
        print(f"{item.name}\t{item.type}")
    return 0


def cmd_export(args: argparse.Namespace) -> int:
    bag_path = Path(args.bag).expanduser()
    config_path = Path(args.config).expanduser()
    output_path = Path(args.output).expanduser()
    config = _load_yaml(config_path)
    topics = _load_topic_specs(config)
    bundle = _export_bundle(bag_path, topics)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(bundle, f, ensure_ascii=False, indent=2 if args.pretty else None)
        f.write("\n")
    print(f"Wrote {output_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="gng-bundle-export")
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="List topics in a rosbag")
    p_list.add_argument("--bag", required=True, help="Bag directory or bag file")
    p_list.set_defaults(func=cmd_list)

    p_export = sub.add_parser("export", help="Export selected topics to JSON")
    p_export.add_argument("--bag", required=True, help="Bag directory or bag file")
    p_export.add_argument("--config", required=True, help="YAML config that defines selected topics")
    p_export.add_argument("--output", required=True, help="Output JSON path")
    p_export.add_argument("--pretty", action="store_true", help="Pretty-print JSON")
    p_export.set_defaults(func=cmd_export)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except Exception as exc:
        parser.exit(1, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
