from __future__ import annotations

import argparse
import gzip
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


def _now_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be a mapping: {path}")
    return data


def _package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_results_dir() -> Path:
    return _package_root() / "results"


def _ensure_unique_output_path(path: Path) -> Path:
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    if stem.endswith(".json") and suffix == ".gz":
        stem = stem[:-5]
    parent = path.parent
    for i in range(1, 1000):
        candidate = parent / f"{stem}_{_now_stamp()}_{i:02d}{suffix}"
        if not candidate.exists():
            return candidate
    raise FileExistsError(f"could not find a free filename for {path}")


def _default_output_path(bag_path: Path, use_gzip: bool) -> Path:
    bag_name = bag_path.name.rstrip("/") or "bag"
    suffix = ".json.gz" if use_gzip else ".json"
    return _default_results_dir() / f"{bag_name}_bundle_{_now_stamp()}{suffix}"


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
    rows: List[Any] = field(default_factory=list)
    message_count: int = 0

    def add(self, topic_name: str, row: Any, stamp_ns: int) -> None:
        if topic_name not in self.resolved_topics:
            self.resolved_topics.append(topic_name)
        self.message_count += 1
        if (self.message_count - 1) % self.spec.sample_every != 0:
            return
        if self.spec.max_messages is not None and len(self.rows) >= self.spec.max_messages:
            return
        self.rows.append(row)


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


def _serialize_pointcloud2_row(msg: Any, stamp_ns: int) -> List[Any]:
    return [stamp_ns, _serialize_pointcloud2(msg)[0]]


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


def _serialize_topological_node_core_compact(node: Any) -> List[Any]:
    return [
        int(node.id),
        float(node.pos.x),
        float(node.pos.y),
        float(node.pos.z),
        float(node.normal.x),
        float(node.normal.y),
        float(node.normal.z),
        float(node.rho),
        int(node.label),
        int(getattr(node, "semantic_label", 0)),
        float(getattr(node, "semantic_reliability", 0.0)),
        int(getattr(node, "age", getattr(node, "frame", 0))),
        int(getattr(node, "frame", 0)),
    ]


def _serialize_topological_node_features(node: Any, include_joint_positions: bool = True) -> Dict[str, Any]:
    ee_pose = getattr(node, "ee_pose", None)
    node_id = int(getattr(node, "node_id", getattr(node, "id", 0)))
    out = {
        "node_id": node_id,
        "is_goal": bool(getattr(node, "is_goal", False)),
        "manip_valid": bool(getattr(node, "manip_valid", False)),
        "manip_value": float(getattr(node, "manip_value", 0.0)),
        "manip_condition_number": float(getattr(node, "manip_condition_number", 0.0)),
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
        "rotational_manip_valid": bool(getattr(node, "rotational_manip_valid", False)),
        "rotational_manip_value": float(getattr(node, "rotational_manip_value", 0.0)),
        "rotational_manip_condition_number": float(getattr(node, "rotational_manip_condition_number", 0.0)),
        "rotational_manip_scale": {
            "x": float(getattr(getattr(node, "rotational_manip_scale", None), "x", 0.0)),
            "y": float(getattr(getattr(node, "rotational_manip_scale", None), "y", 0.0)),
            "z": float(getattr(getattr(node, "rotational_manip_scale", None), "z", 0.0)),
        },
        "rotational_manip_orientation": {
            "x": float(getattr(getattr(node, "rotational_manip_orientation", None), "x", 0.0)),
            "y": float(getattr(getattr(node, "rotational_manip_orientation", None), "y", 0.0)),
            "z": float(getattr(getattr(node, "rotational_manip_orientation", None), "z", 0.0)),
            "w": float(getattr(getattr(node, "rotational_manip_orientation", None), "w", 1.0)),
        },
    }
    return out


def _serialize_topological_node_features_compact(node: Any, include_joint_positions: bool = True) -> List[Any]:
    ee_pose = getattr(node, "ee_pose", None)
    return [
        int(getattr(node, "node_id", getattr(node, "id", 0))),
        1 if getattr(node, "is_goal", False) else 0,
        1 if getattr(node, "manip_valid", False) else 0,
        float(getattr(node, "manip_value", 0.0)),
        float(getattr(node, "manip_condition_number", 0.0)),
        float(getattr(getattr(node, "manip_scale", None), "x", 0.0)),
        float(getattr(getattr(node, "manip_scale", None), "y", 0.0)),
        float(getattr(getattr(node, "manip_scale", None), "z", 0.0)),
        float(getattr(getattr(node, "manip_orientation", None), "x", 0.0)),
        float(getattr(getattr(node, "manip_orientation", None), "y", 0.0)),
        float(getattr(getattr(node, "manip_orientation", None), "z", 0.0)),
        float(getattr(getattr(node, "manip_orientation", None), "w", 1.0)),
        [float(v) for v in getattr(node, "weight_angle", [])],
        [
            float(getattr(getattr(ee_pose, "position", None), "x", 0.0)),
            float(getattr(getattr(ee_pose, "position", None), "y", 0.0)),
            float(getattr(getattr(ee_pose, "position", None), "z", 0.0)),
            float(getattr(getattr(ee_pose, "orientation", None), "x", 0.0)),
            float(getattr(getattr(ee_pose, "orientation", None), "y", 0.0)),
            float(getattr(getattr(ee_pose, "orientation", None), "z", 0.0)),
            float(getattr(getattr(ee_pose, "orientation", None), "w", 1.0)),
        ],
        None,
        1 if getattr(node, "rotational_manip_valid", False) else 0,
        float(getattr(node, "rotational_manip_value", 0.0)),
        float(getattr(node, "rotational_manip_condition_number", 0.0)),
        float(getattr(getattr(node, "rotational_manip_scale", None), "x", 0.0)),
        float(getattr(getattr(node, "rotational_manip_scale", None), "y", 0.0)),
        float(getattr(getattr(node, "rotational_manip_scale", None), "z", 0.0)),
        float(getattr(getattr(node, "rotational_manip_orientation", None), "x", 0.0)),
        float(getattr(getattr(node, "rotational_manip_orientation", None), "y", 0.0)),
        float(getattr(getattr(node, "rotational_manip_orientation", None), "z", 0.0)),
        float(getattr(getattr(node, "rotational_manip_orientation", None), "w", 1.0)),
    ]


def _serialize_topological_map(msg: Any, compact: bool = False) -> Dict[str, Any]:
    nodes = []
    for node in msg.nodes:
        nodes.append(_serialize_topological_node_core_compact(node) if compact else _serialize_topological_node_core(node))

    edges: List[Dict[str, int]] = []
    edge_vals = list(msg.edges)
    for i in range(0, len(edge_vals) - 1, 2):
        a = int(edge_vals[i])
        b = int(edge_vals[i + 1])
        if a == b:
            continue
        edges.append({"a": min(a, b), "b": max(a, b)})

    clusters = []
    cluster_features = None
    if not compact:
        cluster_features = []
        for cluster in msg.clusters:
            clusters.append(
                [
                    int(cluster.id),
                    int(cluster.label),
                    int(getattr(cluster, "label_inferred", 0)),
                    float(cluster.pos.x),
                    float(cluster.pos.y),
                    float(cluster.pos.z),
                    float(cluster.scale.x),
                    float(cluster.scale.y),
                    float(cluster.scale.z),
                    float(cluster.quat.x),
                    float(cluster.quat.y),
                    float(cluster.quat.z),
                    float(cluster.quat.w),
                    int(getattr(cluster, "age", getattr(cluster, "frame", 0))),
                    float(cluster.match),
                    float(cluster.velocity.x),
                    float(cluster.velocity.y),
                    float(cluster.velocity.z),
                    float(getattr(cluster, "reliability", getattr(cluster, "label_reliability", 0.0))),
                    [int(v) for v in getattr(cluster, "nodes", [])],
                ]
            )
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
            node_features = [_serialize_topological_node_features(node, include_joint_positions=not compact) for node in msg.node_features]
        else:
            node_features = [_serialize_topological_node_features(node, include_joint_positions=not compact) for node in msg.nodes]
    elif hasattr(msg, "node_features") and getattr(msg, "node_features", None):
        node_features = [_serialize_topological_node_features_compact(node, include_joint_positions=False) for node in msg.node_features]
    else:
        node_features = [_serialize_topological_node_features_compact(node, include_joint_positions=False) for node in msg.nodes]

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


def _serialize_topological_map_row(msg: Any, stamp_ns: int, compact: bool = False) -> List[Any]:
    payload = _serialize_topological_map(msg, compact=compact)
    if compact:
        return [
            stamp_ns,
            payload.get("frame_id", ""),
            payload.get("frame_number", 0),
            payload.get("nodes", []),
            payload.get("edges", []),
            payload.get("node_features"),
        ]
    return [
        stamp_ns,
        payload.get("frame_id", ""),
        payload.get("frame_number", 0),
        payload.get("nodes", []),
        payload.get("edges", []),
        payload.get("clusters", []),
        payload.get("node_features"),
        payload.get("cluster_features"),
    ]


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


def _serialize_generic_row(msg: Any, stamp_ns: int) -> List[Any]:
    payload = _serialize_generic(msg)
    return [stamp_ns, payload]


def _schema_for_kind(kind: str, compact: bool = False) -> List[str]:
    if kind == "topological_map":
        if compact:
            return [
                "stamp_ns",
                "frame_id",
                "frame_number",
                "nodes",
                "edges",
                "node_features",
            ]
        return [
            "stamp_ns",
            "frame_id",
            "frame_number",
            "nodes",
            "edges",
            "clusters",
            "node_features",
            "cluster_features",
        ]
    if kind == "pointcloud2":
        return ["stamp_ns", "points"]
    return ["stamp_ns", "data"]


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
                row = _serialize_topological_map_row(msg, int(stamp_ns), compact=capture.spec.compact)
            elif capture.spec.kind == "pointcloud2":
                row = _serialize_pointcloud2_row(msg, int(stamp_ns))
            else:
                row = _serialize_generic_row(msg, int(stamp_ns))
            capture.add(topic_name, row, int(stamp_ns))

    topics_output: Dict[str, Any] = {}
    for alias, capture in captures.items():
        entry: Dict[str, Any] = {
            "topic": capture.spec.topic,
            "kind": capture.spec.kind,
            "role": capture.spec.role,
            "schema": _schema_for_kind(capture.spec.kind, compact=capture.spec.compact),
            "row_count": len(capture.rows),
        }
        if capture.rows:
            entry["rows"] = capture.rows
        topics_output[alias] = entry
    out: Dict[str, Any] = {
        "version": "gng_html_bundle_v2",
        "generated_at": _now_iso(),
        "source": {
            "bag_path": str(bag_path),
            "storage_id": storage_id,
        },
        "topics": topics_output,
    }
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
    config = _load_yaml(config_path)
    topics = _load_topic_specs(config)
    bundle = _export_bundle(bag_path, topics)
    output_arg = getattr(args, "output", None)
    use_gzip = bool(args.gzip)
    if output_arg:
        output_path = Path(output_arg).expanduser()
        if output_path.suffix.lower() == ".gz":
            use_gzip = True
    else:
        output_path = _default_output_path(bag_path, use_gzip=True)
        use_gzip = True
    if output_path.suffix.lower() == ".gz":
        use_gzip = True
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path = _ensure_unique_output_path(output_path)
    if use_gzip:
        payload = json.dumps(
            bundle,
            ensure_ascii=False,
            indent=2 if args.pretty else None,
            separators=None if args.pretty else (",", ":"),
        ).encode("utf-8")
        with gzip.open(output_path, "wb", compresslevel=9) as f:
            f.write(payload)
    else:
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(bundle, f, ensure_ascii=False, indent=2 if args.pretty else None, separators=None if args.pretty else (",", ":"))
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
    p_export.add_argument("--output", help="Output JSON path. If omitted, write into gng_bundle_exporter/results/")
    p_export.add_argument("--pretty", action="store_true", help="Pretty-print JSON")
    p_export.add_argument("--gzip", action="store_true", help="Write gzip-compressed JSON")
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
