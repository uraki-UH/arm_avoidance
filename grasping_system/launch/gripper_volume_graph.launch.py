import re
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_GRIPPERS = """[
  {
    name: gripper,
    tf_frame: tool0,
    output_topic: gripper_volume_topological_map,
    shape: box,
    dimensions: [0.08, 0.04, 0.10],
    center: [0.0, 0.0, 0.05],
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    resolution: 0.01,
    label: 0,
    semantic_label: 0
  }
]"""


def _vector(spec, key, size, default):
    value = spec.get(key, default)
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"gripper '{spec.get('name', '?')}' {key} must contain {size} values")
    return [float(item) for item in value]


def _load_grippers(context):
    config_path = LaunchConfiguration("grippers_file").perform(context).strip()
    if config_path:
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"gripper definition file does not exist: {path}")
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    else:
        loaded = yaml.safe_load(LaunchConfiguration("grippers").perform(context))

    if isinstance(loaded, dict):
        loaded = loaded.get("grippers")
    if not isinstance(loaded, list) or not loaded:
        raise ValueError("grippers must be a non-empty YAML list")
    return loaded


def _with_tf_prefix(frame_id, prefix):
    normalized_prefix = prefix.strip().strip("/")
    if not normalized_prefix:
        return frame_id

    normalized_frame = frame_id.lstrip("/")
    if normalized_frame == normalized_prefix or normalized_frame.startswith(
        f"{normalized_prefix}/"
    ):
        return normalized_frame
    return f"{normalized_prefix}/{normalized_frame}"


def _launch_grippers(context):
    actions = []
    node_names = set()
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    for index, spec in enumerate(_load_grippers(context)):
        if not isinstance(spec, dict):
            raise ValueError(f"grippers[{index}] must be a mapping")

        gripper_name = str(spec.get("name", "")).strip()
        tf_frame = str(spec.get("tf_frame", spec.get("frame_id", ""))).strip()
        if not gripper_name or not tf_frame:
            raise ValueError(f"grippers[{index}] requires name and tf_frame")
        tf_frame = _with_tf_prefix(tf_frame, tf_prefix)

        node_name = re.sub(r"[^A-Za-z0-9_]", "_", gripper_name).strip("_")
        node_name = f"{node_name or f'gripper_{index}'}_volume_graph_node"
        if node_name in node_names:
            raise ValueError(f"duplicate gripper node name: {node_name}")
        node_names.add(node_name)

        parameters = {
            "output_topic": str(
                spec.get("output_topic", f"{gripper_name}/gripper_volume_topological_map")
            ),
            "frame_id": tf_frame,
            "shape": str(spec.get("shape", "box")),
            "dimensions": _vector(spec, "dimensions", 3, [0.08, 0.04, 0.10]),
            "center": _vector(spec, "center", 3, [0.0, 0.0, 0.05]),
            "orientation_xyzw": _vector(
                spec, "orientation_xyzw", 4, [0.0, 0.0, 0.0, 1.0]
            ),
            "resolution": float(spec.get("resolution", 0.01)),
            "label": int(spec.get("label", 0)),
            "semantic_label": int(spec.get("semantic_label", 0)),
        }
        actions.append(
            Node(
                package="grasping_system",
                executable="gripper_volume_graph_node",
                name=node_name,
                output="screen",
                parameters=[parameters],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "grippers",
            default_value=DEFAULT_GRIPPERS,
            description="YAML list of gripper definitions and their TF frames",
        ),
        DeclareLaunchArgument(
            "grippers_file",
            default_value="",
            description="Optional YAML file; when set it takes precedence over grippers",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Optional prefix applied to every configured gripper TF frame",
        ),
        OpaqueFunction(function=_launch_grippers),
    ])
