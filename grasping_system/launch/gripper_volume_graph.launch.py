import math
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
    output_topic: grip_V_topological_map,
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


def _lerp(start, end, fraction):
    return [
        (1.0 - fraction) * float(start[index]) + fraction * float(end[index])
        for index in range(len(start))
    ]


def _normalize_quaternion(quaternion, description):
    squared_norm = sum(component * component for component in quaternion)
    if squared_norm <= 1.0e-16:
        raise ValueError(f"{description} must be a non-zero quaternion")
    inverse_norm = 1.0 / math.sqrt(squared_norm)
    return [component * inverse_norm for component in quaternion]


def _slerp_quaternion(start, end, fraction):
    """Interpolate XYZW quaternions without adding a runtime dependency."""
    dot = sum(start[index] * end[index] for index in range(4))
    if dot < 0.0:
        end = [-component for component in end]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))

    # Nearly identical orientations are more accurately handled linearly.
    if dot > 0.9995:
        return _normalize_quaternion(_lerp(start, end, fraction), "interpolated orientation")

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    first_weight = math.sin((1.0 - fraction) * theta) / sin_theta
    second_weight = math.sin(fraction * theta) / sin_theta
    return [
        first_weight * start[index] + second_weight * end[index]
        for index in range(4)
    ]


def _expand_swept_meshes(spec):
    swept_meshes = spec.get("swept_meshes", [])
    if not isinstance(swept_meshes, list):
        raise ValueError(f"gripper '{spec.get('name', '?')}' swept_meshes must be a list")

    expanded = []
    for index, sweep in enumerate(swept_meshes):
        if not isinstance(sweep, dict):
            raise ValueError(
                f"gripper '{spec.get('name', '?')}' swept_meshes[{index}] must be a mapping"
            )
        path = str(sweep.get("path", "")).strip()
        if not path:
            raise ValueError(
                f"gripper '{spec.get('name', '?')}' swept_meshes[{index}] requires path"
            )
        samples = sweep.get("samples", 2)
        if isinstance(samples, bool) or not isinstance(samples, int) or samples < 2:
            raise ValueError(
                f"gripper '{spec.get('name', '?')}' swept_meshes[{index}] samples must be an integer >= 2"
            )

        default_position = _vector(sweep, "position", 3, [0.0, 0.0, 0.0])
        start_position = _vector(sweep, "start_position", 3, default_position)
        end_position = _vector(sweep, "end_position", 3, default_position)
        default_orientation = _vector(
            sweep, "orientation_xyzw", 4, [0.0, 0.0, 0.0, 1.0]
        )
        start_orientation = _normalize_quaternion(
            _vector(sweep, "start_orientation_xyzw", 4, default_orientation),
            f"gripper '{spec.get('name', '?')}' swept_meshes[{index}] start_orientation_xyzw",
        )
        end_orientation = _normalize_quaternion(
            _vector(sweep, "end_orientation_xyzw", 4, default_orientation),
            f"gripper '{spec.get('name', '?')}' swept_meshes[{index}] end_orientation_xyzw",
        )
        scale = _vector(sweep, "scale", 3, [1.0, 1.0, 1.0])

        for sample_index in range(samples):
            fraction = float(sample_index) / float(samples - 1)
            expanded.append({
                "path": path,
                "scale": scale,
                "position": _lerp(start_position, end_position, fraction),
                "orientation_xyzw": _slerp_quaternion(
                    start_orientation, end_orientation, fraction
                ),
            })
    return expanded


def _mesh_exclusions(spec):
    mesh_key = "meshes" if "meshes" in spec else "exclude_closed_meshes"
    configured_meshes = spec.get(mesh_key, [])
    if not isinstance(configured_meshes, list):
        raise ValueError(
            f"gripper '{spec.get('name', '?')}' {mesh_key} must be a list"
        )
    exclusions = list(configured_meshes)
    exclusions.extend(_expand_swept_meshes(spec))

    paths = []
    scales = []
    positions = []
    orientations = []
    positive_indices = []
    negative_indices = []
    for index, exclusion in enumerate(exclusions):
        if not isinstance(exclusion, dict):
            raise ValueError(
                f"gripper '{spec.get('name', '?')}' {mesh_key}[{index}] "
                "must be a mapping"
            )
        path = str(exclusion.get("path", "")).strip()
        if not path:
            raise ValueError(
                f"gripper '{spec.get('name', '?')}' {mesh_key}[{index}] "
                "requires path"
            )
        paths.append(path)
        internal_side = str(exclusion.get("internal_side", "")).strip().lower()
        if internal_side == "positive":
            positive_indices.append(index)
        elif internal_side == "negative":
            negative_indices.append(index)
        elif internal_side not in ("", "body"):
            raise ValueError(
                f"gripper '{spec.get('name', '?')}' {mesh_key}[{index}] "
                "internal_side must be positive, negative, body, or empty"
            )
        scales.extend(_vector(exclusion, "scale", 3, [1.0, 1.0, 1.0]))
        positions.extend(_vector(exclusion, "position", 3, [0.0, 0.0, 0.0]))
        orientations.extend(
            _vector(exclusion, "orientation_xyzw", 4, [0.0, 0.0, 0.0, 1.0])
        )
    return paths, scales, positions, orientations, positive_indices, negative_indices


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
        (
            exclusion_paths,
            exclusion_scales,
            exclusion_positions,
            exclusion_orientations,
            positive_finger_indices,
            negative_finger_indices,
        ) = _mesh_exclusions(spec)

        node_name = re.sub(r"[^A-Za-z0-9_]", "_", gripper_name).strip("_")
        node_name = f"{node_name or f'gripper_{index}'}_volume_graph_node"
        if node_name in node_names:
            raise ValueError(f"duplicate gripper node name: {node_name}")
        node_names.add(node_name)

        parameters = {
            "output_topic": str(
                spec.get("output_topic", f"{gripper_name}/grip_V_topological_map")
            ),
            "frame_id": tf_frame,
            "shape": str(spec.get("shape", "box")),
            "dimensions": _vector(spec, "dimensions", 3, [0.08, 0.04, 0.10]),
            "center": _vector(spec, "center", 3, [0.0, 0.0, 0.05]),
            "orientation_xyzw": _vector(
                spec, "orientation_xyzw", 4, [0.0, 0.0, 0.0, 1.0]
            ),
            "resolution": float(spec.get("resolution", 0.01)),
            "exclusion_clearance": float(spec.get("exclusion_clearance", 0.0)),
            "retain_occupied_meshes": bool(spec.get("retain_occupied_meshes", False)),
            "retain_internal_only": bool(spec.get("retain_internal_only", False)),
            "closing_axis": _vector(spec, "closing_axis", 3, [0.0, 1.0, 0.0]),
            "include_cluster": bool(spec.get("include_cluster", True)),
            "label": int(spec.get("label", 0)),
            "semantic_label": int(spec.get("semantic_label", 0)),
        }
        if exclusion_paths:
            parameters.update({
                "exclusion_mesh_paths": exclusion_paths,
                "exclusion_mesh_scales": exclusion_scales,
                "exclusion_mesh_positions": exclusion_positions,
                "exclusion_mesh_orientations_xyzw": exclusion_orientations,
            })
        if positive_finger_indices:
            parameters["positive_finger_mesh_indices"] = positive_finger_indices
        if negative_finger_indices:
            parameters["negative_finger_mesh_indices"] = negative_finger_indices
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
