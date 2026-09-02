import gzip
import json
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


TOPIC_TOKEN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def read_json(path):
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def inspect_dataset(path):
    try:
        root = read_json(path)
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None

    kind = root.get("kind", "")
    if kind == "object_template":
        template = root
    elif kind == "object_surface_dataset" and isinstance(root.get("gng_template"), dict):
        template = root["gng_template"]
    else:
        return None

    graph = template.get("gng")
    if not isinstance(graph, dict) or not isinstance(graph.get("nodes"), list) or not graph["nodes"]:
        return None
    template_id = root.get("dataset_id") or root.get("template_id") or template.get("template_id")
    if not isinstance(template_id, str) or not TOPIC_TOKEN.fullmatch(template_id):
        return None
    return template_id


def discover_templates(dataset_dir):
    discovered = {}
    for root_dir, _, filenames in os.walk(dataset_dir):
        for filename in sorted(filenames):
            if not (filename.endswith(".json") or filename.endswith(".json.gz")):
                continue
            path = os.path.join(root_dir, filename)
            template_id = inspect_dataset(path)
            if template_id and template_id not in discovered:
                discovered[template_id] = path
    return discovered


def create_summon_nodes(context):
    dataset_dir = LaunchConfiguration("dataset_dir").perform(context).rstrip("/")
    if not os.path.isdir(dataset_dir):
        raise RuntimeError(f"dataset_dirがありません: {dataset_dir}")

    discovered = discover_templates(dataset_dir)
    requested_text = LaunchConfiguration("template_ids").perform(context).strip()
    requested = [item.strip() for item in requested_text.split(",") if item.strip()]
    if requested:
        missing = [template_id for template_id in requested if template_id not in discovered]
        if missing:
            raise RuntimeError(f"未登録のtemplate_idです: {', '.join(missing)}")
        selected = {template_id: discovered[template_id] for template_id in requested}
    else:
        selected = discovered
    if not selected:
        raise RuntimeError(f"召喚可能な物体テンプレートがありません: {dataset_dir}")

    state_topic = LaunchConfiguration("state_topic")
    template_ids = sorted(selected)
    nodes = [Node(
        package="gng_vlut_system",
        executable="object_hypothesis_summon_node",
        name="object_hypothesis_summon_node",
        output="screen",
        parameters=[{
            "template_ids": template_ids,
            "initial_template_id": LaunchConfiguration("initial_template_id"),
            "state_topic": state_topic,
            "selection_topic": LaunchConfiguration("selection_topic"),
            "map_topic": LaunchConfiguration("map_topic"),
            "switch_interval_sec": ParameterValue(
                LaunchConfiguration("switch_interval_sec"), value_type=float),
            "random_seed": ParameterValue(LaunchConfiguration("random_seed"), value_type=int),
        }],
    )]

    nodes.append(Node(
        package="gng_vlut_system",
        executable="object_template_map_publisher_node",
        name="object_hypothesis_map_publisher_node",
        output="screen",
        parameters=[{
            "dataset_paths": [selected[template_id] for template_id in template_ids],
            "template_ids": template_ids,
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_hz": ParameterValue(
                LaunchConfiguration("publish_hz"), value_type=float),
            "output_topic": LaunchConfiguration("map_topic"),
            "activation_state_topic": state_topic,
            "publish_empty_on_deactivate": True,
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dataset_dir", default_value="/datasets"),
        DeclareLaunchArgument("template_ids", default_value=""),
        DeclareLaunchArgument("initial_template_id", default_value=""),
        DeclareLaunchArgument("switch_interval_sec", default_value="5.0"),
        DeclareLaunchArgument("random_seed", default_value="0"),
        DeclareLaunchArgument("state_topic", default_value="/object_hypothesis/summon_state"),
        DeclareLaunchArgument("selection_topic", default_value="/object_hypothesis/select"),
        DeclareLaunchArgument("map_topic", default_value="/object_hypothesis/topological_map"),
        DeclareLaunchArgument("frame_id", default_value="object_template"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        OpaqueFunction(function=create_summon_nodes),
    ])
