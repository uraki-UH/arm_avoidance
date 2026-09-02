import glob
import gzip
import json
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_dataset_path(dataset_dir, dataset_file):
    if not dataset_file:
        raise RuntimeError("dataset_fileの指定が必要です。")
    if os.path.isabs(dataset_file):
        candidates = [dataset_file]
    elif dataset_file.endswith((".json", ".json.gz")):
        candidates = [os.path.join(dataset_dir, dataset_file)]
    else:
        candidates = [
            os.path.join(dataset_dir, f"{dataset_file}_gng_template.json.gz"),
            os.path.join(dataset_dir, f"{dataset_file}_gng_template.json"),
        ]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    if not os.path.isabs(dataset_file) and not dataset_file.endswith((".json", ".json.gz")):
        matched = sorted(
            glob.glob(os.path.join(dataset_dir, f"{dataset_file}*_gng_template.json.gz")) +
            glob.glob(os.path.join(dataset_dir, f"{dataset_file}*_gng_template.json"))
        )
        if len(matched) == 1:
            return matched[0]
        if len(matched) > 1:
            raise RuntimeError(f"dataset_fileに一致するデータセットが複数あります: {dataset_file}")
    raise RuntimeError(f"データセットを見つけられません: {dataset_file}")


def read_template_id(dataset_path):
    opener = gzip.open if dataset_path.endswith(".gz") else open
    with opener(dataset_path, "rt", encoding="utf-8") as stream:
        dataset = json.load(stream)
    kind = dataset.get("kind", "")
    nested_template = dataset.get("gng_template", {})
    is_template_dataset = (
        kind == "object_template" or
        (kind == "object_surface_dataset" and isinstance(nested_template, dict))
    )
    if not is_template_dataset:
        return ""
    template_id = dataset.get("dataset_id") or dataset.get("template_id") or nested_template.get("template_id")
    if not template_id:
        raise RuntimeError(f"template_idを取得できません: {dataset_path}")
    return template_id


def read_template_sources(source_file):
    with open(source_file, encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    sources = root.get("template_sources", {})
    if not isinstance(sources, dict):
        raise RuntimeError("template_sourcesはYAMLのマッピングで指定してください。")
    values = {}
    for key in (
            "dataset_files", "dataset_dirs", "exclude_dirs", "exclude_template_ids"):
        value = sources.get(key, [])
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            raise RuntimeError(f"template_sources.{key}は文字列配列で指定してください。")
        values[key] = value
    return values


def iter_dataset_files(dataset_dir, excluded_dirs):
    for root_dir, dir_names, file_names in os.walk(dataset_dir):
        dir_names.sort()
        dir_names[:] = [
            name for name in dir_names
            if os.path.realpath(os.path.join(root_dir, name)) not in excluded_dirs
        ]
        for file_name in sorted(file_names):
            if file_name.endswith((".json", ".json.gz")):
                yield os.path.join(root_dir, file_name)


def resolve_matching_templates(dataset_dir, dataset_file, source_file):
    if dataset_file:
        dataset_path = resolve_dataset_path(dataset_dir, dataset_file)
        template_id = read_template_id(dataset_path)
        if not template_id:
            raise RuntimeError(f"物体テンプレートではありません: {dataset_path}")
        return [(dataset_path, template_id)]

    sources = read_template_sources(source_file)
    if not sources["dataset_files"] and not sources["dataset_dirs"]:
        raise RuntimeError(
            "template_sources_fileへdataset_filesまたはdataset_dirsを設定してください。")

    excluded_ids = set(sources["exclude_template_ids"])
    excluded_dirs = {
        os.path.realpath(
            configured_dir if os.path.isabs(configured_dir)
            else os.path.join(dataset_dir, configured_dir))
        for configured_dir in sources["exclude_dirs"]
    }
    template_paths = {}

    def add_template(dataset_path, can_skip_non_template):
        canonical_path = os.path.realpath(dataset_path)
        template_id = read_template_id(canonical_path)
        if not template_id:
            if can_skip_non_template:
                return
            raise RuntimeError(f"物体テンプレートではありません: {dataset_path}")
        if template_id in excluded_ids:
            return
        previous_path = template_paths.get(template_id)
        if previous_path and previous_path != canonical_path:
            raise RuntimeError(
                f"template_idが重複しています: {template_id}: {previous_path}, {canonical_path}")
        template_paths[template_id] = canonical_path

    for configured_file in sources["dataset_files"]:
        add_template(resolve_dataset_path(dataset_dir, configured_file), False)
    for configured_dir in sources["dataset_dirs"]:
        source_dir = (
            configured_dir if os.path.isabs(configured_dir)
            else os.path.join(dataset_dir, configured_dir))
        if not os.path.isdir(source_dir):
            raise RuntimeError(f"テンプレートフォルダを見つけられません: {configured_dir}")
        for dataset_path in iter_dataset_files(source_dir, excluded_dirs):
            add_template(dataset_path, True)

    if not template_paths:
        raise RuntimeError("有効な物体テンプレートが選択されていません。")
    return [(path, template_id) for template_id, path in sorted(template_paths.items())]


def read_node_profile(profile_file, node_name):
    with open(profile_file, encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    node_config = root.get(node_name, {})
    params = node_config.get("ros__parameters", {})
    if not isinstance(params, dict):
        raise RuntimeError(f"{node_name}.ros__parametersはYAMLのマッピングで指定してください。")
    return params


def create_matching_nodes(context):
    dataset_dir = LaunchConfiguration("dataset_dir").perform(context).rstrip("/")
    dataset_file = LaunchConfiguration("dataset_file").perform(context)
    source_file = LaunchConfiguration("template_sources_file").perform(context)
    templates = resolve_matching_templates(dataset_dir, dataset_file, source_file)
    profile_file = LaunchConfiguration("profile_file").perform(context)
    matcher_profile = read_node_profile(profile_file, "object_template_matcher_node")
    validator_profile = read_node_profile(profile_file, "object_template_match_validator_node")
    environment_topic = LaunchConfiguration("environment_topological_map_topic")
    plane_clusters_topic = LaunchConfiguration("plane_clusters_topic")
    template_ids = [template_id for _, template_id in templates]
    dataset_paths = [dataset_path for dataset_path, _ in templates]
    candidate_topics = [
        f"/{template_id}/object_template_match_candidates"
        for template_id in template_ids
    ]
    nodes = [
        Node(
            package="gng_vlut_system",
            executable="object_template_matcher_node",
            name="object_template_matcher_node",
            output="screen",
            parameters=[matcher_profile, {
                "template_ids": template_ids,
                "template_dataset_paths": dataset_paths,
                "environment_topological_map_topic": environment_topic,
                "plane_clusters_topic": plane_clusters_topic,
                "candidate_topics": candidate_topics,
            }],
        ),
    ]
    has_multiple_templates = len(templates) > 1
    for dataset_path, template_id in templates:
        candidate_topic = f"/{template_id}/object_template_match_candidates"
        state_topic = f"/{template_id}/object_template_match_state"
        node_suffix = f"_{template_id}" if has_multiple_templates else ""
        nodes.extend([
            Node(
                package="gng_vlut_system",
                executable="object_template_match_validator_node",
                name=f"object_template_match_validator_node{node_suffix}",
                output="screen",
                parameters=[validator_profile, {
                "template_id": template_id,
                "candidate_topic": candidate_topic,
                "state_topic": state_topic,
            }],
            ),
            Node(
                package="gng_vlut_system",
                executable="object_template_map_publisher_node",
                name=f"object_template_map_publisher_node{node_suffix}",
                output="screen",
                parameters=[{
                    "dataset_path": dataset_path,
                    "template_id": template_id,
                    "frame_id": LaunchConfiguration("frame_id"),
                    "publish_hz": LaunchConfiguration("publish_hz"),
                    "activation_state_topic": state_topic,
                }],
            ),
        ])
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("dataset_dir", default_value="/datasets"),
        DeclareLaunchArgument("dataset_file", default_value=""),
        DeclareLaunchArgument(
            "template_sources_file",
            default_value=os.path.join(
                package_share, "config", "object_template_matching_sources.yaml"),
        ),
        DeclareLaunchArgument(
            "profile_file",
            default_value=os.path.join(package_share, "config", "object_template_matching.yaml"),
        ),
        DeclareLaunchArgument("environment_topological_map_topic", default_value="/topological_map"),
        DeclareLaunchArgument("plane_clusters_topic", default_value="/plane_clusters"),
        DeclareLaunchArgument("frame_id", default_value="object_template"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        OpaqueFunction(function=create_matching_nodes),
    ])
