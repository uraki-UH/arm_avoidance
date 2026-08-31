import glob
import gzip
import json
import os

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
    nested_template = dataset.get("gng_template", {})
    template_id = dataset.get("dataset_id") or dataset.get("template_id") or nested_template.get("template_id")
    if not template_id:
        raise RuntimeError(f"template_idを取得できません: {dataset_path}")
    return template_id


def create_matching_nodes(context):
    dataset_dir = LaunchConfiguration("dataset_dir").perform(context).rstrip("/")
    dataset_file = LaunchConfiguration("dataset_file").perform(context)
    dataset_path = resolve_dataset_path(dataset_dir, dataset_file)
    template_id = read_template_id(dataset_path)
    profile_file = LaunchConfiguration("profile_file").perform(context)
    environment_topic = LaunchConfiguration("environment_topological_map_topic")
    candidate_topic = f"/{template_id}/object_template_match_candidates"
    state_topic = f"/{template_id}/object_template_match_state"
    return [
        Node(
            package="gng_vlut_system",
            executable="object_template_matcher_node",
            name="object_template_matcher_node",
            output="screen",
            parameters=[profile_file, {
                "template_id": template_id,
                "template_dataset_path": dataset_path,
                "environment_topological_map_topic": environment_topic,
                "candidate_topic": candidate_topic,
            }],
        ),
        Node(
            package="gng_vlut_system",
            executable="object_template_match_validator_node",
            name="object_template_match_validator_node",
            output="screen",
            parameters=[profile_file, {
                "template_id": template_id,
                "candidate_topic": candidate_topic,
                "state_topic": state_topic,
            }],
        ),
        Node(
            package="gng_vlut_system",
            executable="object_template_map_publisher_node",
            name="object_template_map_publisher_node",
            output="screen",
            parameters=[{
                "dataset_path": dataset_path,
                "template_id": template_id,
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_hz": LaunchConfiguration("publish_hz"),
                "activation_state_topic": state_topic,
            }],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("dataset_dir", default_value="/datasets"),
        DeclareLaunchArgument("dataset_file"),
        DeclareLaunchArgument(
            "profile_file",
            default_value=os.path.join(package_share, "config", "object_template_matching.yaml"),
        ),
        DeclareLaunchArgument("environment_topological_map_topic", default_value="/topological_map"),
        DeclareLaunchArgument("frame_id", default_value="object_template"),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        OpaqueFunction(function=create_matching_nodes),
    ])
