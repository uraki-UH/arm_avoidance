import json
import math
import os
import struct

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VLUT_FILE_SIGNATURE = b"VLUT"
VLUT_FILE_ID = int.from_bytes(VLUT_FILE_SIGNATURE, byteorder="big")
DEFAULT_VOXEL_SIZE = 0.02


def _root_parameters(params_yaml):
    for root_key in ("/**", "ros__parameters"):
        candidate = params_yaml.get(root_key, {})
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate["ros__parameters"]
        if isinstance(candidate, dict):
            return candidate
    return {}


def _load_parameters(params_file):
    try:
        with open(params_file, "r", encoding="utf-8") as stream:
            return _root_parameters(yaml.safe_load(stream) or {})
    except (OSError, yaml.YAMLError) as ex:
        raise RuntimeError(f"環境VLUT設定YAMLの読込失敗: {ex}") from ex


def _is_enabled(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _as_launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _value(config, name, fallback):
    value = config.get(name, fallback)
    return fallback if value is None or value == "" else value


def _namespaced_frame(robot_name, frame_id):
    normalized = str(frame_id).strip().lstrip("/")
    if not normalized:
        normalized = "base_link"
    if "/" in normalized:
        return normalized
    return f"{robot_name}/{normalized}" if robot_name else normalized


def _world_index_modes(world_index):
    legacy_enable = _is_enabled(_value(world_index, "enable", False))
    enable_build = _is_enabled(_value(
        world_index, "enable_build", legacy_enable))
    enable_roi_query = _is_enabled(_value(
        world_index, "enable_roi_query", legacy_enable))
    if enable_roi_query and not enable_build:
        raise RuntimeError(
            "world_index.enable_roi_queryにはworld_index.enable_buildが必要")
    return enable_build, enable_roi_query


def _vlut_file_from_parameters(root_params):
    gng = root_params.get("gng", {})
    if not isinstance(gng, dict):
        return ""
    data_directory = str(gng.get("data_directory", "")).strip()
    experiment_id = str(gng.get("experiment_id", "")).strip()
    vlut_filename = str(gng.get("vlut_filename", "vlut.bin")).strip()
    if not data_directory or not experiment_id or not vlut_filename:
        return ""
    return os.path.join(data_directory, experiment_id, vlut_filename)


def _read_vlut_voxel_size(vlut_file):
    if not vlut_file:
        return None
    try:
        with open(vlut_file, "rb") as stream:
            header = stream.read(12)
    except OSError:
        return None
    if len(header) != 12:
        return None
    file_id, version, voxel_size = struct.unpack("<IIf", header)
    if file_id != VLUT_FILE_ID or version < 1:
        return None
    if not math.isfinite(voxel_size) or voxel_size <= 0.0:
        return None
    return voxel_size


def _shared_consumer_parameters(entry, default_input_topic, default_source_frame_id):
    if not isinstance(entry, dict):
        raise RuntimeError("world_index.consumersの各要素にはYAML map形式が必要")
    params_file = str(entry.get("params_file", "")).strip()
    if not params_file:
        raise RuntimeError("world_index.consumersの各要素にはparams_fileが必要")
    root_params = _load_parameters(params_file)
    environment = root_params.get("environment_voxelization", {})
    if not isinstance(environment, dict):
        environment = {}
    gng_params = root_params.get("gng_params", {})
    if not isinstance(gng_params, dict):
        gng_params = {}
    voxel_idx_params = root_params.get("voxel_idx_shift", {})
    if not isinstance(voxel_idx_params, dict):
        voxel_idx_params = {}

    default_robot_name = _value(
        environment, "robot_name", root_params.get("robot_name", ""))
    robot_name = str(_value(entry, "robot_name", default_robot_name)).strip()
    if not robot_name:
        raise RuntimeError(f"共有world index consumerのrobot_name未設定: {params_file}")
    base_frame = _value(environment, "base_frame", root_params.get("frame_id", "base_link"))
    target_frame_id = str(_value(
        entry, "target_frame_id", _namespaced_frame(robot_name, base_frame)))
    voxel_topic = str(_value(
        entry, "voxel_topic", _value(
            environment, "voxel_topic", f"/{robot_name}/roi_voxel_ids")))
    input_topic = str(_value(entry, "input_topic", default_input_topic))
    source_frame_id = str(_value(entry, "source_frame_id", default_source_frame_id))

    automatic_voxel_size = _read_vlut_voxel_size(_vlut_file_from_parameters(root_params))
    configured_voxel_size = _value(entry, "voxel_size", None)
    if configured_voxel_size is not None:
        voxel_size = float(configured_voxel_size)
    elif automatic_voxel_size is not None:
        voxel_size = automatic_voxel_size
    else:
        gng = root_params.get("gng", {})
        voxel_size = float(_value(
            gng if isinstance(gng, dict) else {}, "vlut_resolution", DEFAULT_VOXEL_SIZE))
    if voxel_size <= 0.0:
        raise RuntimeError(f"共有world index consumerのvoxel_size不正: robot={robot_name}")

    danger_source = str(_value(
        environment, "danger_source", "environment_inflation")).strip().lower()
    if danger_source not in ("environment_inflation", "vlut_distance"):
        raise RuntimeError(
            "environment_voxelization.danger_sourceはenvironment_inflationまたは"
            f"vlut_distanceが必要: robot={robot_name}")
    danger_inflation = float(_value(environment, "danger_inflation", 0.05))
    if danger_source == "vlut_distance":
        danger_inflation = 0.0

    return {
        "name": robot_name,
        "params_file": params_file,
        "input_topic": input_topic,
        "source_frame_id": source_frame_id,
        "target_frame_id": target_frame_id,
        "voxel_topic": voxel_topic,
        "voxel_size": voxel_size,
        "x_shift": int(_value(voxel_idx_params, "x_shift", 42)),
        "y_shift": int(_value(voxel_idx_params, "y_shift", 21)),
        "z_shift": int(_value(voxel_idx_params, "z_shift", 0)),
        "offset": int(_value(voxel_idx_params, "offset", 1000000)),
        "enable_reachability_filter": _is_enabled(_value(
            environment, "enable_reachability_filter", True)),
        "min_reachability_x": float(_value(
            environment, "min_reachability_x", gng_params.get("min_x", -0.1))),
        "max_reachability_x": float(_value(
            environment, "max_reachability_x", gng_params.get("max_x", 0.5))),
        "min_reachability_y": float(_value(
            environment, "min_reachability_y", gng_params.get("min_y", -1.0))),
        "max_reachability_y": float(_value(
            environment, "max_reachability_y", gng_params.get("max_y", 1.0))),
        "min_reachability_z": float(_value(
            environment, "min_reachability_z", gng_params.get("min_z", -1.0))),
        "max_reachability_z": float(_value(
            environment, "max_reachability_z", gng_params.get("max_z", 1.0))),
        "reachability_margin_x": float(_value(environment, "reachability_margin_x", 0.2)),
        "reachability_margin_y": float(_value(environment, "reachability_margin_y", 0.2)),
        "reachability_margin_z": float(_value(environment, "reachability_margin_z", 0.2)),
        "max_dense_voxel_num": int(_value(environment, "max_dense_voxel_num", 8000000)),
        "danger_inflation": danger_inflation,
        "publish_hz": float(_value(environment, "publish_hz", 30.0)),
        "enable_static_tf": _is_enabled(_value(environment, "enable_static_tf", False)),
        "static_tf_parent_frame": str(_value(environment, "static_tf_parent_frame", "world")),
        "static_tf_x": float(_value(environment, "static_tf_x", 0.0)),
        "static_tf_y": float(_value(environment, "static_tf_y", 0.0)),
        "static_tf_z": float(_value(environment, "static_tf_z", 0.0)),
        "static_tf_roll": float(_value(environment, "static_tf_roll", 0.0)),
        "static_tf_pitch": float(_value(environment, "static_tf_pitch", 0.0)),
        "static_tf_yaw": float(_value(environment, "static_tf_yaw", 0.0)),
    }


def _additional_consumer_json(consumer):
    keys = (
        "name", "target_frame_id", "voxel_topic", "voxel_size", "x_shift", "y_shift",
        "z_shift", "offset", "enable_reachability_filter", "min_reachability_x",
        "max_reachability_x", "min_reachability_y", "max_reachability_y",
        "min_reachability_z", "max_reachability_z", "reachability_margin_x",
        "reachability_margin_y", "reachability_margin_z", "max_dense_voxel_num")
    values = {key: consumer[key] for key in keys}
    values["output_topic"] = values.pop("voxel_topic")
    return values


def _shared_world_index_actions(
    package_share, params_file, environment, world_index):
    consumer_entries = world_index.get("consumers", [])
    if not isinstance(consumer_entries, list) or len(consumer_entries) < 2:
        raise RuntimeError("共有world indexには2台以上のworld_index.consumersが必要")
    input_topic = str(_value(environment, "input_topic", "/topo_points"))
    source_frame_id = str(_value(environment, "source_frame_id", ""))
    consumers = [
        _shared_consumer_parameters(entry, input_topic, source_frame_id)
        for entry in consumer_entries
    ]
    if any(consumer["input_topic"] != input_topic for consumer in consumers):
        raise RuntimeError("共有world index consumerのinput_topicは全台で共通指定が必要")
    if any(consumer["source_frame_id"] != source_frame_id for consumer in consumers):
        raise RuntimeError("共有world index consumerのsource_frame_idは全台で共通指定が必要")
    robot_names = [consumer["name"] for consumer in consumers]
    voxel_topics = [consumer["voxel_topic"] for consumer in consumers]
    if len(set(robot_names)) != len(robot_names):
        raise RuntimeError("共有world index consumerのrobot_name重複")
    if len(set(voxel_topics)) != len(voxel_topics):
        raise RuntimeError("共有world index consumerのvoxel_topic重複")

    primary_consumer = consumers[0]
    world_frame_id = str(_value(world_index, "frame_id", "world"))
    bucket_topic = str(_value(world_index, "bucket_topic", "/world_index/buckets"))
    bucket_size = float(_value(world_index, "bucket_size", 0.2))
    if bucket_size <= 0.0:
        raise RuntimeError("world_index.bucket_sizeには正の値が必要")
    enable_build, enable_roi_query = _world_index_modes(world_index)
    enable_bucket_publish = _is_enabled(_value(
        world_index, "enable_bucket_publish", True)) and enable_build
    additional_consumers_json = json.dumps([
        _additional_consumer_json(consumer) for consumer in consumers[1:]
    ], separators=(",", ":"))

    actions = [
        Node(
            package="gng_vlut_system",
            executable="world_index_to_voxel_node",
            name="shared_world_index_to_voxel_node",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "output_topic": primary_consumer["voxel_topic"],
                "source_frame_id": source_frame_id,
                "world_frame_id": world_frame_id,
                "target_frame_id": primary_consumer["target_frame_id"],
                "enable_world_index": enable_build,
                "enable_roi_query": enable_roi_query,
                "world_bucket_topic": bucket_topic,
                "enable_world_bucket_publish": enable_bucket_publish,
                "bucket_size": bucket_size,
                "additional_consumers_json": additional_consumers_json,
                **{
                    key: primary_consumer[key]
                    for key in (
                        "voxel_size", "x_shift", "y_shift", "z_shift", "offset",
                        "enable_reachability_filter", "min_reachability_x",
                        "max_reachability_x", "min_reachability_y", "max_reachability_y",
                        "min_reachability_z", "max_reachability_z",
                        "reachability_margin_x", "reachability_margin_y",
                        "reachability_margin_z", "max_dense_voxel_num")
                },
            }],
        )
    ]
    for consumer_idx, consumer in enumerate(consumers):
        if consumer["enable_static_tf"]:
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"shared_world_index_static_tf_{consumer_idx}",
                    arguments=[
                        str(consumer["static_tf_x"]), str(consumer["static_tf_y"]),
                        str(consumer["static_tf_z"]), str(consumer["static_tf_yaw"]),
                        str(consumer["static_tf_pitch"]), str(consumer["static_tf_roll"]),
                        consumer["static_tf_parent_frame"], consumer["target_frame_id"],
                    ],
                )
            )
        actions.append(
            Node(
                package="gng_vlut_system",
                executable="voxel_to_vlut_node",
                name=f"voxel_to_vlut_node_{consumer_idx}",
                namespace=consumer["name"],
                output="screen",
                parameters=[{
                    "input_topic": consumer["voxel_topic"],
                    "occupied_voxels_topic": "occupied_voxels",
                    "danger_voxels_topic": "danger_voxels",
                    "target_frame_id": consumer["target_frame_id"],
                    "danger_inflation": consumer["danger_inflation"],
                    "output_voxel_size": consumer["voxel_size"],
                    "publish_hz": consumer["publish_hz"],
                }],
            )
        )

    print(
        "[environment_to_vlut] 共有world index起動設定: "
        f"input={input_topic} world_frame={world_frame_id} bucket_size={bucket_size:.6g} "
        f"consumer_num={len(consumers)} build={enable_build} roi_query={enable_roi_query} "
        f"bucket_topic={bucket_topic if enable_bucket_publish else 'disabled'}"
    )
    for consumer in consumers:
        print(
            "[environment_to_vlut] consumer: "
            f"robot={consumer['name']} target_frame={consumer['target_frame_id']} "
            f"voxel_size={consumer['voxel_size']:.6g} voxel_topic={consumer['voxel_topic']}"
        )
    return actions


def _launch_setup(context, *_args, **_kwargs):
    package_share = get_package_share_directory("gng_vlut_system")
    params_file = LaunchConfiguration("params_file").perform(context)
    root_params = _load_parameters(params_file)
    environment = root_params.get("environment_voxelization", {})
    if not isinstance(environment, dict):
        raise RuntimeError("environment_voxelizationはYAML map形式が必要")
    world_index = environment.get("world_index", {})
    if world_index is None:
        world_index = {}
    if not isinstance(world_index, dict):
        raise RuntimeError("environment_voxelization.world_indexはYAML map形式が必要")
    consumer_entries = world_index.get("consumers", [])
    if isinstance(consumer_entries, list) and len(consumer_entries) >= 2:
        return _shared_world_index_actions(
            package_share, params_file, environment, world_index)

    gng_params = root_params.get("gng_params", {})
    if not isinstance(gng_params, dict):
        gng_params = {}
    voxel_idx_params = root_params.get("voxel_idx_shift", {})
    if not isinstance(voxel_idx_params, dict):
        voxel_idx_params = {}

    configured_robot_name = _value(
        environment, "robot_name", root_params.get("robot_name", "ToPoDualArm"))
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or str(configured_robot_name)
    base_frame = _value(environment, "base_frame", root_params.get("frame_id", "base_link"))
    target_frame_id = _namespaced_frame(robot_name, base_frame)
    input_topic = _value(environment, "input_topic", "/topo_points")
    voxel_topic = _value(environment, "voxel_topic", f"/{robot_name}/roi_voxel_ids")
    source_frame_id = _value(environment, "source_frame_id", "")
    enable_world_index_build, enable_world_index_roi_query = _world_index_modes(world_index)
    world_index_frame_id = _value(world_index, "frame_id", "world")
    world_index_bucket_topic = _value(
        world_index, "bucket_topic", f"/{robot_name}/world_index_buckets")
    enable_world_index_bucket_publish = _is_enabled(
        _value(world_index, "enable_bucket_publish", True)) and enable_world_index_build
    world_index_bucket_size = _value(world_index, "bucket_size", 0.2)
    danger_source = str(_value(
        environment, "danger_source", "environment_inflation")).strip().lower()
    if danger_source not in ("environment_inflation", "vlut_distance"):
        raise RuntimeError(
            "environment_voxelization.danger_sourceはenvironment_inflationまたは"
            "vlut_distanceが必要")
    danger_inflation = _value(environment, "danger_inflation", 0.05)
    if danger_source == "vlut_distance":
        danger_inflation = 0.0

    bridge_arguments = {
        "robot_name": robot_name,
        "input_topic": input_topic,
        "voxel_topic": voxel_topic,
        "source_frame_id": source_frame_id,
        "target_frame_id": target_frame_id,
        "params_file": params_file,
        "x_shift": _value(voxel_idx_params, "x_shift", 42),
        "y_shift": _value(voxel_idx_params, "y_shift", 21),
        "z_shift": _value(voxel_idx_params, "z_shift", 0),
        "offset": _value(voxel_idx_params, "offset", 1000000),
        "enable_reachability_filter": _value(
            environment, "enable_reachability_filter", True),
        "min_reachability_x": _value(environment, "min_reachability_x", gng_params.get("min_x", -0.1)),
        "max_reachability_x": _value(environment, "max_reachability_x", gng_params.get("max_x", 0.5)),
        "min_reachability_y": _value(environment, "min_reachability_y", gng_params.get("min_y", -1.0)),
        "max_reachability_y": _value(environment, "max_reachability_y", gng_params.get("max_y", 1.0)),
        "min_reachability_z": _value(environment, "min_reachability_z", gng_params.get("min_z", -1.0)),
        "max_reachability_z": _value(environment, "max_reachability_z", gng_params.get("max_z", 1.0)),
        "reachability_margin_x": _value(environment, "reachability_margin_x", 0.2),
        "reachability_margin_y": _value(environment, "reachability_margin_y", 0.2),
        "reachability_margin_z": _value(environment, "reachability_margin_z", 0.2),
        "max_dense_voxel_num": _value(environment, "max_dense_voxel_num", 8000000),
        "world_index_enable": enable_world_index_build,
        "world_index_enable_build": enable_world_index_build,
        "world_index_enable_roi_query": enable_world_index_roi_query,
        "world_index_frame_id": world_index_frame_id,
        "world_index_bucket_topic": world_index_bucket_topic,
        "world_index_enable_bucket_publish": enable_world_index_bucket_publish,
        "world_index_bucket_size": world_index_bucket_size,
        "danger_inflation": danger_inflation,
        "publish_hz": _value(environment, "publish_hz", 30.0),
    }

    actions = []

    if _is_enabled(_value(environment, "enable_static_tf", False)):
        static_parent_frame = _value(environment, "static_tf_parent_frame", "world")
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="environment_to_vlut_static_tf",
                arguments=[
                    _as_launch_value(_value(environment, "static_tf_x", 0.0)),
                    _as_launch_value(_value(environment, "static_tf_y", 0.0)),
                    _as_launch_value(_value(environment, "static_tf_z", 0.0)),
                    _as_launch_value(_value(environment, "static_tf_yaw", 0.0)),
                    _as_launch_value(_value(environment, "static_tf_pitch", 0.0)),
                    _as_launch_value(_value(environment, "static_tf_roll", 0.0)),
                    _as_launch_value(static_parent_frame),
                    target_frame_id,
                ],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", "point_to_vlut.launch.py")),
            launch_arguments={
                name: _as_launch_value(value)
                for name, value in bridge_arguments.items()
            }.items(),
        )
    )

    tf_mode = "static" if _is_enabled(_value(environment, "enable_static_tf", False)) else "external"
    print(
        "[environment_to_vlut] 統合起動設定: "
        f"robot={robot_name} input={input_topic} target_frame={target_frame_id} "
        f"voxel_topic={voxel_topic} tf={tf_mode} danger_source={danger_source} "
        f"world_index_build={enable_world_index_build} "
        f"world_index_roi_query={enable_world_index_roi_query}"
    )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("robot_name", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
