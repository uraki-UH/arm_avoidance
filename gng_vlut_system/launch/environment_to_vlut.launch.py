import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_VOXEL_SIZE = 0.02


import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_config import (
    root_parameters as _root_parameters,
    load_root_parameters as _load_parameters,
    is_enabled as _is_enabled,
    config_value as _value,
    namespaced_frame as _namespaced_frame,
    namespaced_topic as _namespaced_topic,
    world_index_modes as _world_index_modes,
    read_vlut_voxel_size as _read_vlut_voxel_size,
    roi_filter_parameters,
    world_index_parameters,
    environment_danger_inflation,
)

def _as_launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _reachability_map_topic(environment, robot_name):
    topic = str(environment.get("reachability_map_topic") or "").strip()
    return _namespaced_topic(robot_name, topic) if topic else ""


def _vlut_file_from_parameters(root_params):
    gng = root_params.get("gng", {})
    if not isinstance(gng, dict):
        return ""
    data_directory = str(gng.get("data_directory", "")).strip()
    experiment_id = str(gng.get("experiment_id", "")).strip()
    vlut_filename = str(gng.get("vlut_filename", "vlut.bin")).strip()
    if not data_directory or not experiment_id or not vlut_filename:
        return ""
    # 左右で共通のボクセル幅。独立モデルでは左腕のヘッダを参照
    if str(gng.get("enable_independent_arms", False)).lower() in ("true", "1", "yes", "on"):
        return os.path.join(data_directory, experiment_id, "left_arm", vlut_filename)
    return os.path.join(data_directory, experiment_id, vlut_filename)


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
    self_recognition = root_params.get("self_recognition", {})
    if not isinstance(self_recognition, dict):
        self_recognition = {}

    default_robot_name = _value(
        environment, "robot_name", root_params.get("robot_name", ""))
    robot_name = str(_value(entry, "robot_name", default_robot_name)).strip()
    if not robot_name:
        raise RuntimeError(f"共有world index consumerのrobot_name未設定: {params_file}")
    base_frame = _value(environment, "base_frame", root_params.get("frame_id", "base_link"))
    target_frame_id = str(_value(
        entry, "target_frame_id", _namespaced_frame(robot_name, base_frame)))
    static_tf_child_frame = _namespaced_frame(
        robot_name,
        _value(
            entry,
            "static_tf_child_frame",
            _value(environment, "static_tf_child_frame", base_frame)))
    voxel_topic = str(_value(
        entry, "voxel_topic", _value(
            environment, "voxel_topic", f"/{robot_name}/self_filter_roi_voxels")))
    enable_environment_self_filter = _is_enabled(_value(
        self_recognition, "enable_environment_self_filter", False))
    raw_voxel_topic = voxel_topic
    if enable_environment_self_filter:
        raw_voxel_topic = _namespaced_topic(robot_name, _value(
            self_recognition, "raw_environment_voxel_topic", "roi_voxels"))
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

    danger_inflation = environment_danger_inflation(environment)

    return {
        "name": robot_name,
        "params_file": params_file,
        "input_topic": input_topic,
        "source_frame_id": source_frame_id,
        "target_frame_id": target_frame_id,
        "voxel_topic": voxel_topic,
        "raw_voxel_topic": raw_voxel_topic,
        "enable_environment_self_filter": enable_environment_self_filter,
        "voxel_size": voxel_size,
        **roi_filter_parameters({**environment, **entry}, robot_name, voxel_idx_params, gng_params),
        "danger_inflation": danger_inflation,
        "publish_hz": float(_value(environment, "publish_hz", 30.0)),
        "enable_static_tf": _is_enabled(_value(environment, "enable_static_tf", False)),
        "static_tf_parent_frame": str(_value(environment, "static_tf_parent_frame", "world")),
        "static_tf_child_frame": static_tf_child_frame,
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
        "z_shift", "offset", "enable_reachability_filter", "reachability_map_topic",
        "min_reachability_x",
        "max_reachability_x", "min_reachability_y", "max_reachability_y",
        "min_reachability_z", "max_reachability_z", "reachability_margin_x",
        "reachability_margin_y", "reachability_margin_z", "max_dense_voxel_num")
    values = {key: consumer[key] for key in keys}
    values["output_topic"] = consumer["raw_voxel_topic"]
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
    index_params = world_index_parameters(world_index, primary_consumer["target_frame_id"])
    enable_build = index_params["enable_world_index"]
    enable_roi_query = index_params["enable_roi_query"]
    enable_bucket_publish = index_params["enable_world_bucket_publish"]
    parallel_thread_num = index_params["parallel_thread_num"]
    allow_unconnected_source_as_world = _is_enabled(_value(
        environment, "allow_unconnected_source_as_world", True))
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
                "output_topic": primary_consumer["raw_voxel_topic"],
                "source_frame_id": source_frame_id,
                "world_frame_id": world_frame_id,
                "target_frame_id": primary_consumer["target_frame_id"],
                "allow_unconnected_source_as_world":
                    allow_unconnected_source_as_world,
                "enable_world_index": enable_build,
                "enable_roi_query": enable_roi_query,
                "world_bucket_topic": bucket_topic,
                "enable_world_bucket_publish": enable_bucket_publish,
                "bucket_size": bucket_size,
                "parallel_thread_num": parallel_thread_num,
                "additional_consumers_json": additional_consumers_json,
                **{
                    key: primary_consumer[key]
                    for key in (
                        "voxel_size", "x_shift", "y_shift", "z_shift", "offset",
                        "enable_reachability_filter", "reachability_map_topic",
                        "min_reachability_x",
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
                        consumer["static_tf_parent_frame"],
                        consumer["static_tf_child_frame"],
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
        f"parallel_thread_num={parallel_thread_num} "
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
    self_recognition = root_params.get("self_recognition", {})
    if not isinstance(self_recognition, dict):
        self_recognition = {}

    configured_robot_name = _value(
        environment, "robot_name", root_params.get("robot_name", "ToPoDualArm"))
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or str(configured_robot_name)
    base_frame = _value(environment, "base_frame", root_params.get("frame_id", "base_link"))
    target_frame_id = _namespaced_frame(robot_name, base_frame)
    input_topic = _value(environment, "input_topic", "/topo_points")
    voxel_topic = _value(environment, "voxel_topic", f"/{robot_name}/self_filter_roi_voxels")
    enable_environment_self_filter = _is_enabled(_value(
        self_recognition, "enable_environment_self_filter", False))
    source_voxel_topic = voxel_topic
    if enable_environment_self_filter:
        source_voxel_topic = _namespaced_topic(robot_name, _value(
            self_recognition, "raw_environment_voxel_topic", "roi_voxels"))
    source_frame_id = _value(environment, "source_frame_id", "")
    index_params = world_index_parameters(world_index, target_frame_id)
    enable_world_index_build = index_params["enable_world_index"]
    enable_world_index_roi_query = index_params["enable_roi_query"]
    world_index_frame_id = index_params["world_frame_id"]
    world_index_bucket_topic = _value(
        world_index, "bucket_topic", f"/{robot_name}/world_index_buckets")
    enable_world_index_bucket_publish = index_params["enable_world_bucket_publish"]
    world_index_bucket_size = index_params["bucket_size"]
    world_index_parallel_thread_num = index_params["parallel_thread_num"]
    danger_inflation = environment_danger_inflation(environment)

    bridge_arguments = {
        "robot_name": robot_name,
        "input_topic": input_topic,
        "voxel_topic": voxel_topic,
        "source_voxel_topic": source_voxel_topic,
        "source_frame_id": source_frame_id,
        "target_frame_id": target_frame_id,
        "params_file": params_file,
        **roi_filter_parameters(environment, robot_name, voxel_idx_params, gng_params),
        "world_index_enable": enable_world_index_build,
        "world_index_enable_build": enable_world_index_build,
        "world_index_enable_roi_query": enable_world_index_roi_query,
        "allow_unconnected_source_as_world": _value(
            environment, "allow_unconnected_source_as_world", True),
        "world_index_frame_id": world_index_frame_id,
        "world_index_bucket_topic": world_index_bucket_topic,
        "world_index_enable_bucket_publish": enable_world_index_bucket_publish,
        "world_index_bucket_size": world_index_bucket_size,
        "world_index_parallel_thread_num": world_index_parallel_thread_num,
        "danger_inflation": danger_inflation,
        "publish_hz": _value(environment, "publish_hz", 30.0),
    }

    actions = []

    if _is_enabled(_value(environment, "enable_static_tf", False)):
        static_parent_frame = _value(environment, "static_tf_parent_frame", "world")
        static_child_frame = _namespaced_frame(
            robot_name, _value(environment, "static_tf_child_frame", base_frame))
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
                    static_child_frame,
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
        f"source_voxel_topic={source_voxel_topic} voxel_topic={voxel_topic} "
        f"self_filter={enable_environment_self_filter} tf={tf_mode} danger_source={_value(environment, 'danger_source', 'environment_inflation')} "
        f"world_index_build={enable_world_index_build} "
        f"world_index_roi_query={enable_world_index_roi_query}"
    )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_share, "config", "topo_dual_arm_max_long.yaml")),
        DeclareLaunchArgument("robot_name", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
