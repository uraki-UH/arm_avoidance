"""launch間で共通の設定読込み・環境ROI設定。"""

from enum import Enum
import math
from pathlib import Path
import struct

import yaml


class danger_source(str, Enum):
    environment_inflation = "environment_inflation"
    vlut_distance = "vlut_distance"


def parameter_file_data(params_file):
    if not params_file:
        return {}
    try:
        with open(params_file, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"設定YAMLの読込失敗: {params_file}: {error}") from error
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ValueError(f"設定YAMLには辞書が必要です: {params_file}")
    return data


def root_parameters(data):
    if not isinstance(data, dict):
        raise ValueError("設定YAMLには辞書が必要です")
    for key in ("/**", "ros__parameters"):
        if key not in data:
            continue
        candidate = data[key]
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate["ros__parameters"]
        if not isinstance(candidate, dict):
            raise ValueError(f"{key}: ros__parametersには辞書が必要です")
        return candidate
    return {}


def load_root_parameters(params_file):
    return root_parameters(parameter_file_data(params_file))


def node_parameter_source(params_file, data):
    # 直下ros__parameters形式のROSノード用辞書化。ノード別設定ファイルの保持
    return root_parameters(data) if "ros__parameters" in data and "/**" not in data else params_file


def resolve_package_path(raw_path, get_package_share):
    if not raw_path.startswith("package://"):
        return raw_path
    package, separator, relative = raw_path[len("package://"):].partition("/")
    if not package or not separator or not relative:
        raise ValueError(f"package URIが不正です: {raw_path}")
    return str(Path(get_package_share(package)) / relative)


def read_vlut_voxel_size(vlut_file):
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
    if file_id != int.from_bytes(b"VLUT", byteorder="big") or version < 1:
        return None
    return voxel_size if math.isfinite(voxel_size) and voxel_size > 0.0 else None


def config_value(config, name, fallback):
    value = config.get(name, fallback)
    return fallback if value is None or value == "" else value


def is_enabled(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"真偽値の設定が不正です: {value}")


def namespaced_frame(robot_name, frame_id):
    frame = str(frame_id).strip().lstrip("/") or "base_link"
    return frame if "/" in frame or not robot_name else f"{robot_name}/{frame}"


def namespaced_topic(robot_name, topic):
    topic = str(topic).strip()
    if topic.startswith("/"):
        return topic
    return f"/{robot_name}/{topic}" if robot_name else f"/{topic}"


def world_index_modes(config):
    enable_legacy = is_enabled(config_value(config, "enable", False))
    enable_build = is_enabled(config_value(config, "enable_build", enable_legacy))
    enable_query = is_enabled(config_value(config, "enable_roi_query", enable_build))
    if enable_query and not enable_build:
        raise ValueError("world_index.enable_roi_queryにはworld_index.enable_buildが必要です")
    return enable_build, enable_query


def world_index_parameters(config, target_frame, *, enable_build=None):
    if enable_build is None:
        enable_build, enable_query = world_index_modes(config)
    else:
        # 明示的な索引停止時のROI照会停止。単独Viewerの直接登録との互換
        enable_query = enable_build and is_enabled(config_value(config, "enable_roi_query", True))
    bucket_size = float(config_value(config, "bucket_size", 0.2))
    if not math.isfinite(bucket_size) or bucket_size <= 0.0:
        raise ValueError("world_index.bucket_sizeには有限の正の値が必要です")
    return {
        "world_frame_id": str(config_value(config, "frame_id", "world")) if enable_build else target_frame,
        "enable_world_index": enable_build,
        "enable_roi_query": enable_query,
        "enable_world_bucket_publish": enable_build and is_enabled(config_value(config, "enable_bucket_publish", True)),
        "world_bucket_topic": str(config_value(config, "bucket_topic", "world_index_buckets")),
        "bucket_size": bucket_size,
        "parallel_thread_num": max(1, int(config_value(config, "parallel_thread_num", 1))),
    }


def roi_filter_parameters(environment, robot_name, voxel_idx, sampling):
    topic = str(config_value(environment, "reachability_map_topic", "")).strip()
    parameters = {
        key: int(config_value(voxel_idx, key, default))
        for key, default in (("x_shift", 42), ("y_shift", 21), ("z_shift", 0), ("offset", 1000000))
    }
    parameters.update({
        "enable_reachability_filter": is_enabled(config_value(environment, "enable_reachability_filter", True)),
        "reachability_map_topic": namespaced_topic(robot_name, topic) if topic else "",
        "max_dense_voxel_num": int(config_value(environment, "max_dense_voxel_num", 8000000)),
    })
    for axis in "xyz":
        for direction, fallback in (("min", -0.1 if axis == "x" else -1.0), ("max", 0.5 if axis == "x" else 1.0)):
            key = f"{direction}_reachability_{axis}"
            parameters[key] = float(config_value(environment, key, sampling.get(f"{direction}_{axis}", fallback)))
        key = f"reachability_margin_{axis}"
        parameters[key] = float(config_value(environment, key, 0.2))
    return parameters


def environment_danger_inflation(environment):
    try:
        source = danger_source(str(config_value(environment, "danger_source", "environment_inflation")).strip().lower())
    except ValueError as error:
        raise ValueError("environment_voxelization.danger_sourceにはenvironment_inflationまたはvlut_distanceが必要です") from error
    return 0.0 if source == danger_source.vlut_distance else float(config_value(environment, "danger_inflation", 0.05))
