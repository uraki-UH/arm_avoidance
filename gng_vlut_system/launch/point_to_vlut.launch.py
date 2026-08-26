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
DEFAULT_VOXEL_SIZE = "0.02"


def _find_root_parameters(params_yaml):
    for root_key in ("/**", "ros__parameters"):
        candidate = params_yaml.get(root_key, {})
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate["ros__parameters"]
        if isinstance(candidate, dict):
            return candidate
    return {}


def _resolve_vlut_file(params_file, explicit_vlut_file):
    if explicit_vlut_file:
        return explicit_vlut_file
    if not params_file or not os.path.isfile(params_file):
        return ""

    try:
        with open(params_file, "r", encoding="utf-8") as stream:
            params_yaml = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as ex:
        print(f"[point_to_vlut] VLUT設定YAMLの読込失敗: {ex}")
        return ""

    root_parameters = _find_root_parameters(params_yaml)
    gng_parameters = root_parameters.get("gng", {})
    if not isinstance(gng_parameters, dict):
        return ""

    data_directory = str(gng_parameters.get("data_directory", "")).strip()
    experiment_id = str(gng_parameters.get("experiment_id", "")).strip()
    vlut_filename = str(gng_parameters.get("vlut_filename", "vlut.bin")).strip()
    if not data_directory or not experiment_id or not vlut_filename:
        return ""
    return os.path.join(data_directory, experiment_id, vlut_filename)


def _read_vlut_voxel_size(vlut_file):
    if not vlut_file:
        return None
    try:
        with open(vlut_file, "rb") as stream:
            header = stream.read(12)
    except OSError as ex:
        print(f"[point_to_vlut] VLUT読込失敗: file={vlut_file} error={ex}")
        return None

    if len(header) != 12:
        print(f"[point_to_vlut] VLUTヘッダ長不足: file={vlut_file}")
        return None

    file_id, version, voxel_size = struct.unpack("<IIf", header)
    if file_id != VLUT_FILE_ID or version < 1:
        print(
            "[point_to_vlut] VLUTヘッダ未対応: "
            f"file={vlut_file} file_id=0x{file_id:08x} version={version}"
        )
        return None
    if not math.isfinite(voxel_size) or voxel_size <= 0.0:
        print(
            "[point_to_vlut] VLUT解像度不正: "
            f"file={vlut_file} voxel_size={voxel_size}"
        )
        return None
    return voxel_size


def _is_enabled(raw_value):
    return raw_value.strip().lower() in ("1", "true", "yes", "on")


def _float_argument(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _bool_argument(context, name):
    return _is_enabled(LaunchConfiguration(name).perform(context))


def _optional_bool_argument(context, name, fallback):
    raw_value = LaunchConfiguration(name).perform(context).strip()
    return fallback if not raw_value else _is_enabled(raw_value)


def _launch_setup(context, *_args, **_kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    explicit_vlut_file = LaunchConfiguration("vlut_file").perform(context)
    point_voxel_size = _float_argument(context, "voxel_size")
    output_voxel_size = _float_argument(context, "output_voxel_size")

    if _bool_argument(context, "enable_vlut_voxel_size_auto"):
        vlut_file = _resolve_vlut_file(params_file, explicit_vlut_file)
        vlut_voxel_size = _read_vlut_voxel_size(vlut_file)
        if vlut_voxel_size is not None:
            point_voxel_size = vlut_voxel_size
            output_voxel_size = vlut_voxel_size
            print(
                "[point_to_vlut] VLUTヘッダからvoxel_sizeを自動設定: "
                f"file={vlut_file} voxel_size={vlut_voxel_size:.9g}"
            )
        else:
            print(
                "[point_to_vlut] VLUTヘッダを利用できないため、"
                f"手動設定値を使用: voxel_size={point_voxel_size:.9g} "
                f"output_voxel_size={output_voxel_size:.9g}"
            )

    robot_name = LaunchConfiguration("robot_name").perform(context)
    input_topic = LaunchConfiguration("input_topic").perform(context)
    voxel_topic = LaunchConfiguration("voxel_topic").perform(context)
    source_frame_id = LaunchConfiguration("source_frame_id").perform(context)
    target_frame_id = LaunchConfiguration("target_frame_id").perform(context)
    legacy_enable_world_index = _bool_argument(context, "world_index_enable")
    enable_world_index_build = _optional_bool_argument(
        context, "world_index_enable_build", legacy_enable_world_index)
    enable_world_index_roi_query = _optional_bool_argument(
        context, "world_index_enable_roi_query", legacy_enable_world_index)
    if enable_world_index_roi_query and not enable_world_index_build:
        raise RuntimeError("world_index_enable_roi_queryにはworld_index_enable_buildが必要")

    voxel_source_arguments = {
        "input_topic": input_topic,
        "output_topic": voxel_topic,
        "source_frame_id": source_frame_id,
        "target_frame_id": target_frame_id,
        "voxel_size": f"{point_voxel_size:.9g}",
        "x_shift": LaunchConfiguration("x_shift"),
        "y_shift": LaunchConfiguration("y_shift"),
        "z_shift": LaunchConfiguration("z_shift"),
        "offset": LaunchConfiguration("offset"),
        "enable_reachability_filter": LaunchConfiguration("enable_reachability_filter"),
        "min_reachability_x": LaunchConfiguration("min_reachability_x"),
        "max_reachability_x": LaunchConfiguration("max_reachability_x"),
        "min_reachability_y": LaunchConfiguration("min_reachability_y"),
        "max_reachability_y": LaunchConfiguration("max_reachability_y"),
        "min_reachability_z": LaunchConfiguration("min_reachability_z"),
        "max_reachability_z": LaunchConfiguration("max_reachability_z"),
        "reachability_margin_x": LaunchConfiguration("reachability_margin_x"),
        "reachability_margin_y": LaunchConfiguration("reachability_margin_y"),
        "reachability_margin_z": LaunchConfiguration("reachability_margin_z"),
        "max_dense_voxel_num": LaunchConfiguration("max_dense_voxel_num"),
    }

    voxel_source_launch = "point_to_voxel.launch.py"
    if enable_world_index_build:
        voxel_source_launch = "world_index_to_voxel.launch.py"
        voxel_source_arguments.update({
            "enable_world_index": "true",
            "enable_roi_query": (
                "true" if enable_world_index_roi_query else "false"),
            "allow_unconnected_source_as_world": LaunchConfiguration(
                "allow_unconnected_source_as_world"),
            "world_frame_id": LaunchConfiguration("world_index_frame_id"),
            "world_bucket_topic": LaunchConfiguration("world_index_bucket_topic"),
            "enable_world_bucket_publish": LaunchConfiguration(
                "world_index_enable_bucket_publish"),
            "bucket_size": LaunchConfiguration("world_index_bucket_size"),
            "parallel_thread_num": LaunchConfiguration(
                "world_index_parallel_thread_num"),
        })

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("gng_vlut_system"),
                    "launch",
                    voxel_source_launch)),
            launch_arguments=voxel_source_arguments.items()),
        Node(
            package="gng_vlut_system",
            executable="voxel_to_vlut_node",
            name="voxel_to_vlut_node",
            namespace=robot_name,
            output="screen",
            parameters=[{
                "input_topic": voxel_topic,
                "occupied_voxels_topic": "occupied_voxels",
                "danger_voxels_topic": "danger_voxels",
                "target_frame_id": target_frame_id,
                "danger_inflation": _float_argument(context, "danger_inflation"),
                "output_voxel_size": output_voxel_size,
                "publish_hz": _float_argument(context, "publish_hz"),
            }],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    default_params_file = os.path.join(package_share, "config", "ToPoDualArm.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("input_topic", default_value="/topo_points"),
        DeclareLaunchArgument("voxel_topic", default_value="/topo_voxel_ids"),
        DeclareLaunchArgument("source_frame_id", default_value=""),
        DeclareLaunchArgument("target_frame_id", default_value="ToPoDualArm/base_link"),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("vlut_file", default_value=""),
        DeclareLaunchArgument("enable_vlut_voxel_size_auto", default_value="true"),
        DeclareLaunchArgument("world_index_enable", default_value="false"),
        DeclareLaunchArgument("world_index_enable_build", default_value=""),
        DeclareLaunchArgument("world_index_enable_roi_query", default_value=""),
        DeclareLaunchArgument(
            "allow_unconnected_source_as_world", default_value="false"),
        DeclareLaunchArgument("world_index_frame_id", default_value="world"),
        DeclareLaunchArgument(
            "world_index_bucket_topic", default_value="/world_index/buckets"),
        DeclareLaunchArgument("world_index_enable_bucket_publish", default_value="true"),
        DeclareLaunchArgument("world_index_bucket_size", default_value="0.2"),
        DeclareLaunchArgument("world_index_parallel_thread_num", default_value="1"),
        DeclareLaunchArgument("voxel_size", default_value=DEFAULT_VOXEL_SIZE),
        DeclareLaunchArgument("x_shift", default_value="42"),
        DeclareLaunchArgument("y_shift", default_value="21"),
        DeclareLaunchArgument("z_shift", default_value="0"),
        DeclareLaunchArgument("offset", default_value="1000000"),
        DeclareLaunchArgument("enable_reachability_filter", default_value="true"),
        DeclareLaunchArgument("min_reachability_x", default_value="-0.1"),
        DeclareLaunchArgument("max_reachability_x", default_value="0.5"),
        DeclareLaunchArgument("min_reachability_y", default_value="-1.0"),
        DeclareLaunchArgument("max_reachability_y", default_value="1.0"),
        DeclareLaunchArgument("min_reachability_z", default_value="-1.0"),
        DeclareLaunchArgument("max_reachability_z", default_value="1.0"),
        DeclareLaunchArgument("reachability_margin_x", default_value="0.2"),
        DeclareLaunchArgument("reachability_margin_y", default_value="0.2"),
        DeclareLaunchArgument("reachability_margin_z", default_value="0.2"),
        DeclareLaunchArgument("max_dense_voxel_num", default_value="8000000"),
        DeclareLaunchArgument("danger_inflation", default_value="0.05"),
        DeclareLaunchArgument("output_voxel_size", default_value=DEFAULT_VOXEL_SIZE),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),
        OpaqueFunction(function=_launch_setup),
    ])
