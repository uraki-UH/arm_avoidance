import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_vector(entry, key, transform_idx):
    values = entry.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"sensor_static_transforms[{transform_idx}].{key} must have three values")
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"sensor_static_transforms[{transform_idx}].{key} must be numeric") from error
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"sensor_static_transforms[{transform_idx}].{key} must be finite")
    return result


def _launch_setup(context):
    params_file = Path(LaunchConfiguration("params_file").perform(context))
    if not params_file.is_file():
        raise FileNotFoundError(f"sensor static TF設定ファイルが見つかりません: {params_file}")
    with params_file.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    transforms = config.get("sensor_static_transforms", [])
    if not isinstance(transforms, list):
        raise ValueError("sensor_static_transforms must be a list")

    nodes = []
    for transform_idx, entry in enumerate(transforms):
        if not isinstance(entry, dict):
            raise ValueError(f"sensor_static_transforms[{transform_idx}] must be a mapping")
        parent_frame = entry.get("parent_frame", "")
        child_frame = entry.get("child_frame", "")
        if not isinstance(parent_frame, str) or not isinstance(child_frame, str):
            raise ValueError(f"sensor_static_transforms[{transform_idx}] frame name must be a string")
        if not parent_frame or not child_frame or parent_frame == child_frame:
            raise ValueError(f"sensor_static_transforms[{transform_idx}] frame pair is invalid")
        translation_m = _read_vector(entry, "translation_m", transform_idx)
        rpy_deg = _read_vector(entry, "rpy_deg", transform_idx)
        roll, pitch, yaw = (math.radians(value) for value in rpy_deg)
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"sensor_static_tf_{transform_idx}",
                arguments=[
                    "--x", str(translation_m[0]),
                    "--y", str(translation_m[1]),
                    "--z", str(translation_m[2]),
                    "--roll", str(roll),
                    "--pitch", str(pitch),
                    "--yaw", str(yaw),
                    "--frame-id", parent_frame,
                    "--child-frame-id", child_frame,
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    default_params_file = str(
        Path(get_package_share_directory("gng_vlut_system"))
        / "config"
        / "sensor_static_tf.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            OpaqueFunction(function=_launch_setup),
        ]
    )
