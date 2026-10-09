"""MID-360実機のPointCloud2・IMU出力と任意の取付TF。"""

import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def read_config(path):
    config = yaml.safe_load(Path(path).read_text())
    for key in ("host_ip", "lidar_ip"):
        addr = ipaddress.IPv4Address(config[key])
        if addr.is_unspecified or addr.is_multicast or addr.is_loopback:
            raise ValueError(f"{key}: 実機接続用IPv4を指定してください")
    if config["host_ip"] == config["lidar_ip"]:
        raise ValueError("PCとセンサに異なるIPを指定してください")
    freq = float(config["publish_freq"])
    if not math.isfinite(freq) or not 5.0 <= freq <= 100.0:
        raise ValueError("publish_freq: 5〜100 Hzを指定してください")
    config["publish_freq"] = freq
    if type(config["enable_mount_tf"]) is not bool:
        raise ValueError("enable_mount_tf: trueまたはfalseを指定してください")
    for key in ("pos", "rot_deg"):
        if len(config[key]) != 3 or not all(math.isfinite(float(v)) for v in config[key]):
            raise ValueError(f"{key}: 有限値3個を指定してください")
    for key in ("frame_id", "parent_frame_id"):
        if not isinstance(config[key], str) or not config[key] or config[key].startswith("/") or any(c.isspace() for c in config[key]):
            raise ValueError(f"{key}: 空白・先頭スラッシュなしのTF名を指定してください")
    if config["enable_mount_tf"] and config["frame_id"] == config["parent_frame_id"]:
        raise ValueError("親と子のTF名が同一です")
    from rclpy.validate_full_topic_name import validate_full_topic_name
    for key in ("points_topic", "imu_topic"):
        validate_full_topic_name(config[key])
    if config["points_topic"] == config["imu_topic"]:
        raise ValueError("点群とIMUに異なるトピック名を指定してください")
    return config


def sdk_config(config):
    ports = {"cmd_data": 56100, "push_msg": 56200, "point_data": 56300,
             "imu_data": 56400, "log_data": 56500}
    host = {}
    for name, port in ports.items():
        host[name + "_ip"] = "" if name == "log_data" else config["host_ip"]
        host[name + "_port"] = port + 1
    return {
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {"lidar_net_info": {name + "_port": port for name, port in ports.items()},
                   "host_net_info": host},
        "lidar_configs": [{
            "ip": config["lidar_ip"], "pcl_data_type": 1, "pattern_mode": 0,
            # TF側への外部変換集約。SDK側の二重変換防止
            "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0},
        }],
    }


def start(context):
    config = read_config(LaunchConfiguration("config").perform(context))
    # 指定IPのホスト割当確認。NIC設定の変更なし
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((config["host_ip"], 0))
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as tmp:
        json.dump(sdk_config(config), tmp)
        config_path = tmp.name

    def cleanup(context, *args, **kwargs):
        Path(config_path).unlink(missing_ok=True)
        return []

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node",
             name="mid360_driver", output="screen", parameters=[{
                 "xfer_format": 0, "multi_topic": 0, "data_src": 0,
                 "publish_freq": config["publish_freq"], "output_data_type": 0,
                 "frame_id": config["frame_id"], "user_config_path": config_path,
                 "use_sim_time": False,
             }], remappings=[("/livox/lidar", config["points_topic"]),
                              ("/livox/imu", config["imu_topic"])]),
    ]
    if config["enable_mount_tf"]:
        roll, pitch, yaw = [math.radians(float(v)) for v in config["rot_deg"]]
        x, y, z = config["pos"]
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="mid360_mount_tf", arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                "--frame-id", config["parent_frame_id"], "--child-frame-id", config["frame_id"],
            ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.environ.get("MID360_CONFIG", "/config/mid360.yaml")),
        OpaqueFunction(function=start),
    ])
