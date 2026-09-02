import os
import tempfile
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parse_vector(raw_value: str, value_name: str) -> tuple[float, float, float]:
    values = raw_value.replace(",", " ").split()
    if len(values) != 3:
        raise ValueError(f"{value_name}には3個の数値が必要です: '{raw_value}'")
    try:
        return tuple(float(value) for value in values)
    except ValueError as error:
        raise ValueError(f"{value_name}に数値以外が含まれています: '{raw_value}'") from error


def load_lidar_params(params_file: str) -> dict:
    if not params_file or not os.path.exists(params_file):
        raise FileNotFoundError(f"LiDAR設定ファイルがありません: {params_file}")
    with open(params_file, "r", encoding="utf-8") as params_stream:
        params = yaml.safe_load(params_stream) or {}
    if not isinstance(params, dict):
        raise ValueError(f"LiDAR設定のrootがmappingではありません: {params_file}")
    return params


def get_lidar_section(params: dict, section_name: str) -> dict:
    section = params.get(section_name, {})
    if not isinstance(section, dict):
        raise ValueError(f"LiDAR設定の'{section_name}'がmappingではありません")
    return section


def resolve_lidar_value(context, argument_name: str, params: dict, key: str, fallback) -> str:
    launch_value = LaunchConfiguration(argument_name).perform(context).strip()
    value = launch_value if launch_value else params.get(key, fallback)
    if isinstance(value, (list, tuple)):
        return " ".join(str(item) for item in value)
    return str(value).strip()


def write_world_lidar_sdf(
    model_name: str,
    frame_id: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    topic: str,
    update_hz: float,
    num_horizontal_samples: int,
    num_vertical_samples: int,
    min_horizontal_angle: float,
    max_horizontal_angle: float,
    min_vertical_angle: float,
    max_vertical_angle: float,
    min_range: float,
    max_range: float,
    noise_mean: float,
    noise_std_dev: float,
) -> str:
    if not model_name or not frame_id or not topic:
        raise ValueError("環境LiDARのmodel、frame、topicには空でない値が必要です")
    if num_horizontal_samples < 1 or num_vertical_samples < 1:
        raise ValueError("環境LiDARのsamplesは1以上が必要です")
    if update_hz <= 0.0:
        raise ValueError("lidar_update_hzは正数が必要です")
    if min_range < 0.0 or max_range <= min_range:
        raise ValueError("環境LiDARのrange設定が不正です")
    if max_horizontal_angle <= min_horizontal_angle:
        raise ValueError("環境LiDARの水平angle設定が不正です")
    if max_vertical_angle < min_vertical_angle:
        raise ValueError("環境LiDARの垂直angle設定が不正です")
    if noise_std_dev < 0.0:
        raise ValueError("lidar_noise_std_devは0以上が必要です")

    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=model_name)
    ET.SubElement(model, "static").text = "true"
    pose_values = (*xyz, *rpy)
    ET.SubElement(model, "pose").text = " ".join(str(value) for value in pose_values)

    link = ET.SubElement(model, "link", name="world_lidar_link")
    sensor = ET.SubElement(link, "sensor", name="world_lidar_sensor", type="ray")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "visualize").text = "false"
    ET.SubElement(sensor, "update_rate").text = str(update_hz)
    ray = ET.SubElement(sensor, "ray")
    scan = ET.SubElement(ray, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    ET.SubElement(horizontal, "samples").text = str(num_horizontal_samples)
    ET.SubElement(horizontal, "resolution").text = "1"
    ET.SubElement(horizontal, "min_angle").text = str(min_horizontal_angle)
    ET.SubElement(horizontal, "max_angle").text = str(max_horizontal_angle)
    vertical = ET.SubElement(scan, "vertical")
    ET.SubElement(vertical, "samples").text = str(num_vertical_samples)
    ET.SubElement(vertical, "resolution").text = "1"
    ET.SubElement(vertical, "min_angle").text = str(min_vertical_angle)
    ET.SubElement(vertical, "max_angle").text = str(max_vertical_angle)
    lidar_range = ET.SubElement(ray, "range")
    ET.SubElement(lidar_range, "min").text = str(min_range)
    ET.SubElement(lidar_range, "max").text = str(max_range)
    ET.SubElement(lidar_range, "resolution").text = "0.01"
    noise = ET.SubElement(ray, "noise")
    ET.SubElement(noise, "type").text = "gaussian"
    ET.SubElement(noise, "mean").text = str(noise_mean)
    ET.SubElement(noise, "stddev").text = str(noise_std_dev)

    plugin = ET.SubElement(
        sensor,
        "plugin",
        name="world_lidar_ros_pointcloud",
        filename="libgazebo_ros_ray_sensor.so",
    )
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "remapping").text = f"~/out:={topic}"
    ET.SubElement(plugin, "output_type").text = "sensor_msgs/PointCloud2"
    ET.SubElement(plugin, "frame_name").text = frame_id

    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", suffix=".sdf", prefix="world_lidar_", delete=False
    ) as sdf_file:
        sdf_file.write(ET.tostring(sdf, encoding="unicode"))
        return sdf_file.name


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    lidar_params_file = LaunchConfiguration("lidar_params_file").perform(context).strip()
    all_lidar_params = load_lidar_params(lidar_params_file)
    world_lidar_params = get_lidar_section(all_lidar_params, "world_lidar")
    scan_params = get_lidar_section(all_lidar_params, "scan")
    model_name = resolve_lidar_value(
        context, "world_lidar_model_name", world_lidar_params, "model_name", "environment_lidar"
    )
    frame_id = resolve_lidar_value(
        context, "world_lidar_frame_id", world_lidar_params, "frame_id", "world_lidar_link"
    )
    xyz = parse_vector(
        resolve_lidar_value(context, "world_lidar_xyz", world_lidar_params, "xyz", "1.5 0 1.2"),
        "world_lidar_xyz",
    )
    rpy = parse_vector(
        resolve_lidar_value(context, "world_lidar_rpy", world_lidar_params, "rpy", "0 0 0"),
        "world_lidar_rpy",
    )
    sdf_path = write_world_lidar_sdf(
        model_name=model_name,
        frame_id=frame_id,
        xyz=xyz,
        rpy=rpy,
        topic=resolve_lidar_value(
            context, "world_lidar_topic", world_lidar_params, "topic", "/lidar/points"
        ),
        update_hz=float(resolve_lidar_value(
            context, "lidar_update_hz", scan_params, "update_hz", 10.0
        )),
        num_horizontal_samples=int(resolve_lidar_value(
            context, "num_lidar_horizontal_samples", scan_params, "num_horizontal_samples", 360
        )),
        num_vertical_samples=int(resolve_lidar_value(
            context, "num_lidar_vertical_samples", scan_params, "num_vertical_samples", 16
        )),
        min_horizontal_angle=float(resolve_lidar_value(
            context, "min_lidar_horizontal_angle", scan_params, "min_horizontal_angle", -3.14159265
        )),
        max_horizontal_angle=float(resolve_lidar_value(
            context, "max_lidar_horizontal_angle", scan_params, "max_horizontal_angle", 3.14159265
        )),
        min_vertical_angle=float(resolve_lidar_value(
            context, "min_lidar_vertical_angle", scan_params, "min_vertical_angle", -0.261799
        )),
        max_vertical_angle=float(resolve_lidar_value(
            context, "max_lidar_vertical_angle", scan_params, "max_vertical_angle", 0.261799
        )),
        min_range=float(resolve_lidar_value(
            context, "min_lidar_range", scan_params, "min_range", 0.1
        )),
        max_range=float(resolve_lidar_value(
            context, "max_lidar_range", scan_params, "max_range", 20.0
        )),
        noise_mean=float(resolve_lidar_value(
            context, "lidar_noise_mean", scan_params, "noise_mean", 0.0
        )),
        noise_std_dev=float(resolve_lidar_value(
            context, "lidar_noise_std_dev", scan_params, "noise_std_dev", 0.005
        )),
    )

    def cleanup_sdf(_context, *unused_args, **unused_kwargs):
        try:
            os.unlink(sdf_path)
        except FileNotFoundError:
            pass
        return []

    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", sdf_path, "-entity", model_name],
        output="screen",
    )
    x, y, z = xyz
    roll, pitch, yaw = rpy
    return [
        spawn_node,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_node, on_exit=[OpaqueFunction(function=cleanup_sdf)])
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_lidar_static_transform_publisher",
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                "--frame-id", "world",
                "--child-frame-id", frame_id,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_params_file", default_value=os.path.join(pkg_share, "config", "gazebo_lidar.yaml")
        ),
        DeclareLaunchArgument("world_lidar_model_name", default_value=""),
        DeclareLaunchArgument("world_lidar_frame_id", default_value=""),
        DeclareLaunchArgument("world_lidar_xyz", default_value=""),
        DeclareLaunchArgument("world_lidar_rpy", default_value=""),
        DeclareLaunchArgument("world_lidar_topic", default_value=""),
        DeclareLaunchArgument("lidar_update_hz", default_value=""),
        DeclareLaunchArgument("num_lidar_horizontal_samples", default_value=""),
        DeclareLaunchArgument("num_lidar_vertical_samples", default_value=""),
        DeclareLaunchArgument("min_lidar_horizontal_angle", default_value=""),
        DeclareLaunchArgument("max_lidar_horizontal_angle", default_value=""),
        DeclareLaunchArgument("min_lidar_vertical_angle", default_value=""),
        DeclareLaunchArgument("max_lidar_vertical_angle", default_value=""),
        DeclareLaunchArgument("min_lidar_range", default_value=""),
        DeclareLaunchArgument("max_lidar_range", default_value=""),
        DeclareLaunchArgument("lidar_noise_mean", default_value=""),
        DeclareLaunchArgument("lidar_noise_std_dev", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
