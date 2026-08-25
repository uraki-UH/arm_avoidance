import os
import re
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_package_uri(raw_path: str) -> str:
    if not raw_path.startswith("package://"):
        return raw_path

    pkg_and_path = raw_path[len("package://"):]
    pkg_name, _, rel_path = pkg_and_path.partition("/")
    if not pkg_name or not rel_path:
        return raw_path

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except Exception:
        return raw_path
    return os.path.join(pkg_share, rel_path)


def load_root_params(params_file: str) -> dict:
    if not params_file or not os.path.exists(params_file):
        return {}
    try:
        with open(params_file, "r", encoding="utf-8") as f:
            params_yaml = yaml.safe_load(f) or {}
    except Exception:
        return {}

    for root_key in ("/**", "ros__parameters"):
        candidate = params_yaml.get(root_key, {})
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate.get("ros__parameters", {})
        if isinstance(candidate, dict):
            return candidate
    return {}


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


def gazebo_material_name(rgba: str) -> str | None:
    try:
        red, green, blue, _ = (float(value) for value in rgba.split())
    except ValueError:
        return None

    palette = {
        "Gazebo/Black": (0.0, 0.0, 0.0),
        "Gazebo/DarkGrey": (0.175, 0.175, 0.175),
        "Gazebo/Grey": (0.7, 0.7, 0.7),
        "Gazebo/White": (1.0, 1.0, 1.0),
        "Gazebo/Red": (1.0, 0.0, 0.0),
        "Gazebo/Green": (0.0, 1.0, 0.0),
        "Gazebo/Blue": (0.0, 0.0, 1.0),
    }
    return min(
        palette,
        key=lambda name: sum((value - target) ** 2 for value, target in zip((red, green, blue), palette[name])),
    )


def parse_vector(raw_value: str, value_name: str) -> tuple[float, float, float]:
    values = raw_value.replace(",", " ").split()
    if len(values) != 3:
        raise ValueError(f"{value_name}には3個の数値が必要です: '{raw_value}'")
    try:
        return tuple(float(value) for value in values)
    except ValueError as error:
        raise ValueError(f"{value_name}に数値以外が含まれています: '{raw_value}'") from error


def add_lidar_sensor(
    root: ET.Element,
    parent_link: str,
    lidar_link: str,
    lidar_xyz: str,
    lidar_rpy: str,
    lidar_topic: str,
    lidar_frame_id: str,
    lidar_update_hz: float,
    num_lidar_horizontal_samples: int,
    num_lidar_vertical_samples: int,
    min_lidar_horizontal_angle: float,
    max_lidar_horizontal_angle: float,
    min_lidar_vertical_angle: float,
    max_lidar_vertical_angle: float,
    min_lidar_range: float,
    max_lidar_range: float,
) -> None:
    link_names = {link.get("name") for link in root.findall("link")}
    if parent_link not in link_names:
        raise ValueError(f"LiDARの親link '{parent_link}'がURDFにありません")
    if lidar_link in link_names:
        raise ValueError(f"LiDAR用link '{lidar_link}'がURDF内で重複しています")
    if num_lidar_horizontal_samples < 1 or num_lidar_vertical_samples < 1:
        raise ValueError("LiDARのsamplesは1以上が必要です")
    if min_lidar_range < 0.0 or max_lidar_range <= min_lidar_range:
        raise ValueError("LiDARのrange設定が不正です")
    if max_lidar_horizontal_angle <= min_lidar_horizontal_angle:
        raise ValueError("LiDARの水平angle設定が不正です")
    if max_lidar_vertical_angle < min_lidar_vertical_angle:
        raise ValueError("LiDARの垂直angle設定が不正です")
    if lidar_update_hz <= 0.0:
        raise ValueError("lidar_update_hzは正数が必要です")
    if not lidar_topic:
        raise ValueError("lidar_topicが空です")

    lidar_xyz_values = parse_vector(lidar_xyz, "lidar_xyz")
    lidar_rpy_values = parse_vector(lidar_rpy, "lidar_rpy")
    normalized_xyz = " ".join(str(value) for value in lidar_xyz_values)
    normalized_rpy = " ".join(str(value) for value in lidar_rpy_values)

    link = ET.SubElement(root, "link", name=lidar_link)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value="0.01")
    ET.SubElement(
        inertial,
        "inertia",
        ixx="1e-5",
        ixy="0",
        ixz="0",
        iyy="1e-5",
        iyz="0",
        izz="1e-5",
    )
    visual = ET.SubElement(link, "visual")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "cylinder", radius="0.04", length="0.06")
    material = ET.SubElement(visual, "material", name="lidar_black")
    ET.SubElement(material, "color", rgba="0.05 0.05 0.05 1")

    joint = ET.SubElement(root, "joint", name=f"{parent_link}_to_{lidar_link}", type="fixed")
    ET.SubElement(joint, "parent", link=parent_link)
    ET.SubElement(joint, "child", link=lidar_link)
    ET.SubElement(joint, "origin", xyz=normalized_xyz, rpy=normalized_rpy)

    gazebo = ET.SubElement(root, "gazebo", reference=lidar_link)
    sensor = ET.SubElement(gazebo, "sensor", name="lidar_sensor", type="ray")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "visualize").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(lidar_update_hz)
    ray = ET.SubElement(sensor, "ray")
    scan = ET.SubElement(ray, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    ET.SubElement(horizontal, "samples").text = str(num_lidar_horizontal_samples)
    ET.SubElement(horizontal, "resolution").text = "1"
    ET.SubElement(horizontal, "min_angle").text = str(min_lidar_horizontal_angle)
    ET.SubElement(horizontal, "max_angle").text = str(max_lidar_horizontal_angle)
    vertical = ET.SubElement(scan, "vertical")
    ET.SubElement(vertical, "samples").text = str(num_lidar_vertical_samples)
    ET.SubElement(vertical, "resolution").text = "1"
    ET.SubElement(vertical, "min_angle").text = str(min_lidar_vertical_angle)
    ET.SubElement(vertical, "max_angle").text = str(max_lidar_vertical_angle)
    lidar_range = ET.SubElement(ray, "range")
    ET.SubElement(lidar_range, "min").text = str(min_lidar_range)
    ET.SubElement(lidar_range, "max").text = str(max_lidar_range)
    ET.SubElement(lidar_range, "resolution").text = "0.01"
    noise = ET.SubElement(ray, "noise")
    ET.SubElement(noise, "type").text = "gaussian"
    ET.SubElement(noise, "mean").text = "0.0"
    ET.SubElement(noise, "stddev").text = "0.005"

    plugin = ET.SubElement(
        sensor,
        "plugin",
        name="lidar_ros_pointcloud",
        filename="libgazebo_ros_ray_sensor.so",
    )
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "remapping").text = f"~/out:={lidar_topic}"
    ET.SubElement(plugin, "output_type").text = "sensor_msgs/PointCloud2"
    ET.SubElement(plugin, "frame_name").text = lidar_frame_id


def write_gazebo_urdf(
    robot_urdf: str,
    mesh_root_dir: str,
    static_model: bool,
    fixed_base_link: str,
    lidar_config: dict | None = None,
) -> str:
    with open(robot_urdf, "r", encoding="utf-8") as f:
        urdf_text = f.read()

    mesh_root = mesh_root_dir or os.path.join(os.path.dirname(robot_urdf), "meshes")
    mesh_root = os.path.abspath(mesh_root)
    uri_prefix = "file://" + mesh_root.rstrip("/") + "/"
    urdf_text = re.sub(
        r"filename=([\"'])meshes/",
        lambda match: "filename=" + match.group(1) + uri_prefix,
        urdf_text,
    )

    root = ET.fromstring(urdf_text)
    if lidar_config:
        add_lidar_sensor(root, **lidar_config)
    if static_model:
        gazebo = ET.SubElement(root, "gazebo")
        ET.SubElement(gazebo, "static").text = "true"
    elif fixed_base_link:
        link_names = {link.get("name") for link in root.findall("link")}
        if fixed_base_link not in link_names:
            raise ValueError(
                f"fixed_base_link '{fixed_base_link}' is not defined in {robot_urdf}"
            )
        if "world" in link_names:
            raise ValueError(
                "Cannot inject a fixed Gazebo base because the URDF already defines a 'world' link"
            )

        # Keep the model dynamic while anchoring its base in Gazebo's world.
        # A model-level <static> flag would also freeze the arm joints.
        ET.SubElement(root, "link", name="world")
        joint = ET.SubElement(root, "joint", name="world_to_robot_base", type="fixed")
        ET.SubElement(joint, "parent", link="world")
        ET.SubElement(joint, "child", link=fixed_base_link)
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")

    joints = root.findall("joint")
    fixed_children = {
        joint.find("child").get("link")
        for joint in joints
        if joint.get("type") == "fixed" and joint.find("child") is not None
    }
    movable_parents = {
        joint.find("parent").get("link")
        for joint in joints
        if joint.get("type") != "fixed" and joint.find("parent") is not None
    }

    # Gazebo Classic discards a massless fixed-link junction and its movable subtree.
    for link in root.findall("link"):
        if (
            link.get("name") in fixed_children & movable_parents
            and link.find("inertial") is None
        ):
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass", value="0.001")
            ET.SubElement(
                inertial,
                "inertia",
                ixx="1e-6",
                ixy="0",
                ixz="0",
                iyy="1e-6",
                iyz="0",
                izz="1e-6",
            )

        color = link.find("./visual/material/color")
        rgba = color.get("rgba") if color is not None else None
        material_name = gazebo_material_name(rgba) if rgba else None
        if material_name:
            gazebo = ET.SubElement(root, "gazebo", reference=link.get("name"))
            ET.SubElement(gazebo, "material").text = material_name

    urdf_text = ET.tostring(root, encoding="unicode")

    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", suffix=".urdf", prefix="gazebo_robot_", delete=False
    ) as f:
        f.write(urdf_text)
        return f.name


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    follow_tf_frame = LaunchConfiguration("follow_tf_frame").perform(context).strip()
    follow_tf_reference_frame = LaunchConfiguration("follow_tf_ref").perform(context).strip() or "world"
    legacy_reference_frame = LaunchConfiguration("follow_tf_reference_frame").perform(context).strip()
    if legacy_reference_frame:
        follow_tf_reference_frame = legacy_reference_frame
    follow_tf_update_hz = float(LaunchConfiguration("follow_tf_update_hz").perform(context))
    follow_tf_service_name = LaunchConfiguration("follow_tf_service_name").perform(context).strip() or "/gazebo/set_entity_state"
    static_model = LaunchConfiguration("static_model").perform(context).lower() in ("true", "1", "yes", "on")
    fixed_base_link = LaunchConfiguration("fixed_base_link").perform(context).strip()
    enable_lidar = LaunchConfiguration("enable_lidar").perform(context).lower() in ("true", "1", "yes", "on")
    mesh_root_dir = LaunchConfiguration("mesh_root_dir").perform(context).strip()
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    root_params = load_root_params(params_file)
    robot_urdf_raw = LaunchConfiguration("urdf_path").perform(context) or str(root_params.get("urdf_path", "")).strip()
    if not robot_urdf_raw:
        raise FileNotFoundError(
            "No robot description path was provided. "
            "Set urdf_path in the params file or pass urdf_path explicitly."
        )
    robot_urdf = resolve_package_uri(robot_urdf_raw)
    if not os.path.exists(robot_urdf):
        raise FileNotFoundError(f"Robot description file does not exist: {robot_urdf}")

    mesh_root_dir = mesh_root_dir or str(root_params.get("mesh_root_dir", "")).strip()
    static_model = static_model or str(root_params.get("static_model", "")).lower() in ("true", "1", "yes", "on")
    fixed_base_link = str(root_params.get("fixed_base_link", fixed_base_link)).strip()
    if follow_tf_frame:
        static_model = False

    lidar_config = None
    lidar_tf_config = None
    if enable_lidar:
        lidar_params_file = LaunchConfiguration("lidar_params_file").perform(context).strip()
        all_lidar_params = load_lidar_params(lidar_params_file)
        robot_lidar_params = get_lidar_section(all_lidar_params, "robot_lidar")
        scan_params = get_lidar_section(all_lidar_params, "scan")
        lidar_parent_link = resolve_lidar_value(
            context, "lidar_parent_link", robot_lidar_params, "parent_link", "camera_link"
        )
        lidar_link = resolve_lidar_value(context, "lidar_link", robot_lidar_params, "link", "lidar_link")
        lidar_xyz = resolve_lidar_value(context, "lidar_xyz", robot_lidar_params, "xyz", "0 0 0.08")
        lidar_rpy = resolve_lidar_value(context, "lidar_rpy", robot_lidar_params, "rpy", "0 0 0")
        lidar_frame_id = resolve_lidar_value(
            context, "lidar_frame_id", robot_lidar_params, "frame_id", ""
        )
        if not lidar_frame_id:
            lidar_frame_id = f"{robot_name}/{lidar_link}"
        lidar_config = {
            "parent_link": lidar_parent_link,
            "lidar_link": lidar_link,
            "lidar_xyz": lidar_xyz,
            "lidar_rpy": lidar_rpy,
            "lidar_topic": resolve_lidar_value(
                context, "lidar_topic", robot_lidar_params, "topic", "/lidar/points"
            ),
            "lidar_frame_id": lidar_frame_id,
            "lidar_update_hz": float(resolve_lidar_value(
                context, "lidar_update_hz", scan_params, "update_hz", 10.0
            )),
            "num_lidar_horizontal_samples": int(resolve_lidar_value(
                context, "num_lidar_horizontal_samples", scan_params, "num_horizontal_samples", 360
            )),
            "num_lidar_vertical_samples": int(resolve_lidar_value(
                context, "num_lidar_vertical_samples", scan_params, "num_vertical_samples", 16
            )),
            "min_lidar_horizontal_angle": float(resolve_lidar_value(
                context, "min_lidar_horizontal_angle", scan_params, "min_horizontal_angle", -3.14159265
            )),
            "max_lidar_horizontal_angle": float(resolve_lidar_value(
                context, "max_lidar_horizontal_angle", scan_params, "max_horizontal_angle", 3.14159265
            )),
            "min_lidar_vertical_angle": float(resolve_lidar_value(
                context, "min_lidar_vertical_angle", scan_params, "min_vertical_angle", -0.261799
            )),
            "max_lidar_vertical_angle": float(resolve_lidar_value(
                context, "max_lidar_vertical_angle", scan_params, "max_vertical_angle", 0.261799
            )),
            "min_lidar_range": float(resolve_lidar_value(
                context, "min_lidar_range", scan_params, "min_range", 0.1
            )),
            "max_lidar_range": float(resolve_lidar_value(
                context, "max_lidar_range", scan_params, "max_range", 20.0
            )),
        }
        lidar_tf_config = {
            "parent_frame_id": f"{robot_name}/{lidar_parent_link}",
            "child_frame_id": lidar_frame_id,
            "xyz": parse_vector(lidar_xyz, "lidar_xyz"),
            "rpy": parse_vector(lidar_rpy, "lidar_rpy"),
        }
    gazebo_urdf = write_gazebo_urdf(
        robot_urdf,
        mesh_root_dir,
        static_model,
        fixed_base_link,
        lidar_config,
    )

    def cleanup_gazebo_urdf(_context, *unused_args, **unused_kwargs):
        try:
            os.unlink(gazebo_urdf)
        except FileNotFoundError:
            pass
        return []

    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", gazebo_urdf,
            "-entity", robot_name,
            "-x", "0", "-y", "0", "-z", spawn_z,
        ],
        output="screen",
    )
    nodes = [
        spawn_node,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_node, on_exit=[OpaqueFunction(function=cleanup_gazebo_urdf)])
        ),
    ]
    if follow_tf_frame:
        nodes.append(
            Node(
                package="gng_vlut_system",
                executable="gazebo_entity_tf_follower_node",
                name="gazebo_entity_tf_follower_node",
                parameters=[{
                    "entity_name": robot_name,
                    "source_frame": follow_tf_frame,
                    "reference_frame": follow_tf_reference_frame,
                    "service_name": follow_tf_service_name,
                    "update_hz": follow_tf_update_hz,
                }],
                output="screen",
            )
        )
    if lidar_tf_config:
        x, y, z = lidar_tf_config["xyz"]
        roll, pitch, yaw = lidar_tf_config["rpy"]
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_static_transform_publisher",
                arguments=[
                    "--x", str(x), "--y", str(y), "--z", str(z),
                    "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                    "--frame-id", lidar_tf_config["parent_frame_id"],
                    "--child-frame-id", lidar_tf_config["child_frame_id"],
                ],
                output="screen",
            )
        )
    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("mesh_root_dir", default_value=""),
        DeclareLaunchArgument("spawn_z", default_value="0.5"),
        DeclareLaunchArgument("static_model", default_value="false"),
        DeclareLaunchArgument("fixed_base_link", default_value=""),
        DeclareLaunchArgument("follow_tf_frame", default_value=""),
        DeclareLaunchArgument("follow_tf_ref", default_value="world"),
        DeclareLaunchArgument("follow_tf_reference_frame", default_value=""),
        DeclareLaunchArgument("follow_tf_update_hz", default_value="20.0"),
        DeclareLaunchArgument("follow_tf_service_name", default_value="/gazebo/set_entity_state"),
        DeclareLaunchArgument("enable_lidar", default_value="false"),
        DeclareLaunchArgument(
            "lidar_params_file", default_value=os.path.join(pkg_share, "config", "gazebo_lidar.yaml")
        ),
        DeclareLaunchArgument("lidar_parent_link", default_value=""),
        DeclareLaunchArgument("lidar_link", default_value=""),
        DeclareLaunchArgument("lidar_xyz", default_value=""),
        DeclareLaunchArgument("lidar_rpy", default_value=""),
        DeclareLaunchArgument("lidar_topic", default_value=""),
        DeclareLaunchArgument("lidar_frame_id", default_value=""),
        DeclareLaunchArgument("lidar_update_hz", default_value=""),
        DeclareLaunchArgument("num_lidar_horizontal_samples", default_value=""),
        DeclareLaunchArgument("num_lidar_vertical_samples", default_value=""),
        DeclareLaunchArgument("min_lidar_horizontal_angle", default_value=""),
        DeclareLaunchArgument("max_lidar_horizontal_angle", default_value=""),
        DeclareLaunchArgument("min_lidar_vertical_angle", default_value=""),
        DeclareLaunchArgument("max_lidar_vertical_angle", default_value=""),
        DeclareLaunchArgument("min_lidar_range", default_value=""),
        DeclareLaunchArgument("max_lidar_range", default_value=""),
        # Gazebo起動済み、または別途起動する構成の前提
        OpaqueFunction(function=launch_setup)
    ])
