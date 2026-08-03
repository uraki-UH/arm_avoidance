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


def write_gazebo_urdf(robot_urdf: str, mesh_root_dir: str) -> str:
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
    gazebo_urdf = write_gazebo_urdf(robot_urdf, mesh_root_dir)

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
    return [
        spawn_node,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_node, on_exit=[OpaqueFunction(function=cleanup_gazebo_urdf)])
        ),
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("mesh_root_dir", default_value=""),
        DeclareLaunchArgument("spawn_z", default_value="0.5"),
        # We assume Gazebo is already running or started separately
        OpaqueFunction(function=launch_setup)
    ])
