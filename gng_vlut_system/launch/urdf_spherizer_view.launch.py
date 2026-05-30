import os
import re
import shlex
import shutil
import subprocess
import copy
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PACKAGE_URI_RE = re.compile(r"package://([A-Za-z0-9_]+)/")


def _extract_root_params(params_file: str) -> dict:
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


def _resolve_package_uri(raw_path: str) -> str:
    if not raw_path.startswith("package://"):
        return raw_path
    pkg_and_path = raw_path[len("package://") :]
    pkg_name, _, rel_path = pkg_and_path.partition("/")
    if not pkg_name or not rel_path:
        return raw_path
    try:
        pkg_share = get_package_share_directory(pkg_name)
    except Exception:
        return raw_path
    return os.path.join(pkg_share, rel_path)


def _pick_robot_description(root_params: dict) -> str:
    candidates = [
        root_params.get("robot_description_file", ""),
        root_params.get("robot_urdf_path", ""),
    ]
    for candidate in candidates:
        if candidate:
            return candidate

    try:
        topoarm_pkg = get_package_share_directory("topoarm_description")
        default = os.path.join(topoarm_pkg, "urdf", "topo_dual_arm.urdf.xacro")
        if os.path.exists(default):
            return default
    except Exception:
        pass

    return "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro"


def _collect_package_names(text: str) -> set[str]:
    return set(_PACKAGE_URI_RE.findall(text or ""))


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except Exception:
        return ""


def _prepare_expanded_urdf(source_path: Path, workdir: Path) -> Path:
    expanded_path = workdir / f"{source_path.stem}.expanded.urdf"
    if source_path.suffix.lower() == ".xacro":
        result = subprocess.run(
            ["xacro", str(source_path)],
            check=True,
            capture_output=True,
            text=True,
        )
        expanded_path.write_text(result.stdout, encoding="utf-8")
    else:
        shutil.copy2(source_path, expanded_path)
    return expanded_path


def _build_package_dir_args(package_names: set[str]) -> list[str]:
    args: list[str] = []
    for pkg_name in sorted(package_names):
        try:
            pkg_share = get_package_share_directory(pkg_name)
        except Exception:
            continue
        args.extend(["--package-dir", f"{pkg_name}={pkg_share}"])
    return args


def _resolve_spherizer_executable(spherizer_cmd: str) -> str:
    candidate = shlex.split(spherizer_cmd)[0] if spherizer_cmd else "urdf-spherizer"
    resolved = shutil.which(candidate)
    if resolved:
        return resolved

    home = Path.home()
    fallback = home / ".local" / "bin" / candidate
    if fallback.exists() and os.access(fallback, os.X_OK):
        return str(fallback)

    return candidate


def _geometry_kind(geometry_elem: ET.Element | None) -> str:
    if geometry_elem is None:
        return "none"
    if geometry_elem.find("mesh") is not None:
        return "mesh"
    if geometry_elem.find("box") is not None:
        return "box"
    if geometry_elem.find("cylinder") is not None:
        return "cylinder"
    if geometry_elem.find("sphere") is not None:
        return "sphere"
    return "other"


def _link_is_primitive_only(link_elem: ET.Element) -> bool:
    has_geometry = False
    for section_name in ("visual", "collision"):
        for section in link_elem.findall(section_name):
            geometry = section.find("geometry")
            kind = _geometry_kind(geometry)
            if kind == "mesh":
                return False
            if kind != "none":
                has_geometry = True
    return has_geometry


def _clone_collision_from_source(source_link: ET.Element) -> list[ET.Element]:
    collisions = source_link.findall("collision")
    if collisions:
        return [copy.deepcopy(elem) for elem in collisions]

    visuals = source_link.findall("visual")
    cloned: list[ET.Element] = []
    for idx, visual in enumerate(visuals):
        collision = ET.Element("collision", visual.attrib if visual.attrib else {})
        origin = visual.find("origin")
        geometry = visual.find("geometry")
        if origin is not None:
            collision.append(copy.deepcopy(origin))
        if geometry is not None:
            collision.append(copy.deepcopy(geometry))
        else:
            continue
        cloned.append(collision)
    return cloned


def _restore_primitive_link_collisions(original_urdf: Path, spheres_urdf: Path) -> list[str]:
    original_tree = ET.parse(original_urdf)
    original_root = original_tree.getroot()
    output_tree = ET.parse(spheres_urdf)
    output_root = output_tree.getroot()

    output_links = {link.attrib.get("name", ""): link for link in output_root.findall("link")}
    restored: list[str] = []

    for original_link in original_root.findall("link"):
        link_name = original_link.attrib.get("name", "")
        if not link_name or not _link_is_primitive_only(original_link):
            continue
        output_link = output_links.get(link_name)
        if output_link is None:
            continue

        for collision in list(output_link.findall("collision")):
            output_link.remove(collision)

        cloned_collisions = _clone_collision_from_source(original_link)
        for collision in cloned_collisions:
            output_link.append(collision)
        if cloned_collisions:
            restored.append(link_name)

    ET.indent(output_tree, space="  ")
    output_tree.write(spheres_urdf, encoding="utf-8", xml_declaration=True)
    return restored


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    root_params = _extract_root_params(params_file)

    yaml_robot_name = root_params.get("robot_name", "topoarm")
    yaml_robot_description = _pick_robot_description(root_params)
    yaml_joint_state_topic = root_params.get("joint_state_topic", "")
    yaml_stream_topic = root_params.get("stream_topic", "/viewer/internal/stream/robot")
    yaml_frame_id = root_params.get("frame_id", "base_link")
    yaml_publish_hz = root_params.get("publish_hz", 30.0)
    yaml_resource_root_dir = root_params.get("resource_root_dir", "")
    yaml_mesh_root_dir = root_params.get("mesh_root_dir", "")
    yaml_eef_link_names = root_params.get("robot.eef_link_names", "")
    yaml_arm_leaf_link_names = root_params.get("robot.arm_leaf_link_names", "")

    robot_name = LaunchConfiguration("robot_name").perform(context) or yaml_robot_name
    robot_description_file = (
        LaunchConfiguration("robot_description_file").perform(context)
        or yaml_robot_description
    )
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context)
    if not joint_state_topic:
        joint_state_topic = yaml_joint_state_topic or f"/{robot_name}/joint_states"
        if joint_state_topic == "joint_states":
            joint_state_topic = f"/{robot_name}/joint_states"
    stream_topic = LaunchConfiguration("stream_topic").perform(context) or yaml_stream_topic
    frame_id = LaunchConfiguration("frame_id").perform(context) or yaml_frame_id
    publish_hz = LaunchConfiguration("publish_hz").perform(context) or str(yaml_publish_hz)
    spherizer_cmd = LaunchConfiguration("spherizer_cmd").perform(context) or "urdf-spherizer"
    output_spheres_urdf = LaunchConfiguration("output_spheres_urdf").perform(context)
    max_spheres = int(LaunchConfiguration("max_spheres").perform(context) or "64")
    min_gain_ratio = float(LaunchConfiguration("min_gain_ratio").perform(context) or "0.02")
    spherizer_margin = float(LaunchConfiguration("spherizer_margin").perform(context) or "0.0")
    epsilon = float(LaunchConfiguration("spherizer_epsilon").perform(context) or "1e-6")

    resolved_source = Path(_resolve_package_uri(robot_description_file)).expanduser().resolve()
    if not resolved_source.exists():
        raise FileNotFoundError(f"robot_description_file not found: {resolved_source}")

    workdir = Path(tempfile.mkdtemp(prefix=f"urdf_spherizer_{robot_name}_", dir="/tmp"))
    expanded_urdf = _prepare_expanded_urdf(resolved_source, workdir)

    package_names = _collect_package_names(_read_text(resolved_source))
    package_names |= _collect_package_names(_read_text(expanded_urdf))
    package_dir_args = _build_package_dir_args(package_names)

    spheres_output = (
        Path(output_spheres_urdf).expanduser().resolve()
        if output_spheres_urdf
        else workdir / f"{resolved_source.stem}.spheres.urdf"
    )

    cmd = shlex.split(_resolve_spherizer_executable(spherizer_cmd))
    cmd.extend(
        [
            str(expanded_urdf),
            "-o",
            str(spheres_output),
            "--max-spheres",
            str(max_spheres),
            "--min-gain-ratio",
            str(min_gain_ratio),
            "--margin",
            str(spherizer_margin),
            "--epsilon",
            str(epsilon),
        ]
    )
    cmd.extend(package_dir_args)

    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"Failed to execute spherizer command: {cmd[0]!r}. "
            "Install the upstream urdf-spherizer tool first."
        ) from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"urdf-spherizer failed with exit code {exc.returncode}: {' '.join(cmd)}"
        ) from exc

    restored_links = _restore_primitive_link_collisions(expanded_urdf, spheres_output)
    if restored_links:
        print(
            "[urdf_spherizer_view] restored primitive links without spherizing: "
            + ", ".join(restored_links)
        )

    common_params = {
        "robot_name": robot_name,
        "robot_description_file": str(spheres_output),
        "joint_state_topic": joint_state_topic,
        "stream_topic": stream_topic,
        "frame_id": frame_id,
        "publish_hz": float(publish_hz),
    }
    if yaml_resource_root_dir:
        common_params["resource_root_dir"] = yaml_resource_root_dir
    if yaml_mesh_root_dir:
        common_params["mesh_root_dir"] = yaml_mesh_root_dir
    if yaml_eef_link_names:
        common_params["robot.eef_link_names"] = yaml_eef_link_names
    if yaml_arm_leaf_link_names:
        common_params["robot.arm_leaf_link_names"] = yaml_arm_leaf_link_names

    print(
        "[urdf_spherizer_view] "
        f"params_file={params_file} robot_name={robot_name} "
        f"joint_state_topic={joint_state_topic} "
        f"robot_description_file={robot_description_file} "
        f"spheres_output={spheres_output}"
    )

    return [
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge_node",
            namespace=robot_name,
            parameters=[params_file, common_params],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml"),
                description="ロボット設定YAML",
            ),
            DeclareLaunchArgument("robot_name", default_value="", description="ロボット名"),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value="",
                description="入力URDF/Xacroへのパス",
            ),
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="",
                description="購読する joint_states トピック名",
            ),
            DeclareLaunchArgument(
                "stream_topic",
                default_value="",
                description="Viewerへ流す内部ストリームトピック",
            ),
            DeclareLaunchArgument("frame_id", default_value="", description="基準フレーム"),
            DeclareLaunchArgument("publish_hz", default_value="", description="描画更新周期"),
            DeclareLaunchArgument(
                "spherizer_cmd",
                default_value="urdf-spherizer",
                description="本家 urdf-spherizer 実行コマンド",
            ),
            DeclareLaunchArgument(
                "output_spheres_urdf",
                default_value="",
                description="生成した spheres URDF の出力先",
            ),
            DeclareLaunchArgument(
                "max_spheres",
                default_value="16",
                description="球化時の最大球数",
            ),
            DeclareLaunchArgument(
                "min_gain_ratio",
                default_value="0.1",
                description="球分割を止める相対閾値",
            ),
            DeclareLaunchArgument(
                "spherizer_margin",
                default_value="0.0",
                description="球化時の絶対閾値[m]",
            ),
            DeclareLaunchArgument(
                "spherizer_epsilon",
                default_value="1e-6",
                description="数値安定化用の膨張",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
