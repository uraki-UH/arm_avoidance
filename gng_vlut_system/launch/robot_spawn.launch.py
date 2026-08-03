import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    enable_joint_state_publisher = LaunchConfiguration("enable_joint_state_publisher").perform(context).lower() in ("true", "1", "yes", "on")
    publish_initial_joint_state = LaunchConfiguration("publish_initial_joint_state").perform(context).lower() in ("true", "1", "yes", "on")
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context).strip()
    robot_description_topic = LaunchConfiguration("robot_description_topic").perform(context).strip() or "robot_description"
    if not robot_description_topic.startswith("/"):
        robot_description_topic = f"/{robot_name}/{robot_description_topic}"
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    root_params = load_root_params(params_file)
    resource_root_dir = LaunchConfiguration("resource_root_dir").perform(context).strip() or str(root_params.get("resource_root_dir", "")).strip()
    mesh_root_dir = LaunchConfiguration("mesh_root_dir").perform(context).strip() or str(root_params.get("mesh_root_dir", "")).strip()

    robot_urdf_raw = LaunchConfiguration("urdf_path").perform(context) or str(root_params.get("urdf_path", "")).strip()
    if not robot_urdf_raw:
        raise FileNotFoundError(
            "No robot description path was provided. "
            "Set urdf_path in the params file or pass urdf_path explicitly."
        )

    robot_urdf = resolve_package_uri(robot_urdf_raw)
    if not os.path.exists(robot_urdf):
        raise FileNotFoundError(
            f"Robot description file does not exist: {robot_urdf}. "
            "Pass a valid urdf_path or install the matching package."
        )

    xacro_cmd = ["xacro ", robot_urdf]

    nodes = []

    if enable_joint_state_publisher:
        joint_state_params = {
            "robot_description": ParameterValue(Command(xacro_cmd), value_type=str),
        }
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=robot_name,
                parameters=[joint_state_params],
                remappings=[
                    ("joint_states", joint_state_topic or f"/{robot_name}/joint_states"),
                    ("robot_description", robot_description_topic),
                ],
            )
        )

    if publish_initial_joint_state:
        nodes.append(
            Node(
                package="gng_vlut_system",
                executable="initial_joint_state_publisher_node",
                name="initial_joint_state_publisher_node",
                namespace=robot_name,
                parameters=[{
                    "robot_description": ParameterValue(Command(xacro_cmd), value_type=str),
                    "joint_state_topic": joint_state_topic or f"/{robot_name}/joint_states",
                }],
            )
        )

    nodes.extend([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_name,
            parameters=[{
                "robot_description": ParameterValue(Command(xacro_cmd), value_type=str),
                "frame_prefix": robot_name + "/"
            }],
            remappings=[
                ("joint_states", joint_state_topic or f"/{robot_name}/joint_states"),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ]
        ),
        Node(
            package="gng_vlut_system",
            executable="robot_description_player_node",
            name="robot_description_player",
            namespace=robot_name,
            parameters=[{
                "urdf_path": robot_urdf,
                "resource_root_dir": resource_root_dir,
                "mesh_root_dir": mesh_root_dir,
                "topic_name": robot_description_topic,
            }]
        )
    ])

    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("resource_root_dir", default_value=""),
        DeclareLaunchArgument("mesh_root_dir", default_value=""),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("publish_initial_joint_state", default_value="false"),
        DeclareLaunchArgument("joint_state_topic", default_value=""),
        DeclareLaunchArgument("robot_description_topic", default_value="robot_description"),
        OpaqueFunction(function=launch_setup)
    ])
