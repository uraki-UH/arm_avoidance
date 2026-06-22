import os
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


def resolve_robot_description(robot_name: str) -> tuple[str, str, str]:
    robot_desc_pkg = get_package_share_directory(f"{robot_name}_description")
    candidates = [
        os.path.join(robot_desc_pkg, "urdf", f"{robot_name}.urdf.xacro"),
        os.path.join(robot_desc_pkg, "urdf", f"{robot_name}_pro_normal.urdf.xacro"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate, robot_desc_pkg, os.path.join(robot_desc_pkg, "meshes")

    raise FileNotFoundError(
        f"No URDF/Xacro found for robot_name='{robot_name}'. "
        f"Checked: {', '.join(candidates)}"
    )


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    enable_joint_state_publisher = LaunchConfiguration("enable_joint_state_publisher").perform(context).lower() in ("true", "1", "yes", "on")
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context).strip()

    robot_desc_default, resource_root, mesh_root = resolve_robot_description(robot_name)

    robot_urdf_raw = LaunchConfiguration("urdf_path").perform(context)
    robot_urdf = resolve_package_uri(robot_urdf_raw) if robot_urdf_raw else robot_desc_default
    if not os.path.exists(robot_urdf):
        raise FileNotFoundError(
            f"Robot description file does not exist: {robot_urdf}. "
            "Pass urdf_path explicitly or install the matching description package."
        )

    xacro_cmd = ["xacro ", robot_urdf]

    nodes = []

    if enable_joint_state_publisher:
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=robot_name,
                parameters=[{"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}],
                remappings=[("joint_states", joint_state_topic or f"/{robot_name}/joint_states")],
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
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
            }]
        )
    ])

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("joint_state_topic", default_value=""),
        OpaqueFunction(function=launch_setup)
    ])
