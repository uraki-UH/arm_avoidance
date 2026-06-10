import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
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


def pick_robot_description(robot_name: str, fallback_pkg_share: str) -> str:
    candidates = []
    try:
        topoarm_pkg = get_package_share_directory("topoarm_description")
        candidates.extend([
            os.path.join(topoarm_pkg, "urdf", "topo_dual_arm.urdf.xacro"),
            os.path.join(topoarm_pkg, "urdf", "topoarm.urdf.xacro"),
        ])
    except Exception:
        pass

    try:
        robot_desc_pkg = get_package_share_directory(f"{robot_name}_description")
        candidates.extend([
            os.path.join(robot_desc_pkg, "urdf", f"{robot_name}.urdf.xacro"),
            os.path.join(robot_desc_pkg, "urdf", f"{robot_name}_pro_normal.urdf.xacro"),
        ])
    except Exception:
        pass

    candidates.append(os.path.join(fallback_pkg_share, "urdf", "topoarm_description", "urdf", "topoarm_dual.urdf.xacro"))
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    return candidates[-1]


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    enable_joint_state_publisher = LaunchConfiguration("enable_joint_state_publisher").perform(context).lower() in ("true", "1", "yes", "on")
    
    # Auto-detect robot description package
    robot_desc_default = pick_robot_description(robot_name, pkg_share)
    try:
        if "topoarm_description" in robot_desc_default:
            robot_desc_pkg = get_package_share_directory("topoarm_description")
        else:
            robot_desc_pkg = get_package_share_directory(f"{robot_name}_description")
        resource_root = robot_desc_pkg
        mesh_root = os.path.join(robot_desc_pkg, "meshes")
    except PackageNotFoundError:
        resource_root = os.path.join(pkg_share, "urdf")
        mesh_root = os.path.join(resource_root, "meshes", "topoarm")

    robot_urdf_raw = LaunchConfiguration("urdf_path").perform(context)
    robot_urdf = resolve_package_uri(robot_urdf_raw) if robot_urdf_raw else robot_desc_default

    xacro_cmd = ["xacro ", robot_urdf]

    nodes = []

    if enable_joint_state_publisher:
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=robot_name,
                parameters=[{"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)}],
                remappings=[
                    ("joint_states", f"/{robot_name}/joint_states"),
                ],
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
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="true"),
        OpaqueFunction(function=launch_setup)
    ])
