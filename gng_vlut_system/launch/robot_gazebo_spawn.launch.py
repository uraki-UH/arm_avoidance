import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _resolve_urdf_for_gazebo(robot_urdf_path, resource_root_dir, output_name):
    """
    Gazebo often struggles with package:// syntax in URDFs depending on version.
    This resolves package:// to absolute file:// paths in a temporary file.
    """
    if not os.path.exists(robot_urdf_path):
        return None
        
    text = Path(robot_urdf_path).read_text(encoding="utf-8")
    replacement = "file://" + resource_root_dir.rstrip("/") + "/"
    resolved_text = text
    pos = 0
    while True:
        pos = resolved_text.find("package://", pos)
        if pos == -1:
            break
        subpath_start = resolved_text.find("/", pos + len("package://"))
        if subpath_start == -1:
            break
        resolved_text = (
            resolved_text[:pos]
            + replacement
            + resolved_text[subpath_start + 1 :]
        )
        pos += len(replacement)
    
    tmp_path = Path("/tmp") / output_name
    tmp_path.write_text(resolved_text, encoding="utf-8")
    return str(tmp_path)

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    
    # Auto-detect robot description package
    try:
        robot_desc_pkg = get_package_share_directory(f"{robot_name}_description")
        potential_urdf = os.path.join(robot_desc_pkg, "urdf", f"{robot_name}.urdf.xacro")
        if not os.path.exists(potential_urdf):
            potential_urdf = os.path.join(robot_desc_pkg, "urdf", f"{robot_name}_pro_normal.urdf.xacro")
        
        robot_desc_default = potential_urdf
        resource_root = robot_desc_pkg
    except PackageNotFoundError:
        robot_desc_default = os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")
        resource_root = os.path.join(pkg_share, "urdf")

    # Resolve URDF for Gazebo
    resolved_urdf = _resolve_urdf_for_gazebo(
        robot_desc_default, 
        resource_root, 
        f"resolved_{robot_name}.urdf"
    )

    return [
        # Spawn Entity in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "robot_description",
                "-entity", robot_name,
                "-x", "0", "-y", "0", "-z", "0"
            ],
            output="screen",
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        # We assume Gazebo is already running or started separately
        OpaqueFunction(function=launch_setup)
    ])
