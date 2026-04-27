import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

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
        mesh_root = os.path.join(robot_desc_pkg, "meshes")
    except PackageNotFoundError:
        robot_desc_default = os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")
        resource_root = os.path.join(pkg_share, "urdf")
        mesh_root = os.path.join(resource_root, "meshes", "topoarm")

    robot_urdf = LaunchConfiguration("robot_description_file").perform(context) or robot_desc_default

    return [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", robot_urdf])}]
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", robot_urdf])}]
        ),
        Node(
            package="gng_vlut_system",
            executable="robot_description_player_node",
            name="robot_description_player",
            parameters=[{
                "robot_description_file": robot_urdf,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
            }]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("robot_description_file", default_value=""),
        OpaqueFunction(function=launch_setup)
    ])
