import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    experiment_id = LaunchConfiguration("experiment_id").perform(context)
    if not experiment_id:
        experiment_id = robot_name
    
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

    robot_urdf = LaunchConfiguration("robot_description_file", default=robot_desc_default)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_runtime.launch.py")),
            launch_arguments={
                "robot_name": robot_name,
                "experiment_id": experiment_id,
                "robot_description_file": robot_urdf,
                "enable_safety_monitor": LaunchConfiguration("enable_safety_monitor"),
                "joint_state_source": LaunchConfiguration("joint_state_source"),
            }.items()
        ),
        Node(
            package="gng_vlut_system",
            executable="robot_arm_bridge_node",
            name="robot_arm_bridge_node",
            output="screen",
            parameters=[{
                "robot_description_file": robot_urdf,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
                "publish_hz": 20.0,
                "frame_id": LaunchConfiguration("frame_id"),
            }]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("robot_description_file", default_value=""),
        DeclareLaunchArgument("enable_safety_monitor", default_value="false"),
        DeclareLaunchArgument("joint_state_source", default_value="rviz"),
        DeclareLaunchArgument("frame_id", default_value="world"),

        OpaqueFunction(function=launch_setup)
    ])
