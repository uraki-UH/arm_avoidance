import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    except Exception:
        robot_desc_default = os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")
        resource_root = os.path.join(pkg_share, "urdf")
        mesh_root = os.path.join(resource_root, "meshes", "topoarm")

    return [
        # 1. GNG Bridge (Topofuzzy)
        Node(
            package="gng_vlut_system",
            executable="topofuzzy_bridge_node",
            name="topofuzzy_bridge_node",
            parameters=[{
                "input_topic": "/gng_status",
                "stream_topic": "/viewer/internal/stream/gng",
            }]
        ),

        # 2. Robot Viewer Bridge
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge_node",
            parameters=[{
                "robot_description_file": robot_desc_default,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
                "joint_state_topic": "/joint_states",
                "stream_topic": "/viewer/internal/stream/robot",
                "publish_hz": 20.0,
            }]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        OpaqueFunction(function=launch_setup)
    ])
