import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    data_dir = LaunchConfiguration("dir").perform(context)
    exp_id = LaunchConfiguration("id").perform(context)
    gng_model_path = LaunchConfiguration("gng_model_path").perform(context)
    vlut_path = LaunchConfiguration("vlut_path").perform(context)
    robot_base_frame = LaunchConfiguration("robot_base_frame").perform(context)
    gng_frame_id = LaunchConfiguration("gng_frame_id").perform(context)
    gng_source_frame_id = LaunchConfiguration("gng_source_frame_id").perform(context)
    publish_hz = float(LaunchConfiguration("publish_hz").perform(context))
    topic_name = LaunchConfiguration("topic_name").perform(context)
    
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

    def resolve_result_path(path: str, default_filename: str) -> str:
        if path:
            if os.path.isabs(path):
                return path
            if path.startswith("gng_results/") or "/" in path:
                return os.path.join(pkg_share, path)
        filename = path or default_filename
        return os.path.join(pkg_share, data_dir, exp_id, filename)

    return [
        # 1. GNG Bridge (Topofuzzy)
        Node(
            package="gng_vlut_system",
            executable="topofuzzy_bridge_node",
            name="topofuzzy_bridge_node",
            parameters=[{
                "gng_model_path": resolve_result_path(gng_model_path, "gng.bin"),
                "vlut_path": resolve_result_path(vlut_path, "vlut.bin"),
                "data_directory": data_dir,
                "experiment_id": exp_id,
                "frame_id": gng_frame_id,
                "source_frame_id": gng_source_frame_id,
                "publish_hz": publish_hz,
                "topic_name": topic_name,
            }]
        ),

        # 2. Robot Viewer Bridge
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge_node",
            parameters=[{
                "robot_name": robot_name,
                "robot_description_file": robot_desc_default,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
                "joint_state_topic": "/joint_states",
                "stream_topic": "/viewer/internal/stream/robot",
                "frame_id": robot_base_frame,
                "publish_hz": publish_hz,
            }]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("dir", default_value="gng_results"),
        DeclareLaunchArgument("id", default_value="topoarm"),
        DeclareLaunchArgument("gng_model_path", default_value=""),
        DeclareLaunchArgument("vlut_path", default_value=""),
        DeclareLaunchArgument("robot_base_frame", default_value="base_link"),
        DeclareLaunchArgument("gng_frame_id", default_value="world"),
        DeclareLaunchArgument("gng_source_frame_id", default_value="world"),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),
        DeclareLaunchArgument("topic_name", default_value="/topological_map_static"),
        OpaqueFunction(function=launch_setup)
    ])
