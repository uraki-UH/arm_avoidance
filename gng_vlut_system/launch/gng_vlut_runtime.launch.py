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
    experiment_id = LaunchConfiguration("id").perform(context)
    if not experiment_id:
        experiment_id = LaunchConfiguration("experiment_id").perform(context)
    if not experiment_id:
        experiment_id = robot_name

    data_dir = LaunchConfiguration("dir").perform(context)
    if not data_dir:
        data_dir = LaunchConfiguration("data_directory").perform(context)
    if not os.path.isabs(data_dir):
        data_dir = os.path.join(pkg_share, data_dir)

    gng_path = os.path.join(data_dir, experiment_id, "gng.bin")
    vlut_path = os.path.join(data_dir, experiment_id, "vlut.bin")
    enable_safety_monitor = LaunchConfiguration("enable_safety_monitor").perform(context).lower() in ("true", "1", "yes", "on")

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

    return [
        # 1. ロボットモデルの展開 (Digital Twin / TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
                "joint_state_topic": LaunchConfiguration("joint_state_sim_topic"),
            }.items()
        ),

        # 2. センサー位置の静的TF配信 (Calibration)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_calibration_publisher',
            arguments=[
                LaunchConfiguration("sensor_x"), LaunchConfiguration("sensor_y"), LaunchConfiguration("sensor_z"),
                LaunchConfiguration("sensor_yaw"), LaunchConfiguration("sensor_pitch"), LaunchConfiguration("sensor_roll"),
                LaunchConfiguration("base_frame"), LaunchConfiguration("sensor_frame_id")
            ]
        ),

        # 3. GNG 座標変換ノード (Sensor -> Base Frame 高速変換)
        Node(
            package="pointcloud_transformer_cpp",
            executable="gng_transformer_node_cpp",
            name="gng_transformer",
            parameters=[{
                "target_frame": LaunchConfiguration("base_frame"),
                "input_topic": "/gng_map",
                "output_topic": "/topological_map_transformed",
                "filter_radius": 3.0
            }]
        ),

        # 4b. JointState 中継 (sim / real / test -> /joint_states)
        Node(
            package="gng_vlut_system",
            executable="joint_state_mux_node",
            name="joint_state_mux",
            parameters=[{
                "sim_topic": LaunchConfiguration("joint_state_sim_topic"),
                "real_topic": LaunchConfiguration("joint_state_real_topic"),
                "test_topic": LaunchConfiguration("joint_state_test_topic"),
                "output_topic": LaunchConfiguration("joint_state_output_topic"),
                "active_source": LaunchConfiguration("joint_state_mux_mode"),
                "max_age_sec": LaunchConfiguration("joint_state_max_age_sec"),
            }]
        ),

        # 5. 安全監視・判定ノード (Core Logic)
        *(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_monitor.launch.py")),
                    launch_arguments={
                        "robot_name": LaunchConfiguration("robot_name"),
                        "id": LaunchConfiguration("id"),
                        "experiment_id": LaunchConfiguration("experiment_id"),
                        "dir": LaunchConfiguration("dir"),
                        "data_directory": LaunchConfiguration("data_directory"),
                        "frame_id": LaunchConfiguration("base_frame"),
                        "safety_margin": LaunchConfiguration("safety_margin"),
                        "tag": LaunchConfiguration("tag"),
                        "mode": LaunchConfiguration("mode"),
                    }.items()
                )
            ]
            if enable_safety_monitor and os.path.exists(gng_path) and os.path.exists(vlut_path)
            else []
        ),

        # 6. 可視化ブリッジ (React Viewer用)
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge",
            parameters=[{
                "robot_description_file": robot_desc_default,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
                "joint_state_topic": LaunchConfiguration("joint_state_output_topic"),
                "stream_topic": "/viewer/internal/stream/robot",
                "publish_hz": 20.0,
            }]
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        # --- 基本設定 ---
        DeclareLaunchArgument("robot_name", default_value="topoarm", description="ロボット名（URDF等の検索に使用）"),
        DeclareLaunchArgument("id", default_value="", description="VLUTデータセットのID"),
        DeclareLaunchArgument("experiment_id", default_value="", description="(deprecated) VLUTデータセットのID"),
        DeclareLaunchArgument("dir", default_value="gng_results", description="GNG/VLUT データの置き場所"),
        DeclareLaunchArgument("data_directory", default_value="", description="(deprecated) GNG/VLUT データの置き場所"),
        DeclareLaunchArgument("base_frame", default_value="base_link", description="ロボットの基準座標系"),
        DeclareLaunchArgument("tag", default_value="dynamic", description="GNG layer tag"),
        DeclareLaunchArgument("mode", default_value="dynamic", description="GNG layer mode"),
        
        # --- 安全設定 ---
        DeclareLaunchArgument("enable_safety_monitor", default_value="true"),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("joint_state_sim_topic", default_value="/joint_states_sim"),
        DeclareLaunchArgument("joint_state_real_topic", default_value="/joint_states_real"),
        DeclareLaunchArgument("joint_state_test_topic", default_value="/joint_states_test"),
        DeclareLaunchArgument("joint_state_output_topic", default_value="/joint_states"),
        DeclareLaunchArgument("joint_state_mux_mode", default_value="priority"),
        DeclareLaunchArgument("joint_state_max_age_sec", default_value="0.5"),
        DeclareLaunchArgument("safety_margin", default_value="0.05", description="安全マージン [m]"),
        
        # --- センサーキャリブレーション（実測値をここに入力） ---
        DeclareLaunchArgument("sensor_x", default_value="0.5"),
        DeclareLaunchArgument("sensor_y", default_value="0.0"),
        DeclareLaunchArgument("sensor_z", default_value="1.0"),
        DeclareLaunchArgument("sensor_roll", default_value="0.0"),
        DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
        DeclareLaunchArgument("sensor_yaw", default_value="0.0"),
        DeclareLaunchArgument("sensor_frame_id", default_value="camera_link", description="バッグファイル等のセンサー座標系名"),
        OpaqueFunction(function=launch_setup)
    ])
