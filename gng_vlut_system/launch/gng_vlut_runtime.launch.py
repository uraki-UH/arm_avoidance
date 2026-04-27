import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
        DeclareLaunchArgument("safety_margin", default_value="0.05", description="安全マージン [m]"),
        
        # --- センサーキャリブレーション（実測値をここに入力） ---
        DeclareLaunchArgument("sensor_x", default_value="0.5"),
        DeclareLaunchArgument("sensor_y", default_value="0.0"),
        DeclareLaunchArgument("sensor_z", default_value="1.0"),
        DeclareLaunchArgument("sensor_roll", default_value="0.0"),
        DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
        DeclareLaunchArgument("sensor_yaw", default_value="0.0"),
        DeclareLaunchArgument("sensor_frame_id", default_value="camera_link", description="バッグファイル等のセンサー座標系名"),

        # 1. ロボットモデルの展開 (Digital Twin / TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()
        ),

        # 2. センサー位置の静的TF配信 (Calibration)
        # ロボット基準(base_frame)からセンサー(sensor_frame_id)への相対位置を固定
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
                "filter_radius": 3.0  # ワークスペース外をカットする場合の半径
            }]
        ),

        # 4. 安全監視・判定ノード (Core Logic)
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
        ),
        
        # 5. 可視化ブリッジ (React Viewer用)
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge",
            parameters=[{
                "robot_name": LaunchConfiguration("robot_name"),
                "base_frame": LaunchConfiguration("base_frame"),
                "stream_port": 9090
            }]
        )
    ])
