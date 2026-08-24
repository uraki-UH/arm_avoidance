import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """左グリッパの把持ボクセルテンプレート照合。"""
    package_share = get_package_share_directory("grasping_system")
    graph_launch = os.path.join(package_share, "launch", "gripper_volume_graph.launch.py")
    gripper_definitions = os.path.join(
        package_share, "config", "ToPoDualArm_gripper_volumes.yaml"
    )

    object_voxels_topic = LaunchConfiguration("object_voxels_topic")
    candidate_topic = LaunchConfiguration("candidate_topic")
    candidate_voxels_topic = LaunchConfiguration("candidate_voxels_topic")
    summary_topic = LaunchConfiguration("summary_topic")
    enable_depth_visibility = LaunchConfiguration("enable_depth_visibility")

    return LaunchDescription([
        DeclareLaunchArgument("object_voxels_topic", default_value="/topo_voxel_ids"),
        DeclareLaunchArgument("candidate_topic", default_value="/grasp_pose_cands"),
        DeclareLaunchArgument(
            "candidate_voxels_topic", default_value="/grasp_pose_cand_cells"
        ),
        DeclareLaunchArgument(
            "summary_topic", default_value="/grasp_pose_cands/summary"
        ),
        DeclareLaunchArgument("enable_depth_visibility", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(graph_launch),
            launch_arguments={"grippers_file": gripper_definitions}.items(),
        ),
        Node(
            package="grasping_system",
            executable="grasp_voxel_matcher_node",
            name="left_grasp_voxel_matcher",
            output="screen",
            parameters=[{
                "object_voxels_topic": object_voxels_topic,
                "required_graph_topic": "/ToPoDualArm/L_grip_V_topological_map",
                "undersize_graph_topic": "/ToPoDualArm/L_grip_minV_topological_map",
                # 基部と開閉中の指を含む掃引グラフ
                # 把持対象との接触を許容する初期開口内部 `grip_V` の衝突判定除外
                "forbidden_graph_topic": "/ToPoDualArm/L_grip_sweptV_topological_map",
                "candidate_topic": candidate_topic,
                "candidate_voxels_topic": candidate_voxels_topic,
                "summary_topic": summary_topic,
                "enable_depth_visibility": enable_depth_visibility,
            }],
        ),
        Node(
            package="gng_vlut_system",
            executable="grasp_pose_marker_bridge_node",
            name="grasp_pose_marker_bridge_node",
            output="screen",
            parameters=[{
                "input_topic": candidate_topic,
                "output_topic": "/grasp_pose_markers",
            }],
        ),
    ])
