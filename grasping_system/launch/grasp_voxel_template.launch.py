import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run the left-gripper voxel-template POC against topological-grid output."""
    package_share = get_package_share_directory("grasping_system")
    graph_launch = os.path.join(package_share, "launch", "gripper_volume_graph.launch.py")
    gripper_definitions = os.path.join(
        package_share, "config", "ToPoDualArm_gripper_volumes.yaml"
    )

    object_voxels_topic = LaunchConfiguration("object_voxels_topic")
    candidate_topic = LaunchConfiguration("candidate_topic")
    summary_topic = LaunchConfiguration("summary_topic")

    return LaunchDescription([
        DeclareLaunchArgument("object_voxels_topic", default_value="/topo_voxel_ids"),
        DeclareLaunchArgument("candidate_topic", default_value="/grasp_voxel_candidates"),
        DeclareLaunchArgument(
            "summary_topic", default_value="/grasp_voxel_candidates/summary"
        ),
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
                # The swept graph contains the base and open-to-closed fingers.
                # The matcher exempts the initial open interior (grip_V), where
                # contact with the grasped object is expected.
                "forbidden_graph_topic": "/ToPoDualArm/L_grip_sweptV_topological_map",
                "candidate_topic": candidate_topic,
                "summary_topic": summary_topic,
            }],
        ),
    ])
