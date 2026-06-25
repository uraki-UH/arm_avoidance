from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("input_topic", default_value="/topo_points"),
        DeclareLaunchArgument("voxel_topic", default_value="/topo_voxel_ids"),
        DeclareLaunchArgument("source_frame_id", default_value=""),
        DeclareLaunchArgument("target_frame_id", default_value="ToPoDualArm/base_link"),
        DeclareLaunchArgument("voxel_size", default_value="0.02"),
        DeclareLaunchArgument("x_shift", default_value="42"),
        DeclareLaunchArgument("y_shift", default_value="21"),
        DeclareLaunchArgument("z_shift", default_value="0"),
        DeclareLaunchArgument("offset", default_value="1000000"),
        DeclareLaunchArgument("danger_inflation", default_value="0.05"),
        DeclareLaunchArgument("output_voxel_size", default_value="0.02"),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),

        Node(
            package="gng_vlut_system",
            executable="pointcloud_voxel_bridge_node",
            name="pointcloud_voxel_bridge_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("voxel_topic"),
                "source_frame_id": LaunchConfiguration("source_frame_id"),
                "target_frame_id": LaunchConfiguration("target_frame_id"),
                "voxel_size": LaunchConfiguration("voxel_size"),
                "x_shift": LaunchConfiguration("x_shift"),
                "y_shift": LaunchConfiguration("y_shift"),
                "z_shift": LaunchConfiguration("z_shift"),
                "offset": LaunchConfiguration("offset"),
            }],
        ),

        Node(
            package="gng_vlut_system",
            executable="voxel_to_vlut_bridge_node",
            name="voxel_to_vlut_bridge_node",
            namespace=LaunchConfiguration("robot_name"),
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("voxel_topic"),
                "occupied_voxels_topic": "occupied_voxels",
                "danger_voxels_topic": "danger_voxels",
                "target_frame_id": LaunchConfiguration("target_frame_id"),
                "danger_inflation": LaunchConfiguration("danger_inflation"),
                "output_voxel_size": LaunchConfiguration("output_voxel_size"),
                "publish_hz": LaunchConfiguration("publish_hz"),
            }],
        ),
    ])
