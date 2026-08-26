from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/topo_points"),
        DeclareLaunchArgument("output_topic", default_value="/topo_voxel_ids"),
        DeclareLaunchArgument("source_frame_id", default_value=""),
        DeclareLaunchArgument("world_frame_id", default_value="world"),
        DeclareLaunchArgument("target_frame_id", default_value="ToPoDualArm/base_link"),
        DeclareLaunchArgument("enable_world_index", default_value="true"),
        DeclareLaunchArgument("enable_roi_query", default_value="true"),
        DeclareLaunchArgument(
            "world_bucket_topic", default_value="/ToPoDualArm/world_index_buckets"),
        DeclareLaunchArgument("enable_world_bucket_publish", default_value="true"),
        DeclareLaunchArgument("voxel_size", default_value="0.02"),
        DeclareLaunchArgument("bucket_size", default_value="0.2"),
        DeclareLaunchArgument("x_shift", default_value="42"),
        DeclareLaunchArgument("y_shift", default_value="21"),
        DeclareLaunchArgument("z_shift", default_value="0"),
        DeclareLaunchArgument("offset", default_value="1000000"),
        DeclareLaunchArgument("enable_reachability_filter", default_value="true"),
        DeclareLaunchArgument("min_reachability_x", default_value="-0.1"),
        DeclareLaunchArgument("max_reachability_x", default_value="0.5"),
        DeclareLaunchArgument("min_reachability_y", default_value="-1.0"),
        DeclareLaunchArgument("max_reachability_y", default_value="1.0"),
        DeclareLaunchArgument("min_reachability_z", default_value="-1.0"),
        DeclareLaunchArgument("max_reachability_z", default_value="1.0"),
        DeclareLaunchArgument("reachability_margin_x", default_value="0.2"),
        DeclareLaunchArgument("reachability_margin_y", default_value="0.2"),
        DeclareLaunchArgument("reachability_margin_z", default_value="0.2"),
        DeclareLaunchArgument("max_dense_voxel_num", default_value="8000000"),
        DeclareLaunchArgument("parallel_thread_num", default_value="1"),
        DeclareLaunchArgument("additional_consumers_json", default_value="[]"),
        Node(
            package="gng_vlut_system",
            executable="world_index_to_voxel_node",
            name="world_index_to_voxel_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "source_frame_id": LaunchConfiguration("source_frame_id"),
                "world_frame_id": LaunchConfiguration("world_frame_id"),
                "target_frame_id": LaunchConfiguration("target_frame_id"),
                "enable_world_index": LaunchConfiguration("enable_world_index"),
                "enable_roi_query": LaunchConfiguration("enable_roi_query"),
                "world_bucket_topic": LaunchConfiguration("world_bucket_topic"),
                "enable_world_bucket_publish": LaunchConfiguration(
                    "enable_world_bucket_publish"),
                "voxel_size": LaunchConfiguration("voxel_size"),
                "bucket_size": LaunchConfiguration("bucket_size"),
                "x_shift": LaunchConfiguration("x_shift"),
                "y_shift": LaunchConfiguration("y_shift"),
                "z_shift": LaunchConfiguration("z_shift"),
                "offset": LaunchConfiguration("offset"),
                "enable_reachability_filter": LaunchConfiguration(
                    "enable_reachability_filter"),
                "min_reachability_x": LaunchConfiguration("min_reachability_x"),
                "max_reachability_x": LaunchConfiguration("max_reachability_x"),
                "min_reachability_y": LaunchConfiguration("min_reachability_y"),
                "max_reachability_y": LaunchConfiguration("max_reachability_y"),
                "min_reachability_z": LaunchConfiguration("min_reachability_z"),
                "max_reachability_z": LaunchConfiguration("max_reachability_z"),
                "reachability_margin_x": LaunchConfiguration("reachability_margin_x"),
                "reachability_margin_y": LaunchConfiguration("reachability_margin_y"),
                "reachability_margin_z": LaunchConfiguration("reachability_margin_z"),
                "max_dense_voxel_num": LaunchConfiguration("max_dense_voxel_num"),
                "parallel_thread_num": LaunchConfiguration("parallel_thread_num"),
                "additional_consumers_json": LaunchConfiguration(
                    "additional_consumers_json"),
            }],
        ),
    ])
