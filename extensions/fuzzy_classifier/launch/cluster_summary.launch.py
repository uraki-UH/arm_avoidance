from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    include_cluster_details = LaunchConfiguration("include_cluster_details")
    max_detail_clusters = LaunchConfiguration("max_detail_clusters")
    moving_speed_threshold_mps = LaunchConfiguration("moving_speed_threshold_mps")
    log_interval_sec = LaunchConfiguration("log_interval_sec")
    pretty_json = LaunchConfiguration("pretty_json")
    box_topic = LaunchConfiguration("box_topic")
    node_topic = LaunchConfiguration("node_topic")
    publish_cluster_boxes = LaunchConfiguration("publish_cluster_boxes")
    publish_cluster_nodes = LaunchConfiguration("publish_cluster_nodes")
    render_safe_terrain_box = LaunchConfiguration("render_safe_terrain_box")
    render_safe_terrain_node = LaunchConfiguration("render_safe_terrain_node")
    box_alpha = LaunchConfiguration("box_alpha")
    node_alpha = LaunchConfiguration("node_alpha")
    node_scale = LaunchConfiguration("node_scale")
    color_label_source = LaunchConfiguration("color_label_source")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/topological_map",
                description="TopologicalMap input topic",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/fuzzy_classifier/cluster_summary",
                description="Summary JSON output topic (std_msgs/String)",
            ),
            DeclareLaunchArgument(
                "include_cluster_details",
                default_value="true",
                description="Include per-cluster details in output JSON",
            ),
            DeclareLaunchArgument(
                "max_detail_clusters",
                default_value="200",
                description="Maximum number of cluster detail rows",
            ),
            DeclareLaunchArgument(
                "moving_speed_threshold_mps",
                default_value="0.2",
                description="Speed threshold used for moving cluster counting",
            ),
            DeclareLaunchArgument(
                "log_interval_sec",
                default_value="1.0",
                description="Summary log interval in seconds",
            ),
            DeclareLaunchArgument(
                "pretty_json",
                default_value="false",
                description="Publish indented JSON when true",
            ),
            DeclareLaunchArgument(
                "box_topic",
                default_value="/fuzzy_classifier/cluster_boxes",
                description="MarkerArray output topic for cluster boxes",
            ),
            DeclareLaunchArgument(
                "node_topic",
                default_value="/fuzzy_classifier/cluster_nodes",
                description="MarkerArray output topic for cluster nodes",
            ),
            DeclareLaunchArgument(
                "publish_cluster_boxes",
                default_value="true",
                description="Publish RViz cluster box markers",
            ),
            DeclareLaunchArgument(
                "publish_cluster_nodes",
                default_value="false",
                description="Publish RViz cluster node markers",
            ),
            DeclareLaunchArgument(
                "render_safe_terrain_box",
                default_value="false",
                description="Render SAFE_TERRAIN clusters as boxes",
            ),
            DeclareLaunchArgument(
                "render_safe_terrain_node",
                default_value="false",
                description="Render SAFE_TERRAIN clusters as nodes",
            ),
            DeclareLaunchArgument(
                "box_alpha",
                default_value="0.35",
                description="Cluster box marker alpha [0.0-1.0]",
            ),
            DeclareLaunchArgument(
                "node_alpha",
                default_value="0.9",
                description="Cluster node marker alpha [0.0-1.0]",
            ),
            DeclareLaunchArgument(
                "node_scale",
                default_value="0.08",
                description="Cluster node marker diameter in meters",
            ),
            DeclareLaunchArgument(
                "color_label_source",
                default_value="label",
                description="Marker color source: label",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock for ROS time",
            ),
            Node(
                package="fuzzy_classifier",
                executable="cluster_summary_node",
                name="fuzzy_cluster_summary",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "include_cluster_details": include_cluster_details,
                        "max_detail_clusters": max_detail_clusters,
                        "moving_speed_threshold_mps": moving_speed_threshold_mps,
                        "log_interval_sec": log_interval_sec,
                        "pretty_json": pretty_json,
                        "box_topic": box_topic,
                        "node_topic": node_topic,
                        "publish_cluster_boxes": publish_cluster_boxes,
                        "publish_cluster_nodes": publish_cluster_nodes,
                        "render_safe_terrain_box": render_safe_terrain_box,
                        "render_safe_terrain_node": render_safe_terrain_node,
                        "box_alpha": box_alpha,
                        "node_alpha": node_alpha,
                        "node_scale": node_scale,
                        "color_label_source": color_label_source,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
