from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/topological_map/merged",
    )
    output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/topological_grid_voxels",
    )
    summary_topic = DeclareLaunchArgument(
        "summary_topic",
        default_value="/topological_grid_voxels/summary",
    )
    grid_size = DeclareLaunchArgument(
        "grid_size",
        default_value="0.5",
    )
    origin_x = DeclareLaunchArgument(
        "origin_x",
        default_value="0.0",
    )
    origin_y = DeclareLaunchArgument(
        "origin_y",
        default_value="0.0",
    )
    origin_z = DeclareLaunchArgument(
        "origin_z",
        default_value="0.0",
    )
    origin_shift_half = DeclareLaunchArgument(
        "origin_shift_half",
        default_value="false",
    )
    x_shift = DeclareLaunchArgument(
        "x_shift",
        default_value="42",
    )
    y_shift = DeclareLaunchArgument(
        "y_shift",
        default_value="21",
    )
    z_shift = DeclareLaunchArgument(
        "z_shift",
        default_value="0",
    )
    offset = DeclareLaunchArgument(
        "offset",
        default_value="1000000",
    )

    return LaunchDescription([
        input_topic,
        output_topic,
        summary_topic,
        grid_size,
        origin_x,
        origin_y,
        origin_z,
        origin_shift_half,
        x_shift,
        y_shift,
        z_shift,
        offset,
        Node(
            package="ais_gng",
            executable="topological_grid_node",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "summary_topic": LaunchConfiguration("summary_topic"),
                "grid_size": LaunchConfiguration("grid_size"),
                "origin_x": LaunchConfiguration("origin_x"),
                "origin_y": LaunchConfiguration("origin_y"),
                "origin_z": LaunchConfiguration("origin_z"),
                "origin_shift_half": LaunchConfiguration("origin_shift_half"),
                "x_shift": LaunchConfiguration("x_shift"),
                "y_shift": LaunchConfiguration("y_shift"),
                "z_shift": LaunchConfiguration("z_shift"),
                "offset": LaunchConfiguration("offset"),
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "INFO"],
        ),
    ])
