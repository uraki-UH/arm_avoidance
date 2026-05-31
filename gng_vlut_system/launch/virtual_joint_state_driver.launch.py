from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def safe_float(value, default):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def safe_bool(value, default):
    try:
        if value is None or value == "":
            return default
        return value.lower() in ("1", "true", "yes", "on")
    except Exception:
        return default


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    target_topic = LaunchConfiguration("target_topic").perform(context)
    state_topic = LaunchConfiguration("state_topic").perform(context)
    output_topic = LaunchConfiguration("output_topic").perform(context)
    publish_hz = LaunchConfiguration("publish_hz").perform(context)
    max_joint_velocity = LaunchConfiguration("max_joint_velocity").perform(context)
    position_tolerance = LaunchConfiguration("position_tolerance").perform(context)
    use_wraparound = LaunchConfiguration("use_wraparound").perform(context)
    hold_when_no_target = LaunchConfiguration("hold_when_no_target").perform(context)

    return [
        Node(
            package="gng_vlut_system",
            executable="virtual_joint_state_driver_node",
            name="virtual_joint_state_driver_node",
            namespace=robot_name,
            output="screen",
            parameters=[{
                "target_topic": target_topic,
                "state_topic": state_topic,
                "output_topic": output_topic,
                "publish_hz": safe_float(publish_hz, 50.0),
                "max_joint_velocity": safe_float(max_joint_velocity, 0.6),
                "position_tolerance": safe_float(position_tolerance, 0.01),
                "use_wraparound": safe_bool(use_wraparound, True),
                "hold_when_no_target": safe_bool(hold_when_no_target, True),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("target_topic", default_value="target_joint_states"),
        DeclareLaunchArgument("state_topic", default_value="joint_states"),
        DeclareLaunchArgument("output_topic", default_value="joint_states"),
        DeclareLaunchArgument("publish_hz", default_value="50.0"),
        DeclareLaunchArgument("max_joint_velocity", default_value="0.6"),
        DeclareLaunchArgument("position_tolerance", default_value="0.01"),
        DeclareLaunchArgument("use_wraparound", default_value="true"),
        DeclareLaunchArgument("hold_when_no_target", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
