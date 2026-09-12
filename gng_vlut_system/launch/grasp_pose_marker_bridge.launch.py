from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 未指定値はノードの既定値。launchとros2 runの設定差異の防止
parameter_types = {
    "input_type": str, "input_topic": str, "output_topic": str, "marker_namespace": str,
    "arrow_length": float, "shaft_diameter": float, "head_diameter": float, "head_length": float,
    "color_r": float, "color_g": float, "color_b": float, "color_a": float,
    "anchor": str, "enable_transverse_axes": bool, "helper_axis_length_ratio": float,
    "primary_axis_idx": int, "primary_axis_sign": float, "enable_state_colors": bool,
}


def launch_setup(context):
    parameters = {}
    for name, value_type in parameter_types.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package="gng_vlut_system", executable="grasp_pose_marker_bridge_node",
        name="grasp_pose_marker_bridge_node", output="screen", parameters=[parameters],
    )]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value="", description="空欄はノード既定値")
          for name in parameter_types],
        OpaqueFunction(function=launch_setup),
    ])
