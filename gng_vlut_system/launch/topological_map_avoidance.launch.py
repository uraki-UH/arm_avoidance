import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")

    robot_name = LaunchConfiguration("robot_name").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    gng_model_path = LaunchConfiguration("gng_model_path").perform(context)
    robot_urdf = LaunchConfiguration("robot_urdf_path").perform(context)

    node_params = {}
    if robot_urdf:
        node_params["robot_urdf_path"] = robot_urdf
    if gng_model_path:
        node_params["gng_model_path"] = gng_model_path

    final_params_list = []
    if params_file and os.path.exists(params_file):
        final_params_list.append(params_file)
    final_params_list.append(node_params)

    return [
        Node(
            package="gng_vlut_system",
            executable="topological_map_avoidance_node",
            name="topological_map_avoidance_node",
            namespace=robot_name,
            output="screen",
            parameters=final_params_list,
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm",
                              description="ロボットの名前空間"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml"),
                              description="設定YAMLファイル"),
        DeclareLaunchArgument("robot_urdf_path",
                              default_value="package://topoarm_description/urdf/topo_dual_arm.urdf.xacro",
                              description="ロボットURDF/Xacroのパス"),
        DeclareLaunchArgument("gng_model_path",
                              default_value="",
                              description="読み込むGNGモデル(gng.bin)のパス"),
        OpaqueFunction(function=launch_setup),
    ])
