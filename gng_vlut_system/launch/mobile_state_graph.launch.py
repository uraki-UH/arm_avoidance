"""機体YAMLを利用する移動状態グラフの事前生成・表示。ロボット召喚は別起動。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    share = Path(get_package_share_directory('gng_vlut_system'))
    params_path = Path(LaunchConfiguration('params_file').perform(context))
    if not params_path.is_file() and params_path.parent == Path('.'):
        params_path = share/'config'/params_path
    params = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']
    output_path = LaunchConfiguration('output_path').perform(context)
    if not output_path:
        output_path = str(share/params['gng']['data_directory']/params['gng']['experiment_id']/'motion_graph.json')
    overrides = {'output_path': output_path}
    for name in ('build_only', 'enable_rebuild'):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(name+'はtrueまたはfalseが必要です')
        overrides[name] = value == 'true'
    return [Node(package='gng_vlut_system', executable='mobile_state_graph_node.py',
                 namespace=params['robot_name'], output='screen', parameters=[str(params_path), overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(Path(
            get_package_share_directory('gng_vlut_system'))/'config/fuzzbot.yaml')),
        DeclareLaunchArgument('output_path', default_value=''),
        DeclareLaunchArgument('build_only', default_value='false'),
        DeclareLaunchArgument('enable_rebuild', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
