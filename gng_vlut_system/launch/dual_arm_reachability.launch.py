"""左右到達セルmapの独立表示。経路計画用GNGとの分離。"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    model_dir = Path(LaunchConfiguration('model_dir').perform(context))
    namespace = LaunchConfiguration('namespace').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    nodes = []
    for side in ('left_arm', 'right_arm'):
        path = model_dir / (side + '.bin')
        if not path.is_file():
            raise FileNotFoundError(path)
        nodes.append(Node(
            package='gng_vlut_system', executable='visualization_gng_static_node',
            name='reachability_' + side, namespace=namespace,
            parameters=[{'model_path': str(path), 'topic_name': 'reachability_' + side + '_Tmap',
                         'frame_id': frame_id}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_dir'),
        DeclareLaunchArgument('namespace', default_value='topo_dual_arm_max'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        OpaqueFunction(function=create_nodes),
    ])
