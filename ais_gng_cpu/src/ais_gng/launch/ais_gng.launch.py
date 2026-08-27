import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

package_dir = get_package_share_directory("ais_gng")


def parse_bool(value, argument_name):
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"Invalid boolean for {argument_name}: {value}")


def generate_launch_description():
    declar_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='femto.yaml',
        description='config'
    )
    declar_backend = DeclareLaunchArgument(
        'backend',
        default_value='cpu',
        description='backend to launch: cpu or gpu'
    )
    declar_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='',
        description='入力PointCloud2トピックの上書き'
    )
    declar_plane_params_file = DeclareLaunchArgument(
        'plane_params_file',
        default_value=os.path.join(
            package_dir, 'config', 'plane_cluster_incremental.yaml'),
        description='平面クラスタと可視化の設定ファイル'
    )
    declar_start_plane_cluster = DeclareLaunchArgument(
        'start_plane_cluster',
        default_value='true',
        description=(
            'CPU版では内蔵平面クラスタのマーカー変換、'
            'GPU版では平面クラスタ生成とマーカー変換を起動'
        )
    )
    declar_topological_map_topic = DeclareLaunchArgument(
        'topological_map_topic',
        default_value='/topological_map',
    )
    declar_planar_clusters_topic = DeclareLaunchArgument(
        'planar_clusters_topic',
        default_value='/topological_planar_clusters_incremental',
    )
    declar_plane_clusters_input_topic = DeclareLaunchArgument(
        'plane_clusters_input_topic',
        default_value='auto',
        description=(
            'autoはCPU版で内蔵クラスタ出力を利用。'
            '空文字は独立ノードでクラスタを再計算'
        )
    )

    def launch_setup(context, *args, **kwargs):
        backend = LaunchConfiguration('backend').perform(context)
        if backend not in ('cpu', 'gpu'):
            raise RuntimeError(f"Unsupported backend: {backend}")

        lidar = LaunchConfiguration('lidar').perform(context)
        gng_config_path = os.path.join(package_dir, 'config', f'gng_{backend}', lidar)
        if not os.path.exists(gng_config_path):
            raise RuntimeError(f"Config file not found: {gng_config_path}")

        executable_path = os.path.join(
            get_package_prefix("ais_gng"),
            "lib",
            "ais_gng",
            f"ais_gng_{backend}",
        )
        if not os.path.exists(executable_path):
            raise RuntimeError(f"Backend executable not found: {executable_path}")

        parameters = [gng_config_path]
        input_topic = LaunchConfiguration('input_topic').perform(context)
        if input_topic:
            parameters.append({'input.topic_names': [input_topic]})

        topological_map_topic = LaunchConfiguration(
            'topological_map_topic').perform(context)
        planar_clusters_topic = LaunchConfiguration(
            'planar_clusters_topic').perform(context)
        if backend == 'cpu':
            parameters.append({
                'plane_cluster.output_topic': planar_clusters_topic,
            })

        nodes = [
            Node(
                package="ais_gng",
                executable=f"ais_gng_{backend}",
                parameters=parameters,
                remappings=[('topological_map', topological_map_topic)],
                output='screen',
                # prefix='gdb -ex run -ex bt --args', # for debugging
                # arguments=['--ros-args', '--log-level', 'WARN'] # no screen log
            ),
            Node(
                package='ais_gng',
                executable='object_gng_dataset_exporter_node',
                parameters=[{'map_topic': topological_map_topic}],
                output='screen',
            ),
        ]

        start_plane_cluster = parse_bool(
            LaunchConfiguration('start_plane_cluster').perform(context),
            'start_plane_cluster')
        if start_plane_cluster:
            clusters_input_topic = LaunchConfiguration(
                'plane_clusters_input_topic').perform(context)
            if clusters_input_topic == 'auto':
                clusters_input_topic = (
                    planar_clusters_topic if backend == 'cpu' else '')
            nodes.append(
                Node(
                    package='ais_gng',
                    executable='plane_cluster_incremental_node',
                    name='plane_cluster_visualization_node',
                    parameters=[
                        LaunchConfiguration('plane_params_file'),
                        {
                            'input_topic': topological_map_topic,
                            'output_topic': planar_clusters_topic,
                            'clusters_input_topic': clusters_input_topic,
                        },
                    ],
                    output='screen',
                )
            )

        return nodes

    return LaunchDescription([
        declar_lidar,
        declar_backend,
        declar_input_topic,
        declar_plane_params_file,
        declar_start_plane_cluster,
        declar_topological_map_topic,
        declar_planar_clusters_topic,
        declar_plane_clusters_input_topic,
        OpaqueFunction(function=launch_setup),
    ])
