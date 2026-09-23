import os
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

package_dir = get_package_share_directory("ais_gng")


def configured_input_topic(config_path):
    with open(config_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    for node_config in config.values():
        params = node_config.get('ros__parameters', {})
        topic_names = params.get('input.topic_names', [])
        if topic_names:
            return str(topic_names[0])
    return ''


def automatic_camera_info_topic(point_cloud_topic):
    known_topics = {
        '/camera/camera/depth/color/points':
            '/camera/camera/depth/camera_info',
        '/dataset/points': '/dataset/camera_info',
    }
    return known_topics.get(point_cloud_topic, '')


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
    declar_enable_grasp_attention = DeclareLaunchArgument(
        'enable_grasp_attention', default_value='auto',
        description='把持候補近傍の重点学習。autoはYAML設定、未指定は無効（CPU専用）'
    )
    declar_source_point_cloud_topic = DeclareLaunchArgument(
        'source_point_cloud_topic',
        default_value='auto',
        description=(
            '保存用元PointCloud2トピック。autoはinput_topicを使用し、'
            '空文字は元点群保存を無効化'
        )
    )
    declar_source_camera_info_topic = DeclareLaunchArgument(
        'source_camera_info_topic',
        default_value='auto',
        description=(
            'RGB-D PNG自動保存用CameraInfoトピック。autoは既知の'
            'PointCloud2トピックから選択し、空文字はPCD保存のみ'
        )
    )
    declar_plane_params_file = DeclareLaunchArgument(
        'plane_params_file',
        default_value=os.path.join(
            package_dir, 'config', 'plane_cluster_incremental.yaml'),
        description='平面クラスタと可視化の設定ファイル'
    )
    declar_enable_support_regions = DeclareLaunchArgument(
        'enable_support_regions',
        default_value='auto',
        description='曲面の支持領域分離。trueは有効、falseは旧動作、autoはYAML設定'
    )
    declar_start_plane_cluster = DeclareLaunchArgument(
        'start_plane_cluster',
        default_value='true',
        description=(
            'CPU版では内蔵平面クラスタのマーカー変換、'
            'GPU版では平面クラスタ生成とマーカー変換を起動'
        )
    )
    declar_surface_method = DeclareLaunchArgument(
        'surface_method', default_value='auto',
        choices=['auto', 'model', 'smooth_graph'],
        description='曲面抽出方式。autoはYAML設定、modelは形状推定、smooth_graphは接続判定'
    )
    declar_topological_map_topic = DeclareLaunchArgument(
        'topological_map_topic',
        default_value='/topological_map',
    )
    declar_plane_clusters_topic = DeclareLaunchArgument(
        'plane_clusters_topic',
        default_value='/plane_clusters',
    )
    declar_plane_clusters_input_topic = DeclareLaunchArgument(
        'plane_clusters_input_topic',
        default_value='auto',
        description=(
            'autoはCPU版で内蔵クラスタ出力を利用。'
            '空文字は独立ノードでクラスタを再計算'
        )
    )
    declar_use_node_rho_for_seed_order = DeclareLaunchArgument(
        'use_node_rho_for_seed_order',
        default_value='auto',
        description=(
            'CPU平面クラスタのGNG rho再利用をtrue/falseで上書き。'
            'autoはYAMLまたはCPU直結側の既定値を使用'
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
        with open(gng_config_path, encoding='utf-8') as config_file:
            gng_parameters = yaml.safe_load(config_file).get(
                'ais_gng_node', {}).get('ros__parameters', {})
        # センサー別YAMLの短い切替名から内部パラメータへの変換。旧名との併記時は短い名前を優先。
        clustering_switches = {}
        for yaml_name, parameter_name in (
                ('plane_clustering', 'plane_cluster.direct_enabled'),
                ('curve_clustering', 'surface_model.enable')):
            if yaml_name in gng_parameters:
                enable_clustering = gng_parameters[yaml_name]
                if not isinstance(enable_clustering, bool):
                    raise RuntimeError(f'{yaml_name} must be a YAML boolean (true/false)')
                clustering_switches[parameter_name] = enable_clustering
        gng_parameters.update(clustering_switches)

        executable_path = os.path.join(
            get_package_prefix("ais_gng"),
            "lib",
            "ais_gng",
            f"ais_gng_{backend}",
        )
        if not os.path.exists(executable_path):
            raise RuntimeError(f"Backend executable not found: {executable_path}")

        surface_config_path = os.path.join(package_dir, 'config', 'surface_model.yaml')
        # センサー設定と短名変換の統合。同一セレクターへの展開による共通設定との優先順維持。
        parameters = [surface_config_path, gng_parameters]
        launch_parameter_overrides = {}
        grasp_attention = LaunchConfiguration('enable_grasp_attention').perform(context)
        if grasp_attention != 'auto':
            enable_grasp_attention = parse_bool(grasp_attention, 'enable_grasp_attention')
            if enable_grasp_attention and backend != 'cpu':
                raise RuntimeError('enable_grasp_attention is supported only by the CPU backend')
            launch_parameter_overrides['enable_grasp_attention'] = enable_grasp_attention
        input_topic = LaunchConfiguration('input_topic').perform(context)
        if input_topic:
            launch_parameter_overrides['input.topic_names'] = [input_topic]
        source_point_cloud_topic = LaunchConfiguration(
            'source_point_cloud_topic').perform(context)
        if source_point_cloud_topic == 'auto':
            source_point_cloud_topic = (
                input_topic or configured_input_topic(gng_config_path))
        source_camera_info_topic = LaunchConfiguration(
            'source_camera_info_topic').perform(context)
        if source_camera_info_topic == 'auto':
            source_camera_info_topic = automatic_camera_info_topic(
                source_point_cloud_topic)

        topological_map_topic = LaunchConfiguration(
            'topological_map_topic').perform(context)
        plane_clusters_topic = LaunchConfiguration(
            'plane_clusters_topic').perform(context)
        with open(LaunchConfiguration('plane_params_file').perform(context),
                  encoding='utf-8') as config_file:
            plane_config = yaml.safe_load(config_file)
        plane_parameters = plane_config.get(
            'plane_cluster_incremental_node', {}).get('ros__parameters', {})
        enable_nonplane_component = False
        if backend == 'cpu':
            # 共通設定のCPU直結名前空間への転写。センサー別YAML、起動引数の順で優先。
            plane_parameter_overrides = {
                f'plane_cluster.{name}': value for name, value in plane_parameters.items()
            }
            plane_parameter_overrides.update(
                plane_config.get('ais_gng_node', {}).get('ros__parameters', {}))
            plane_parameter_overrides.update({
                name: value for name, value in gng_parameters.items()
                if name.startswith(('plane_cluster.', 'nonplane_component.'))
            })
            plane_parameter_overrides['plane_cluster.output_topic'] = plane_clusters_topic
            enable_nonplane_component = bool(plane_parameter_overrides.get(
                'nonplane_component.direct_enabled', True))
            rho_mode = LaunchConfiguration(
                'use_node_rho_for_seed_order').perform(context)
            if rho_mode != 'auto':
                plane_parameter_overrides[
                    'plane_cluster.use_node_rho_for_seed_order'
                ] = parse_bool(rho_mode, 'use_node_rho_for_seed_order')
            parameters.append(plane_parameter_overrides)

        # センサー・平面設定より明示launch引数を優先。
        parameters.append(launch_parameter_overrides)
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
                parameters=[{
                    'map_topic': topological_map_topic,
                    'plane_clusters_topic': plane_clusters_topic,
                    'point_cloud_topic': source_point_cloud_topic,
                    'camera_info_topic': source_camera_info_topic,
                }],
                output='screen',
            ),
        ]

        start_plane_cluster = parse_bool(
            LaunchConfiguration('start_plane_cluster').perform(context),
            'start_plane_cluster')
        if start_plane_cluster:
            # センサー別の曲面設定を実際の計算ノードへも転送。
            surface_parameter_overrides = {
                name: value for name, value in gng_parameters.items()
                if name.startswith('surface_model.')
            }
            surface_method = LaunchConfiguration('surface_method').perform(context)
            if surface_method != 'auto':
                surface_parameter_overrides['surface_model.method'] = surface_method
            support_mode = LaunchConfiguration('enable_support_regions').perform(context)
            if support_mode != 'auto':
                surface_parameter_overrides['surface_model.enable_support_regions'] = (
                    parse_bool(support_mode, 'enable_support_regions'))
            clusters_input_topic = LaunchConfiguration(
                'plane_clusters_input_topic').perform(context)
            if clusters_input_topic == 'auto':
                clusters_input_topic = (
                    plane_clusters_topic if backend == 'cpu' else '')
            nodes.append(
                Node(
                    package='ais_gng',
                    executable='plane_cluster_incremental_node',
                    name='plane_cluster_visualization_node',
                    parameters=[
                        plane_parameters,
                        LaunchConfiguration('plane_params_file'),
                        surface_config_path,
                        surface_parameter_overrides,
                        {
                            'input_topic': topological_map_topic,
                            'output_topic': plane_clusters_topic,
                            'clusters_input_topic': clusters_input_topic,
                            # CPU直結の所属情報をViewerで描画。MarkerArrayによる二重生成の抑止。
                            'enable_nonplane_markers': not enable_nonplane_component,
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
        declar_enable_grasp_attention,
        declar_source_point_cloud_topic,
        declar_source_camera_info_topic,
        declar_plane_params_file,
        declar_enable_support_regions,
        declar_start_plane_cluster,
        declar_surface_method,
        declar_topological_map_topic,
        declar_plane_clusters_topic,
        declar_plane_clusters_input_topic,
        declar_use_node_rho_for_seed_order,
        OpaqueFunction(function=launch_setup),
    ])
