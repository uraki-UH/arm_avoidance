import builtins
import importlib.util
import io
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.utilities import evaluate_parameters
from rclpy.context import Context
from rclpy.node import Node as parameter_node


package_path = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'ais_gng_launch_test', package_path / 'launch' / 'ais_gng.launch.py')
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)


class test_clustering_yaml(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ros_context = Context()
        cls.ros_context.init(args=[], initialize_logging=False, domain_id=229)

    @classmethod
    def tearDownClass(cls):
        cls.ros_context.shutdown()

    def resolve_parameters(self, overrides, *, enable_common=False, arguments=None):
        # YAML入力だけの差し替え。実行ファイルの起動なし、隔離ドメインでのROSパラメータ解決。
        backend = (arguments or {}).get('backend', 'cpu')
        configs = {
            str(package_path / f'config/gng_{backend}/at128.yaml'): {
                'ais_gng_node': {'ros__parameters': {
                    'input.topic_names': ['/yaml_test/points'], **overrides}}},
            str(package_path / 'config/surface_model.yaml'): {
                '/**': {'ros__parameters': {
                    'surface_model.enable': enable_common,
                    'surface_model.output_topic': '/curved_surface_clusters',
                    'surface_model.method': 'model',
                    'surface_model.enable_support_regions': True}}},
            str(package_path / 'config/plane_cluster_incremental.yaml'): {
                'plane_cluster_incremental_node': {'ros__parameters': {
                    'use_node_rho_for_seed_order': False}},
                'ais_gng_node': {'ros__parameters': {
                    'plane_cluster.direct_enabled': enable_common,
                    'nonplane_component.direct_enabled': True}}},
        }
        original_open = builtins.open

        def config_open(file, *args, **kwargs):
            if str(file) in configs:
                return io.StringIO(yaml.safe_dump(configs[str(file)]))
            return original_open(file, *args, **kwargs)

        context = LaunchContext()
        context.launch_configurations.update({
            'backend': 'cpu', 'lidar': 'at128.yaml', **(arguments or {})})
        with patch.object(launch_module, 'package_dir', str(package_path)), \
                patch('builtins.open', side_effect=config_open):
            nodes = []
            for action in launch_module.generate_launch_description().entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
                elif isinstance(action, OpaqueFunction):
                    nodes = action.execute(context)
            results = []
            for idx, node in enumerate(nodes):
                node_name = ('ais_gng_node', 'object_gng_dataset_exporter_node',
                             'plane_cluster_visualization_node')[idx]
                with TemporaryDirectory(prefix='gng-launch-params-') as directory:
                    args = ['--ros-args']
                    for param_idx, params in enumerate(
                            evaluate_parameters(context, node._Node__parameters)):
                        if isinstance(params, dict):
                            config = {'/**': {'ros__parameters': params}}
                        else:
                            with open(params, encoding='utf-8') as config_file:
                                config = yaml.safe_load(config_file)
                        path = Path(directory) / f'{param_idx}.yaml'
                        path.write_text(yaml.safe_dump(config), encoding='utf-8')
                        args.extend(['--params-file', str(path)])
                    ros_node = parameter_node(
                        node_name, context=self.ros_context, cli_args=args,
                        use_global_arguments=False, enable_rosout=False,
                        start_parameter_services=False,
                        automatically_declare_parameters_from_overrides=True)
                    try:
                        results.append({name: value.value for name, value in
                                        ros_node.get_parameters_by_prefix('').items()})
                    finally:
                        ros_node.destroy_node()
            return results

    def test_input_topic_argument_overrides_sensor(self):
        for backend in ('cpu', 'gpu'):
            with self.subTest(backend=backend):
                params = self.resolve_parameters({}, arguments={
                    'backend': backend, 'input_topic': '/lidar_points'})
                self.assertEqual(params[0]['input.topic_names'], ['/lidar_points'])
                self.assertEqual(params[1]['point_cloud_topic'], '/lidar_points')

    def test_input_topics_without_argument(self):
        topics = ['/yaml_test/front', '/yaml_test/rear']
        params = self.resolve_parameters({'input.topic_names': topics})
        self.assertEqual(params[0]['input.topic_names'], topics)
        self.assertEqual(params[1]['point_cloud_topic'], topics[0])

    def test_common_fallback(self):
        for enable_common in (False, True):
            with self.subTest(enable_common=enable_common):
                params = self.resolve_parameters({}, enable_common=enable_common)
                self.assertEqual(params[0]['plane_cluster.direct_enabled'], enable_common)
                self.assertEqual(len(params), 3 if enable_common else 2)
                self.assertEqual(params[0]['surface_model.enable'], enable_common)

    def test_sensor_overrides_common(self):
        for enable_plane in (False, True):
            for enable_curve in (False, True):
                with self.subTest(enable_plane=enable_plane, enable_curve=enable_curve):
                    params = self.resolve_parameters({
                        'plane_cluster.direct_enabled': enable_plane,
                        'surface_model.enable': enable_curve,
                    }, enable_common=not enable_plane)
                    self.assertEqual(params[0]['plane_cluster.direct_enabled'], enable_plane)
                    self.assertEqual(params[0]['surface_model.enable'], enable_plane and enable_curve)
                    self.assertEqual(len(params), 3 if enable_plane else 2)
                    if enable_plane:
                        self.assertEqual(params[2]['surface_model.enable'], enable_curve)

    def test_surface_topic_consistency(self):
        params = self.resolve_parameters({'surface_model.output_topic': '/yaml_test/curves'},
                                         enable_common=True)
        self.assertEqual(params[0]['surface_model.output_topic'], '/yaml_test/curves')
        self.assertEqual(params[2]['surface_model.output_topic'], '/yaml_test/curves')

    def test_short_switches(self):
        for enable_plane in (False, True):
            for enable_curve in (False, True):
                with self.subTest(enable_plane=enable_plane, enable_curve=enable_curve):
                    params = self.resolve_parameters({
                        'plane_clustering': enable_plane,
                        'curve_clustering': enable_curve,
                    }, enable_common=not enable_curve)
                    self.assertEqual(params[0]['plane_cluster.direct_enabled'], enable_plane)
                    self.assertEqual(params[0]['surface_model.enable'], enable_plane and enable_curve)
                    self.assertEqual(len(params), 3 if enable_plane else 2)
                    if enable_plane:
                        self.assertEqual(params[2]['surface_model.enable'], enable_curve)

    def test_short_switches_override_legacy_names(self):
        for enable_clustering in (False, True):
            with self.subTest(enable_clustering=enable_clustering):
                params = self.resolve_parameters({
                    'plane_clustering': enable_clustering,
                    'curve_clustering': enable_clustering,
                    'plane_cluster.direct_enabled': not enable_clustering,
                    'surface_model.enable': not enable_clustering,
                })
                self.assertEqual(params[0]['plane_cluster.direct_enabled'], enable_clustering)
                self.assertEqual(params[0]['surface_model.enable'], enable_clustering)
                self.assertEqual(len(params), 3 if enable_clustering else 2)
                if enable_clustering:
                    self.assertEqual(params[2]['surface_model.enable'], enable_clustering)

    def test_invalid_short_switches(self):
        for name in ('plane_clustering', 'curve_clustering'):
            for value in ('false', 0, 1, None):
                with self.subTest(name=name, value=value):
                    with self.assertRaisesRegex(RuntimeError, name):
                        self.resolve_parameters({name: value})

    def test_explicit_arguments_take_priority(self):
        params = self.resolve_parameters({
            'surface_model.method': 'smooth_graph',
            'surface_model.enable_support_regions': True,
            'plane_cluster.use_node_rho_for_seed_order': True,
        }, enable_common=True, arguments={'surface_method': 'model', 'enable_support_regions': 'false',
                      'use_node_rho_for_seed_order': 'false'})
        self.assertEqual(params[2]['surface_model.method'], 'model')
        self.assertFalse(params[2]['surface_model.enable_support_regions'])
        self.assertFalse(params[0]['plane_cluster.use_node_rho_for_seed_order'])

    def test_nonplane_override(self):
        params = self.resolve_parameters({'nonplane_component.direct_enabled': False},
                                         enable_common=True)
        self.assertFalse(params[0]['nonplane_component.direct_enabled'])
        self.assertTrue(params[2]['enable_nonplane_markers'])

    def test_explicit_stop_of_surface_node(self):
        params = self.resolve_parameters({'curve_clustering': True, 'plane_clustering': True},
                                         arguments={'start_plane_cluster': 'false'})
        self.assertEqual(len(params), 2)
        self.assertFalse(params[0]['surface_model.enable'])
        self.assertEqual(params[1]['plane_clusters_topic'], '/plane_clusters')

    def test_disabled_clustering_has_no_consumers(self):
        params = self.resolve_parameters({'plane_clustering': False, 'curve_clustering': False})
        self.assertEqual(len(params), 2)
        self.assertEqual(params[1]['plane_clusters_topic'], '')
        self.assertFalse(params[0]['nonplane_component.direct_enabled'])
        self.assertFalse(params[0]['surface_model.enable'])

    def test_external_plane_source_keeps_surface_node(self):
        params = self.resolve_parameters({'plane_clustering': False, 'curve_clustering': True},
                                         arguments={'plane_clusters_input_topic': '/external/planes'})
        self.assertEqual(len(params), 3)
        self.assertTrue(params[0]['surface_model.enable'])
        self.assertFalse(params[0]['nonplane_component.direct_enabled'])
        self.assertEqual(params[1]['plane_clusters_topic'], '/external/planes')
        self.assertEqual(params[2]['clusters_input_topic'], '/external/planes')

    def test_explicit_standalone_plane_node(self):
        params = self.resolve_parameters({'plane_clustering': False},
                                         arguments={'plane_clusters_input_topic': ''})
        self.assertEqual(len(params), 3)
        self.assertEqual(params[2]['clusters_input_topic'], '')
        self.assertEqual(params[1]['plane_clusters_topic'], '/plane_clusters')

    def test_gpu_switch_controls_plane_node(self):
        for enable_plane in (False, True):
            params = self.resolve_parameters({'plane_clustering': enable_plane},
                                             arguments={'backend': 'gpu'})
            self.assertEqual(len(params), 3 if enable_plane else 2)
            self.assertEqual(params[1]['plane_clusters_topic'], '/plane_clusters' if enable_plane else '')


if __name__ == '__main__':
    unittest.main()
