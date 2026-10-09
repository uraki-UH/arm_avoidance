#!/usr/bin/env python3
"""実CPU GNG・テンプレート照合・ファジークラス認識を通す有限検証。"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from ais_gng_msgs.msg import TopologicalMap, PlaneClusterArray
import yaml

share = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(share / 'scripts'))
from object_class_recognition import class_recognizer

model_ids = {'car_surface': 'drivaer_sedan', 'kei_car_surface': 'compact_tall', 'truck_surface': 'box_truck'}
class_ids = {'car_surface': 'passenger_car', 'kei_car_surface': 'kei_car', 'truck_surface': 'truck', 'mug': 'mug'}


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def make_points(name, seed, is_variant=False):
    rng = np.random.default_rng(seed)
    models = json.loads((share.parent / 'ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/config/vehicle_models/models.json').read_text())['models']
    if name in model_ids:
        model_id = 'passenger_van' if name == 'car_surface' and is_variant else model_ids[name]
        points = np.array(next(item['points'] for item in models if item['id'] == model_id), dtype=float)
        if is_variant and name == 'kei_car_surface':
            points[:, 2] *= 0.88
            points[:, 0] *= 1.03
        if is_variant and name == 'truck_surface':
            points[:, 0] *= 1.10
            points[:, 2] *= 1.08
            points[points[:, 0] > 1.0, 2] *= 0.86
    elif name in ('mug', 'cylinder'):
        num = 3500
        angle = rng.uniform(-math.pi, math.pi, num)
        height = rng.uniform(0, 0.095 if not is_variant else 0.11, num)
        radius = np.full(num, 0.045)
        if is_variant:
            radius *= 0.8 + 0.4 * height / 0.11
        # 胴体外面・内面・底と取っ手の既知合成形状。
        radius[:700] *= 0.86
        points = np.column_stack((radius * np.cos(angle), radius * np.sin(angle), height))
        points[:300, :2] *= rng.uniform(0, 1, (300, 1))
        points[:300, 2] = 0.003
        if name == 'mug':
            handle_angle = rng.uniform(-math.pi / 2, math.pi / 2, 900)
            tube_angle = rng.uniform(-math.pi, math.pi, 900)
            handle_radius = 0.035 if not is_variant else 0.043
            ring = handle_radius + 0.007 * np.cos(tube_angle)
            handle = np.column_stack((0.043 + ring * np.cos(handle_angle),
                                      0.007 * np.sin(tube_angle), 0.05 + ring * np.sin(handle_angle)))
            points = np.concatenate((points, handle))
    elif name == 'box':
        points = rng.uniform(-0.5, 0.5, (3500, 3))
        axes = rng.integers(0, 3, 3500)
        points[np.arange(3500), axes] = rng.choice([-0.5, 0.5], 3500)
        points *= [3.5, 1.6, 1.7]
        points[:, 2] += 0.85
    elif name == 'plane':
        points = rng.uniform(-1, 1, (3500, 3)) * [2.5, 2.0, 0.0]
    else:
        raise ValueError(name)
    return points


class runtime:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.processes = []
        self.node = Node('class_verification_probe')
        self.counter = 0

    def spawn(self, command, name):
        log = (self.output_dir / f'{name}.log').open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))
        save(self.output_dir / f'{name}_process.json', {'pid': process.pid, 'command': command})
        return process

    def stop(self, process):
        for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
            if process.poll() is not None:
                break
            process.send_signal(sig)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
        if process.poll() is None:
            raise RuntimeError(f'試験プロセス停止失敗: {process.pid}')

    def wait_for(self, predicate, process, timeout=30.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError(f'試験ノード終了: {process.returncode}')
            if time.monotonic() > deadline:
                raise TimeoutError('試験ノードの応答待ち期限切れ')
            rclpy.spin_once(self.node, timeout_sec=0.03)

    def close(self):
        for process, log in reversed(self.processes):
            self.stop(process)
            log.close()
        self.node.destroy_node()

    def learn(self, points, name, max_nodes, num_frames, node_grid=None):
        self.counter += 1
        prefix = f'/class_verification/learn_{self.counter}'
        params = yaml.safe_load((share.parent / 'ais_gng_cpu/src/ais_gng/config/gng_cpu/mesh_surface.yaml').read_text())['ais_gng_node']['ros__parameters']
        plane_params = yaml.safe_load((share.parent / 'ais_gng_cpu/src/ais_gng/config/plane_cluster_incremental.yaml').read_text())['plane_cluster_incremental_node']['ros__parameters']
        params.update({f'plane_cluster.{key}': value for key, value in plane_params.items()})
        span = float(np.ptp(points, axis=0).max())
        params.update({'node.num_max': max_nodes, 'node.learning_num': 1200,
                       'plane_clustering': True, 'nonplane_component.direct_enabled': True,
                       'plane_cluster.output_topic': prefix + '/planes',
                       'input.topic_names': [prefix + '/points'], 'input.visualize': False,
                       'input.local_coordinates': True, 'input.base_frame_id': 'object_template',
                       'node.interval': [span / 45.0] * 4,
                       'input.voxel_grid_unit': max(0.002, span / 150.0)})
        if node_grid is not None:
            params['node.grid'] = node_grid
        for idx, axis in enumerate('xyz'):
            params[f'input.{axis}_min'] = float(points[:, idx].min() - max(0.05, span * 0.05))
            params[f'input.{axis}_max'] = float(points[:, idx].max() + max(0.05, span * 0.05))
        config_path = self.output_dir / f'{name}_gng.yaml'
        config_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        maps, planes = {}, {}
        map_sub = self.node.create_subscription(TopologicalMap, prefix + '/map', lambda msg: maps.update({msg.frame_number: msg}), 10)
        plane_sub = self.node.create_subscription(PlaneClusterArray, prefix + '/planes', lambda msg: planes.update({msg.frame_number: msg}), 10)
        publisher = self.node.create_publisher(PointCloud2, prefix + '/points', 10)
        process = self.spawn(['/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu', '--ros-args',
                              '--params-file', str(config_path), '-r', '__node:=class_verification_gng',
                              '-r', 'topological_map:=' + prefix + '/map'], name + '_gng')
        try:
            self.wait_for(lambda: publisher.get_subscription_count() > 0, process)
            deadline = time.monotonic() + 45.0
            next_publish = 0.0
            point_values = points.astype(np.float32).tolist()
            while len(set(maps) & set(planes)) < num_frames:
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f'{name}: GNG入力失敗 map={len(maps)} planes={len(planes)} return={process.poll()}')
                if time.monotonic() >= next_publish:
                    header = Header(frame_id='object_template', stamp=self.node.get_clock().now().to_msg())
                    publisher.publish(create_cloud_xyz32(header, point_values))
                    next_publish = time.monotonic() + 0.05
                rclpy.spin_once(self.node, timeout_sec=0.01)
            last = max(set(maps) & set(planes))
            result = maps[last], planes[last]
            if len(result[0].nodes) < 10:
                raise RuntimeError(f'{name}: GNG node不足: {len(result[0].nodes)}')
            return result
        finally:
            self.stop(process)
            self.node.destroy_subscription(map_sub)
            self.node.destroy_subscription(plane_sub)
            self.node.destroy_publisher(publisher)

    def match(self, template_dir, graphs, matcher):
        template_ids = list(class_ids)
        params = yaml.safe_load((share / 'config/object_template_matching.yaml').read_text())['object_template_matcher_node']['ros__parameters']
        params.update({'template_ids': template_ids,
                       'template_dataset_paths': [str(template_dir / f'{name}.json') for name in template_ids],
                       'environment_topological_map_topic': '/class_verification/environment',
                       'plane_clusters_topic': '/class_verification/planes',
                       'candidate_topics': [f'/class_verification/{name}/candidate' for name in template_ids]})
        self.counter += 1
        matcher_name = f'matcher_{self.counter}'
        config_path = self.output_dir / f'{matcher_name}.yaml'
        config_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        outputs = {}
        subscriptions = [self.node.create_subscription(
            String, topic, lambda message, name=name: outputs.update({name: json.loads(message.data)}), 10)
            for name, topic in zip(template_ids, params['candidate_topics'])]
        map_pub = self.node.create_publisher(TopologicalMap, params['environment_topological_map_topic'], 10)
        plane_pub = self.node.create_publisher(PlaneClusterArray, params['plane_clusters_topic'], 10)
        process = self.spawn([str(matcher), '--ros-args', '--params-file', str(config_path)], matcher_name)
        results = []
        try:
            self.wait_for(lambda: map_pub.get_subscription_count() > 0 and plane_pub.get_subscription_count() > 0, process)
            class_config = yaml.safe_load((share / 'config/object_class_recognition.yaml').read_text())
            for idx, (name, expected, condition, graph, planes) in enumerate(graphs):
                # 時間方向の候補混合を避けるため、条件ごとに認識キャッシュを初期化。
                recognizer = class_recognizer(class_config, template_ids)
                outputs.clear()
                begin = time.monotonic()
                graph = deepcopy(graph)
                planes = deepcopy(planes)
                for repeat in range(24):
                    graph.frame_number = planes.frame_number = idx * 100 + repeat
                    map_pub.publish(graph)
                    plane_pub.publish(planes)
                    previous_num = len(outputs)
                    # 索引で候補から外れたtemplateの不在も試験結果として保持。
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        rclpy.spin_once(self.node, timeout_sec=0.02)
                        if len(outputs) > previous_num:
                            break
                    if len(outputs) == len(template_ids):
                        break
                    if time.monotonic() - begin > 8.0:
                        break
                for template_id, candidate in outputs.items():
                    recognizer.update(template_id, candidate, 0.0)
                result = recognizer.snapshot(0.0)
                detail_classes = {class_id: result['classes'][class_id] for class_id in class_ids.values()}
                supported = [class_id for class_id, item in detail_classes.items() if item['state'] == 'supported']
                row = {'name': name, 'expected': expected, 'condition': condition,
                       'num_nodes': len(graph.nodes), 'num_planes': len(planes.clusters),
                       'supported': supported, 'classes': result['classes'], 'candidates': deepcopy(outputs),
                       'elapsed_sec': time.monotonic() - begin}
                results.append(row)
                print(json.dumps({key: row[key] for key in ('name', 'expected', 'supported', 'num_nodes', 'num_planes', 'elapsed_sec')}, ensure_ascii=False), flush=True)
            return results
        finally:
            self.stop(process)
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(map_pub)
            self.node.destroy_publisher(plane_pub)


def store_graph(path, graph, planes):
    nodes = [{'x': node.pos.x, 'y': node.pos.y, 'z': node.pos.z,
              'nx': node.normal.x, 'ny': node.normal.y, 'nz': node.normal.z, 'rho': node.rho}
             for node in graph.nodes]
    compact_planes = [{'id': plane.id, 'idx': list(plane.node_indices),
                       'normal': [plane.normal.x, plane.normal.y, plane.normal.z],
                       'centroid': [plane.centroid.x, plane.centroid.y, plane.centroid.z],
                       'extent': [plane.extent_u, plane.extent_v]} for plane in planes.clusters]
    save(path, {'kind': 'object_template', 'schema_version': 1, 'template_id': path.stem,
                'gng': {'nodes': nodes, 'edges': [list(graph.edges[idx:idx + 2]) for idx in range(0, len(graph.edges), 2)],
                        'plane_clusters': compact_planes}})
    path.with_suffix('.map.cdr').write_bytes(serialize_message(graph))
    path.with_suffix('.planes.cdr').write_bytes(serialize_message(planes))



def check_mug_detail(test, args):
    """小物のグリッド密度による識別性能変化の切り分け。"""
    templates = args.output.parent / 'templates'
    templates.mkdir(exist_ok=True)
    for source in args.template_dir.glob('*.json'):
        shutil.copy2(source, templates / source.name)
    graph, planes = test.learn(make_points('mug', 0), 'mug_fine_template', 256, 24, node_grid=0.02)
    store_graph(templates / 'mug.json', graph, planes)
    graphs = [('mug_fine_replay', 'mug', 'registered_graph', graph, planes)]
    for idx, condition in enumerate(('independent', 'occluded', 'held_out_shape', 'cylinder')):
        points = make_points('cylinder' if condition == 'cylinder' else 'mug',
                             args.seed + idx, condition == 'held_out_shape')
        if condition == 'occluded':
            points = points[points[:, 0] < np.median(points[:, 0])]
        points += np.random.default_rng(args.seed + idx).normal(0, 0.0001, points.shape)
        graph, planes = test.learn(points, 'mug_fine_' + condition, 256, 24, node_grid=0.02)
        store_graph(args.output.parent / ('mug_fine_' + condition + '.json'), graph, planes)
        graphs.append(('mug_fine_' + condition, None if condition == 'cylinder' else 'mug',
                       condition, graph, planes))
    results = []
    for graph in graphs:
        results.extend(test.match(templates, [graph], args.matcher))
    save(args.output.parent / 'details.json', results)
    save(args.output, {'num_cases': len(results),
                      'num_positive_supported': sum(row['expected'] in row['supported'] for row in results if row['expected']),
                      'num_negative_false_support': sum(bool(row['supported']) for row in results if row['expected'] is None)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', choices=['prepare', 'evaluate', 'smoke', 'mug_detail'], required=True)
    parser.add_argument('--template-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--matcher', type=Path, default=Path('/tmp/uraki_object_class_build/src/object_template_matcher_node'))
    parser.add_argument('--seed', type=int, default=100)
    parser.add_argument('--max-nodes', type=int, default=128)
    parser.add_argument('--num-frames', type=int, default=24)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.template_dir.mkdir(parents=True, exist_ok=True)
    begin = time.monotonic()
    rclpy.init()
    test = runtime(args.output.parent)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        if args.stage == 'mug_detail':
            check_mug_detail(test, args)
            return
        if args.stage in ('prepare', 'smoke'):
            names = ['mug'] if args.stage == 'smoke' else list(class_ids)
            summaries = []
            for name in names:
                points = make_points(name, 0)
                graph, planes = test.learn(points, name, args.max_nodes, args.num_frames)
                store_graph(args.template_dir / f'{name}.json', graph, planes)
                row = {'name': name, 'num_nodes': len(graph.nodes), 'num_planes': len(planes.clusters),
                       'num_nonplane_nodes': sum(node.nonplane_component_id != node.NONPLANE_COMPONENT_NONE for node in graph.nodes),
                       'has_dense_ids': all(idx == node.id for idx, node in enumerate(graph.nodes))}
                summaries.append(row)
                print(json.dumps(row), flush=True)
            save(args.output.parent / 'templates.json', summaries)
            save(args.output, {'num_templates': len(names), 'elapsed_sec': time.monotonic() - begin})
            return
        graphs = []
        for idx, (name, expected) in enumerate(class_ids.items()):
            graph = deserialize_message((args.template_dir / f'{name}.map.cdr').read_bytes(), TopologicalMap)
            planes = deserialize_message((args.template_dir / f'{name}.planes.cdr').read_bytes(), PlaneClusterArray)
            graphs.append((name + '_replay', expected, 'registered_graph', graph, planes))
            for condition in ('independent', 'occluded', 'held_out_shape'):
                points = make_points(name, args.seed + idx, condition == 'held_out_shape')
                if condition == 'occluded':
                    points = points[points[:, 0] < np.median(points[:, 0])]
                span = float(np.ptp(points, axis=0).max())
                points += np.random.default_rng(args.seed + idx).normal(0.0, span * 0.001, points.shape)
                angle = math.pi / 4
                rotation = np.array([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
                points = points @ rotation.T + [0.25, -0.15, 0.0]
                graph, planes = test.learn(points, name + '_' + condition, args.max_nodes, args.num_frames)
                store_graph(args.output.parent / f'{name}_{condition}.json', graph, planes)
                graphs.append((name + '_' + condition, expected, condition, graph, planes))
        for name in ('box', 'plane', 'cylinder'):
            points = make_points(name, args.seed)
            graph, planes = test.learn(points, name, args.max_nodes, args.num_frames)
            graphs.append((name, None, 'negative', graph, planes))
        results = []
        for graph in graphs:
            results.extend(test.match(args.template_dir, [graph], args.matcher))
        save(args.output.parent / 'details.json', results)
        positives = [row for row in results if row['expected'] is not None]
        negatives = [row for row in results if row['expected'] is None]
        def has_wrong_support(row):
            allowed = {row['expected']}
            if row['expected'] == 'kei_car':
                allowed.add('passenger_car')
            return bool(set(row['supported']) - allowed)

        metrics = {'num_cases': len(results), 'num_positive_cases': len(positives),
                   'num_positive_wrong_support': sum(has_wrong_support(row) for row in positives),
                   'num_positive_supported': sum(row['expected'] in row['supported'] for row in positives),
                   'num_negative_cases': len(negatives),
                   'num_negative_false_support': sum(bool(row['supported']) for row in negatives),
                   'elapsed_sec': time.monotonic() - begin}
        save(args.output, metrics)
    finally:
        test.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
