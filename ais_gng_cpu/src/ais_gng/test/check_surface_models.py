"""保存GNGまたは実行中GNGを用いた有限時間のSurfaceModel通信・色分け検証。"""

import argparse
from collections import Counter
import gzip
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from ais_gng_msgs.msg import PlaneCluster, PlaneClusterArray, TopologicalMap, TopologicalNode
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rosidl_runtime_py.convert import message_to_ordereddict


def load_template(path):
    opener = gzip.open if path.endswith('.gz') else open
    with opener(path, 'rt') as source:
        graph = json.load(source)['gng']
    msg, planes = TopologicalMap(), PlaneClusterArray()
    msg.header.frame_id = planes.header.frame_id = 'map'
    for idx, raw in enumerate(graph['nodes']):
        node = TopologicalNode()
        node.id = idx
        node.pos.x, node.pos.y, node.pos.z = [float(raw[k]) for k in ('x', 'y', 'z')]
        node.normal.x, node.normal.y, node.normal.z = [float(raw[k]) for k in ('nx', 'ny', 'nz')]
        msg.nodes.append(node)
    msg.edges = [int(idx) for edge in graph['edges'] for idx in edge]
    for raw in graph.get('plane_clusters', []):
        plane = PlaneCluster()
        plane.id = raw['id']
        plane.node_indices = raw['idx']
        if 'normal' in raw:
            plane.normal.x, plane.normal.y, plane.normal.z = [float(v) for v in raw['normal']]
        planes.clusters.append(plane)
    return msg, planes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--template')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--executable', default='/ros2_ws/build/ais_gng/plane_cluster_incremental_node')
    parser.add_argument('--output')
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--markers', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--graph', action='store_true')
    parser.add_argument('--params-file')
    parser.add_argument('--disconnect-after', type=float)
    args = parser.parse_args()
    if not 2 <= args.seconds <= 60:
        parser.error('--seconds must be within [2, 60]')
    if args.disconnect_after is not None and (
            not args.template or not 1 <= args.disconnect_after < args.seconds - 1):
        parser.error('--disconnect-after requires --template and an interval within the test')
    prefix = f'/surface_model_check_{os.getpid()}'
    map_topic = prefix + '/map' if args.template else '/topological_map'
    plane_topic = prefix + '/planes' if args.template else '/plane_clusters'
    command = [args.executable, '--ros-args', '-r', '__node:=surface_model_check',
               '-p', 'input_topic:=' + map_topic, '-p', 'clusters_input_topic:=' + plane_topic,
               '-p', 'surface_model.output_topic:=' + prefix,
               '-p', 'surface_model.enable_markers:=' + str(args.markers).lower(),
               '-p', 'surface_model.enable_graph:=' + str(args.graph).lower(),
               '-p', 'enable_plane_markers:=false', '-p', 'enable_nonplane_markers:=false']
    if args.launch:
        command = ['ros2', 'launch', 'ais_gng', 'surface_models.launch.py',
                   'input_topic:=' + map_topic, 'plane_clusters_topic:=' + plane_topic,
                   'output_topic:=' + prefix, 'enable_markers:=' + str(args.markers).lower(),
                   'enable_graph:=' + str(args.graph).lower()]
    if args.params_file:
        command += (['params_file:=' + args.params_file] if args.launch
                    else ['--params-file', args.params_file])
    rclpy.init()
    node = rclpy.create_node('surface_model_check_observer')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    data, markers, graphs = [], [], []
    node.create_subscription(String, prefix + '/models', lambda m: data.append(json.loads(m.data)), qos)
    node.create_subscription(TopologicalMap, prefix, graphs.append, qos)
    node.create_subscription(MarkerArray, prefix + '/markers', markers.append, qos)
    process = None
    try:
        if args.template:
            msg, planes = load_template(args.template)
            map_pub = node.create_publisher(TopologicalMap, map_topic, qos)
            plane_pub = node.create_publisher(PlaneClusterArray, plane_topic, qos)
            publish_begin = time.monotonic()
            disconnect_frame = None

            def publish():
                nonlocal disconnect_frame
                msg.frame_number += 1
                if args.disconnect_after is not None and time.monotonic() - publish_begin >= args.disconnect_after:
                    # 座標・法線を保ち、全edgeだけを消す接続ゆらぎの再現。
                    msg.edges = []
                    if disconnect_frame is None:
                        disconnect_frame = msg.frame_number
                planes.frame_number = msg.frame_number
                msg.header.stamp = planes.header.stamp = node.get_clock().now().to_msg()
                map_pub.publish(msg)
                plane_pub.publish(planes)

            timer = node.create_timer(0.2, publish)
        process = subprocess.Popen(command, start_new_session=True)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert process.poll() is None, 'SurfaceModel process exited unexpectedly'
        assert len(data) >= 3, 'SurfaceModel was not received'
        assert not args.markers or markers, 'MarkerArray was not received'
        assert not args.graph or graphs, 'TopologicalMap was not received'
        assert node.count_publishers(prefix + '/markers') == int(args.markers), 'Unexpected marker publisher'
        assert node.count_publishers(prefix) == int(args.graph), 'Unexpected graph publisher'
        latest = data[-1]
        graph = next((g for g in reversed(graphs) if g.frame_number == latest['frame_number']), None)
        members = [idx for model in latest['models'] for idx in model['node_indices']]
        assert len(members) == len(set(members)), 'Duplicate membership'
        for model in latest['models']:
            patches = set(model['patch_indices'])
            assert all(a not in patches or b not in patches for a, b in latest['sharp_edges']), \
                'A surface crosses a sharp plane boundary'
        if args.template:
            assert set(members) == set(range(len(msg.nodes))), 'Missing node membership'
        if args.graph:
            assert graph, 'No graph matches the model frame'
            expected = {m['id']: [graph.nodes[i].id for i in m['node_indices']]
                        for m in latest['models'] if m['type'] != 'unknown'}
            assert {c.id: list(c.nodes) for c in graph.clusters} == expected, 'Cluster membership differs'
            assert all(i < len(graph.nodes) for i in graph.edges), 'Invalid edge index'
        active = {(m.ns, m.id): m for m in markers[-1].markers if m.action == 0} if markers else {}
        display_models = [m for m in latest['models'] if m['is_display_candidate']]
        if args.template and args.markers:
            expected = {m['id']: len(m['node_indices']) for m in display_models}
            actual = {idx: len(marker.points) for (ns, idx), marker in active.items()
                      if ns == 'surface_nodes'}
            assert actual == expected, 'Marker membership differs from the display filter'
            assert all(ns != 'surface_unknown_nodes' for ns, _ in active), 'Unknown nodes are visible'
        colors = []
        for (ns, idx), marker in active.items():
            if ns != 'surface_nodes':
                continue
            color = (marker.color.r, marker.color.g, marker.color.b)
            colors.append(color)
            edges = active.get(('surface_edges', idx))
            if edges:
                assert marker.color == edges.color, 'Node/edge colors differ in one model'
        assert len(colors) == len(set(colors)), 'Different models use the same color'
        def timing(key):
            samples = sorted(x[key] for x in data)
            return {'mean_ms': statistics.mean(samples), 'p95_ms': samples[int(.95*(len(samples)-1))]}
        summary = {'frames': len(data), 'node_num': len(members),
                   'patch_num': len(latest['patches']),
                   'types': dict(Counter(m['type'] for m in latest['models'])),
                   'mixed_regions': sum(len({latest['patches'][p]['kind'] for p in m['patch_indices']}) > 1
                                        for m in latest['models'] if m['type'] != 'unknown'),
                   'display_models': len(display_models),
                   'display_nodes': sum(len(m['node_indices']) for m in display_models),
                   'model_fits': latest['model_fits'],
                   'sharp_edges': len(latest['sharp_edges']),
                   'valid_curvature': sum(p['curvature']['valid'] for p in latest['patches']),
                   'graph_cdr_bytes': len(serialize_message(graph)) if graph else 0,
                   'marker_cdr_bytes': len(serialize_message(markers[-1])) if markers else 0,
                   'graph_clusters': len(graph.clusters) if graph else 0, 'total': timing('update_ms'),
                   'curvature': timing('curvature_ms'), 'retention': timing('retention_ms'),
                   'retained_models': sum(m['is_retained'] for m in latest['models'])}
        if args.disconnect_after is not None:
            after = [d for d in data if d['frame_number'] >= disconnect_frame]
            summary['disconnected_frames'] = len(after)
            summary['visible_disconnected_frames'] = sum(
                any(m['is_display_candidate'] for m in d['models']) for d in after)
            summary['disconnected_total_mean_ms'] = statistics.mean(d['update_ms'] for d in after)
        print('RESULT ' + json.dumps(summary), flush=True)
        if args.output:
            output = {'summary': summary, 'snapshot': latest}
            if graph:
                output['graph'] = message_to_ordereddict(graph)
            if markers:
                output['markers'] = message_to_ordereddict(markers[-1])
            Path(args.output).write_text(json.dumps(output, indent=2))
    finally:
        if process:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=5)
                    break
                except subprocess.TimeoutExpired:
                    pass
            print(f'STOPPED pid={process.pid} returncode={process.poll()}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
