"""再学習GNG・VLUT・集約GNGの実launchによる読込・配信照合。"""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from ais_gng_msgs.msg import TopologicalMap
from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray

root = Path('/ros2_ws/src')
output = root / 'artifacts/topodualarm_retrain_20261001'
model_id = 'ToPoDualArm10000_selfchecked_20261001'
model = root / 'gng_vlut_system/gng_results' / model_id
audit = json.loads((output / 'final_audit.json').read_text())
states = json.loads((output / 'final_audit.json.states.json').read_text())
assert audit['config']['enable_edges'] and not audit['config'].get('edge_pairs')
assert audit['config'].get('enable_geometry_checks', True)
assert audit['num_colliding'] == audit['num_limit_failures'] == audit['num_stored_unsafe'] == 0
assert audit['max_fk_error_m'] < 1e-6
assert all(layer['num_unsafe_edges'] == 0 for layer in audit['layers'])
assert len(states) == audit['num_nodes'] > 0
spec = importlib.util.spec_from_file_location('vlut_check',
    root / 'benchmarks/voxel_pose_compression_20260930/export_preview.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
records = module.read_gng(model / 'gng.bin', expected_num_layers=1)
assert [len(layer) for layer in records['edge_layers']] == [layer['num_edges'] for layer in audit['layers']]
header, relations = module.read_vlut(model / 'vlut.bin', set(map(int, states)))
assert set(map(int, np.unique(relations['node']))) == set(map(int, states))
assert os.environ['ROS_DOMAIN_ID'] == '218' and os.environ['ROS_LOCALHOST_ONLY'] == '1'
rclpy.init()
node = rclpy.create_node('topodualarm_retrain_verify')
topics = ['/ToPoDualArm/Tmap_static', '/ToPoDualArm/Tmap_vis_L0']
received = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
for topic in topics:
    node.create_subscription(TopologicalMap, topic,
        lambda msg, name=topic: received.__setitem__(name, msg), qos)
node.create_subscription(TopologicalNodeFeatureArray, '/ToPoDualArm/topological_node_features',
    lambda msg: received.__setitem__('features', msg), qos)
command = ['ros2', 'launch', 'gng_vlut_system', 'gng_viewer_bridge.launch.py',
    f'params_file:={root}/gng_vlut_system/config/ToPoDualArm.yaml', f'id:={model_id}',
    'enable_dynamixel_current_pose:=false', 'enable_dynamixel_input:=false',
    'enable_environment_voxelization:=false', 'enable_self_recognition_viz:=false',
    'enable_realsense_mount_tf:=false']
process = None
try:
    (output / 'viewer_command.json').write_text(json.dumps(command, ensure_ascii=False, indent=2))
    with (output / 'viewer.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        deadline = time.monotonic() + 60
        while len(received) < 3 and time.monotonic() < deadline:
            assert process.poll() is None, 'launch exited'
            rclpy.spin_once(node, timeout_sec=.1)
        assert len(received) == 3, f'missing topics: {received.keys()}'
        source, coarse = (received[topic] for topic in topics)
        assert len(source.nodes) == len(states)
        assert {str(n.id) for n in source.nodes} == set(states)
        assert source.header.frame_id == coarse.header.frame_id == 'ToPoDualArm/base_link'
        for vertex in source.nodes:
            expected = states[str(vertex.id)]
            assert np.allclose([vertex.pos.x, vertex.pos.y, vertex.pos.z], expected['pos'], rtol=0, atol=1e-6)
        features = received['features'].features
        assert len(features) == len(states)
        for feature in features:
            assert np.allclose(feature.weight_angle, states[str(feature.node_id)]['q'], rtol=0, atol=1e-6)
        assert len(source.edges) == 2 * audit['layers'][1]['num_edges']
        assert len(coarse.nodes) == 150
        for graph in (source, coarse):
            assert len(graph.edges) % 2 == 0
            assert all(0 <= idx < len(graph.nodes) for idx in graph.edges)
        result = {'num_nodes': len(source.nodes), 'num_edges': len(source.edges)//2,
            'num_coarse_nodes': len(coarse.nodes), 'num_coarse_edges': len(coarse.edges)//2,
            'num_vlut_relations': len(relations), 'frame': source.header.frame_id}
        (output / 'viewer_result.json').write_text(json.dumps(result, indent=2))
        print('PASS', json.dumps(result), flush=True)
finally:
    if process is not None:
        for sig, sec in ((signal.SIGINT, 15), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=sec)
            except subprocess.TimeoutExpired:
                continue
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
        process.wait(timeout=3)
        print('STOPPED', process.pid, process.returncode, flush=True)
    node.destroy_node()
    rclpy.shutdown()
