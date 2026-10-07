"""左右別GNGの配信・組合せ検査の隔離統合試験。保存済みモデルを引数で指定。"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from ais_gng_msgs.msg import TopologicalMap
from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray
from gng_control_msgs.srv import CheckArmPair


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=187)
    parser.add_argument('--max_pairs', type=int, default=500)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output.mkdir(parents=True, exist_ok=True)
    models = args.models.resolve()
    manifest = json.loads((models/'independent_arms.json').read_text())
    metadata = {item['name']: json.loads((models/item['metadata']).read_text())
                for item in manifest['profiles']}
    command = ['ros2', 'launch', 'gng_vlut_system', 'gng_viewer_bridge.launch.py',
               'enable_independent_arms:=true', 'dir:='+str(models.parent), 'id:='+models.name]
    print('launch_command:', ' '.join(command), flush=True)
    rclpy.init()
    def handle_stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, handle_stop)
    node = rclpy.create_node('check_independent_arm_pair')
    received = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for arm in ('left_arm', 'right_arm'):
        for suffix, kind in (('map', TopologicalMap), ('features', TopologicalNodeFeatureArray)):
            topic = 'Tmap_'+arm if suffix == 'map' else arm+'/topological_node_features'
            key = arm+'_'+suffix
            node.create_subscription(kind, '/topo_dual_arm_max_long/'+topic,
                                     lambda message, name=key: received.update({name: message}), qos)
    client = node.create_client(CheckArmPair, '/topo_dual_arm_max_long/check_arm_pair')
    process = None
    owned_pids = set()
    try:
        with (args.output/'launch.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            deadline = time.monotonic()+90
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.2)
                if len(received) == 4 and client.service_is_ready():
                    break
                assert process.poll() is None, 'launchの早期終了'
            assert len(received) == 4 and client.service_is_ready(), list(received)
            features = {}
            for arm in ('left_arm', 'right_arm'):
                graph = received[arm+'_map']
                features[arm] = {item.node_id: item for item in received[arm+'_features'].features}
                assert len(graph.nodes) == metadata[arm]['num_nodes']
                assert graph.edges, arm
                assert {item.id for item in graph.nodes} == set(features[arm])
                assert all(len(item.weight_angle) == 7 and all(math.isfinite(x) for x in item.weight_angle)
                           for item in features[arm].values())

            def call(left, right, start=None):
                request = CheckArmPair.Request(left_node_id=left, right_node_id=right)
                if start is not None:
                    request.start_state = start
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=30)
                assert future.done(), '組合せ検査の応答期限超過'
                return future.result()

            rng = random.Random(71)
            left_ids, right_ids = list(features['left_arm']), list(features['right_arm'])
            safe_pairs, colliding_pairs = [], []
            for _ in range(args.max_pairs):
                left, right = rng.choice(left_ids), rng.choice(right_ids)
                response = call(left, right)
                assert response.is_valid, response.message
                assert not response.has_path_check
                assert response.joint_state.name == metadata['left_arm']['joint_names']+metadata['right_arm']['joint_names']
                expected = list(features['left_arm'][left].weight_angle)+list(features['right_arm'][right].weight_angle)
                assert list(response.joint_state.position) == expected
                if response.is_collision_free:
                    safe_pairs.append((left, right, response.joint_state))
                else:
                    colliding_pairs.append({'left': left, 'right': right, 'pairs': response.collision_pairs})
                if colliding_pairs and len(safe_pairs) >= 16:
                    break
            assert safe_pairs, '安全な組合せなし'
            left, right, start = safe_pairs[0]
            result = call(left, right, start)
            assert result.is_valid and result.is_collision_free and result.has_path_check
            assert not call(-1, right).is_valid
            for kind in ('missing', 'duplicate', 'nonfinite', 'fixed', 'outside_limits'):
                invalid = copy.deepcopy(start)
                if kind == 'missing':
                    invalid.name.pop(); invalid.position.pop()
                elif kind == 'duplicate':
                    invalid.name.append(invalid.name[0]); invalid.position.append(invalid.position[0])
                elif kind == 'nonfinite':
                    invalid.position[0] = float('nan')
                elif kind == 'fixed':
                    invalid.name.append('waist_joint'); invalid.position.append(.1)
                else:
                    invalid.position[1] = 100.0
                assert not call(left, right, invalid).is_valid, kind
            paths = []
            for next_left, next_right, _ in safe_pairs[1:17]:
                response = call(next_left, next_right, start)
                assert response.is_valid and response.has_path_check
                paths.append({'left': next_left, 'right': next_right,
                              'is_collision_free': response.is_collision_free,
                              'pairs': response.collision_pairs})
            report = {'result': 'pass', 'nodes': {a: metadata[a]['num_nodes'] for a in metadata},
                      'joint_dimensions': [7, 7], 'num_safe_pairs': len(safe_pairs),
                      'colliding_pairs': colliding_pairs, 'paths': paths,
                      'invalid_inputs_rejected': True, 'launch_command': command}
            (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report), flush=True)
    finally:
        if process is not None:
            # 起動したlaunchと同じプロセスグループだけの終了処理
            if process.poll() is None:
                owned_pids = {int(value) for value in subprocess.check_output(
                    ['ps', '-o', 'pid=', '-g', str(process.pid)], text=True).split()}
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            for pid in owned_pids:
                for path in Path('/tmp').glob('gng_safety_resolved_robot_'+str(pid)+'_*.urdf'):
                    path.unlink(missing_ok=True)
        node.destroy_node()
        rclpy.shutdown()
        print('所有試験launch: 終了済み', flush=True)


if __name__ == '__main__':
    main()
