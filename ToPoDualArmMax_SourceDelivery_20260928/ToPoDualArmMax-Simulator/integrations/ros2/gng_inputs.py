"""実環境の読取り専用GNG入力。シミュレーション姿勢・指令・TFの配信なし。"""
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
from scipy.spatial import cKDTree
import yaml

workspace = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(workspace / 'gng_vlut_system/scripts'))
from gng_avoidance_planner import graph_topology_hash, voxel_centers


def load_models(params_file):
    path = Path(params_file).resolve()
    params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    config = params['gng']
    if not config.get('enable_independent_arms'):
        raise ValueError('ブラウザGNG回避には独立グループの学習モデルが必要です')
    directory = Path(config['data_directory']) / config['experiment_id']
    manifest = json.loads((directory / 'independent_arms.json').read_text())
    models = {}
    for entry in manifest['profiles']:
        model_path = (directory / entry['metadata']).resolve()
        if not model_path.is_relative_to(directory.resolve()):
            raise ValueError('モデル保存先の不正')
        model = json.loads(model_path.read_text())
        if model['profile'] != entry['name'] or model['urdf_path'] != params['urdf_path']:
            raise ValueError('学習モデルと機体設定の不一致')
        for key in ('gng_file', 'vlut_file'):
            if not (model_path.parent / model[key]).is_file():
                raise ValueError('学習済みファイルがありません: '+str(model_path.parent / model[key]))
        names = model['joint_names']
        if not names or len(names) != len(set(names)) or model['num_nodes'] <= 0:
            raise ValueError('学習モデルの関節名またはノード数の不正')
        models[entry['name']] = model
    if not models:
        raise ValueError('学習グループが空です')
    return params, models


class gng_inputs:
    def __init__(self, node, params, models, root_link, max_age_sec=1.0):
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from tf2_ros import Buffer, TransformListener
        from ais_gng_msgs.msg import TopologicalMap, TopologicalNodeStates
        from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray
        from voxel_msgs.msg import Voxel
        self.node, self.models, self.max_age_sec = node, models, max_age_sec
        self.lock = threading.Lock()
        self.is_closed = False
        self.graphs, self.stamps = {}, {}
        self.cloud = None
        self.error = ''
        self.subscriptions = []
        self.fixed_subscriptions = {}
        self.root_frame = params['robot_name']+'/'+root_link
        self.graph_frame = params['robot_name']+'/'+params['environment_voxelization'].get('base_frame', 'base_link')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            for name in models:
                prefix = '/'+params['robot_name']+'/'
                for topic, kind, callback in (
                    ('Tmap_'+name, TopologicalMap, self.on_graph),
                    (name+'/topological_node_features', TopologicalNodeFeatureArray, self.on_features),
                    (name+'/gng_node_states_stamped', TopologicalNodeStates, self.on_states)):
                    subscription = node.create_subscription(
                        kind, prefix+topic, lambda msg, key=name, fn=callback: self.receive(fn, key, msg), qos)
                    self.subscriptions.append(subscription)
                    if callback.__name__ != 'on_states':
                        self.fixed_subscriptions[name, callback.__name__] = subscription
            self.subscriptions.append(node.create_subscription(Voxel,
                params['environment_voxelization']['voxel_topic'],
                lambda msg: self.receive(self.on_voxels, 'voxels', msg), qos))
        except Exception:
            self.close()
            raise

    def receive(self, callback, name, message):
        from tf2_ros import TransformException
        with self.lock:
            if self.is_closed:
                return
            try:
                callback(name, message)
                key = (name, callback.__name__)
                if key in self.fixed_subscriptions:
                    subscription = self.fixed_subscriptions.pop(key)
                    self.node.destroy_subscription(subscription)
                    self.subscriptions.remove(subscription)
            except TransformException:
                # 起動時のTF未受信は入力待機。運転中の継続欠測は期限切れ停止
                self.cloud = None
            except Exception as error:
                self.error = str(error)

    def accept_stamp(self, name, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds-stamp)*1e-9
        if not 0 <= age <= self.max_age_sec or stamp <= self.stamps.get(name, -1):
            return False
        self.stamps[name] = stamp
        return True

    def on_graph(self, name, message):
        ids, edges = tuple(node.id for node in message.nodes), tuple(message.edges)
        if (message.header.frame_id != self.graph_frame or len(ids) != self.models[name]['num_nodes']
                or len(set(ids)) != len(ids) or len(edges) % 2
                or any(idx < 0 or idx >= len(ids) for idx in edges)):
            raise ValueError('GNGトポロジー・座標系の不一致: '+name)
        old = self.graphs.get(name, {})
        topology = graph_topology_hash(ids, edges)
        if old.get('hash') == topology:
            return
        if 'hash' in old:
            raise ValueError('GNG再読込みのため物理セッションの再開始が必要です')
        adjacency = {idx: [] for idx in ids}
        for first, second in zip(edges[::2], edges[1::2]):
            adjacency[ids[first]].append(ids[second])
            adjacency[ids[second]].append(ids[first])
        self.graphs[name] = dict(old, ids=ids, adjacency=adjacency, hash=topology)

    def on_features(self, name, message):
        old = self.graphs.get(name, {})
        if 'angles' in old:
            return
        angles = {item.node_id: np.asarray(item.weight_angle, dtype=float) for item in message.features}
        num = len(self.models[name]['joint_names'])
        if (len(angles) != self.models[name]['num_nodes'] or len(angles) != len(message.features)
                or any(value.shape != (num,) or not np.all(np.isfinite(value)) for value in angles.values())):
            raise ValueError('GNG関節角配列の不一致: '+name)
        self.graphs[name] = dict(old, angles=angles, angle_node_ids=tuple(angles),
                                 angle_tree=cKDTree(np.array(list(angles.values()))))

    def on_states(self, name, message):
        old = self.graphs.get(name, {})
        if 'ids' not in old:
            return
        if (message.header.frame_id != self.graph_frame or message.topology_hash != old['hash']
                or tuple(message.node_ids) != old['ids'] or len(message.labels) != len(old['ids'])
                or any(value not in (1, 2, 3) for value in message.labels)):
            raise ValueError('VLUT状態とGNGトポロジーの不一致: '+name)
        if self.accept_stamp(name, message):
            self.graphs[name] = dict(old, labels=dict(zip(message.node_ids, message.labels)), received=time.monotonic())

    def on_voxels(self, name, message):
        from rclpy.time import Time
        from scipy.spatial.transform import Rotation
        if (not np.isfinite(message.voxel_size) or message.voxel_size <= 0
                or sorted((message.x_shift, message.y_shift, message.z_shift)) != [0, 21, 42]
                or not np.all(np.isfinite([message.origin_x, message.origin_y, message.origin_z]))):
            raise ValueError('環境ボクセル形式の不正')
        if not self.accept_stamp(name, message):
            return
        transform = np.eye(4)
        if message.header.frame_id != self.root_frame:
            # 専用TFスレッド不要の即時参照。変換なしの暗黙処理は禁止
            tf = self.buffer.lookup_transform(self.root_frame, message.header.frame_id,
                                               Time.from_msg(message.header.stamp)).transform
            transform[:3, :3] = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]).as_matrix()
            transform[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
        if not len(message.data):
            self.cloud = None
            return
        points = voxel_centers(message, transform)
        self.cloud = (cKDTree(points), message.voxel_size*np.sqrt(3)/2, time.monotonic())

    def snapshot(self):
        with self.lock:
            if self.error:
                raise ValueError(self.error)
            now = time.monotonic()
            missing = []
            if self.cloud is None or now-self.cloud[2] > self.max_age_sec:
                missing.append('自己除去後の環境ボクセル')
            for name in self.models:
                graph = self.graphs.get(name, {})
                if ('angles' not in graph or 'labels' not in graph
                        or set(graph.get('ids', ())) != set(graph.get('angles', {}))
                        or now-graph.get('received', 0) > self.max_age_sec):
                    missing.append(name+' GNG/VLUT')
            return (None, '入力待機: '+', '.join(missing)) if missing else ((self.cloud, dict(self.graphs)), '')

    def close(self):
        with self.lock:
            self.is_closed = True
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        self.listener.unregister()
