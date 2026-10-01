#!/usr/bin/env python3
"""実レイ点群・VLUT状態・学習済み姿勢グラフによるGazebo専用退避。"""
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
import heapq
import json
import time

import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String, UInt16MultiArray
from voxel_msgs.msg import Voxel
from ais_gng_msgs.msg import TopologicalMap, TopologicalNodeStates
from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray

from dual_arm_avoidance_demo import avoidance_demo


def graph_topology_hash(ids, edges):
    """固定グラフのID順序・辺配列に対する64 bit照合値。"""
    result = 14695981039346656037
    for value in (len(ids), *ids, len(edges), *edges):
        result = ((result ^ int(value)) * 1099511628211) & ((1 << 64)-1)
    return result


def has_safe_node_neighbors(labels, adjacency, node_id):
    """退避先自身と辺で直接つながるノードの安全確認。欠測は不許可。"""
    neighbors = adjacency.get(node_id)
    return neighbors is not None and all(labels.get(idx) == 1 for idx in (node_id, *neighbors))


def has_stable_return_clearance(state, has_clear_return):
    """危険時の即時解除と、安全継続時間による復帰許可。実時間基準。"""
    now = time.monotonic()
    previous = getattr(state, 'return_check_sec', None)
    state.return_check_sec = now
    if not has_clear_return:
        state.return_clear_since_sec = None
        return False
    if (getattr(state, 'return_clear_since_sec', None) is None or previous is None
            or now < previous or now-previous > state.config.get('max_state_age_sec', 1.0)):
        # 入力確認の中断・時計の巻戻りをまたぐ安全時間の持越し防止
        state.return_clear_since_sec = now
    return now-state.return_clear_since_sec >= state.config.get('return_clear_sec', .5)


class gng_path_search:
    def has_safe_measured_neighbors(self):
        """Python計画の実測最寄り姿勢と一次隣接の安全確認。"""
        if not self.angles:
            return False
        current = self.positions[self.arm_indices]
        node_id = min(self.angles, key=lambda idx: float(np.linalg.norm(self.angles[idx]-current)))
        return has_safe_node_neighbors(self.labels, self.adjacency, node_id)

    def pose(self, node_id):
        result = self.positions.copy()
        result[np.asarray(self.arm_indices)[self.active_angle_indices]] = self.angles[node_id][self.active_angle_indices]
        return result

    def select_active_arms(self):
        centers = self.geometry.centers(self.positions)
        groups = getattr(self, 'planning_groups', None)
        if groups is None:
            # 旧maxデモの設定との互換。共通構成では任意名のグループを指定
            groups = [{'name': side, 'joint_names': [name for name in self.arm_names if name.startswith(side+'_')],
                       'link_names': [name for name, _, _ in self.geometry.spheres if name.startswith(side+'_')]}
                      for side in ('L', 'R')]
        active_groups = []
        for group in groups:
            is_side = self.geometry.is_arm & np.array([
                name in group['link_names'] for name, _, _ in self.geometry.spheres])
            if not np.any(is_side):
                continue
            gap = np.min(self.cloud_tree.query(centers[is_side])[0]
                         -self.geometry.radii[is_side]-self.cell_radius)
            if gap < self.config['min_retreat_dist_th']:
                active_groups.append(group)
        if not active_groups:
            # 退避経路の完了まで対象腕を維持。復帰は片腕ずつの実行
            if self.path:
                return self.active_angle_indices.copy()
            for group in groups:
                indices = [self.arm_indices[self.arm_names.index(name)] for name in group['joint_names']]
                if np.max(np.abs(self.positions[indices]-self.home[indices])) > 1e-5:
                    active_groups = [group]
                    break
        active_names = {name for group in active_groups for name in group['joint_names']}
        requested_indices = np.array([idx for idx, name in enumerate(self.arm_names)
                                      if name in active_names], dtype=int)
        if self.path and np.array_equal(requested_indices, self.coordination_source_indices):
            return self.active_angle_indices.copy()
        return requested_indices

    def cloud_clearance(self, positions):
        centers = self.geometry.centers(positions)
        is_arm = self.geometry.is_arm
        distances = self.cloud_tree.query(centers[is_arm])[0]
        return float(np.min(distances-self.geometry.radii[is_arm]-self.cell_radius)), centers

    def can_bridge(self, first, second, min_gap):
        # 実行監視の停止距離と経路検査下限の整合
        min_gap = max(min_gap, self.config['min_clearance_th'])
        count = max(2, int(np.ceil(np.max(np.abs(second-first))/self.config['max_bridge_step'])))
        for ratio in np.linspace(0, 1, count+1)[1:]:
            gap, centers = self.cloud_clearance(first+(second-first)*ratio)
            if gap < min_gap:
                return False
            if not self.has_planning_clearance(centers):
                if not self.geometry.has_inter_arm_clearance(centers, self.config.get('min_internal_clearance_th', .005)):
                    self.has_inter_arm_rejection = True
                return False
        return True

    def has_planning_clearance(self, centers):
        if 'min_planning_clearance_th' in self.config:
            return self.geometry.has_internal_clearance(centers, self.config['min_planning_clearance_th'])
        return self.geometry.has_internal_clearance(centers)

    def plan_with_coordination(self, current_gap):
        source_indices = self.active_angle_indices.copy()
        path = self.plan(current_gap)
        # 片腕探索の失敗理由に左右干渉が含まれる場合だけの協調再探索
        if (not path and self.has_inter_arm_rejection and not self.has_timed_out
                and 0 < len(source_indices) < len(self.arm_indices)):
            self.active_angle_indices = np.arange(len(self.arm_indices))
            path = self.plan(current_gap)
        if not path:
            self.active_angle_indices = source_indices
        return dict(path=path, active_angle_indices=self.active_angle_indices,
                    coordination_source_indices=(source_indices if path and
                        not np.array_equal(source_indices, self.active_angle_indices)
                        else np.array([], dtype=int)))

    def plan(self, current_gap):
        self.has_inter_arm_rejection = False
        self.has_timed_out = False
        deadline = time.monotonic()+self.config['max_plan_sec']
        safe_ids = [idx for idx in self.angles if self.labels.get(idx) == 1]
        safe_ids.sort(key=lambda idx: float(np.max(np.abs(self.pose(idx)-self.positions))))
        queue, costs, previous = [], {}, {}
        min_gap = max(self.config['min_cloud_clearance_th'], min(current_gap-0.005, self.config['target_clearance']))
        for idx in safe_ids[:self.config['max_entry_candidates']]:
            if time.monotonic() > deadline:
                self.has_timed_out = True
                return []
            target = self.pose(idx)
            if self.can_bridge(self.positions, target, min_gap):
                cost = float(np.max(np.abs(target-self.positions)))
                costs[idx], previous[idx] = cost, None
                heapq.heappush(queue, (cost, idx))
                # 実測姿勢から接続可能な近傍始点の上限
                if len(queue) >= 3:
                    break
        while queue:
            if time.monotonic() > deadline:
                self.has_timed_out = True
                return []
            cost, idx = heapq.heappop(queue)
            if cost != costs[idx]:
                continue
            pose = self.pose(idx)
            if (has_safe_node_neighbors(self.labels, self.adjacency, idx)
                    and self.cloud_clearance(pose)[0] >= self.config['target_clearance']):
                path = []
                while idx is not None:
                    path.append(idx)
                    idx = previous[idx]
                path = path[::-1]
                if all(self.can_bridge(self.pose(a), self.pose(b), min_gap) for a, b in zip(path, path[1:])):
                    return path
                continue
            for adjacent in self.adjacency.get(idx, []):
                if self.labels.get(adjacent) != 1 or adjacent not in self.angles:
                    continue
                target = self.pose(adjacent)
                next_cost = cost+float(np.max(np.abs(target-pose)))
                if next_cost < costs.get(adjacent, float('inf')):
                    costs[adjacent], previous[adjacent] = next_cost, idx
                    heapq.heappush(queue, (next_cost, adjacent))
        self.has_timed_out = time.monotonic() > deadline
        return []



class gng_lidar_demo(avoidance_demo, gng_path_search):
    def __init__(self):
        self.cloud_time = self.voxel_time = self.graph_time = 0.0
        self.cloud_tree = None
        self.angles = {}
        self.labels = {}
        self.adjacency = {}
        self.graph_ids = ()
        self.graph_edges = None
        self.path = []
        self.plan_future = None
        self.plan_pool = ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn'))
        self.num_local_steps = 0
        self.next_plan_sec = 0.0
        self.num_plans = 0
        self.num_selected_gng = 0
        self.num_cloud = 0
        self.num_voxels = 0
        self.cloud_gap = None
        self.last_cloud_stamp = -1
        self.last_voxel_stamp = -1
        self.graph_message = None
        self.cell_radius = 0.02*np.sqrt(3)/2
        self.return_clear_since_sec = None
        self.return_check_sec = None
        super().__init__()
        return_clear_sec = self.config.get('return_clear_sec', .5)
        if (isinstance(return_clear_sec, bool) or not isinstance(return_clear_sec, (int, float))
                or not np.isfinite(return_clear_sec) or return_clear_sec < 0):
            raise ValueError('return_clear_secには有限の非負数が必要です')
        for key in ('max_plan_sec', 'min_retreat_dist_th', 'min_cloud_clearance_th', 'max_bridge_step', 'max_entry_candidates'):
            if not np.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key}は有限の正数が必要です')
        if not isinstance(self.config['max_entry_candidates'], int):
            raise ValueError('max_entry_candidatesは整数が必要です')
        self.planning_groups = self.config.get('planning_groups')
        self.arm_names = ([name for group in self.planning_groups for name in group['joint_names']]
                          if self.planning_groups is not None else
                          [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)])
        self.arm_indices = [self.geometry.joint_names.index(name) for name in self.arm_names]
        self.active_angle_indices = np.array([], dtype=int)
        self.qp = None
        if self.config.get('local_qp', {}).get('enable_qp', False):
            from local_qp import local_qp
            self.qp = local_qp(self.geometry,
                [self.joint_limits[name]['velocity'] for name in self.geometry.joint_names], self.config)
        self.enable_native_planner = bool(self.config.get('enable_native_planner', False))
        self.max_home_error_th = self.config.get('max_home_error_th', .015)
        if not np.isfinite(self.max_home_error_th) or self.max_home_error_th <= 0:
            raise ValueError('復帰判定の関節誤差は有限の正数が必要です')
        self.angle_tree = None
        self.current_node_id = None
        self.has_safe_neighbors = False
        self.native_target = None
        self.native_target_time = 0.0
        self.native_target_stamp = -1
        self.native_node_path = []
        self.num_native_plans = 0
        if self.enable_native_planner:
            self.create_subscription(JointState, 'native_avoidance_target', self.on_native_target, qos_profile_sensor_data)
        self.coordination_source_indices = np.array([], dtype=int)
        self.diag = self.create_publisher(String, 'avoidance/gng_status', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.enable_native_planner:
            self.create_subscription(TopologicalMap, 'plan_Tmap', self.on_native_path, qos)
        self.path_pub = self.create_publisher(TopologicalMap, 'plan_Tmap', qos)
        pipeline = self.config.get('pipeline', {})
        self.external_environment = pipeline.get('external_environment')
        self.source_stamps = {}
        self.real_joint_time = 0.0
        self.voxel_frame = self.get_namespace().strip('/') + '/' + pipeline.get('base_frame', 'base_link')
        source = self.external_environment or {}
        self.create_subscription(PointCloud2, source.get('points_topic', pipeline.get('points_topic', 'lidar_points')), self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Voxel, source.get('voxel_topic', 'self_filter_roi_voxels'), self.on_voxels, qos)
        self.graph_sub = self.create_subscription(TopologicalMap, source.get('graph_topic', 'Tmap_static'), self.on_graph, qos)
        if self.external_environment:
            self.create_subscription(JointState, source['joint_topic'], self.on_real_joints, qos_profile_sensor_data)
            if source.get('state_topic'):
                self.create_subscription(TopologicalNodeStates, source['state_topic'], self.on_stamped_states, qos)
        else:
            self.create_subscription(UInt16MultiArray, 'gng_node_states', self.on_states, qos)
        self.feature_sub = self.create_subscription(TopologicalNodeFeatureArray, source.get('feature_topic', 'topological_node_features'), self.on_features, qos)

    def can_accept_environment_sample(self, message, kind, frame=None):
        # 実時間の入力期限と単調増加stamp。Gazebo時刻との混同・キャッシュの再利用防止
        source = getattr(self, 'external_environment', None)
        if not source:
            return True
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (time.time_ns()-stamp)*1e-9
        if (frame is not None and message.header.frame_id != frame) or not 0 <= age <= source['max_input_age_sec']:
            return False
        if stamp <= self.source_stamps.get(kind, -1):
            return False
        self.source_stamps[kind] = stamp
        return True

    def has_fresh_environment(self):
        source = getattr(self, 'external_environment', None)
        if not source:
            return True
        now = time.time_ns()
        # 到着遅延込みの発生時刻基準。受信後に入力期限を延長しない継続判定
        return all(kind in self.source_stamps and
                   0 <= (now-self.source_stamps[kind])*1e-9 < source['max_input_age_sec']
                   for kind in ('cloud', 'voxels', 'graph', 'joints'))

    def on_real_joints(self, message):
        # 自己除去の根拠となる実機全関節。センサー校正値の丸め・URDF制限への置換なし
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and np.isfinite(positions[name]) for name in self.geometry.joint_names):
            return
        if self.can_accept_environment_sample(message, 'joints'):
            self.real_joint_time = time.monotonic()

    def on_start(self, request, response):
        response = super().on_start(request, response)
        if response.success:
            has_stable_return_clearance(self, False)
            self.path = []
            if self.plan_future is not None:
                self.plan_future.cancel()
                self.plan_future = None
            self.next_plan_sec = 0.0
        return response

    def on_safety_stop(self, message):
        super().on_safety_stop(message)
        if self.is_stop_latched:
            has_stable_return_clearance(self, False)
            self.path = []
            if self.plan_future is not None:
                self.plan_future.cancel()
                self.plan_future = None
            self.next_plan_sec = 0.0
            self.coordination_source_indices = np.array([], dtype=int)

    def on_cloud(self, message):
        if not self.can_accept_environment_sample(message, 'cloud'):
            return
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if stamp <= self.last_cloud_stamp:
            return
        self.last_cloud_stamp = stamp
        if message.width*message.height > 0:
            self.cloud_time = time.monotonic()
            self.num_cloud += 1

    def on_voxels(self, message):
        source = getattr(self, 'external_environment', None)
        frame = source['source_frame'] if source else self.voxel_frame
        if message.header.frame_id != frame or not self.can_accept_environment_sample(message, 'voxels', frame):
            return
        if (not np.isfinite(message.voxel_size) or message.voxel_size <= 0 or
                sorted((message.x_shift, message.y_shift, message.z_shift)) != [0, 21, 42] or
                not np.all(np.isfinite([message.origin_x, message.origin_y, message.origin_z]))):
            return
        stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if stamp <= self.last_voxel_stamp:
            return
        self.last_voxel_stamp = stamp
        ids = np.asarray(message.data, dtype=np.int64)
        if len(ids) == 0:
            self.cloud_tree = None
            return
        mask = (1 << 21)-1
        cells = np.column_stack([((ids >> shift) & mask)-message.offset for shift in
                                 (message.x_shift, message.y_shift, message.z_shift)])
        points = (cells+.5)*message.voxel_size+np.array([message.origin_x, message.origin_y, message.origin_z])
        if source:
            transform = np.asarray(source['root_from_source'])
            points = points @ transform[:3, :3].T + transform[:3, 3]
        self.cloud_tree = cKDTree(points)
        self.cell_radius = message.voxel_size*np.sqrt(3)/2
        self.voxel_time = time.monotonic()
        self.num_voxels = len(ids)

    def on_features(self, message):
        if any(len(feature.weight_angle) != len(self.arm_names) or not np.all(np.isfinite(feature.weight_angle))
               for feature in message.features):
            self.fail(f'GNG角度配列と計画関節の不一致: 期待関節数 {len(self.arm_names)}')
            return
        for feature in message.features:
            if feature.node_id in self.angles:
                continue
            self.angles[feature.node_id] = np.asarray(feature.weight_angle)
        # 起動時固定の学習済み関節角。初回完全取得後の反復受信を停止
        if self.feature_sub is not None and message.features and len(self.angles) == len(message.features):
            self.angle_node_ids = tuple(self.angles)
            self.angle_tree = cKDTree(np.asarray([self.angles[idx] for idx in self.angle_node_ids]))
            self.destroy_subscription(self.feature_sub)
            self.feature_sub = None

    def on_graph(self, message):
        source = getattr(self, 'external_environment', None)
        if source and not self.can_accept_environment_sample(message, 'graph', source['source_frame']):
            return
        self.labels = {node.id: node.label for node in message.nodes}
        ids = tuple(node.id for node in message.nodes)
        if not source or ids != self.graph_ids or message.edges != self.graph_edges:
            if source:
                transform = np.asarray(source['root_from_source'])
                for node in message.nodes:
                    point = transform[:3, :3] @ [node.pos.x, node.pos.y, node.pos.z] + transform[:3, 3]
                    node.pos.x, node.pos.y, node.pos.z = map(float, point)
                    normal = transform[:3, :3] @ [node.normal.x, node.normal.y, node.normal.z]
                    node.normal.x, node.normal.y, node.normal.z = map(float, normal)
                message.header.frame_id = self.voxel_frame
            self.graph_message = message
        # 安全ラベルのみ変わる学習済みグラフの隣接表再利用
        if ids != self.graph_ids or message.edges != self.graph_edges:
            self.adjacency = {idx: [] for idx in ids}
            for first, second in zip(message.edges[::2], message.edges[1::2]):
                if first < len(ids) and second < len(ids):
                    a, b = ids[first], ids[second]
                    self.adjacency[a].append(b)
                    self.adjacency[b].append(a)
            self.graph_ids, self.graph_edges = ids, message.edges
            self.graph_topology_hash = graph_topology_hash(ids, message.edges)
        self.graph_time = time.monotonic()
        if ids and (not source or source.get('state_topic')) and getattr(self, 'graph_sub', None) is not None:
            self.destroy_subscription(self.graph_sub)
            self.graph_sub = None

    def on_stamped_states(self, message):
        source = self.external_environment
        if (not self.graph_ids or message.topology_hash != self.graph_topology_hash
                or tuple(message.node_ids) != self.graph_ids or len(message.labels) != len(self.graph_ids)
                or any(value not in (1, 2, 3) for value in message.labels)):
            return
        if not self.can_accept_environment_sample(message, 'graph', source['source_frame']):
            return
        self.labels = dict(zip(message.node_ids, message.labels))
        self.graph_time = time.monotonic()

    def on_states(self, message):
        # 起動時固定のトポロジーに対する安全状態だけの更新
        if not self.graph_ids or len(message.data) != 2*len(self.graph_ids):
            return
        labels = dict(zip(message.data[::2], message.data[1::2]))
        if set(labels) != set(self.graph_ids) or any(value not in (1, 2, 3) for value in labels.values()):
            return
        self.labels = labels
        self.graph_time = time.monotonic()

    def is_fresh(self):
        now = time.monotonic()
        has_native_target = (not self.enable_native_planner or (self.native_target is not None and now-self.native_target_time < .5))
        return (has_native_target and super().is_fresh() and self.has_fresh_environment() and self.cloud_tree is not None and bool(self.angles) and
                set(self.angles) == set(self.graph_ids) and
                all(now-stamp < self.config['max_state_age_sec'] for stamp in
                    (self.cloud_time, self.voxel_time, self.graph_time)))

    def observe_clearance(self):
        if not self.config.get('enable_live_obstacles', False):
            return super().observe_clearance()
        centers = self.geometry.centers(self.positions)
        indices = np.flatnonzero(self.geometry.is_arm)
        distances, nearest = self.cloud_tree.query(centers[indices])
        gaps = distances-self.geometry.radii[indices]-self.cell_radius
        idx = int(np.argmin(gaps))
        self.obstacle_time = self.voxel_time
        return float(gaps[idx]), centers, int(indices[idx]), self.cloud_tree.data[nearest[idx]]

    def freshness_detail(self):
        now = time.monotonic()
        ages = {name+'_age_sec': now-stamp for name, stamp in (
            ('joint', self.joint_time), ('cloud', self.cloud_time), ('voxel', self.voxel_time),
            ('graph', self.graph_time), ('real_joint', self.real_joint_time),
            ('native_target', self.native_target_time))}
        ages.update(has_native_target=self.native_target is not None,
                    has_cloud_tree=self.cloud_tree is not None,
                    has_matching_graph=set(self.angles) == set(self.graph_ids),
                    source_age_sec={key: (time.time_ns()-stamp)*1e-9 for key, stamp in self.source_stamps.items()})
        return json.dumps(ages, ensure_ascii=False)

    def refine_target(self, step):
        if not self.config['enable_local_refinement']:
            return self.positions.copy(), False
        if getattr(self, 'qp', None) is not None:
            # 出力直前のQPで近傍表面からの退避方向を決定。
            return self.positions.copy(), True
        # 疎なGNGで橋渡しが成立しない場合の、観測点群による微小退避
        best = self.positions.copy()
        best_cost = float('inf')
        candidates = [best]
        for idx in np.asarray(self.arm_indices)[self.active_angle_indices]:
            for sign in (-1, 1):
                candidate = self.positions.copy()
                candidate[idx] += sign*step
                candidate[idx] = np.clip(candidate[idx], *self.geometry.limits[idx])
                candidates.append(candidate)
        for candidate in candidates:
            gap, centers = self.cloud_clearance(candidate)
            if gap < self.cloud_gap-.0002 or not self.has_planning_clearance(centers):
                continue
            cost = 200*max(0, self.config['target_clearance']-gap)**2+.003*float(np.sum((candidate-self.home)**2))
            if cost < best_cost and self.can_bridge(self.positions, candidate, self.config['min_cloud_clearance_th']):
                best, best_cost = candidate, cost
        self.num_local_steps += int(np.max(np.abs(best-self.positions)) > 1e-5)
        return best, bool(np.isfinite(best_cost))

    def on_native_path(self, message):
        # C++計画の診断用経路。現在姿勢を表す仮想ノードは集計対象外
        node_path = [node.id for node in message.nodes if node.id != 65535]
        if node_path and node_path != self.native_node_path:
            self.num_native_plans += 1
        self.native_node_path = node_path

    def on_native_target(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        if stamp <= self.native_target_stamp or not 0 <= age < .5:
            return
        values = dict(zip(message.name, message.position))
        if (len(message.name) != len(message.position) or len(values) != len(message.name)
                or not all(name in values and np.isfinite(values[name]) for name in self.arm_names)):
            self.native_target = None
            return
        for name, idx in zip(self.arm_names, self.arm_indices):
            if not self.geometry.limits[idx, 0] <= values[name] <= self.geometry.limits[idx, 1]:
                self.native_target = None
                return
        self.native_target = np.array([values[name] for name in self.arm_names])
        self.native_target_stamp, self.native_target_time = stamp, time.monotonic()

    def has_safe_first_neighbors(self):
        """実測関節に最も近いGNGノードと一次近傍の安全ラベル判定。欠測は不許可。"""
        self.has_safe_neighbors = False
        if self.angle_tree is None:
            self.current_node_id = None
            return False
        _, idx = self.angle_tree.query(self.positions[self.arm_indices])
        self.current_node_id = self.angle_node_ids[int(idx)]
        self.has_safe_neighbors = has_safe_node_neighbors(self.labels, self.adjacency, self.current_node_id)
        return self.has_safe_neighbors

    def can_resume_obstacle(self):
        """実測姿勢の最寄りGNGノードと直接隣接の安全確認。"""
        return self.has_safe_first_neighbors()

    def select_native_target(self, step):
        """既存C++退避目標の速度制限・観測点群での区間検査。疎なグラフでは局所補正。"""
        self.cloud_gap, _ = self.cloud_clearance(self.positions)
        if self.cloud_gap < self.config['min_cloud_clearance_th'] or self.native_target is None:
            has_stable_return_clearance(self, False)
            return self.positions.copy(), False
        self.active_angle_indices = self.select_active_arms()
        has_safe_neighbors = self.has_safe_first_neighbors()
        if not has_safe_neighbors:
            # 距離による片腕選択でグラフ退避目標を変形しない、計画対象関節全体の追従
            self.active_angle_indices = np.arange(len(self.arm_indices))
        if not len(self.active_angle_indices):
            has_stable_return_clearance(self, False)
            self.phase = 'monitoring'
            return self.positions.copy(), True
        active = np.asarray(self.arm_indices)[self.active_angle_indices]
        home = self.positions.copy()
        home[active] = self.home[active]
        target = self.positions.copy()
        # 点群接近時の退避優先。退避完了後の復帰中は隣接状態と経路の再検査
        # 点群退避目標の離散化許容 [m]
        max_retreat_clearance_dev_th = .001
        can_finish_retreat = (getattr(self, 'phase', '') == 'returning'
                              or self.config['target_clearance']-self.cloud_gap <= max_retreat_clearance_dev_th)
        has_clear_return = (has_safe_neighbors
                            and can_finish_retreat
                            and self.can_bridge(self.positions, home, self.config['min_clearance_th']))
        can_return = has_stable_return_clearance(self, has_clear_return)
        if can_return:
            self.phase = ('monitoring' if np.max(np.abs(home-self.positions)) <= self.max_home_error_th
                          else 'returning')
            target = home
        elif can_finish_retreat and has_safe_neighbors:
            # 安全継続の確認中・復帰経路の閉塞中は退避と復帰の間で保持
            self.phase = 'waiting_for_clearance'
            return self.positions.copy(), True
        else:
            self.phase = 'avoiding'
            target[active] = self.native_target[self.active_angle_indices]
        delta = target-self.positions
        target = self.positions+delta*min(1., step/max(float(np.max(np.abs(delta))), 1e-9))
        target_gap = self.cloud_clearance(target)[0]
        if np.max(np.abs(target-self.positions)) < 1e-5 and can_return:
            return target, True
        if (np.max(np.abs(target-self.positions)) < 1e-5
                or (has_safe_neighbors and not can_return and target_gap < min(self.cloud_gap, self.config['target_clearance'])-.0002)
                or (has_safe_neighbors and not can_return and self.cloud_gap < self.config['target_clearance'] and target_gap <= self.cloud_gap+.0002)
                or not self.can_bridge(self.positions, target, self.config['min_cloud_clearance_th'])):
            return self.refine_target(step)
        if not can_return and self.native_node_path and np.max(np.abs(target-self.positions)) > 1e-5:
            self.num_selected_gng += 1
        return target, True

    def select_target(self, _hand, _elbow, step):
        if self.enable_native_planner:
            return self.select_native_target(step)
        # 障害物の真値位置は計画に不使用。自己除去後の観測点とVLUT状態のみの利用
        is_gng_target = False
        self.cloud_gap, _ = self.cloud_clearance(self.positions)
        if self.cloud_gap < self.config['min_cloud_clearance_th']:
            has_stable_return_clearance(self, False)
            centers = self.geometry.centers(self.positions)
            indices = np.flatnonzero(self.geometry.is_arm)
            distances, nearest = self.cloud_tree.query(centers[indices])
            idx = int(np.argmin(distances-self.geometry.radii[indices]-self.cell_radius))
            self.get_logger().error(f'点群クリアランス不足: link={self.geometry.spheres[indices[idx]][0]}, '
                                    f'gap={self.cloud_gap:.6f}, point={self.cloud_tree.data[nearest[idx]].tolist()}')
            return self.positions.copy(), False
        has_safe_neighbors = self.has_safe_measured_neighbors()
        active_angle_indices = (self.select_active_arms() if has_safe_neighbors
                                else np.arange(len(self.arm_indices)))
        if not np.array_equal(active_angle_indices, self.active_angle_indices):
            self.active_angle_indices = active_angle_indices
            self.coordination_source_indices = np.array([], dtype=int)
            self.path = []
            if self.plan_future is not None:
                self.plan_future.cancel()
                self.plan_future = None
            self.next_plan_sec = 0.0
        if not len(self.active_angle_indices):
            has_stable_return_clearance(self, False)
            return self.positions.copy(), True
        # 非対象腕・胴体・指は実測姿勢に固定
        home_target = self.positions.copy()
        active_joint_indices = np.asarray(self.arm_indices)[self.active_angle_indices]
        home_target[active_joint_indices] = self.home[active_joint_indices]
        home_gap, _ = self.cloud_clearance(home_target)
        has_clear_return = (has_safe_neighbors and not self.path
                            and home_gap >= self.config['min_retreat_dist_th']
                            and self.can_bridge(self.positions, home_target, self.config['min_cloud_clearance_th']))
        can_return = has_stable_return_clearance(self, has_clear_return)
        if has_clear_return:
            target = home_target if can_return else self.positions
        elif has_safe_neighbors and self.cloud_gap >= self.config['min_retreat_dist_th'] and not self.path:
            target = self.positions
        else:
            if self.path and (any(self.labels.get(idx) != 1 for idx in self.path)
                              or not has_safe_node_neighbors(self.labels, self.adjacency, self.path[-1])):
                self.path = []
            if not self.path:
                if len(self.coordination_source_indices):
                    self.coordination_source_indices = np.array([], dtype=int)
                    self.active_angle_indices = (self.select_active_arms() if has_safe_neighbors
                                                 else np.arange(len(self.arm_indices)))
                if self.plan_future is None:
                    now_sec = self.get_clock().now().nanoseconds*1e-9
                    if now_sec < self.next_plan_sec:
                        return self.refine_target(step)
                    self.next_plan_sec = now_sec+1.0
                    snapshot = gng_path_search()
                    for name in ('geometry', 'config', 'cloud_tree', 'cell_radius', 'arm_indices', 'active_angle_indices'):
                        setattr(snapshot, name, getattr(self, name))
                    for name in ('angles', 'labels', 'adjacency'):
                        setattr(snapshot, name, dict(getattr(self, name)))
                    snapshot.positions, snapshot.home = self.positions.copy(), self.home.copy()
                    self.plan_future = self.plan_pool.submit(snapshot.plan_with_coordination, self.cloud_gap)
                    return self.refine_target(step) if self.config['enable_local_refinement'] else (self.positions.copy(), True)
                if not self.plan_future.done():
                    return self.refine_target(step) if self.config['enable_local_refinement'] else (self.positions.copy(), True)
                result = self.plan_future.result()
                self.plan_future = None
                self.path = result['path']
                self.active_angle_indices = result['active_angle_indices']
                self.coordination_source_indices = result['coordination_source_indices']
                if not self.path:
                    return self.refine_target(step)
                self.num_plans += 1
                if (any(self.labels.get(idx) != 1 for idx in self.path)
                        or not has_safe_node_neighbors(self.labels, self.adjacency, self.path[-1])):
                    self.path = []
                    return self.positions.copy(), True
                if not self.can_bridge(self.positions, self.pose(self.path[0]), self.config['min_cloud_clearance_th']):
                    self.path = []
                    return self.refine_target(step)
            target = self.pose(self.path[0])
            if np.max(np.abs(target-self.positions)) < .05:
                self.path.pop(0)
            is_gng_target = True
        delta = target-self.positions
        target = self.positions+delta*min(1.0, step/max(float(np.max(np.abs(delta))), 1e-9))
        if ((has_safe_neighbors and self.cloud_clearance(target)[0] < min(self.cloud_gap, self.config['target_clearance'])-.0002)
                or not self.can_bridge(self.positions, target, self.config['min_cloud_clearance_th'])):
            self.path = []
            return self.refine_target(step)
        if is_gng_target and np.max(np.abs(target-self.positions)) > 1e-5:
            self.num_selected_gng += 1
        return target, True

    def publish_target(self, target):
        if self.state == 'running' and self.qp is not None and not self.is_stop_latched:
            active = np.asarray(self.arm_indices)[self.active_angle_indices]
            target = self.qp.project(self.positions, target, active,
                                     self.cloud_tree, self.cell_radius, self.can_bridge)
            if target is not None and not self.is_fresh():
                self.qp.report['status'], target = 'input_stale', None
            if target is not None and self.qp.report['total_ms'] >= self.config['control_period_sec']*1000:
                self.qp.report['status'], target = 'cycle_overrun', None
            if target is None:
                self.fail(f"QP補正失敗・入力失効・制御周期超過: {self.qp.report['status']}, "
                          f"{self.qp.report['total_ms']:.2f} ms")
                return
        super().publish_target(target)

    def tick(self):
        super().tick()
        if self.state != 'running' or self.phase == 'obstacle_wait':
            has_stable_return_clearance(self, False)
        if hasattr(self, 'diag'):
            if self.state != 'running':
                self.path = []
            if self.graph_message is not None and not self.enable_native_planner:
                message = TopologicalMap()
                message.header = self.graph_message.header
                message.header.stamp = self.get_clock().now().to_msg()
                nodes = {node.id: node for node in self.graph_message.nodes}
                message.nodes = [nodes[idx] for idx in self.path if idx in nodes]
                for node in message.nodes:
                    node.label = self.labels.get(node.id, 0)
                message.edges = [value for idx in range(len(message.nodes)-1) for value in (idx, idx+1)]
                self.path_pub.publish(message)
            self.diag.publish(String(data=json.dumps({
                'num_cloud': self.num_cloud, 'num_voxels': self.num_voxels,
                'local_qp': self.qp.report if self.qp is not None else None,
                'planner_backend': 'topological_map_avoidance_node' if self.enable_native_planner else 'python',
                'native_target_age_sec': time.monotonic()-self.native_target_time if self.enable_native_planner else None,
                'num_native_plans': self.num_native_plans, 'native_node_path': self.native_node_path,
                'current_node_id': self.current_node_id, 'has_safe_first_neighbors': self.has_safe_neighbors,
                'cloud_age_sec': time.monotonic()-self.cloud_time,
                'voxel_age_sec': time.monotonic()-self.voxel_time,
                'graph_age_sec': time.monotonic()-self.graph_time,
                'real_joint_age_sec': time.monotonic()-self.real_joint_time if self.external_environment else None,
                'environment_namespace': self.external_environment['source_namespace'] if self.external_environment else None,
                'num_safe': sum(value == 1 for value in self.labels.values()),
                'num_danger': sum(value == 3 for value in self.labels.values()),
                'num_collision': sum(value == 2 for value in self.labels.values()),
                'num_local_steps': self.num_local_steps,
                'num_plans': self.num_plans, 'num_selected_gng': self.num_selected_gng,
                'node_path': self.path, 'cloud_clearance_m': self.cloud_gap,
                'is_coordinated': bool(len(self.coordination_source_indices) and self.path),
                'active_arm_joints': [self.arm_names[idx] for idx in self.active_angle_indices],
            })))


def main():
    rclpy.init()
    node = None
    try:
        node = gng_lidar_demo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.plan_pool.shutdown(wait=True, cancel_futures=True)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
