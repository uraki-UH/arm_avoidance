#!/usr/bin/env python3
"""実レイ点群・VLUT状態・学習済み姿勢グラフによるGazebo専用退避。"""
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
from dataclasses import asdict
from multiprocessing import get_context
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
from gng_avoidance_planner import (
    gng_avoidance_policy, has_stable_return_clearance, graph_topology_hash, voxel_centers)


def build_path_message(graph, path, labels, stamp):
    """元グラフの時刻・ラベルを保持した、表示用経路メッセージの生成。"""
    message = TopologicalMap()
    message.header = deepcopy(graph.header)
    message.header.stamp = stamp
    nodes = {node.id: node for node in graph.nodes}
    message.nodes = [deepcopy(nodes[idx]) for idx in path if idx in nodes]
    for node in message.nodes:
        node.label = labels.get(node.id, 0)
    message.edges = [value for idx in range(len(message.nodes)-1) for value in (idx, idx+1)]
    return message


class gng_lidar_demo(gng_avoidance_policy, avoidance_demo):
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
        self.motion_phase = 'monitoring'
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
        self.max_home_error_th = self.config.get('max_home_error_th', .015)
        if not np.isfinite(self.max_home_error_th) or self.max_home_error_th <= 0:
            raise ValueError('復帰判定の関節誤差は有限の正数が必要です')
        self.angle_tree = None
        self.current_node_id = None
        self.has_safe_neighbors = False
        self.coordination_source_indices = np.array([], dtype=int)
        self.diag = self.create_publisher(String, 'avoidance/gng_status', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
        # 自己除去の根拠となる実機全関節。センサ校正値の丸め・URDF制限への置換なし
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
            self.motion_phase = 'monitoring'
            self.reset_planning_cycle()
        return response

    def on_safety_stop(self, message):
        super().on_safety_stop(message)
        if self.is_stop_latched:
            self.reset_planning_cycle()
            self.coordination_source_indices = np.array([], dtype=int)

    def reset_planning_cycle(self):
        """開始・停止時の復帰確認、未採用探索結果、再探索時刻の初期化。"""
        has_stable_return_clearance(self, False)
        self.clear_plan()
        self.next_plan_sec = 0.0

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
        if len(message.data) == 0:
            self.cloud_tree = None
            return
        points = voxel_centers(message, source['root_from_source'] if source else None)
        self.cloud_tree = cKDTree(points)
        self.cell_radius = message.voxel_size*np.sqrt(3)/2
        self.voxel_time = time.monotonic()
        self.num_voxels = len(points)

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
        return (super().is_fresh() and self.has_fresh_environment() and self.cloud_tree is not None and bool(self.angles) and
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
            ('graph', self.graph_time), ('real_joint', self.real_joint_time))}
        ages.update(has_cloud_tree=self.cloud_tree is not None,
                    has_matching_graph=set(self.angles) == set(self.graph_ids),
                    source_age_sec={key: (time.time_ns()-stamp)*1e-9 for key, stamp in self.source_stamps.items()})
        return json.dumps(ages, ensure_ascii=False)

    def can_resume_obstacle(self):
        """実測姿勢の最寄りGNGノードと直接隣接の安全確認。"""
        return self.has_safe_measured_neighbors()

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

    def diagnostic_status(self, now_sec):
        """同一観測時刻を基準とする診断値の構築。配信処理からの分離。"""
        return {
            'num_cloud': self.num_cloud, 'num_voxels': self.num_voxels,
            'local_qp': self.qp.report if self.qp is not None else None,
            'planner_backend': 'gng_avoidance_policy',
            'motion_phase': self.motion_phase,
            'motion_flags': asdict(self.motion_flags) if hasattr(self, 'motion_flags') else None,
            'current_node_id': self.current_node_id, 'has_safe_first_neighbors': self.has_safe_neighbors,
            'cloud_age_sec': now_sec-self.cloud_time,
            'voxel_age_sec': now_sec-self.voxel_time,
            'graph_age_sec': now_sec-self.graph_time,
            'real_joint_age_sec': now_sec-self.real_joint_time if self.external_environment else None,
            'environment_namespace': self.external_environment['source_namespace'] if self.external_environment else None,
            'num_safe': sum(value == 1 for value in self.labels.values()),
            'num_danger': sum(value == 3 for value in self.labels.values()),
            'num_collision': sum(value == 2 for value in self.labels.values()),
            'num_local_steps': self.num_local_steps,
            'num_plans': self.num_plans, 'num_selected_gng': self.num_selected_gng,
            'node_path': list(self.path), 'cloud_clearance_m': self.cloud_gap,
            'is_coordinated': bool(len(self.coordination_source_indices) and self.path),
            'active_arm_joints': [self.arm_names[idx] for idx in self.active_angle_indices],
        }

    def publish_diagnostics(self):
        """経路表示と診断JSONの配信境界。"""
        if not hasattr(self, 'diag'):
            return
        if self.graph_message is not None:
            message = build_path_message(self.graph_message, self.path, self.labels, self.get_clock().now().to_msg())
            self.path_pub.publish(message)
        self.diag.publish(String(data=json.dumps(self.diagnostic_status(time.monotonic()))))

    def tick(self):
        super().tick()
        if self.state != 'running' or self.phase == 'obstacle_wait':
            has_stable_return_clearance(self, False)
            self.clear_plan()
            self.motion_phase = self.phase
        self.publish_diagnostics()


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
