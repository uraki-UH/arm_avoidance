"""Gazeboと共通のGNG退避・復帰方策を使用するMuJoCo接続。実機指令なし。"""
from concurrent.futures import ThreadPoolExecutor
import copy
from pathlib import Path
import time
import xml.etree.ElementTree as et

import numpy as np

from gng_inputs import gng_inputs, load_models, workspace
from dual_arm_avoidance_geometry import robot_geometry, origin
from gng_avoidance_planner import gng_avoidance_policy


def check_kinematics(urdf_path, model):
    """同名関節の接続・原点・軸・mimic条件の照合。外装用固定リンクの追加は許容。"""
    app = Path(__file__).resolve().parents[2] / 'app'
    browser_path = app / ('source.urdf' if model == 'long' else 'models/standard/source.urdf')
    browser = {item.get('name'): item for item in et.parse(browser_path).getroot().findall('joint')}
    for joint in et.parse(urdf_path).getroot().findall('joint'):
        other = browser.get(joint.get('name'))
        if other is None or joint.get('type') != other.get('type'):
            raise ValueError('学習URDFとブラウザモデルの関節不一致')
        for tag in ('parent', 'child', 'mimic', 'limit'):
            first, second = joint.find(tag), other.find(tag)
            if (first is None) != (second is None) or (first is not None and first.attrib != second.attrib):
                raise ValueError('学習URDFとブラウザモデルの不一致: '+joint.get('name'))
        def axis(item):
            value = item.find('axis')
            return np.fromstring(value.get('xyz') if value is not None else '1 0 0', sep=' ')
        if not np.allclose(origin(joint.find('origin')), origin(other.find('origin')), atol=1e-9) or not np.allclose(axis(joint), axis(other)):
            raise ValueError('学習URDFとブラウザモデルの座標不一致: '+joint.get('name'))


class independent_policy:
    """片腕ずつの探索と全身形状による組合せ検査。左右同時の積グラフ探索は対象外。"""
    def __init__(self, geometry, models):
        self.geometry, self.models = geometry, models
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='gng-path')
        self.policies = {}
        self.selected = None
        for name, model in models.items():
            policy = gng_avoidance_policy()
            policy.config = dict(max_plan_sec=.5, max_entry_candidates=20, max_bridge_step=.025,
                min_clearance_th=.005, min_cloud_clearance_th=.005, min_planning_clearance_th=.003,
                min_internal_clearance_th=.003, min_retreat_dist_th=.10, target_clearance=.14,
                max_state_age_sec=1., return_clear_sec=.5, enable_local_refinement=False)
            policy.geometry = copy.copy(geometry)
            policy.arm_names = model['joint_names']
            policy.arm_indices = [geometry.joint_names.index(item) for item in policy.arm_names]
            links = {child for joint, _, child, *_ in geometry.joints if joint in policy.arm_names}
            while True:
                expanded = links | {child for _, parent, child, *_ in geometry.joints if parent in links}
                if expanded == links:
                    break
                links = expanded
            policy.geometry.is_arm = np.array([link in links for link, _, _ in geometry.spheres])
            policy.planning_groups = [dict(name=name, joint_names=policy.arm_names, link_names=links)]
            policy.path, policy.plan_future, policy.plan_pool = [], None, self.pool
            policy.active_angle_indices = np.array([], dtype=int)
            policy.coordination_source_indices = np.array([], dtype=int)
            policy.next_plan_sec = 0.
            policy.num_plans = policy.num_selected_gng = policy.num_local_steps = 0
            policy.max_home_error_th = .015
            policy.motion_phase = 'monitoring'
            self.policies[name] = policy

    def propose(self, positions, home, cloud, graphs, environment_shapes):
        candidates = []
        for name, policy in self.policies.items():
            policy.positions, policy.home = positions.copy(), home.copy()
            policy.geometry.environment_shapes = environment_shapes
            policy.cloud_tree, policy.cell_radius = cloud[:2]
            graph = graphs[name]
            for field in ('angles', 'labels', 'adjacency', 'angle_tree', 'angle_node_ids'):
                setattr(policy, field, graph[field])
            gap = policy.cloud_clearance(positions)[0]
            has_safe_neighbors = policy.has_safe_measured_neighbors()
            if not has_safe_neighbors or gap < policy.config['min_retreat_dist_th']:
                candidates.append((0, gap, name))
            elif policy.path or np.max(np.abs(home[policy.arm_indices]-positions[policy.arm_indices])) > policy.max_home_error_th:
                candidates.append((1, 0 if name == self.selected else 1, name))
        if not candidates:
            return positions.copy(), dict(mode='gng', phase='monitoring', profile='', num_plans=sum(p.num_plans for p in self.policies.values()))
        name = min(candidates)[2]
        if name != self.selected:
            for policy in self.policies.values():
                policy.clear_plan()
            self.selected = name
        policy = self.policies[name]
        target, has_candidate = policy.select_target(None, None, .02)
        if not has_candidate:
            target = positions.copy()
        return target, dict(mode='gng', phase=policy.motion_phase if has_candidate else 'blocked',
                            profile=name, num_plans=sum(p.num_plans for p in self.policies.values()))

    def close(self):
        for policy in self.policies.values():
            policy.clear_plan()
        self.pool.shutdown(wait=True, cancel_futures=True)


class gng_filter:
    def __init__(self, scene, settings, node, robot_config):
        if node is None:
            raise ValueError('GNG回避にはROS接続が必要です')
        self.scene, self.robot = scene, scene.robot
        params, self.models = load_models(settings.get('params_file') or str(workspace / 'gng_vlut_system/config/topo_dual_arm_max_long.yaml'))
        check_kinematics(params['urdf_path'], robot_config['model'])
        self.geometry = robot_geometry(params['urdf_path'], environment_shapes=())
        self.names = self.geometry.joint_names
        active_names = [name for model in self.models.values() for name in model['joint_names']]
        if len(active_names) != len(set(active_names)) or not set(active_names) <= set(self.robot.independent):
            raise ValueError('独立グループの関節重複またはモデル不一致')
        if set(active_names) & set(self.robot.locks):
            raise ValueError('GNG対象関節の固定を解除してください')
        self.fixed = {name: value for model in self.models.values() for name, value in model['fixed_joints'].items()
                      if name in self.names and name not in active_names}
        self.home = dict(self.robot.targets)
        self.goal_generation = 0
        self.set_targets(self.home)
        self.policy = independent_policy(self.geometry, self.models)
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='gng-control')
        self.future = None
        self.is_ready = False
        self.started = time.monotonic()
        self.next_plan_sec = 0.
        self.target = self.positions()
        self.applied_labels = None
        self.status = dict(mode='gng', phase='waiting', message='GNG/VLUT入力待機')
        self.is_closed = False
        try:
            self.inputs = gng_inputs(node, params, self.models, self.geometry.root_link)
        except Exception:
            self.pool.shutdown(wait=True, cancel_futures=True)
            self.policy.close()
            raise

    def set_targets(self, targets):
        for name, value in self.fixed.items():
            if abs(targets[name]-value) > .015:
                raise ValueError('学習時の固定関節条件と異なります: '+name)
        if targets != self.home:
            self.goal_generation += 1
            if hasattr(self, 'target'):
                self.target = self.positions()
        self.home = dict(targets)

    def positions(self):
        state = self.robot.state()
        return np.array([state[name] for name in self.names])

    def environment_shapes(self):
        model, data, mj = self.scene.model, self.scene.data, self.scene.mujoco
        base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'robot_base')
        root_rotation, root_position = data.xmat[base].reshape(3, 3), data.xpos[base]
        shapes = []
        for idx, body in enumerate(model.geom_bodyid):
            ancestor = int(body)
            while ancestor and ancestor != base:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == base:
                continue
            kind = {int(mj.mjtGeom.mjGEOM_PLANE): 'plane', int(mj.mjtGeom.mjGEOM_BOX): 'box'}.get(int(model.geom_type[idx]))
            if kind is None:
                raise ValueError('GNG回避の環境形状は平面・箱のみです')
            shapes.append((kind, (data.geom_xpos[idx]-root_position) @ root_rotation,
                           root_rotation.T @ data.geom_xmat[idx].reshape(3, 3), model.geom_size[idx].copy()))
            if len(shapes) > 256:
                raise ValueError('GNG回避の環境衝突形状数は256個までです')
        return tuple(shapes)

    def update(self):
        snapshot, message = self.inputs.snapshot()
        if snapshot is None:
            if self.is_ready or time.monotonic()-self.started > 10:
                raise RuntimeError('GNG停止: '+message)
            self.status = dict(mode='gng', phase='waiting', message=message)
            return False
        self.is_ready = True
        cloud, graphs = snapshot
        positions = self.positions()
        if not np.all(np.isfinite(positions)):
            raise RuntimeError('GNG停止: 関節状態が非有限値です')
        if np.any(positions < self.geometry.limits[:, 0]-.03) or np.any(positions > self.geometry.limits[:, 1]+.03):
            raise RuntimeError('GNG停止: 学習関節範囲から外れました')
        for name, _, dof, max_velocity in self.robot.control_entries:
            velocity = self.scene.data.qvel[dof]
            if not np.isfinite(velocity) or abs(velocity) > max_velocity*1.5:
                raise RuntimeError('GNG停止: 関節速度超過: '+name)
        for name, value in self.fixed.items():
            if abs(positions[self.names.index(name)]-value) > .03:
                raise RuntimeError('GNG停止: 固定関節が学習条件から外れました: '+name)
        self.geometry.environment_shapes = self.environment_shapes()
        checker = self.policy.policies[next(iter(self.models))]
        # 探索workerの状態に触れない、実行直前専用の全腕検査
        from gng_avoidance_planner import gng_path_search
        guard = gng_path_search()
        guard.geometry, guard.config = self.geometry, checker.config
        guard.cloud_tree, guard.cell_radius = cloud[:2]
        gap, centers = guard.cloud_clearance(positions)
        if gap < .005 or not guard.has_planning_clearance(centers):
            internal_gap = float(np.min(self.geometry.internal_clearances(centers), initial=np.inf))
            raise RuntimeError(f'GNG停止: 現在姿勢の余裕不足。点群={gap:.3f} m、自己・配置物体={internal_gap:.3f} m')
        now = time.monotonic()
        if self.future is not None and self.future.done():
            target, status = self.future.result()
            self.future = None
            has_same_labels = all(self.requested_labels[name] == graph['labels'] for name, graph in graphs.items())
            if (has_same_labels and self.requested_generation == self.goal_generation
                    and now-self.requested_sec <= 1. and np.max(np.abs(target-positions)) <= .08
                    and guard.can_bridge(positions, target, .005)):
                self.target, self.status = target, status
                self.applied_labels = self.requested_labels
            else:
                self.target = positions.copy()
                self.status = dict(mode='gng', phase='replanning', message='環境・姿勢更新による再検査')
        # 計算済み目標にも最新環境による再検査。未検査目標への切替え防止
        if (self.applied_labels is not None and any(self.applied_labels[name] != graph['labels'] for name, graph in graphs.items())
                or not guard.can_bridge(positions, self.target, .005)):
            self.target = positions.copy()
        # 旧目標に向かう補間状態の持越し防止。検査済みの実測姿勢からの駆動
        self.robot.set_targets(dict(zip(self.names, map(float, self.target))),
                               command_pose=dict(zip(self.names, map(float, positions))))
        self.status['clearance_m'] = gap
        if self.future is None and now >= self.next_plan_sec:
            self.requested_sec = now
            self.requested_generation = self.goal_generation
            self.requested_labels = {name: graph['labels'] for name, graph in graphs.items()}
            self.next_plan_sec = now+.1
            self.future = self.pool.submit(self.policy.propose, positions, np.array([self.home[name] for name in self.names]),
                                           cloud, graphs, self.geometry.environment_shapes)

    def apply(self):
        pass

    def close(self):
        if self.is_closed:
            return
        self.is_closed = True
        self.inputs.close()
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.policy.close()
