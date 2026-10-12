"""Gazeboと共通のGNG退避・復帰方策を使用するMuJoCo接続。実機指令なし。"""
from concurrent.futures import ThreadPoolExecutor
import copy
from pathlib import Path
import time
import xml.etree.ElementTree as et

import numpy as np

from gng_inputs import gng_inputs, load_models, workspace
from gng_runtime import gng_runtime
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


class simulation_policy(gng_avoidance_policy):
    """MuJoCoの経由姿勢計画。近接・接触状態からの退避を停止条件としない区間検査。"""
    def bridge_clearance(self, first, min_gap):
        # 経路の衝突余裕と退避先の目標余裕の分離。接近済み区間は悪化しない退避経路。
        return min(self.config['min_cloud_clearance_th'], self.cloud_clearance(first)[0])

    def create_path_search(self):
        return simulation_policy()

    def has_safe_endpoint(self, node_id):
        # 経由姿勢自体のVLUT判定。区間は点群・全身形状で別途検査。
        return self.labels.get(node_id) == 1


class independent_policy:
    """片腕ずつの探索と全身形状による組合せ検査。左右同時の積グラフ探索は対象外。"""
    def __init__(self, geometry, models):
        self.geometry, self.models = geometry, models
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='gng-path')
        self.policies = {}
        self.selected = None
        for name, model in models.items():
            policy = simulation_policy()
            policy.config = dict(max_plan_sec=.5, max_entry_candidates=20, max_bridge_step=.025,
                min_clearance_th=.005, min_cloud_clearance_th=.005, min_planning_clearance_th=.003,
                min_internal_clearance_th=.003, min_retreat_dist_th=.10,
                max_state_age_sec=1., return_clear_sec=.5, enable_local_refinement=False)
            policy.config['target_clearance'] = policy.config['min_retreat_dist_th']
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
        target, has_candidate = policy.select_waypoint()
        if not has_candidate:
            target = positions.copy()
        return target, dict(mode='gng', phase=policy.motion_phase if has_candidate else 'blocked',
                            profile=name, plan_node_ids=tuple(policy.path),
                            num_plans=sum(p.num_plans for p in self.policies.values()))

    def close(self):
        for policy in self.policies.values():
            policy.clear_plan()
        self.pool.shutdown(wait=True, cancel_futures=True)


class gng_filter:
    def __init__(self, scene, settings, node, robot_config, learned_models=None):
        if node is None:
            raise ValueError('GNG回避にはROS接続が必要です')
        self.scene, self.robot = scene, scene.robot
        params, self.models = learned_models or load_models(settings.get('params_file') or str(workspace / 'gng_vlut_system/config/topo_dual_arm_max_long.yaml'))
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
        self.next_plan_sec = 0.
        self.target = self.positions()
        self.has_applied_target = False
        self.applied_plan_nodes = ()
        self.status = dict(mode='gng', phase='waiting', message='GNG/VLUT入力待機')
        self.is_closed = False
        self.runtime = None
        try:
            self.runtime = gng_runtime(node, params, settings, scene, self.geometry.root_link)
            self.inputs = gng_inputs(node, self.runtime.params, self.models, self.geometry.root_link)
        except Exception:
            self.pool.shutdown(wait=True, cancel_futures=True)
            self.policy.close()
            if self.runtime is not None:
                self.runtime.close()
            raise

    def set_targets(self, targets):
        targets = self.robot.validate({**self.home, **targets})
        for name, value in self.fixed.items():
            if abs(targets[name]-value) > .015:
                raise ValueError('学習時の固定関節条件と異なります: '+name)
        if targets != self.home:
            self.goal_generation += 1
            if hasattr(self, 'target'):
                self.target = self.positions()
                self.has_applied_target = False
                self.applied_plan_nodes = ()
                self.robot.hold_position()
        self.home = dict(targets)

    def positions(self):
        state = self.robot.state()
        return np.array([state[name] for name in self.names])

    def has_current_plan_state(self, status, plan_nodes, graphs):
        """実行中の経由姿勢のVLUT再検査。後続経路は計画workerで再検査。"""
        profile = status.get('profile')
        graph = graphs.get(profile)
        if graph is not None and status.get('phase') == 'returning':
            goal = [self.home[name] for name in self.models[profile]['joint_names']]
            _, idx = graph['angle_tree'].query(goal)
            node_id = graph['angle_node_ids'][int(idx)]
            return graph['labels'].get(node_id) == 1
        if status.get('phase') == 'avoiding':
            return graph is not None and bool(plan_nodes) and graph['labels'].get(plan_nodes[0]) == 1
        return status.get('phase') not in ('returning', 'avoiding')

    def hold_for_environment(self, message, clearance_m=None):
        """観測不足・進入済み障害物に対する、旧探索目標の無効化と姿勢保持。"""
        if self.status.get('phase') != 'blocked_clearance':
            self.goal_generation += 1
        self.target = self.positions()
        self.has_applied_target = False
        self.applied_plan_nodes = ()
        self.robot.hold_position()
        self.status = dict(mode='gng', phase='blocked_clearance', message=message, **self.runtime.status())
        if clearance_m is not None:
            self.status['clearance_m'] = clearance_m

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
        positions = self.positions()
        self.runtime.update(positions, self.names, self.inputs.buffer)
        snapshot, message = self.inputs.snapshot()
        if snapshot is None:
            message = self.runtime.input_message() or message
            if self.is_ready:
                self.hold_for_environment('姿勢保持: '+message)
                return False
            self.status = dict(mode='gng', phase='waiting', message=message, **self.runtime.status())
            self.robot.hold_position()
            return False
        self.is_ready = True
        cloud, graphs = snapshot
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
        if not cloud[0].n:
            # 新しい空ROIと通信途絶の区別。空観測による安全な自由空間との誤認防止。
            self.hold_for_environment('姿勢保持: ROI内の環境観測なし')
            return False
        self.geometry.environment_shapes = self.environment_shapes()
        checker = self.policy.policies[next(iter(self.models))]
        # 探索workerの状態に触れない、実行直前専用の全腕検査
        guard = simulation_policy()
        guard.geometry, guard.config = self.geometry, checker.config
        guard.cloud_tree, guard.cell_radius = cloud[:2]
        gap, centers = guard.cloud_clearance(positions)
        now = time.monotonic()
        has_valid_target = False
        if self.future is not None and self.future.done():
            target, status = self.future.result()
            self.future = None
            plan_nodes = status.pop('plan_node_ids', ())
            has_valid_plan = self.has_current_plan_state(status, plan_nodes, graphs)
            if (has_valid_plan and self.requested_generation == self.goal_generation
                    and now-self.requested_sec <= 1.
                    and guard.can_bridge(positions, target, .005)
                    and guard.can_bridge(np.array([self.robot.commands[name] for name in self.names]), target, .005)):
                self.target, self.status = target, status
                self.has_applied_target = True
                self.applied_plan_nodes = plan_nodes
                has_valid_target = True
        # 計算済み目標にも最新環境による再検査。未検査目標への切替え防止
        if not has_valid_target:
            # 姿勢保持中の同一姿勢は既算出の形状で照合。動く経路は毎周期の区間検査。
            has_valid_target = (guard.has_planning_clearance(centers) if np.array_equal(positions, self.target)
                                else guard.can_bridge(positions, self.target, .005))
        if (self.has_applied_target and not self.has_current_plan_state(
                self.status, self.applied_plan_nodes, graphs)
                or not has_valid_target):
            self.target = positions.copy()
            self.has_applied_target = False
            self.applied_plan_nodes = ()
            self.status = dict(mode='gng', phase='replanning', message='候補の失効による姿勢保持・再検査')
            self.robot.hold_position()
        # 検査済み経由姿勢への継続追従。補間状態の保持とURDF速度制限はPhysicsRobotの管轄。
        self.robot.set_targets(dict(zip(self.names, map(float, self.target))))
        self.status['clearance_m'] = gap
        self.status.update(self.runtime.status())
        if self.future is None and now >= self.next_plan_sec:
            self.requested_sec = now
            self.requested_generation = self.goal_generation
            self.next_plan_sec = now+.1
            self.future = self.pool.submit(self.policy.propose, positions, np.array([self.home[name] for name in self.names]),
                                           cloud, graphs, self.geometry.environment_shapes)

    def apply(self):
        pass

    def close(self):
        if self.is_closed:
            return
        self.is_closed = True
        try:
            self.inputs.close()
            self.pool.shutdown(wait=True, cancel_futures=True)
            self.policy.close()
        finally:
            self.runtime.close()
