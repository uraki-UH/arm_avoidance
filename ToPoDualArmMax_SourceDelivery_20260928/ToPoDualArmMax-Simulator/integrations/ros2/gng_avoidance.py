"""保存済みGNG接続とライブVLUTノード状態によるMuJoCo接続。実機指令なし。"""
from concurrent.futures import ThreadPoolExecutor
import copy
import heapq
from pathlib import Path
import time
import xml.etree.ElementTree as et

import numpy as np

from gng_inputs import gng_inputs, load_models, workspace
from gng_runtime import gng_runtime
from dual_arm_avoidance_geometry import robot_geometry, origin
from gng_avoidance_planner import gng_path_search
from avoidance_motion import motion_kind


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


class simulation_policy(gng_path_search):
    """保存済み接続とライブVLUT状態によるノード列計画。実行時の区間形状検査なし。"""
    def has_safe_endpoint(self, node_id):
        return self.labels.get(node_id) == 1 and node_id not in self.unavailable_nodes

    def nearest_node(self, positions):
        _, idx = self.angle_tree.query(positions[self.arm_indices])
        return self.angle_node_ids[int(idx)]

    def clear_plan(self):
        self.path = []
        self.plan_goal = None
        self.is_goal_attached = False

    def has_current_path(self):
        ids = (self.plan_source, *self.path)
        return (bool(self.path) and all(self.has_safe_endpoint(idx) for idx in self.path)
                and all(first == second or second in self.adjacency.get(first, ())
                        for first, second in zip(ids, ids[1:])))

    def find_path(self, goal_id=None):
        """最寄りノードからの接続探索。危険な開始ノードからは安全な隣接へ退避。"""
        start = self.nearest_node(self.positions)
        deadline = time.monotonic()+self.config['max_plan_sec']
        queue, costs, previous = [(0., start)], {start: 0.}, {start: None}
        while queue and time.monotonic() < deadline:
            cost, node_id = heapq.heappop(queue)
            if cost != costs[node_id]:
                continue
            if self.has_safe_endpoint(node_id) and (goal_id is None or node_id == goal_id):
                path = []
                while node_id is not None:
                    path.append(node_id)
                    node_id = previous[node_id]
                path.reverse()
                self.plan_source = start
                return path if self.has_safe_endpoint(start) else path[1:]
            for adjacent in self.adjacency.get(node_id, ()):
                if adjacent not in self.angles or not self.has_safe_endpoint(adjacent):
                    continue
                next_cost = cost+float(np.max(np.abs(self.angles[adjacent]-self.angles[node_id])))
                if next_cost < costs.get(adjacent, float('inf')):
                    costs[adjacent], previous[adjacent] = next_cost, node_id
                    heapq.heappush(queue, (next_cost, adjacent))
        return []

    def select_waypoint(self):
        goal = self.nearest_node(self.home)
        goal_key = (goal, self.has_safe_endpoint(goal))
        if not self.has_current_path() or self.plan_goal != goal_key:
            self.clear_plan()
            if self.has_safe_endpoint(goal):
                self.path = self.find_path(goal)
                self.motion_phase = motion_kind.returning
            if not self.path:
                if self.has_safe_measured_neighbors():
                    self.motion_phase = motion_kind.waiting_for_clearance
                    return self.positions.copy(), True
                self.path = self.find_path()
                self.motion_phase = motion_kind.avoiding
            if not self.path:
                return self.positions.copy(), False
            self.plan_goal = goal_key
            self.num_plans += 1
        target = self.pose(self.path[0])
        while len(self.path) > 1 and np.max(np.abs(target-self.positions)) <= self.max_home_error_th:
            self.plan_source = self.path.pop(0)
            target = self.pose(self.path[0])
        if len(self.path) == 1 and np.max(np.abs(target-self.positions)) <= self.max_home_error_th:
            if self.motion_phase != motion_kind.returning:
                self.motion_phase = motion_kind.waiting_for_clearance
                return self.positions.copy(), True
            self.is_goal_attached = True
        if self.is_goal_attached:
            target = self.positions.copy()
            target[self.arm_indices] = self.home[self.arm_indices]
            if np.max(np.abs(target-self.positions)) <= self.max_home_error_th:
                self.clear_plan()
                self.motion_phase = motion_kind.monitoring
        self.num_selected_gng += 1
        return target, True


class independent_policy:
    """片腕ずつのノード列追従と、経由ノードの左右組合せ確認。"""
    def __init__(self, geometry, models, check_node_pair=None):
        self.geometry, self.models = geometry, models
        self.check_node_pair = check_node_pair
        self.policies = {}
        self.selected = None
        for name, model in models.items():
            policy = simulation_policy()
            policy.config = dict(max_plan_sec=.5)
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
            policy.active_angle_indices = np.arange(len(policy.arm_indices))
            policy.path, policy.plan_goal, policy.plan_source = [], None, None
            policy.is_goal_attached = False
            policy.unavailable_nodes, policy.pair_context = set(), None
            policy.num_plans = policy.num_selected_gng = 0
            policy.max_home_error_th = .015
            policy.motion_phase = 'monitoring'
            self.policies[name] = policy

    def propose(self, positions, home, cloud, graphs):
        candidates = []
        gaps = []
        measured_ids = {}
        for name, policy in self.policies.items():
            policy.positions, policy.home = positions.copy(), home.copy()
            for field in ('angles', 'labels', 'adjacency', 'angle_tree', 'angle_node_ids'):
                setattr(policy, field, graphs[name][field])
            measured_ids[name] = policy.nearest_node(positions)
        for name, policy in self.policies.items():
            pair_context = tuple((key, idx) for key, idx in measured_ids.items() if key != name)
            if pair_context != policy.pair_context:
                policy.unavailable_nodes.clear()
                policy.clear_plan()
                policy.pair_context = pair_context
            policy.cloud_tree, policy.cell_radius = cloud[:2]
            gap = policy.cloud_clearance(positions)[0]
            gaps.append(gap)
            has_safe_neighbors = policy.has_safe_endpoint(measured_ids[name])
            if not has_safe_neighbors:
                candidates.append((0, gap, name))
            elif policy.path or np.max(np.abs(home[policy.arm_indices]-positions[policy.arm_indices])) > policy.max_home_error_th:
                candidates.append((1, 0 if name == self.selected else 1, name))
        if not candidates:
            return positions.copy(), dict(mode='gng', phase='monitoring', profile='', clearance_m=min(gaps),
                                          num_plans=sum(p.num_plans for p in self.policies.values()))
        name = min(candidates)[2]
        if name != self.selected:
            for policy in self.policies.values():
                policy.clear_plan()
            self.selected = name
        policy = self.policies[name]
        target, has_candidate = policy.select_waypoint()
        pair_ids = dict(measured_ids)
        if policy.path:
            pair_ids[name] = policy.path[0]
        if has_candidate and policy.path and self.check_node_pair is not None:
            pair_result = self.check_node_pair(pair_ids)
            has_candidate = pair_result is True
            if pair_result is False:
                policy.unavailable_nodes.add(policy.path[0])
                policy.clear_plan()
        if not has_candidate:
            target = positions.copy()
        return target, dict(mode='gng', phase=policy.motion_phase if has_candidate else 'blocked',
                            profile=name, plan_node_ids=tuple(policy.path), entry_node_id=policy.plan_source,
                            pair_node_ids=pair_ids, clearance_m=min(gaps),
                            num_plans=sum(p.num_plans for p in self.policies.values()))

    def close(self):
        for policy in self.policies.values():
            policy.clear_plan()


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
            self.policy.check_node_pair = self.runtime.check_node_pair
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

    def has_current_plan_state(self, status, plan_nodes, graphs, positions=None):
        """ノード列の最新状態・保存済み接続と、非操作腕の組合せノードの照合。"""
        profile = status.get('profile')
        graph = graphs.get(profile)
        if status.get('phase') not in ('returning', 'avoiding'):
            return True
        if graph is None or not plan_nodes or any(graph['labels'].get(idx) != 1 for idx in plan_nodes):
            return False
        entry = status.get('entry_node_id', plan_nodes[0])
        ids = (entry, *plan_nodes)
        if any(first != second and second not in graph['adjacency'].get(first, ())
               for first, second in zip(ids, ids[1:])):
            return False
        if positions is not None:
            for name, node_id in status.get('pair_node_ids', {}).items():
                if name == profile:
                    continue
                other = graphs[name]
                indices = [self.names.index(joint) for joint in self.models[name]['joint_names']]
                _, idx = other['angle_tree'].query(positions[indices])
                if other['angle_node_ids'][int(idx)] != node_id:
                    return False
        return True

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
        now = time.monotonic()
        if self.future is not None and self.future.done():
            target, status = self.future.result()
            self.future = None
            plan_nodes = status.pop('plan_node_ids', ())
            has_valid_plan = self.has_current_plan_state(status, plan_nodes, graphs, positions)
            if (has_valid_plan and self.requested_generation == self.goal_generation
                    and target.shape == positions.shape and np.all(np.isfinite(target))):
                self.target, self.status = target, status
                self.has_applied_target = True
                self.applied_plan_nodes = plan_nodes
        if self.has_applied_target and not self.has_current_plan_state(
                self.status, self.applied_plan_nodes, graphs, positions):
            self.target = positions.copy()
            self.has_applied_target = False
            self.applied_plan_nodes = ()
            self.status = dict(mode='gng', phase='replanning', message='候補の失効による姿勢保持・再検査')
            self.robot.hold_position()
        # 検査済み経由姿勢への継続追従。補間状態の保持とURDF速度制限はPhysicsRobotの管轄。
        self.robot.set_targets(dict(zip(self.names, map(float, self.target))))
        self.status.update(self.runtime.status())
        if self.future is None and now >= self.next_plan_sec:
            self.requested_generation = self.goal_generation
            self.next_plan_sec = now+.1
            self.future = self.pool.submit(self.policy.propose, positions, np.array([self.home[name] for name in self.names]),
                                           cloud, graphs)

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
