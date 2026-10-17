"""保存済みGNG接続とライブVLUT状態による共通回避。実測・指令の接続先から独立した探索。"""
from concurrent.futures import ThreadPoolExecutor
import copy
import heapq
from pathlib import Path
import time
import xml.etree.ElementTree as et

import numpy as np

from gng_inputs import gng_inputs, load_models, workspace
from gng_runtime import gng_runtime
from gng_observation import workspace_mask
from dual_arm_avoidance_geometry import robot_geometry, origin
from gng_avoidance_planner import gng_path_search, has_stable_return_clearance
from avoidance_motion import motion_kind
from gng_prediction import streaming_predictor


def finite_clearance(value):
    """観測障害物なしの場合のJSON互換値。無限離隔を有限の安全距離へ偽装しない表現。"""
    return float(value) if np.isfinite(value) else None


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
    """検査済み名目接続と、保存済みGNG・ライブVLUT状態によるノード列計画。"""
    def has_safe_endpoint(self, node_id):
        return self.labels.get(node_id) == 1 and node_id not in self.unavailable_nodes

    def nearest_node(self, positions):
        _, idx = self.angle_tree.query(positions[self.arm_indices])
        return self.angle_node_ids[int(idx)]

    def clear_plan(self):
        self.path = []
        self.plan_goal = None
        self.is_goal_attached = False

    def select_partial_task(self, min_gap):
        """到達可能な関節目標の合成。固定角度刻み・固定時間刻みへの変換なし。"""
        active = np.asarray(self.arm_indices)[self.active_angle_indices]
        errors = np.abs(self.home[active]-self.positions[active])
        target = self.positions.copy()
        for joint in active[np.argsort(-errors)]:
            if abs(self.home[joint]-target[joint]) <= self.max_home_error_th:
                continue
            candidate = target.copy()
            candidate[joint] = self.home[joint]
            # 各関節の名目目標そのものへの接続。合成後も実測から全区間の再照合。
            if self.can_bridge(self.positions, candidate, min_gap):
                target = candidate
        if np.array_equal(target, self.positions):
            return target, False
        self.clear_plan()
        self.motion_phase = 'partial_task'
        return target, True

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
        target = self.positions.copy()
        target[self.arm_indices] = self.home[self.arm_indices]
        if (self.cloud_clearance(self.positions)[0] >= self.config['min_clearance_th']
                and self.can_bridge(self.positions, target, self.config['min_clearance_th'])):
            # 実測から名目目標への検査済み接続。遠い代表ノードへの不要な迂回なし。
            self.clear_plan()
            self.motion_phase = 'task_following'
            return target, True
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


class avoidance_first_policy(simulation_policy):
    """タスク進行より観測障害物からの退避を優先する実験方式。"""
    def retreat_padding(self):
        return 0.

    def can_follow_task(self):
        visibility = getattr(self, 'visibility', None)
        if not self.cloud_tree.n and (visibility is None or visibility.model != 'fov_shadow'):
            self.is_retreating = True
            self.has_clear_direct_task = False
            has_stable_return_clearance(self, False)
            return False
        target = self.positions.copy()
        target[self.arm_indices] = self.home[self.arm_indices]
        gap, _ = self.cloud_clearance(self.positions)
        is_retreating = getattr(self, 'is_retreating', False)
        margin = self.config['release_clearance_th'] if is_retreating else self.config['trigger_clearance_th']
        margin += self.retreat_padding()
        has_clear_task = gap >= margin and self.can_bridge(self.positions, target, margin)
        self.has_clear_direct_task = has_clear_task
        if not is_retreating and has_clear_task:
            return True
        self.is_retreating = True
        if has_stable_return_clearance(self, has_clear_task):
            self.is_retreating = False
            self.clear_plan()
            return True
        return False

    def retreat_scores(self, poses):
        """近接球の二乗不足距離の総和。固定的な最小距離だけへの依存回避。"""
        centers = self.geometry.centers(poses)
        distances = self.cloud_tree.query(centers[:, self.geometry.is_arm].reshape(-1, 3))[0]
        gaps = distances.reshape(len(poses), -1)-self.geometry.radii[self.geometry.is_arm]-self.cell_radius
        deficits = np.maximum(0., self.config['release_clearance_th']+self.retreat_padding()-gaps)
        return np.sum(deficits*deficits, axis=1), np.min(gaps, axis=1), centers

    def select_retreat(self):
        """両符号の関節微動と合成降下方向の一括評価。目標姿勢の誘引項なし。"""
        self.clear_plan()
        self.motion_phase = 'local_retreat'
        if not self.cloud_tree.n:
            self.motion_phase = 'waiting_for_clearance'
            return self.positions.copy(), False
        active = np.asarray(self.arm_indices)[self.active_angle_indices]
        step = self.config['max_local_step']
        poses = np.repeat(self.positions[None], 1+2*len(active), axis=0)
        for idx, joint in enumerate(active):
            poses[1+2*idx, joint] += step
            poses[2+2*idx, joint] -= step
        poses = np.clip(poses, self.geometry.limits[:, 0], self.geometry.limits[:, 1])
        # 非操作関節の実測値保持。範囲外入力の補正による別関節への指令防止。
        inactive = np.setdiff1d(np.arange(len(self.positions)), active)
        poses[:, inactive] = self.positions[inactive]
        scores, gaps, centers = self.retreat_scores(poses)
        gradient = scores[1::2]-scores[2::2]
        if len(gradient) and np.max(np.abs(gradient)) > 1e-12:
            combined = self.positions.copy()
            combined[active] -= step*gradient/np.max(np.abs(gradient))
            combined[active] = np.clip(combined[active], self.geometry.limits[active, 0], self.geometry.limits[active, 1])
            extra_scores, extra_gaps, extra_centers = self.retreat_scores(combined[None])
            poses = np.concatenate((poses, combined[None]))
            scores, gaps, centers = (np.concatenate(pair) for pair in
                                     ((scores, extra_scores), (gaps, extra_gaps), (centers, extra_centers)))
        has_internal = np.asarray(self.has_planning_clearance(centers))
        min_gap = max(self.config['min_clearance_th'], gaps[0]-.0002)
        candidates = np.flatnonzero((scores < scores[0]-1e-10) & (gaps >= min_gap) & has_internal)
        # 時間予算内の順位順照合。上位候補の死角だけを理由とする早期打切りの防止。
        deadline = time.monotonic()+min(.02, self.config.get('max_plan_sec', .02))
        self.num_local_checks = 0
        self.has_local_timeout = False
        for idx in candidates[np.argsort(scores[candidates])]:
            if time.monotonic() >= deadline:
                self.has_local_timeout = True
                break
            self.num_local_checks += 1
            if self.can_bridge(self.positions, poses[idx], min_gap):
                self.num_local_steps = getattr(self, 'num_local_steps', 0)+1
                return poses[idx].copy(), True
        self.motion_phase = 'waiting_for_clearance'
        return self.positions.copy(), False

    def select_waypoint(self):
        if not self.can_follow_task():
            return self.select_retreat()
        self.clear_plan()
        self.motion_phase = 'task_following'
        target = self.positions.copy()
        target[self.arm_indices] = self.home[self.arm_indices]
        return target, True


class predictive_policy(avoidance_first_policy):
    """vrm_predictiveの先回り経路・不一致適応。現在観測の区間検査と独立した予測層。"""
    def __init__(self):
        self.predictor = streaming_predictor()
        self.node_risks = {}
        self.node_centers = np.empty((0, 0, 3))
        self.num_prediction_fallbacks = 0
        self.prediction_ms = self.max_prediction_ms = self.total_prediction_ms = 0.

    def observe_clearance(self, gap, observation_sec):
        """新規観測だけの相対接近速度。ロールバック・欠測・無限離隔の差分利用なし。"""
        previous = getattr(self, 'previous_clearance', None)
        if previous is not None and observation_sec == previous[0]:
            return
        period = 0. if previous is None else observation_sec-previous[0]
        if previous is None or not 0 < period <= self.predictor.max_age_sec or not np.isfinite([gap, previous[1]]).all():
            self.closing_rate_mps = 0.
        else:
            rate = np.clip((previous[1]-gap)/period, 0., 4.)
            self.closing_rate_mps = .5*getattr(self, 'closing_rate_mps', 0.)+.5*float(rate)
        self.previous_clearance = observation_sec, gap

    def retreat_padding(self):
        """接近中の退避開始余裕。既存予測時間の再利用、現在観測ガードの距離縮小なし。"""
        predictor = getattr(self, 'predictor', None)
        return 0. if predictor is None else min(.2, getattr(self, 'closing_rate_mps', 0.)*predictor.horizon_sec)

    def observe(self, cloud, graph):
        if self.predictor.last_sec == cloud[2]:
            return
        begin = time.perf_counter()
        ids = tuple(self.angle_node_ids)
        # 全ノードは既存VLUTラベルで予測。追加の形状距離照会だけの候補数制限。
        ratios = np.linspace(0., 1., 4)[:, None]
        samples = self.positions[self.arm_indices]+(self.home[self.arm_indices]-self.positions[self.arm_indices])*ratios
        _, indices = self.angle_tree.query(samples, k=min(24, len(ids)))
        indices = np.unique(indices)
        inactive = np.setdiff1d(np.arange(len(self.positions)), self.arm_indices)
        key = (id(self.angles), self.positions[inactive].tobytes(), indices.tobytes())
        if getattr(self, 'centers_key', None) != key:
            poses = np.repeat(self.positions[None], len(indices), axis=0)
            poses[:, self.arm_indices] = np.asarray([self.angles[ids[int(idx)]] for idx in indices])
            self.node_centers = self.geometry.centers(poses)[:, self.geometry.is_arm]
            self.centers_key = key
        dists = self.cloud_tree.query(self.node_centers.reshape(-1, 3))[0].reshape(len(indices), -1)
        gaps = np.full(len(ids), np.inf)
        gaps[indices] = np.min(dists-self.geometry.radii[self.geometry.is_arm]-self.cell_radius, axis=1)
        risks = self.predictor.update(ids, self.adjacency, gaps, self.labels, cloud[2])
        self.node_risks = dict(zip(ids, map(float, risks)))
        self.prediction_ms = (time.perf_counter()-begin)*1000
        self.max_prediction_ms = max(getattr(self, 'max_prediction_ms', 0.), self.prediction_ms)
        self.total_prediction_ms = getattr(self, 'total_prediction_ms', 0.)+self.prediction_ms

    def prediction_status(self):
        return dict(self.predictor.status(), num_fallbacks=self.num_prediction_fallbacks,
                    closing_rate_mps=getattr(self, 'closing_rate_mps', 0.), retreat_padding_m=self.retreat_padding(),
                    num_dist_nodes=len(self.node_centers), total_update_ms=self.prediction_ms,
                    mean_total_update_ms=self.total_prediction_ms/max(1, self.predictor.num_updates),
                    max_total_update_ms=self.max_prediction_ms)

    def select_observed_waypoint(self, cloud, graph, gap):
        """接近時の検査済み局所退避を先行。全ノード予測の更新待ちによる反応遅延の抑制。"""
        if gap < self.config['trigger_clearance_th']+self.retreat_padding():
            self.is_retreating = True
            has_stable_return_clearance(self, False)
            target, can_retreat = self.select_retreat()
            if can_retreat:
                return target, True
        self.observe(cloud, graph)
        return self.select_waypoint()

    def route_risk(self, first, second):
        """関節直線の最寄りノード予測リスク。安全証明ではなく迂回選好の指標。"""
        count = min(21, max(2, int(np.ceil(np.max(np.abs(second-first))/.08))+1))
        poses = first+(second-first)*np.linspace(0., 1., count)[:, None]
        _, indices = self.angle_tree.query(poses[:, self.arm_indices])
        return max(self.node_risks.get(self.angle_node_ids[int(idx)], 0.) for idx in indices)

    def can_follow_task(self):
        # 反応型と共通の離隔・復帰継続条件。予測による現在の安全余裕の縮小なし。
        self.has_clear_task = super().can_follow_task()
        return self.has_clear_task and self.route_risk(self.positions, self.home) < self.predictor.risk_th

    def find_predictive_path(self, goal, allow_prediction):
        """観測安全ノードだけのL∞距離＋予測リスク探索。外れ時の逃げ道数による同点調整。"""
        start = self.nearest_node(self.positions)
        deadline = time.monotonic()+min(.01, self.config['max_plan_sec'])
        def remaining(idx):
            return float(np.max(np.abs(self.angles[idx]-self.angles[goal])))/1.6 if goal is not None else 0.
        queue, costs, previous = [(remaining(start), 0., start)], {start:0.}, {start:None}
        while queue and time.monotonic() < deadline:
            _, cost, idx = heapq.heappop(queue)
            if cost != costs[idx]:
                continue
            risk = self.node_risks.get(idx, 0.) if allow_prediction else 0.
            is_destination = idx == goal if goal is not None else idx != start
            if is_destination and self.has_safe_endpoint(idx) and risk < self.predictor.risk_th:
                path = []
                while idx is not None:
                    path.append(idx)
                    idx = previous[idx]
                path.reverse()
                self.plan_source = start
                return path if self.has_safe_endpoint(start) else path[1:]
            for other in self.adjacency.get(idx, ()):
                if other not in self.angles or not self.has_safe_endpoint(other):
                    continue
                risk = self.node_risks.get(other, 0.) if allow_prediction else 0.
                if risk >= self.predictor.risk_th:
                    continue
                degree = sum(self.has_safe_endpoint(neighbor) for neighbor in self.adjacency.get(other, ()))
                length = float(np.max(np.abs(self.angles[other]-self.angles[idx])))
                next_cost = cost+length*(1.+3.*risk)/(1.+.1*self.predictor.miss_rate*min(degree, 6))
                if next_cost < costs.get(other, float('inf')):
                    costs[other], previous[other] = next_cost, idx
                    heapq.heappush(queue, (next_cost+remaining(other), next_cost, other))
        return []

    def select_waypoint(self):
        self.clear_plan()
        target = self.positions.copy()
        target[self.arm_indices] = self.home[self.arm_indices]
        if self.can_follow_task():
            self.motion_phase = 'predictive_task'
            return target, True
        if not self.has_clear_task:
            retreat, can_retreat = self.select_retreat()
            if can_retreat:
                return retreat, True
            # 復帰待機と経路遮断の区別。安全継続時間による待機の迂回なし。
            if self.has_clear_direct_task:
                return retreat, False
        goal = self.nearest_node(self.home)
        goal = goal if (self.has_safe_endpoint(goal)
                        and self.node_risks.get(goal, 0.) < self.predictor.risk_th) else None
        if goal == self.nearest_node(self.positions) and not self.has_clear_task:
            goal = None
        self.path = self.find_predictive_path(goal, True)
        if not self.path:
            # 行き止まり時だけの予測除外。現在の衝突・遮蔽・自己干渉の除外は禁止。
            self.num_prediction_fallbacks += 1
            self.path = self.find_predictive_path(goal, False)
            if not self.path and self.can_bridge(self.positions, target, self.config['trigger_clearance_th'],
                                                 target_node_id=self.nearest_node(target)):
                self.motion_phase = 'predictive_fallback'
                return target, True
        while len(self.path) > 1 and np.max(np.abs(self.pose(self.path[0])-self.positions)) <= self.max_home_error_th:
            self.plan_source = self.path.pop(0)
        if self.path:
            target = self.pose(self.path[0])
            if self.can_bridge(self.positions, target, self.config['trigger_clearance_th'], target_node_id=self.path[0]):
                self.num_plans += 1
                self.motion_phase = 'predictive_detour'
                return target, True
        gap = self.cloud_clearance(self.positions)[0]
        min_gap = max(self.config['min_clearance_th'],
                      min(self.config['trigger_clearance_th'], gap-.0002))
        partial, can_progress = self.select_partial_task(min_gap)
        if can_progress:
            return partial, True
        # 観測障害物を消さない局所退避。未検査の予測軌道の実行なし。
        return self.select_retreat()


class independent_policy:
    """片腕ずつのノード列追従と、経由ノードの左右組合せ確認。"""
    def __init__(self, geometry, models, check_node_pair=None, strategy='graph'):
        policy_types = dict(graph=simulation_policy, avoidance_first=avoidance_first_policy,
                            vrm_predictive=predictive_policy)
        if strategy not in policy_types:
            raise ValueError('未対応のGNG回避方策: '+str(strategy))
        self.strategy = strategy
        self.geometry, self.models = geometry, models
        self.check_node_pair = check_node_pair
        self.policies = {}
        self.selected = None
        for name, model in models.items():
            policy = policy_types[strategy]()
            policy.config = dict(max_plan_sec=.5, max_bridge_step=.025,
                                 min_clearance_th=.01, min_planning_clearance_th=.005)
            if strategy != 'graph':
                policy.config.update(max_bridge_step=.025, min_clearance_th=.01,
                                     min_planning_clearance_th=.005, trigger_clearance_th=.10,
                                     release_clearance_th=.14, return_clear_sec=.5,
                                     max_state_age_sec=.3, max_local_step=.12)
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
        gaps = {}
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
            if hasattr(policy, 'observe_clearance'):
                policy.observe_clearance(gap, cloud[2])
            gaps[name] = gap
            # 最寄り代表姿勢の衝突と実測姿勢の衝突の区別。別腕の誤退避によるタスク飢餓の防止。
            padding = policy.retreat_padding() if hasattr(policy, 'retreat_padding') else 0.
            trigger = policy.config.get('trigger_clearance_th', policy.config['min_clearance_th'])+padding
            has_near_obstacle = gap < trigger
            is_priority_retreat = (self.strategy != 'graph' and
                                   (getattr(policy, 'is_retreating', False) or has_near_obstacle))
            if self.strategy == 'vrm_predictive':
                is_priority_retreat |= policy.node_risks.get(measured_ids[name], 0.) >= policy.predictor.risk_th
            if has_near_obstacle or is_priority_retreat:
                candidates.append((0, gap, name))
            elif policy.path or np.max(np.abs(home[policy.arm_indices]-positions[policy.arm_indices])) > policy.max_home_error_th:
                candidates.append((1, 0 if name == self.selected else 1, name))
        if not candidates:
            return positions.copy(), dict(mode='gng', phase='monitoring', profile='', clearance_m=finite_clearance(min(gaps.values())),
                                          num_plans=sum(p.num_plans for p in self.policies.values()))
        first_hold = None
        for _, _, name in sorted(candidates):
            policy = self.policies[name]
            if name != self.selected:
                policy.clear_plan()
            if self.strategy == 'vrm_predictive':
                target, has_candidate = policy.select_observed_waypoint(cloud, graphs[name], gaps[name])
            else:
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
            status = dict(mode='gng', phase=policy.motion_phase if has_candidate else 'blocked',
                            profile=name, plan_node_ids=tuple(policy.path), entry_node_id=policy.plan_source,
                            pair_node_ids=pair_ids, clearance_m=finite_clearance(min(gaps.values())),
                            **(dict(prediction=policy.prediction_status()) if self.strategy == 'vrm_predictive' else {}),
                            num_plans=sum(p.num_plans for p in self.policies.values()))
            if has_candidate and not np.array_equal(target, positions):
                self.selected = name
                return target, status
            # 優先腕が保持だけの場合の次候補照合。別腕の実行可能な指令の飢餓防止。
            if first_hold is None:
                first_hold = target, status
        return first_hold

    def close(self):
        for policy in self.policies.values():
            policy.clear_plan()


class nonstop_simulation_policy(independent_policy):
    """非停止の仮想実験。観測は選好、設定済み固定禁止領域は制約。実機使用禁止。"""
    def propose(self, positions, home, cloud, graphs):
        target = home.copy()
        phases, gaps, selected_nodes = [], [], {}
        has_workspace_block = False
        for name, policy in self.policies.items():
            if name not in getattr(self, 'active_profiles', self.policies):
                continue
            policy.positions, policy.home = positions.copy(), home.copy()
            policy.cloud_tree, policy.cell_radius = cloud[:2]
            for field in ('angles', 'labels', 'adjacency', 'angle_tree', 'angle_node_ids'):
                setattr(policy, field, graphs[name][field])
            gap = policy.cloud_clearance(positions)[0]
            gaps.append(gap)
            if hasattr(policy, 'observe_clearance'):
                policy.observe_clearance(gap, cloud[2])
                policy.observe(cloud, graphs[name])
            active = np.asarray(policy.arm_indices)
            # 探索用の絶対姿勢候補。固定時間ごとの微小角度指令への変換なし。
            poses = [home.copy()]
            for joint in active:
                for delta in (-.6, .6):
                    pose = positions.copy()
                    pose[joint] += delta
                    poses.append(pose)
            _, indices = policy.angle_tree.query(positions[active], k=min(16, len(policy.angle_node_ids)))
            node_ids = [policy.angle_node_ids[int(idx)] for idx in np.atleast_1d(indices)]
            for node_id in node_ids:
                pose = positions.copy()
                pose[active] = policy.angles[node_id]
                poses.append(pose)
            poses = np.clip(np.asarray(poses), self.geometry.limits[:, 0], self.geometry.limits[:, 1])
            inactive = np.setdiff1d(np.arange(len(positions)), active)
            poses[:, inactive] = positions[inactive]
            ratios = np.array([.25, .5, .75, 1.])
            samples = positions+(poses[:, None]-positions)*ratios[None, :, None]
            centers = policy.geometry.centers(samples.reshape(-1, len(positions)))
            selected = centers[:, policy.geometry.is_arm]
            distances = policy.cloud_tree.query(selected.reshape(-1, 3))[0].reshape(len(poses), len(ratios), -1)
            surface_gaps = distances-policy.geometry.radii[policy.geometry.is_arm]-policy.cell_radius
            padding = policy.retreat_padding() if hasattr(policy, 'retreat_padding') else 0.
            deficits = np.maximum(0., .2+padding-surface_gaps)
            obstacle_cost = 30.*np.mean(deficits**2, axis=(1, 2))+5.*np.max(deficits, axis=(1, 2))**2
            internal = policy.geometry.internal_clearances(centers).reshape(len(poses), len(ratios), -1)
            internal_cost = 500.*np.sum(np.maximum(0., .005-internal)**2, axis=(1, 2))
            task_cost = .025*np.mean((poses[:, active]-home[active])**2, axis=1)
            travel_cost = .005*np.mean((poses[:, active]-positions[active])**2, axis=1)
            costs = obstacle_cost+internal_cost+task_cost+travel_cost
            # VLUT危険ラベル・予測リスクも停止条件ではなく候補の選好。
            for idx, node_id in enumerate(node_ids, 1+2*len(active)):
                costs[idx] += .05*(policy.labels.get(node_id) == 2)
                costs[idx] += .02*getattr(policy, 'node_risks', {}).get(node_id, 0.)
            has_motion = np.any(np.abs(poses[:, active]-positions[active]) > 1e-6, axis=1)
            if np.any(has_motion) and np.any(np.abs(home[active]-positions[active]) > 1e-6):
                costs[~has_motion] = np.inf
            mask = getattr(policy, 'workspace_mask', None)
            # 選好順の固定領域照合。全候補の再ボクセル化・全経路検査の回避。
            idx = next((int(idx) for idx in np.argsort(costs) if np.isfinite(costs[idx])
                        and (mask is None or mask.can_bridge(policy.geometry, positions, poses[idx]))), None)
            if idx is None:
                target[active] = positions[active]
                has_workspace_block = True
                continue
            target[active] = poses[idx, active]
            phases.append(idx != 0)
            if idx >= 1+2*len(active):
                selected_nodes[name] = node_ids[idx-1-2*len(active)]
        phase = ('workspace_blocked' if has_workspace_block else
                 'simulation_avoiding' if any(phases) else 'simulation_task')
        return target, dict(mode='gng', phase=phase,
            profile='', clearance_m=finite_clearance(min(gaps)), selected_node_ids=selected_nodes,
            is_nonstop_simulation=True, has_hardware_output=False)


class gng_filter:
    def __init__(self, scene, settings, node, robot_config, learned_models=None):
        if node is None:
            raise ValueError('GNG回避にはROS接続が必要です')
        self.scene, self.robot = scene, scene.robot
        self.enable_nonstop_simulation = settings.get('enable_nonstop_simulation') is True
        if self.enable_nonstop_simulation and not getattr(scene, 'is_simulation', False):
            raise ValueError('非停止モードは専用MuJoCo接続だけで使用可能です')
        params, self.models = learned_models or load_models(settings.get('params_file') or str(workspace / 'gng_vlut_system/config/topo_dual_arm_max_long.yaml'))
        sizes = {float(np.float32(model['voxel_size'])) for model in self.models.values()}
        if len(sizes) != 1 or sizes != {float(np.float32(params['gng']['vlut_resolution']))}:
            raise ValueError('入力格子と保存済みVLUTの解像度不一致')
        check_kinematics(params['urdf_path'], robot_config['model'])
        self.geometry = robot_geometry(params['urdf_path'], environment_shapes=())
        self.workspace_mask = workspace_mask(params, self.geometry.root_link)
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
        policy_type = nonstop_simulation_policy if self.enable_nonstop_simulation else independent_policy
        self.policy = policy_type(self.geometry, self.models, strategy=settings.get('gng_strategy', 'graph'))
        for policy in self.policy.policies.values():
            policy.workspace_mask = self.workspace_mask
        if self.enable_nonstop_simulation:
            self.policy.active_profiles = set(settings.get('nonstop_profiles', self.models))
            if not self.policy.active_profiles or not self.policy.active_profiles <= self.models.keys():
                raise ValueError('非停止実験の操作グループが不正です')
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='gng-control')
        self.future = None
        self.is_ready = False
        self.next_plan_sec = 0.
        self.target = self.positions()
        self.next_guard_sec = 0.
        self.has_applied_target = False
        self.applied_plan_nodes = ()
        self.status = dict(mode='gng', phase='waiting', message='GNG/VLUT入力待機')
        self.is_closed = False
        self.runtime = None
        try:
            self.runtime = gng_runtime(node, params, settings, scene, self.geometry.root_link)
            for policy in self.policy.policies.values():
                policy.visibility = self.runtime.visibility
            self.policy.check_node_pair = self.runtime.check_node_pair
            self.inputs = gng_inputs(node, self.runtime.params, self.models, self.geometry.root_link,
                                     control_voxel_topic='/'+self.runtime.namespace+'/control_voxels')
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
                self.target = np.array([targets[name] for name in self.names]) if self.enable_nonstop_simulation else self.positions()
                self.has_applied_target = False
                self.applied_plan_nodes = ()
                if not self.enable_nonstop_simulation:
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
        self.status = dict(mode='gng', phase='blocked_clearance', message=message,
                           profile=self.status.get('profile', ''), **self.runtime.status())
        if clearance_m is not None:
            self.status['clearance_m'] = clearance_m

    def update(self):
        positions = self.positions()
        self.runtime.update(positions, self.names, self.inputs.buffer)
        snapshot, message = self.inputs.snapshot()
        if self.runtime.input_kind == 'pointcloud' and self.runtime.input_message():
            snapshot, message = None, self.runtime.input_message()
        if self.enable_nonstop_simulation:
            return self.update_nonstop_simulation(positions, snapshot, message)
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
        velocities = self.robot.motion_state()['velocity']
        for name, joint in self.robot.independent.items():
            if name in self.robot.locks:
                continue
            velocity = velocities.get(name, float('nan'))
            max_velocity = float(joint.find('limit').get('velocity'))
            if not np.isfinite(velocity) or abs(velocity) > max_velocity*1.5:
                raise RuntimeError('GNG停止: 関節速度超過: '+name)
        for name, value in self.fixed.items():
            if abs(positions[self.names.index(name)]-value) > .03:
                raise RuntimeError('GNG停止: 固定関節が学習条件から外れました: '+name)
        has_fresh_inferred_map = (self.runtime.visibility is not None and self.runtime.visibility.model == 'fov_shadow'
                                  and self.runtime.visibility.has_fresh_observation())
        if not cloud[0].n and not has_fresh_inferred_map:
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
                self.next_guard_sec = 0.
                self.applied_plan_nodes = plan_nodes
        if self.has_applied_target and not self.has_current_plan_state(
                self.status, self.applied_plan_nodes, graphs, positions):
            self.target = positions.copy()
            self.has_applied_target = False
            self.applied_plan_nodes = ()
            self.status = dict(mode='gng', phase='replanning', message='候補の失効による姿勢保持・再検査')
            self.robot.hold_position()
        if ((self.workspace_mask.enable_mask or self.runtime.visibility is not None or self.policy.strategy != 'graph'
                or self.runtime.recorded_filter.num_boundary_voxels)
                and self.has_applied_target
                and now >= self.next_guard_sec):
            # 非同期探索と独立した最新観測での短区間再検査。所有方策の共有状態への書込みなし。
            self.next_guard_sec = now+.02
            selected = self.policy.policies.get(self.status.get('profile'))
            guard = gng_path_search()
            guard.geometry = selected.geometry if selected is not None else self.geometry
            guard.workspace_mask = self.workspace_mask
            guard.visibility = self.runtime.visibility
            guard.config = dict(max_bridge_step=.025, min_clearance_th=.01, min_planning_clearance_th=.005)
            guard.cloud_tree, guard.cell_radius = cloud[:2]
            if not np.array_equal(positions, self.target) and not guard.can_bridge(positions, self.target, .01):
                reason = self.runtime.visibility.last_reason if self.runtime.visibility is not None else 'obstacle'
                if reason == 'free':
                    reason = 'pointcloud_or_internal'
                self.hold_for_environment('姿勢保持: 区間照合 '+reason)
                return False
        # 検査済み経由姿勢への継続追従。補間状態の保持とURDF速度制限はPhysicsRobotの管轄。
        self.target = self.apply_workspace_constraint(positions, self.target)
        self.robot.set_targets(dict(zip(self.names, map(float, self.target))))
        self.status.update(self.runtime.status())
        if self.future is None and now >= self.next_plan_sec:
            self.requested_generation = self.goal_generation
            self.next_plan_sec = now+.1
            self.future = self.pool.submit(self.policy.propose, positions, np.array([self.home[name] for name in self.names]),
                                           cloud, graphs)

    def update_nonstop_simulation(self, positions, snapshot, message):
        """非停止実験の更新。欠測時も名目追従、設定済み固定禁止領域だけは継続遵守。"""
        if not np.isfinite(positions).all():
            raise ValueError('非停止シミュレーションの実測姿勢が非有限値です')
        self.is_ready = True
        if self.future is not None and self.future.done():
            target, status = self.future.result()
            self.future = None
            if self.requested_generation == self.goal_generation:
                if target.shape != positions.shape or not np.isfinite(target).all():
                    raise ValueError('非停止シミュレーションの計画姿勢が不正です')
                self.target, self.status = target, status
        if snapshot is None:
            self.target = np.array([self.home[name] for name in self.names])
            self.status = dict(mode='gng', phase='simulation_open_loop', message=message,
                               is_nonstop_simulation=True, has_hardware_output=False)
        elif self.future is None and time.monotonic() >= self.next_plan_sec:
            self.requested_generation = self.goal_generation
            self.next_plan_sec = time.monotonic()+.05
            self.future = self.pool.submit(self.policy.propose, positions,
                np.array([self.home[name] for name in self.names]), *snapshot)
        self.target = np.clip(self.target, self.geometry.limits[:, 0], self.geometry.limits[:, 1])
        self.target = self.apply_workspace_constraint(positions, self.target)
        self.robot.set_targets(dict(zip(self.names, map(float, self.target))))
        self.status.update(self.runtime.status())

    def apply_workspace_constraint(self, positions, target):
        """通常回避・非停止実験で共通の固定制約。欠測時の名目指令にも同じ判定。"""
        mask = getattr(self, 'workspace_mask', None)
        if mask is None or not mask.enable_mask:
            return target
        velocity = self.robot.motion_state()['velocity']
        velocity = np.array([velocity[name] for name in self.names])
        has_room = mask.has_braking_room(self.geometry, positions, velocity)
        if not has_room or not mask.can_bridge(self.geometry, positions, target):
            self.status.update(phase='workspace_blocked' if has_room else 'workspace_braking',
                               message='設定済み固定禁止領域への指令棄却' if has_room else '固定禁止領域への接近による減速')
            return positions.copy()
        return target

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


# プラグインの共通接続。学習条件の準備と既存回避実装への委譲
class gng_adapter:
    def prepare(self, settings, robot):
        learned = load_models(settings.get('params_file') or str(workspace/'gng_vlut_system/config/topo_dual_arm_max_long.yaml'))
        models = learned[1]
        active = {name for model in models.values() for name in model['joint_names']}
        fixed = {name for model in models.values() for name in model['fixed_joints']} - active
        return dict(learned_models=learned, fixed_joint_names=fixed)

    def create(self, scene, settings, node, robot, prepared):
        return gng_filter(scene, settings, node, robot, prepared['learned_models'])
