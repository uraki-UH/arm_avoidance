"""観測点群・姿勢グラフによるROS非依存の回避計画と復帰方策。"""
import heapq
import time

import numpy as np

from avoidance_motion import motion_flags, motion_components, select_motion


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
        self.current_node_id = None
        self.has_safe_neighbors = False
        current = self.positions[self.arm_indices]
        if getattr(self, 'angle_tree', None) is not None:
            _, idx = self.angle_tree.query(current)
            self.current_node_id = self.angle_node_ids[int(idx)]
        elif self.angles:
            self.current_node_id = min(self.angles, key=lambda idx: float(np.linalg.norm(self.angles[idx]-current)))
        if self.current_node_id is not None:
            self.has_safe_neighbors = has_safe_node_neighbors(self.labels, self.adjacency, self.current_node_id)
        return self.has_safe_neighbors

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



class gng_avoidance_policy(gng_path_search):
    motion_components = motion_components()

    def clear_plan(self):
        """旧経路と未採用探索結果の失効。"""
        self.path = []
        if self.plan_future is not None:
            self.plan_future.cancel()
            self.plan_future = None

    def retreat_target(self, step):
        """退避経路の探索・再検査・次目標生成。"""
        has_safe_neighbors = self.motion_flags.has_safe_neighbors
        if self.path and (any(self.labels.get(idx) != 1 for idx in self.path)
                          or not has_safe_node_neighbors(self.labels, self.adjacency, self.path[-1])):
            self.path = []
        if not self.path:
            if len(self.coordination_source_indices):
                self.coordination_source_indices = np.array([], dtype=int)
                self.active_angle_indices = (self.select_active_arms() if has_safe_neighbors
                                             else np.arange(len(self.arm_indices)))
            if self.plan_future is None:
                now_sec = time.monotonic()
                if now_sec < self.next_plan_sec:
                    return (*self.refine_target(step), False)
                self.next_plan_sec = now_sec+1.0
                snapshot = gng_path_search()
                for name in ('geometry', 'config', 'cloud_tree', 'cell_radius', 'arm_indices', 'active_angle_indices'):
                    setattr(snapshot, name, getattr(self, name))
                for name in ('angles', 'labels', 'adjacency'):
                    setattr(snapshot, name, dict(getattr(self, name)))
                snapshot.positions, snapshot.home = self.positions.copy(), self.home.copy()
                self.plan_future = self.plan_pool.submit(snapshot.plan_with_coordination, self.cloud_gap)
                return (*self.refine_target(step), False) if self.config['enable_local_refinement'] else (self.positions.copy(), True, False)
            if not self.plan_future.done():
                return (*self.refine_target(step), False) if self.config['enable_local_refinement'] else (self.positions.copy(), True, False)
            result = self.plan_future.result()
            self.plan_future = None
            self.path = result['path']
            self.active_angle_indices = result['active_angle_indices']
            self.coordination_source_indices = result['coordination_source_indices']
            if not self.path:
                return (*self.refine_target(step), False)
            self.num_plans += 1
            if (any(self.labels.get(idx) != 1 for idx in self.path)
                    or not has_safe_node_neighbors(self.labels, self.adjacency, self.path[-1])):
                self.path = []
                return self.positions.copy(), True, False
            if not self.can_bridge(self.positions, self.pose(self.path[0]), self.config['min_cloud_clearance_th']):
                self.path = []
                return (*self.refine_target(step), False)
        target = self.pose(self.path[0])
        if np.max(np.abs(target-self.positions)) < .05:
            self.path.pop(0)
        return target, True, True


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

    def select_target(self, _hand, _elbow, step):
        # 障害物の真値位置は計画に不使用。自己除去後の観測点とVLUT状態のみの利用
        if getattr(self, 'is_stop_latched', False):
            has_stable_return_clearance(self, False)
            self.clear_plan()
            self.motion_phase = 'stopped'
            self.motion_flags = motion_flags(is_stop_requested=True)
            target, has_candidate, _ = self.motion_components.execute(
                select_motion(self.motion_flags), self, self.positions, step)
            return target, has_candidate
        self.cloud_gap, _ = self.cloud_clearance(self.positions)
        if self.cloud_gap < self.config['min_cloud_clearance_th']:
            has_stable_return_clearance(self, False)
            self.motion_flags = motion_flags(has_valid_input=False)
            self.clear_plan()
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
            self.motion_flags = motion_flags()
            self.phase = self.motion_phase = select_motion(self.motion_flags)
            target, has_candidate, _ = self.motion_components.execute(self.phase, self, self.positions, step)
            return target, has_candidate
        # 非対象腕・胴体・指は実測姿勢に固定
        home_target = self.positions.copy()
        active_joint_indices = np.asarray(self.arm_indices)[self.active_angle_indices]
        home_target[active_joint_indices] = self.home[active_joint_indices]
        # 退避完了後の復帰中は隣接状態と復帰経路で継続判定
        max_retreat_clearance_dev_th = .001
        can_finish_retreat = (getattr(self, 'motion_phase', '') == 'returning'
                              or self.config['target_clearance']-self.cloud_gap <= max_retreat_clearance_dev_th)
        has_clear_return = (has_safe_neighbors and can_finish_retreat
                            and self.can_bridge(self.positions, home_target, self.config['min_clearance_th']))
        can_return = has_stable_return_clearance(self, has_clear_return)
        self.motion_flags = motion_flags(
            has_active_joints=True, has_safe_neighbors=has_safe_neighbors,
            can_finish_retreat=can_finish_retreat, can_return=can_return,
            is_home=np.max(np.abs(home_target-self.positions)) <= self.max_home_error_th)
        self.phase = self.motion_phase = select_motion(self.motion_flags)
        if self.phase != 'avoiding':
            self.clear_plan()
        target, has_candidate, is_gng_target = self.motion_components.execute(
            self.phase, self, home_target, step)
        if not has_candidate:
            return target, has_candidate
        target = np.asarray(target, dtype=float)
        if target.shape != self.positions.shape or not np.all(np.isfinite(target)):
            return self.positions.copy(), False
        # 差替え部品にも共通の非対象関節固定。協調探索で拡張された対象を使用
        active = np.asarray(self.arm_indices)[self.active_angle_indices]
        projected = self.positions.copy()
        projected[active] = target[active]
        target = projected
        if np.array_equal(target, self.positions):
            return target, True
        delta = target-self.positions
        target = self.positions+delta*min(1.0, step/max(float(np.max(np.abs(delta))), 1e-9))
        if ((has_safe_neighbors and not can_return and self.cloud_clearance(target)[0] < min(self.cloud_gap, self.config['target_clearance'])-.0002)
                or not self.can_bridge(self.positions, target, self.config['min_cloud_clearance_th'])):
            self.path = []
            return self.refine_target(step)
        if is_gng_target and np.max(np.abs(target-self.positions)) > 1e-5:
            self.num_selected_gng += 1
        return target, True
