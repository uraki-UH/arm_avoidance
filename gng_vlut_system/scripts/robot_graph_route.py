"""ロボット姿勢グラフの世代付き更新と経路生成。学習方式に非依存。"""
from dataclasses import dataclass
import heapq
import math
import time
from types import MappingProxyType

from task_program import checked_mapping, positive


@dataclass(frozen=True)
class graph_snapshot:
    graph_id: str
    revision: int
    joint_names: tuple
    nodes: object
    edges: object


def integer(value):
    if type(value) is not int or value < 0:
        raise ValueError('ID・更新世代には非負整数が必要です')
    return value


def edge_key(value):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('辺はノードIDの2要素配列が必要です')
    first, second = map(integer, value)
    if first == second:
        raise ValueError('自己辺は使用できません')
    return tuple(sorted((first, second)))


class robot_graph_route:
    """原子的なスナップショット交換と、実行経路への更新影響の監視。"""

    def __init__(self, settings, joint_names):
        checked_mapping(settings, ('robot_id', 'topic', 'max_state_age_sec', 'max_connect_dist_th',
                                   'max_plan_sec', 'joint_names'), 'robot_graph')
        self.robot_id = settings.get('robot_id')
        self.topic = settings.get('topic', 'robot_Tmap_updates')
        if not isinstance(self.robot_id, str) or not self.robot_id or not isinstance(self.topic, str) or not self.topic:
            raise ValueError('robot_graphにはrobot_idと有効なtopicが必要です')
        self.robot_joint_names = tuple(joint_names)
        self.joint_names = tuple(settings.get('joint_names', joint_names))
        if (not self.joint_names or len(set(self.joint_names)) != len(self.joint_names)
                or any(name not in self.robot_joint_names for name in self.joint_names)):
            raise ValueError('姿勢グラフの対象関節が不正です')
        self.max_state_age_sec = positive(settings.get('max_state_age_sec', 1.0), 'max_state_age_sec')
        self.max_connect_dist_th = positive(settings.get('max_connect_dist_th', 1e-6), 'max_connect_dist_th')
        self.max_plan_sec = positive(settings.get('max_plan_sec', .2), 'max_plan_sec')
        self.snapshot = None
        self.received_sec = None
        self.stamp_sec = None
        self.has_valid_input = False
        self.has_active_plan = False
        self.has_plan_change = False
        self.plan_nodes = frozenset()
        self.plan_edges = frozenset()

    def accept(self, packet, now_sec, received_sec=None):
        """全検査成功後だけの交換。欠落差分・不正更新後は全体再取得待ち。"""
        try:
            self._accept(packet, now_sec, time.monotonic() if received_sec is None else received_sec)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.has_valid_input = False
            self.has_plan_change = self.has_active_plan
            raise ValueError(f'姿勢グラフ更新を拒否: {error}') from error

    def _accept(self, packet, now_sec, received_sec):
        checked_mapping(packet, ('kind', 'robot_id', 'graph_id', 'revision', 'base_revision',
                                 'stamp_sec', 'joint_names', 'nodes', 'edges', 'remove_nodes', 'remove_edges'), 'graph')
        if packet['robot_id'] != self.robot_id:
            raise ValueError('対象ロボットの不一致')
        graph_id, revision, stamp = packet['graph_id'], integer(packet['revision']), packet['stamp_sec']
        if not isinstance(graph_id, str) or not graph_id or type(stamp) not in (int, float) or not math.isfinite(stamp):
            raise ValueError('グラフID・時刻の不正')
        if not 0 <= now_sec - stamp <= self.max_state_age_sec:
            raise ValueError('グラフ更新の時刻期限切れ')
        old = self.snapshot
        if old and graph_id == old.graph_id and (revision < old.revision or stamp < self.stamp_sec):
            raise ValueError('同一グラフの世代・時刻巻戻り')
        kind = packet['kind']
        if kind == 'heartbeat':
            if set(packet) != {'kind', 'robot_id', 'graph_id', 'revision', 'stamp_sec'}:
                raise ValueError('heartbeatへのデータ混入')
            if not self.has_valid_input or old is None or (graph_id, revision) != (old.graph_id, old.revision):
                raise ValueError('heartbeatの世代不一致')
            self.received_sec, self.stamp_sec = received_sec, stamp
            return
        if kind == 'snapshot':
            if any(key in packet for key in ('base_revision', 'remove_nodes', 'remove_edges')):
                raise ValueError('全体更新への差分キー混入')
            names = tuple(packet['joint_names'])
            if names != self.joint_names:
                raise ValueError('関節名・順序の不一致')
            nodes, edges = {}, {}
        elif kind == 'delta':
            if (not self.has_valid_input or old is None or graph_id != old.graph_id
                    or integer(packet['base_revision']) != old.revision or revision != old.revision + 1):
                raise ValueError('差分の欠落・世代不一致。snapshotが必要です')
            if 'joint_names' in packet:
                raise ValueError('差分による関節定義の変更は禁止です')
            names, nodes, edges = old.joint_names, dict(old.nodes), dict(old.edges)
            removed = [integer(value) for value in packet.get('remove_nodes', [])]
            if len(set(removed)) != len(removed) or any(value not in nodes for value in removed):
                raise ValueError('削除ノードの不一致')
            for value in removed:
                del nodes[value]
            for value in packet.get('remove_edges', []):
                key = edge_key(value)
                if key not in edges:
                    raise ValueError('削除辺の不一致')
                del edges[key]
            edges = {key: value for key, value in edges.items() if not set(key).intersection(removed)}
        else:
            raise ValueError('未知の更新種別')
        seen = set()
        for node in packet.get('nodes', []):
            checked_mapping(node, ('id', 'positions', 'can_traverse'), 'node')
            node_id, positions = integer(node['id']), tuple(node['positions'])
            if node_id in seen or len(positions) != len(names) or any(
                    type(value) not in (int, float) or not math.isfinite(value) for value in positions):
                raise ValueError('ノード角度・IDの不正')
            if type(node['can_traverse']) is not bool:
                raise ValueError('ノード通行可否の不正')
            seen.add(node_id)
            nodes[node_id] = (positions, node['can_traverse'])
        seen = set()
        for edge in packet.get('edges', []):
            checked_mapping(edge, ('nodes', 'can_traverse'), 'edge')
            key = edge_key(edge['nodes'])
            if key in seen or any(value not in nodes for value in key) or type(edge['can_traverse']) is not bool:
                raise ValueError('辺参照・通行可否の不正')
            seen.add(key)
            edges[key] = edge['can_traverse']
        if not nodes:
            raise ValueError('空の姿勢グラフ')
        new = graph_snapshot(graph_id, revision, names, MappingProxyType(nodes), MappingProxyType(edges))
        if old and graph_id == old.graph_id and revision == old.revision and (old.nodes != new.nodes or old.edges != new.edges):
            raise ValueError('同一世代の内容変更')
        if self.has_active_plan and old:
            self.has_plan_change |= (graph_id != old.graph_id or any(old.nodes.get(key) != new.nodes.get(key)
                for key in self.plan_nodes) or any(old.edges.get(key) != new.edges.get(key) for key in self.plan_edges))
        self.snapshot, self.has_valid_input = new, True
        self.received_sec, self.stamp_sec = received_sec, stamp

    def is_fresh(self, now_sec=None):
        return (self.has_valid_input and self.received_sec is not None
                and 0 <= time.monotonic() - self.received_sec <= self.max_state_age_sec
                and (now_sec is None or 0 <= now_sec - self.stamp_sec <= self.max_state_age_sec))

    def release(self):
        self.has_active_plan = False
        self.has_plan_change = False

    def __call__(self, request):
        if not self.is_fresh():
            raise ValueError('姿勢グラフの準備待ち・期限切れ')
        snapshot = self.snapshot
        if request.joint_names != self.robot_joint_names:
            raise ValueError('要求と姿勢グラフの関節順序が不一致です')
        deadline = time.monotonic() + self.max_plan_sec
        def check_time():
            if time.monotonic() > deadline:
                raise ValueError('姿勢グラフ探索の期限切れ')
        indices = tuple(request.joint_names.index(name) for name in snapshot.joint_names)
        if any(abs(a-b) > 1e-9 for idx, (a, b) in enumerate(zip(request.start, request.target)) if idx not in indices):
            raise ValueError('姿勢グラフにない関節への移動要求')
        allowed = {}
        for key, (pose, can_traverse) in snapshot.nodes.items():
            check_time()
            full_pose = list(request.start)
            for idx, value in zip(indices, pose):
                full_pose[idx] = value
            if can_traverse and all(bound.min_position <= value <= bound.max_position
                                    for value, bound in zip(full_pose, request.bounds)):
                allowed[key] = tuple(full_pose)
        def connect(pose):
            candidates = [(max(abs(a-b) for a, b in zip(pose, value)), key) for key, value in allowed.items()]
            if not candidates:
                raise ValueError('通行可能な姿勢ノードなし')
            dist, key = min(candidates)
            if dist > self.max_connect_dist_th:
                raise ValueError('実測・目標姿勢とグラフの接続距離超過')
            return key
        start, goal = connect(request.start), connect(request.target)
        adjacency = {key: [] for key in allowed}
        for (first, second), can_traverse in snapshot.edges.items():
            check_time()
            if can_traverse and first in allowed and second in allowed:
                cost = math.dist(allowed[first], allowed[second])
                adjacency[first].append((second, cost))
                adjacency[second].append((first, cost))
        queue, costs, previous = [(0.0, start)], {start: 0.0}, {}
        while queue:
            check_time()
            cost, current = heapq.heappop(queue)
            if cost != costs[current]:
                continue
            if current == goal:
                break
            for neighbor, step in adjacency[current]:
                candidate = cost + step
                if candidate < costs.get(neighbor, math.inf):
                    costs[neighbor], previous[neighbor] = candidate, current
                    heapq.heappush(queue, (candidate, neighbor))
        else:
            raise ValueError('姿勢グラフ上に経路なし')
        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        path.reverse()
        self.plan_nodes = frozenset(path)
        self.plan_edges = frozenset(tuple(sorted(pair)) for pair in zip(path, path[1:]))
        self.has_active_plan, self.has_plan_change = True, False
        points = [request.start]
        for key in path:
            if allowed[key] != points[-1]:
                points.append(allowed[key])
        if request.target != points[-1] or len(points) == 1:
            points.append(request.target)
        return tuple(points)
