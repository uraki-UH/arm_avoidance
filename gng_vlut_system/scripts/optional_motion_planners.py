"""追加依存を実行時に限定したMoveItの経路計画アダプタ。"""
from dataclasses import dataclass
from enum import Enum
import math
import re
import time
from types import MappingProxyType

from task_program import checked_mapping, positive


class motion_planner_kind(str, Enum):
    bit_star = 'bit_star'
    rrt = 'rrt'
    rrt_connect = 'rrt_connect'
    rrt_star = 'rrt_star'
    informed_rrt_star = 'informed_rrt_star'
    chomp = 'chomp'
    stomp = 'stomp'


@dataclass(frozen=True)
class planner_spec:
    pipeline_id: str
    planner_id: str
    plugin: str
    package: str
    ompl_type: str = ''


planner_specs = MappingProxyType({
    motion_planner_kind.bit_star: planner_spec('ompl', 'BITstar', 'ompl_interface/OMPLPlanner',
                                              'moveit_planners_ompl', 'geometric::BITstar'),
    motion_planner_kind.rrt: planner_spec('ompl', 'RRT', 'ompl_interface/OMPLPlanner',
                                         'moveit_planners_ompl', 'geometric::RRT'),
    motion_planner_kind.rrt_connect: planner_spec('ompl', 'RRTConnect', 'ompl_interface/OMPLPlanner',
                                                 'moveit_planners_ompl', 'geometric::RRTConnect'),
    motion_planner_kind.rrt_star: planner_spec('ompl', 'RRTstar', 'ompl_interface/OMPLPlanner',
                                              'moveit_planners_ompl', 'geometric::RRTstar'),
    motion_planner_kind.informed_rrt_star: planner_spec('ompl', 'InformedRRTstar', 'ompl_interface/OMPLPlanner',
                                                       'moveit_planners_ompl', 'geometric::InformedRRTstar'),
    motion_planner_kind.chomp: planner_spec('chomp', '', 'chomp_interface/CHOMPPlanner', 'moveit_planners_chomp'),
    motion_planner_kind.stomp: planner_spec('stomp', '', 'stomp_moveit/StompPlanner', 'moveit_planners_stomp'),
})


@dataclass(frozen=True)
class moveit_route:
    """一方式の設定。ROS接続・プラグイン読込は計画要求時だけ。"""
    enable_async = True
    kind: motion_planner_kind
    service_namespace: str
    group_name: str
    joint_names: tuple
    max_plan_sec: float
    max_wait_sec: float
    num_attempts: int

    def __call__(self, request):
        if len(request.joint_names) != len(request.start) or not set(self.joint_names) <= set(request.joint_names):
            raise ValueError('計画対象の関節名と要求が一致していません')
        for name, start, target in zip(request.joint_names, request.start, request.target):
            if name not in self.joint_names and abs(start - target) > 1e-9:
                raise ValueError('計画グループ外の関節への移動要求: ' + name)
        return call_moveit(self, request)


def make_moveit_route(kind):
    """登録表用の設定工場。YAMLからの任意importなし。"""
    kind = motion_planner_kind(kind)

    def configure(settings, joint_names):
        checked_mapping(settings, ('service_namespace', 'group_name', 'joint_names',
                                   'max_plan_sec', 'max_wait_sec', 'num_attempts'), kind.value)
        namespace = settings.get('service_namespace', '/sim_topo_dual_arm_max_long')
        group = settings.get('group_name', 'dual_arm')
        if not isinstance(namespace, str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*)?', namespace):
            raise ValueError('service_namespaceには絶対ROS名前空間が必要です')
        if not isinstance(group, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', group):
            raise ValueError('group_nameが不正です')
        names = settings.get('joint_names')
        if names is None:
            prefixes = {'left_arm': ('L',), 'right_arm': ('R',), 'dual_arm': ('L', 'R')}.get(group)
            if prefixes is None:
                raise ValueError('独自グループにはjoint_namesが必要です')
            names = [f'{prefix}_joint{idx}' for prefix in prefixes for idx in range(1, 8)]
        if (not isinstance(names, list) or not names or not all(isinstance(name, str) and name for name in names)
                or len(set(names)) != len(names) or not set(names) <= set(joint_names)):
            raise ValueError('joint_namesには実行器内の重複のない関節名が必要です')
        num_attempts = settings.get('num_attempts', 1)
        if isinstance(num_attempts, bool) or not isinstance(num_attempts, int) or not 1 <= num_attempts <= 100:
            raise ValueError('num_attemptsには1から100の整数が必要です')
        max_plan_sec = positive(settings.get('max_plan_sec', 1.0), 'max_plan_sec')
        max_wait_sec = positive(settings.get('max_wait_sec', 2.0), 'max_wait_sec')
        if max_plan_sec > 60 or max_wait_sec > 60:
            raise ValueError('計画・通信待ち時間の上限は60秒です')
        return moveit_route(kind, namespace.rstrip('/'), group, tuple(names),
                            max_plan_sec, max_wait_sec, num_attempts)
    return configure


def check_interface(response, route):
    """未知のplanner_idから既定方式への切替防止。"""
    spec = planner_specs[route.kind]
    for item in response.planner_interfaces:
        if item.pipeline_id != spec.pipeline_id or item.name.casefold() != spec.pipeline_id:
            continue
        if not spec.planner_id or f'{route.group_name}[{spec.planner_id}]' in item.planner_ids:
            return
    raise RuntimeError(f'未登録の計画方式: {route.kind.value} / {route.group_name}。MoveItの版とプラグイン設定を確認してください')


def build_request(route, request, srv_type, constraints_type, joint_constraint_type):
    """全独立関節の実測始点と選択グループの関節目標。"""
    result = srv_type.Request()
    plan = result.motion_plan_request
    spec = planner_specs[route.kind]
    plan.pipeline_id, plan.planner_id, plan.group_name = spec.pipeline_id, spec.planner_id, route.group_name
    plan.num_planning_attempts, plan.allowed_planning_time = route.num_attempts, route.max_plan_sec
    plan.max_velocity_scaling_factor = plan.max_acceleration_scaling_factor = 1.0
    plan.start_state.is_diff = True
    plan.start_state.joint_state.name = list(request.joint_names)
    plan.start_state.joint_state.position = list(request.start)
    goals = constraints_type()
    for name, target in zip(request.joint_names, request.target):
        if name in route.joint_names:
            constraint = joint_constraint_type()
            constraint.joint_name, constraint.position, constraint.weight = name, target, 1.0
            constraint.tolerance_above = constraint.tolerance_below = 1e-10
            goals.joint_constraints.append(constraint)
    plan.goal_constraints = [goals]
    return result


def response_route(response, route, request, check_state=None):
    """計画結果の名前付き照合。未計画関節の実測保持と端点契約の維持。"""
    result = response.motion_plan_response
    if result.error_code.val != 1:
        raise RuntimeError(f'{route.kind.value}の計画失敗: MoveItErrorCodes={result.error_code.val}')
    if result.group_name != route.group_name:
        raise ValueError('計画結果のグループ名が要求と一致していません')
    if result.trajectory.multi_dof_joint_trajectory.points:
        raise ValueError('多自由度関節を含む計画結果は未対応です')
    trajectory = result.trajectory.joint_trajectory
    names = tuple(trajectory.joint_names)
    if len(set(names)) != len(names) or set(names) != set(route.joint_names):
        raise ValueError('計画結果の関節名が要求と一致していません')
    if not 2 <= len(trajectory.points) <= 10000:
        raise ValueError('計画結果の点数が不正です')
    result_route = []
    for point in trajectory.points:
        if len(point.positions) != len(names) or not all(math.isfinite(value) for value in point.positions):
            raise ValueError('計画結果の関節位置が不正です')
        values = dict(zip(names, point.positions))
        result_route.append(tuple(values.get(name, start) for name, start in zip(request.joint_names, request.start)))
    if route.kind == motion_planner_kind.stomp and check_state is not None:
        result_route = connect_stomp_endpoints(result_route, request, check_state)
    # 共通検査と同一の端点・位置制限契約。
    from task_components import checked_route
    return checked_route(result_route, request)


def connect_stomp_endpoints(route, request, check_state):
    """STOMPの端点微差への短区間接続。既存経路の保持と全接続サンプルの再検査。"""
    max_endpoint_error_th = 0.005
    max_endpoint_step = 0.0005
    result = list(route)
    for endpoint, reference in ((route[0], request.start), (route[-1], request.target)):
        delta = max(abs(left-right) for left, right in zip(endpoint, reference))
        if delta > max_endpoint_error_th:
            raise ValueError('STOMPの端点接続許容差を超過しています')
        if delta <= 1e-9:
            continue
        num_steps = max(1, math.ceil(delta / max_endpoint_step))
        for idx in range(num_steps+1):
            ratio = idx / num_steps
            point = tuple(left + (right-left)*ratio for left, right in zip(endpoint, reference))
            if (any(not bound.min_position <= value <= bound.max_position
                    for value, bound in zip(point, request.bounds)) or check_state(point) is not True):
                raise ValueError('STOMPの端点接続区間が干渉検査で拒否されました')
    if max(abs(left-right) for left, right in zip(route[0], request.start)) > 1e-9:
        result.insert(0, request.start)
    if max(abs(left-right) for left, right in zip(route[-1], request.target)) > 1e-9:
        result.append(request.target)
    return result


def call_moveit(route, request):
    """有限待ち・専用ROSコンテキスト・全終了経路での接続解放。"""
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan, QueryPlannerInterfaces, GetStateValidity
    except ImportError as error:
        raise RuntimeError(f'{route.kind.value}にはrclpyとmoveit_msgsが必要です。通常ビルドでは不要です') from error
    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context)
        node = Node('optional_motion_planner', context=context, use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + route.max_wait_sec + route.max_plan_sec

        def call(srv_type, name, message):
            client = node.create_client(srv_type, route.service_namespace + '/' + name)
            try:
                if not client.wait_for_service(timeout_sec=min(route.max_wait_sec, max(0., deadline-time.monotonic()))):
                    raise RuntimeError('計画サービス未起動: ' + client.srv_name)
                future = client.call_async(message)
                executor.spin_until_future_complete(future, timeout_sec=max(0., deadline-time.monotonic()))
                if not future.done():
                    future.cancel()
                    raise RuntimeError('計画サービス応答の期限切れ: ' + client.srv_name)
                response = future.result()
                if response is None:
                    raise RuntimeError('計画サービスの空応答: ' + client.srv_name)
                return response
            finally:
                node.destroy_client(client)

        check_interface(call(QueryPlannerInterfaces, 'query_planner_interface', QueryPlannerInterfaces.Request()), route)
        message = build_request(route, request, GetMotionPlan, Constraints, JointConstraint)
        response = call(GetMotionPlan, 'plan_kinematic_path', message)
        def check_state(point):
            query = GetStateValidity.Request()
            query.group_name = route.group_name
            query.robot_state.is_diff = True
            query.robot_state.joint_state.name = list(request.joint_names)
            query.robot_state.joint_state.position = list(point)
            return call(GetStateValidity, 'check_state_validity', query).valid
        return response_route(response, route, request, check_state)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
