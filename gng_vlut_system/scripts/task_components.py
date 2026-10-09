"""タスク方式を構成する目標解釈・経路・軌道化・検査の登録表。"""
from dataclasses import dataclass, replace
import math
from types import MappingProxyType
from typing import Callable

from task_program import (checked_mapping, check_segment, direct_targets, waypoint_targets,
                          position_targets, plan_segment, task_kind, task_method)


@dataclass(frozen=True)
class planning_request:
    """一回の計画に対応する実測始点・目標・関節制限。"""
    start: tuple
    target: tuple
    bounds: tuple
    limits: object
    joint_names: tuple = ()


@dataclass(frozen=True)
class planning_components:
    """方式名と関数の対応。実行器やYAMLからの任意importなし。"""
    sources: dict
    targets: dict
    routes: dict
    refiners: dict
    trajectories: dict
    validators: dict


def straight_route(request):
    return (request.start, request.target)


def quintic_trajectory(request, route):
    """停止経由点を含む5次補間の連結。"""
    result = []
    elapsed = 0.0
    for start, target in zip(route, route[1:]):
        segment = plan_segment(start, target, request.bounds, request.limits)
        result.extend(replace(point, time_sec=point.time_sec + elapsed)
                      for point in (segment if not result else segment[1:]))
        if len(result) > 100000:
            raise ValueError('経路全体の軌道点数が上限を超過しています')
        elapsed += segment[-1].time_sec
        if elapsed > request.limits['max_task_sec']:
            raise ValueError('経路全体の軌道時間がタスク期限を超過しています')
    return tuple(result)


def default_components():
    """呼出しごとに独立した登録表。別実行器への登録の漏出防止。"""
    return planning_components(
        sources={'robot_graph': make_robot_graph_source},
        targets={(task_kind.move, 'direct'): direct_targets,
                 (task_kind.move, 'waypoints'): waypoint_targets,
                 (task_kind.hold, 'position'): position_targets},
        routes={'straight': straight_route},
        refiners={}, trajectories={'quintic': quintic_trajectory}, validators={})


def checked_route(route, request):
    try:
        route = tuple(tuple(point) for point in route)
        if not 2 <= len(route) <= 10000:
            raise ValueError('経路点数が不正です')
        for point in route:
            if len(point) != len(request.bounds):
                raise ValueError('経路の関節数が不正です')
            for value, bound in zip(point, request.bounds):
                if (isinstance(value, bool) or not math.isfinite(value)
                        or not bound.min_position <= value <= bound.max_position):
                    raise ValueError('経路の関節位置が不正です')
        if any(abs(a-b) > 1e-9 for a, b in zip(route[0], request.start)) or any(
                abs(a-b) > 1e-9 for a, b in zip(route[-1], request.target)):
            raise ValueError('経路の始終点が要求と一致していません')
    except (TypeError, OverflowError) as error:
        raise ValueError('経路の形式が不正です') from error
    return route


def compose_planner(route_planner, refiners, trajectory_planner, validators, joint_names):
    """選択した部品の束縛。失敗時の別方式への暗黙切替なし。"""
    def plan(start, target, bounds, limits):
        request = planning_request(tuple(start), tuple(target), tuple(bounds),
                                   MappingProxyType(dict(limits)), tuple(joint_names))
        try:
            route = checked_route(route_planner(request), request)
            for refine in refiners:
                route = checked_route(refine(request, route), request)
            points = tuple(trajectory_planner(request, route))
            check_segment(points, request.start, request.target, bounds, limits)
            for validate in validators:
                if validate(request, route, points) is not True:
                    raise ValueError('計画検査部品が軌道を拒否しました')
            return points
        except (ValueError, RuntimeError):
            raise
        except Exception as error:
            raise RuntimeError(f'計画部品の失敗: {type(error).__name__}: {error}') from error
    return plan


def configured_methods(definitions, existing, components=None, joint_names=()):
    """YAMLの名前付き構成からタスク方式への変換。既存方式の上書き禁止。"""
    components = default_components() if components is None else components
    checked_mapping(definitions, tuple(kind.value for kind in task_kind), 'methods')
    methods = dict(existing)
    for kind_name, entries in definitions.items():
        kind = task_kind(kind_name)
        if not isinstance(entries, dict):
            raise ValueError('methodsの種別内には方式名の辞書が必要です')
        for name, specification in entries.items():
            if not isinstance(name, str) or not name or (kind, name) in methods:
                raise ValueError(f'方式名が不正または重複しています: {name}')
            checked_mapping(specification, ('targets', 'route', 'refiners', 'trajectory', 'validators'), name)
            try:
                target_name = specification['targets']
                route_name = specification['route']
                trajectory_name = specification['trajectory']
                names = specification.get('validators', [])
                refine_names = specification.get('refiners', [])
                if (not all(isinstance(value, str) for value in (target_name, route_name, trajectory_name))
                        or not isinstance(names, list) or not all(isinstance(value, str) for value in names)
                        or len(set(names)) != len(names)
                        or not isinstance(refine_names, list)
                        or not all(isinstance(value, str) for value in refine_names)
                        or len(set(refine_names)) != len(refine_names)):
                    raise ValueError('部品名と検査部品の配列が不正です')
                resolver = components.targets[(kind, target_name)]
                route = components.routes[route_name]
                trajectory = components.trajectories[trajectory_name]
                refiners = tuple(components.refiners[value] for value in refine_names)
                validators = tuple(components.validators[value] for value in names)
                if not all(callable(value) for value in (resolver, route, *refiners, trajectory, *validators)):
                    raise ValueError('部品には呼出し可能な関数が必要です')
            except (KeyError, TypeError) as error:
                raise ValueError(f'未登録または不足した計画部品: {name}') from error
            methods[(kind, name)] = task_method(resolver, compose_planner(route, refiners, trajectory, validators, joint_names))
    return methods


def make_robot_graph_source(settings, joint_names):
    from robot_graph_route import robot_graph_route
    return robot_graph_route(settings, joint_names)


def bind_sources(definitions, joint_names, components=None):
    """入力アダプタの生成と経路部品への登録。実行器に方式固有分岐なし。"""
    components = default_components() if components is None else components
    if not isinstance(definitions, dict):
        raise ValueError('inputsには名前付き入力の辞書が必要です')
    components = replace(components, routes=dict(components.routes))
    sources = []
    for name, settings in definitions.items():
        if not isinstance(name, str) or not name or name in components.routes:
            raise ValueError(f'入力部品名と経路部品名の重複: {name}')
        if not isinstance(settings, dict):
            raise ValueError('入力設定には辞書が必要です')
        settings = dict(settings)
        source_type = settings.pop('type', name)
        if not isinstance(source_type, str) or source_type not in components.sources:
            raise ValueError(f'未登録の入力部品: {source_type}')
        source = components.sources[source_type](settings, joint_names)
        components.routes[name] = source
        sources.append(source)
    return components, tuple(sources)
