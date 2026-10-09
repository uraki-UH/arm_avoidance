"""計画部品の組替え・再計画・不正出力拒否と既存方式の互換性。"""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from task_components import default_components
from task_program import load_program, joint_bound, plan_segment, task_methods


def data():
    return dict(version=1, poses={'goal': {'arm': .4}},
                methods={'move': {'composed': dict(targets='direct', route='straight', trajectory='quintic')}},
                defaults={'move': 'composed'}, tasks=[dict(kind='move', target='goal')])


def program(settings=None, components=None):
    return load_program(settings or data(), {'arm': joint_bound(-1, 1, .4)}, components=components)


def plan(model, start=(0.,)):
    return model.tasks[0].planner(start, (.4,), model.bounds, model.limits)


def test_builtin_composition_matches_existing_trajectory():
    model = program()
    assert plan(model) == plan_segment((0.,), (.4,), model.bounds, model.limits)
    assert len(task_methods) == 3


def test_route_and_validator_replacement_without_executor_change():
    components = default_components()
    calls = []
    def detour(request):
        calls.append(request.start)
        return (request.start, (-.2,), request.target)
    def validate(request, route, points):
        assert route[1] == (-.2,)
        assert any(point.positions == pytest.approx((-.2,)) for point in points)
        return True
    components.routes['detour'] = detour
    components.validators['check_detour'] = validate
    settings = data()
    settings['methods']['move']['composed'].update(route='detour', validators=['check_detour'])
    model = program(settings, components)
    first = plan(model)
    second = plan(model, (.1,))
    assert calls == [(0.,), (.1,)]
    assert first[0].positions != second[0].positions
    assert all(a.time_sec < b.time_sec for a, b in zip(first, first[1:]))
    assert first[-1].positions == pytest.approx((.4,))
    assert 'detour' not in default_components().routes


@pytest.mark.parametrize('route', [None, [], [(0.,)], [(0.,), (.3,)],
                                   [(0.,), (float('nan'),), (.4,)],
                                   [(0.,), (1.1,), (.4,)], [(0., 0.), (.4,)]])
def test_reject_invalid_route_before_trajectory(route):
    components = default_components()
    components.routes['straight'] = lambda request: route
    components.trajectories['quintic'] = lambda *args: pytest.fail('不正経路の軌道化')
    with pytest.raises(ValueError):
        plan(program(components=components))


def test_validator_rejection_has_no_fallback():
    components = default_components()
    components.validators['collision'] = lambda *args: False
    settings = data()
    settings['methods']['move']['composed']['validators'] = ['collision']
    with pytest.raises(ValueError, match='拒否'):
        plan(program(settings, components))


@pytest.mark.parametrize('change', [
    lambda d: d['methods']['move']['composed'].update(route='gng'),
    lambda d: d['methods']['move']['composed'].update(trajectory='missing'),
    lambda d: d['methods']['move']['composed'].update(typo=True),
    lambda d: d['methods']['move']['composed'].update(validators='collision'),
    lambda d: d['methods']['move']['composed'].update(validators=['a', 'a']),
    lambda d: d['methods']['move']['composed'].update(targets=[]),
    lambda d: d['methods']['move'].update(direct=d['methods']['move']['composed']),
])
def test_bad_registration_rejected_at_load(change):
    settings = data()
    change(settings)
    with pytest.raises(ValueError):
        program(settings)


def test_total_duration_limit():
    components = default_components()
    components.routes['straight'] = lambda request: (request.start, (-.9,), (.9,), (-.9,), request.target)
    settings = data()
    settings['limits'] = {'max_task_sec': 20}
    with pytest.raises(ValueError, match='全体'):
        plan(program(settings, components))


def test_refiner_order_and_joint_name_contract():
    components = default_components()
    calls = []
    def first(request, route):
        assert request.joint_names == ('arm',)
        calls.append('first')
        return (route[0], (.1,), route[-1])
    def second(request, route):
        calls.append('second')
        assert route[1] == (.1,)
        return (route[0], (.2,), route[-1])
    components.refiners.update(first=first, second=second)
    settings = data()
    settings['methods']['move']['composed']['refiners'] = ['first', 'second']
    assert any(point.positions == pytest.approx((.2,)) for point in plan(program(settings, components)))
    assert calls == ['first', 'second']


def test_component_exception_is_runtime_failure():
    components = default_components()
    def broken(request):
        raise KeyError('missing graph')
    components.routes['straight'] = broken
    with pytest.raises(RuntimeError, match='missing graph'):
        plan(program(components=components))
