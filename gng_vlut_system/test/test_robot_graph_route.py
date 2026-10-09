"""局所細分化・差分欠落・経路無効化と共通実行器への接続試験。"""
from copy import deepcopy
from pathlib import Path
import sys
import time

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from robot_graph_route import robot_graph_route
from task_program import joint_bound, load_program, joint_sample, task_runner, run_state
from task_components import planning_request


def source():
    return robot_graph_route({'robot_id': 'test_robot', 'max_state_age_sec': 10}, ('arm',))


def packet():
    return dict(kind='snapshot', robot_id='test_robot', graph_id='training_a', revision=1,
                stamp_sec=1.0, joint_names=['arm'], nodes=[
                    dict(id=10, positions=[0.0], can_traverse=True),
                    dict(id=20, positions=[.2], can_traverse=True),
                    dict(id=30, positions=[.4], can_traverse=True),
                    dict(id=99, positions=[-.2], can_traverse=True)], edges=[
                    dict(nodes=[10, 20], can_traverse=True),
                    dict(nodes=[20, 30], can_traverse=True)])


def delta(**kwargs):
    return dict(kind='delta', robot_id='test_robot', graph_id='training_a', revision=2,
                base_revision=1, stamp_sec=2.0, **kwargs)


def request():
    return planning_request((0.,), (.4,), (joint_bound(-1, 1, .4),), {}, ('arm',))


def test_local_refinement_replaces_edge_and_preserves_snapshot():
    graph = source()
    graph.accept(packet(), 1)
    old = graph.snapshot
    assert graph(request()) == ((0.,), (.2,), (.4,))
    graph.accept(delta(remove_edges=[[20, 30]],
        nodes=[dict(id=25, positions=[.3], can_traverse=True)],
        edges=[dict(nodes=[20, 25], can_traverse=True), dict(nodes=[25, 30], can_traverse=True)]), 2)
    assert graph.has_plan_change
    assert 25 not in old.nodes and (20, 30) in old.edges
    assert graph(request()) == ((0.,), (.2,), (.3,), (.4,))
    assert not graph.has_plan_change


def test_unrelated_update_does_not_interrupt():
    graph = source()
    graph.accept(packet(), 1)
    graph(request())
    graph.accept(delta(nodes=[dict(id=99, positions=[-.3], can_traverse=False)]), 2)
    assert not graph.has_plan_change


@pytest.mark.parametrize('update', [
    delta(base_revision_unused=1),
    {**delta(), 'revision': 3},
    {**delta(), 'robot_id': 'other'},
    delta(nodes=[dict(id=20, positions=[float('nan')], can_traverse=True)]),
    delta(edges=[dict(nodes=[10, 444], can_traverse=True)]),
    {**delta(), 'stamp_sec': -100},
])
def test_bad_update_is_atomic_and_requires_snapshot(update):
    graph = source()
    graph.accept(packet(), 1)
    graph(request())
    old = graph.snapshot
    with pytest.raises(ValueError):
        graph.accept(update, 2)
    assert graph.snapshot is old and not graph.has_valid_input and graph.has_plan_change
    with pytest.raises(ValueError):
        graph.accept(delta(), 2)
    graph.accept({**packet(), 'revision': 3, 'stamp_sec': 3.0}, 3)
    assert graph.is_fresh()


def test_changed_joint_position_and_removed_node_invalidate():
    for update in [delta(nodes=[dict(id=20, positions=[.25], can_traverse=True)]), delta(remove_nodes=[20])]:
        graph = source()
        graph.accept(packet(), 1)
        graph(request())
        graph.accept(update, 2)
        assert graph.has_plan_change


def test_blocked_edge_has_no_straight_fallback():
    graph = source()
    graph.accept(packet(), 1)
    graph.accept(delta(edges=[dict(nodes=[20, 30], can_traverse=False)]), 2)
    with pytest.raises(ValueError, match='経路なし'):
        graph(request())


def test_stale_heartbeat_and_same_revision_mutation():
    graph = source()
    graph.accept(packet(), 1)
    with pytest.raises(ValueError):
        graph.accept(dict(kind='heartbeat', robot_id='test_robot', graph_id='training_a',
                          revision=2, stamp_sec=2.0), 2)
    graph.accept(packet(), 1)
    changed = packet()
    changed['nodes'][1]['positions'] = [.3]
    with pytest.raises(ValueError, match='同一世代'):
        graph.accept(changed, 1)


def test_yaml_route_calls_graph_in_real_task_runner():
    settings = dict(version=1, inputs={'robot_graph': {'robot_id': 'test_robot', 'max_state_age_sec': 10}},
        methods={'move': {'graph_move': dict(targets='direct', route='robot_graph', trajectory='quintic')}},
        defaults={'move': 'graph_move'}, poses={'goal': {'arm': .4}}, tasks=[dict(kind='move', target='goal')])
    program = load_program(settings, {'arm': joint_bound(-1, 1, .4)})
    graph, = program.input_sources
    graph.accept(packet(), 1)
    class backend:
        is_busy = False
        result = None
        def begin(self, points):
            self.points = points
            self.is_busy = True
        def cancel(self):
            self.is_busy = False
    output = backend()
    runner = task_runner(program, output)
    runner.tick(1, joint_sample(1, (0.,), (0.,)))
    runner.tick(1.3, joint_sample(1.3, (0.,), (0.,)))
    runner.start(1.3, joint_sample(1.3, (0.,), (0.,)))
    runner.tick(1.4, joint_sample(1.4, (0.,), (0.,)))
    assert runner.state == run_state.running
    assert any(point.positions == pytest.approx((.2,)) for point in output.points)
    assert output.points[-1].positions == pytest.approx((.4,))


def test_joint_mapping_and_connection_distance_rejected():
    graph = source()
    graph.accept(packet(), 1)
    from dataclasses import replace
    with pytest.raises(ValueError):
        graph(replace(request(), joint_names=('other',)))
    with pytest.raises(ValueError, match='接続距離'):
        graph(replace(request(), start=(.1,)))


def test_native_map_bridge_uses_ids_and_updates_angles():
    from robot_graph_bridge import graph_packet_builder
    from types import SimpleNamespace as obj
    header = obj(frame_id='base_link', stamp=obj(sec=1, nanosec=0))
    graph = obj(header=header, nodes=[obj(id=30, label=1), obj(id=10, label=1)], edges=[0, 1])
    features = obj(header=header, features=[obj(node_id=10, weight_angle=[0.]), obj(node_id=30, weight_angle=[.4])])
    builder = graph_packet_builder('test_robot', ['arm'])
    graph_source = source()
    first = builder.build(graph, features)
    assert first['edges'][0]['nodes'] == [10, 30]
    graph_source.accept(first, 1)
    assert graph_source(request()) == ((0.,), (.4,))
    assert builder.build(graph, features)['revision'] == first['revision']
    features.features[1].weight_angle = [.3]
    graph.header = features.header = obj(frame_id='base_link', stamp=obj(sec=2, nanosec=0))
    second = builder.build(graph, features)
    assert second['revision'] == first['revision'] + 1
    features.header = obj(frame_id='base_link', stamp=obj(sec=3, nanosec=0))
    with pytest.raises(ValueError, match='同一更新時刻'):
        builder.build(graph, features)


def test_ros_callback_interrupts_changed_route():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace as obj
    import json
    from task_executor import task_executor
    graph = source()
    graph.accept(packet(), 1)
    graph(request())
    interruptions = []
    runner = obj(state=run_state.running, interrupt=lambda *args: interruptions.append(args))
    executor = obj(program=obj(input_sources=(graph,)), runner=runner, now_sec=lambda: 2.0,
                   get_logger=lambda: obj(warning=lambda value: None))
    executor.check_inputs = lambda: task_executor.check_inputs(executor)
    task_executor.on_input(executor, graph, obj(data=json.dumps(delta(remove_nodes=[20]))))
    assert len(interruptions) == 1 and graph.has_plan_change


def test_partial_joint_graph_keeps_other_joints_fixed():
    from dataclasses import replace
    graph = robot_graph_route({'robot_id': 'test_robot', 'joint_names': ['arm']}, ('neck', 'arm'))
    graph.accept(packet(), 1)
    req = planning_request((.1, 0.), (.1, .4), (joint_bound(-1, 1, .4),)*2, {}, ('neck', 'arm'))
    assert graph(req) == ((.1, 0.), (.1, .2), (.1, .4))
    with pytest.raises(ValueError, match='ない関節'):
        graph(replace(req, target=(.2, .4)))


def test_independent_arm_sources_have_separate_names():
    settings = dict(version=1, inputs={
        'left_graph': dict(type='robot_graph', robot_id='test_robot', joint_names=['left'], topic='left/robot_Tmap_updates'),
        'right_graph': dict(type='robot_graph', robot_id='test_robot', joint_names=['right'], topic='right/robot_Tmap_updates')},
        methods={'move': {
            'left_move': dict(targets='direct', route='left_graph', trajectory='quintic'),
            'right_move': dict(targets='direct', route='right_graph', trajectory='quintic')}},
        poses={'left_goal': {'left': .4}, 'right_goal': {'right': .4}},
        tasks=[dict(kind='move', method='left_move', target='left_goal'),
               dict(kind='move', method='right_move', target='right_goal')])
    model = load_program(settings, {name: joint_bound(-1, 1, .4) for name in ('left', 'right')})
    assert len(model.input_sources) == 2
    for source, name in zip(model.input_sources, ('left', 'right')):
        source.accept({**packet(), 'joint_names': [name]}, 1)
    first = model.tasks[0].planner((0.,0.), (.4,0.), model.bounds, model.limits)
    second = model.tasks[1].planner((.4,0.), (.4,.4), model.bounds, model.limits)
    assert first[-1].positions == pytest.approx((.4,0.))
    assert second[-1].positions == pytest.approx((.4,.4))
