"""タスク設定・数値軌道・中断再開・Action取消競合の回帰試験。"""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from task_program import (joint_bound, joint_sample, load_program, plan_segment,
                          read_joint_bounds, run_state, task_kind, task_method, task_methods, task_runner)


def config():
    return dict(version=1, poses={'work': {'arm': 0.4}, 'home': {'arm': 0.0}},
                paths={'back': ['work', 'home']}, tasks=[
                    {'kind': 'move', 'target': 'work'},
                    {'kind': 'hold', 'duration_sec': 1.0},
                    {'kind': 'move', 'method': 'waypoints', 'path': 'back'}])


def program(data=None, methods=None):
    return load_program(config() if data is None else data,
                        {'arm': joint_bound(-1.0, 1.0, 0.4)}, methods)


class fake_backend:
    def __init__(self):
        self.is_busy = False
        self.result = None
        self.commands = []
        self.num_cancels = 0

    def begin(self, points):
        assert not self.is_busy
        self.commands.append(points)
        self.is_busy, self.result = True, None

    def cancel(self):
        self.num_cancels += 1

    def finish(self, is_success=True):
        self.is_busy, self.result = False, is_success


def sample(now, position=0.0, velocity=0.0):
    return joint_sample(now, (position,), (velocity,))


def started():
    backend = fake_backend()
    runner = task_runner(program(), backend)
    runner.tick(0.0, sample(0.0))
    runner.tick(0.3, sample(0.3))
    runner.start(0.3, sample(0.3))
    runner.tick(0.4, sample(0.4))
    return runner, backend


def test_defaults_and_per_task_method():
    tasks = program().tasks
    assert [(task.kind.value, task.method) for task in tasks] == [('move', 'direct'), ('hold', 'position'), ('move', 'waypoints')]
    assert tasks[-1].targets == ({'arm': 0.4}, {'arm': 0.0})


@pytest.mark.parametrize('change', [
    lambda data: data.update(version=True),
    lambda data: data.update(typo=1),
    lambda data: data.update(tasks=[]),
    lambda data: data.update(defaults={'move': 'gng'}),
    lambda data: data.update(limits={'max_velocity': float('nan')}),
    lambda data: data.update(limits={'max_acceleration': False}),
    lambda data: data['poses']['work'].update(arm=2.0),
    lambda data: data['poses']['work'].update(unknown=0.0),
    lambda data: data['paths'].update(back=[]),
    lambda data: data['paths'].update(back=['missing']),
    lambda data: data['tasks'][0].update(kind='grasp'),
    lambda data: data['tasks'][0].update(method='local_qp'),
    lambda data: data['tasks'][0].update(target='missing'),
    lambda data: data['tasks'][0].update(duration_sec=1.0),
    lambda data: data['tasks'][1].update(duration_sec=-1),
    lambda data: data['tasks'][1].update(duration_sec=60),
])
def test_reject_invalid_config(change):
    data = config()
    change(data)
    with pytest.raises(ValueError):
        program(data)


def test_method_replacement_without_runner_branch():
    methods = dict(task_methods)
    calls = []
    def custom_planner(start, target, bounds, limits):
        calls.append(start)
        return plan_segment(start, target, bounds, limits)
    methods[(task_kind.move, 'custom')] = task_method(lambda item, poses, paths: ({'arm': 0.1},), custom_planner)
    data = config()
    data['defaults'] = {'move': 'custom'}
    assert program(data, methods).tasks[0].targets == ({'arm': 0.1},)
    runner = task_runner(program(data, methods), fake_backend())
    runner.tick(0.0, sample(0.0, 0.05))
    runner.tick(0.3, sample(0.3, 0.05))
    runner.start(0.3, sample(0.3, 0.05))
    runner.tick(0.4, sample(0.4, 0.05))
    assert calls == [(0.05,)]


def test_custom_planner_cannot_bypass_output_checks():
    methods = dict(task_methods)
    methods[(task_kind.move, 'direct')] = task_method(methods[(task_kind.move, 'direct')].resolve_targets,
                                                    lambda *args: ())
    backend = fake_backend()
    runner = task_runner(program(methods=methods), backend)
    runner.tick(0.0, sample(0.0))
    runner.tick(0.3, sample(0.3))
    runner.start(0.3, sample(0.3))
    runner.tick(0.4, sample(0.4))
    assert runner.state == run_state.failed and not backend.commands


def test_trajectory_bounds_and_endpoints():
    model = program()
    points = plan_segment((0.7,), (-0.9,), model.bounds, model.limits)
    assert points[0].positions == (0.7,)
    assert points[-1].positions == pytest.approx((-0.9,))
    assert points[0].velocities == points[-1].velocities == (0.0,)
    assert points[0].accelerations == points[-1].accelerations == (0.0,)
    assert max(abs(point.velocities[0]) for point in points) <= model.limits['max_velocity'] + 1e-12
    assert max(abs(point.accelerations[0]) for point in points) <= model.limits['max_acceleration'] + 1e-12
    assert all(left.time_sec < right.time_sec for left, right in zip(points, points[1:]))


def test_pause_waits_for_cancel_ack_and_resume_from_measured_position():
    runner, backend = started()
    runner.set_obstacle(True, 0.5)
    assert runner.state == run_state.stopping
    runner.tick(0.8, sample(0.8, 0.15))
    assert runner.state == run_state.stopping
    with pytest.raises(ValueError):
        runner.resume(0.8, sample(0.8, 0.15))
    backend.finish(False)
    runner.tick(0.9, sample(0.9, 0.15))
    assert runner.state == run_state.paused
    with pytest.raises(ValueError):
        runner.resume(0.9, sample(0.9, 0.15))
    runner.set_obstacle(False, 0.9)
    assert runner.state == run_state.paused
    runner.resume(0.9, sample(0.9, 0.15))
    runner.tick(1.0, sample(1.0, 0.15))
    assert backend.commands[-1][0].positions == (0.15,)
    assert backend.commands[-1][-1].positions == pytest.approx((0.4,))
    assert runner.task_idx == runner.waypoint_idx == 0


def test_completion_requires_measured_arrival_and_hold_pause_preserves_time():
    runner, backend = started()
    backend.finish()
    runner.tick(0.6, sample(0.6, 0.1))
    assert runner.task_idx == 0
    runner.tick(0.7, sample(0.7, 0.4))
    assert runner.task_idx == 1
    runner.tick(0.8, sample(0.8, 0.4))
    backend.finish()
    runner.tick(1.1, sample(1.1, 0.4))
    before = runner.hold_sec
    runner.interrupt(1.1)
    runner.tick(1.4, sample(1.4, 0.4))
    runner.tick(9.0, sample(9.0, 0.4))
    assert runner.state == run_state.paused
    assert runner.hold_sec == before
    runner.resume(9.0, sample(9.0, 0.4))
    runner.tick(9.1, sample(9.1, 0.4))
    backend.finish()
    runner.tick(9.3, sample(9.3, 0.4))
    assert runner.hold_sec == pytest.approx(before + 0.2)


@pytest.mark.parametrize('bad', [None, joint_sample(0.0, (0.1,), (0.0,)),
    joint_sample(2.0, (0.1,), (0.0,)), joint_sample(1.0, (float('nan'),), (0.0,)),
    joint_sample(1.0, (0.1,), ()), joint_sample(1.0, (2.0,), (0.0,))])
def test_invalid_feedback_cancels_and_latches_failure(bad):
    runner, backend = started()
    runner.tick(1.0, bad)
    assert runner.state == run_state.failed
    assert backend.num_cancels == 1
    with pytest.raises(ValueError):
        runner.start(1.1, sample(1.1))


def test_stopping_timeout_and_action_failure():
    runner, backend = started()
    runner.interrupt(0.5)
    runner.tick(6.0, sample(6.0))
    assert runner.state == run_state.failed
    runner, backend = started()
    backend.finish(False)
    runner.tick(0.6, sample(0.6))
    assert runner.state == run_state.failed


def test_complete_all_tasks_and_waypoints():
    runner, backend = started()
    for idx in range(1, 80):
        position = backend.commands[-1][-1].positions[0]
        backend.finish()
        now = 0.4 + idx * 0.1
        runner.tick(now, sample(now, position))
        if runner.state == run_state.succeeded:
            break
    assert runner.state == run_state.succeeded
    assert len(backend.commands) == 4
    assert backend.commands[-1][-1].positions == pytest.approx((0.0,))


def test_cancel_cannot_be_downgraded_by_pause():
    runner, backend = started()
    runner.interrupt(0.5, is_cancel=True)
    runner.interrupt(0.6)
    backend.finish(False)
    runner.tick(0.9, sample(0.9))
    assert runner.state == run_state.canceled


def test_hold_preserves_last_command_instead_of_accumulating_static_error():
    runner, backend = started()
    backend.finish()
    runner.tick(0.7, sample(0.7, 0.39))
    assert runner.task_idx == 1
    runner.tick(0.8, sample(0.8, 0.39))
    assert backend.commands[-1][0].positions == pytest.approx((0.39,))
    assert backend.commands[-1][-1].positions == pytest.approx((0.4,))


def test_waypoint_resume_does_not_repeat_completed_waypoints():
    data = config()
    data['tasks'] = [data['tasks'][2]]
    backend = fake_backend()
    runner = task_runner(program(data), backend)
    runner.tick(0.0, sample(0.0))
    runner.tick(0.3, sample(0.3))
    runner.start(0.3, sample(0.3))
    runner.tick(0.4, sample(0.4))
    backend.finish()
    runner.tick(0.7, sample(0.7, 0.4))
    runner.tick(0.8, sample(0.8, 0.4))
    runner.interrupt(0.9)
    backend.finish(False)
    runner.tick(1.0, sample(1.0, 0.3))
    runner.resume(1.0, sample(1.0, 0.3))
    runner.tick(1.1, sample(1.1, 0.3))
    assert runner.waypoint_idx == 1
    assert backend.commands[-1][-1].positions == pytest.approx((0.0,))


def test_clock_rewind_cancels_active_task():
    runner, backend = started()
    runner.tick(0.1, sample(0.1))
    assert runner.state == run_state.failed
    assert backend.num_cancels == 1


def test_shipped_config_with_real_model():
    root = Path(__file__).resolve().parents[2]
    bounds = read_joint_bounds(root / 'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf')
    data = yaml.safe_load((root / 'gng_vlut_system/config/simulation/task_program.yaml').read_text())
    assert len(load_program(data, bounds).tasks) == 3
    assert not any('mimic' in name for name in bounds)


@pytest.mark.parametrize('layout', ['source', 'installed'])
def test_urdf_resolution_without_workspace_absolute_path(tmp_path, monkeypatch, layout):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
    import dual_arm_effort_config
    package_dir = tmp_path / 'share/gng_vlut_system'
    monkeypatch.setattr(dual_arm_effort_config, '__file__', str(package_dir / 'launch/dual_arm_effort_config.py'))
    root = package_dir if layout == 'installed' else package_dir.parent
    for model in ('topo_dual_arm_max', 'topo_dual_arm_max_long'):
        path = root / 'urdf' / model / 'topo_dual_arm_max.urdf'
        path.parent.mkdir(parents=True)
        path.write_text('<robot name="fixture"/>')
        assert dual_arm_effort_config.default_urdf(model) == path
    assert 'topo_dual_arm_max_long' in str(dual_arm_effort_config.default_urdf())


def test_missing_urdf_is_not_a_silent_source_dependency(tmp_path, monkeypatch):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
    import dual_arm_effort_config
    monkeypatch.setattr(dual_arm_effort_config, '__file__', str(tmp_path / 'share/gng_vlut_system/launch/config.py'))
    with pytest.raises(FileNotFoundError):
        dual_arm_effort_config.default_urdf()


def test_action_cancel_before_acceptance(monkeypatch):
    pytest.importorskip('rclpy')
    import task_executor
    from concurrent.futures import Future
    accepted, finished, canceled = Future(), Future(), Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: finished)
    calls = []
    handle.cancel_goal_async = lambda: (calls.append('cancel') or canceled)
    monkeypatch.setattr(task_executor, 'ActionClient', lambda *args: SimpleNamespace(send_goal_async=lambda goal: accepted))
    model = program()
    backend = task_executor.trajectory_backend(SimpleNamespace(), ('arm',), model.limits)
    backend.begin(plan_segment((0.0,), (0.1,), model.bounds, model.limits))
    backend.cancel()
    assert backend.is_busy and not calls
    accepted.set_result(handle)
    assert calls == ['cancel'] and backend.is_busy
    canceled.set_result(SimpleNamespace(goals_canceling=[1]))
    assert backend.is_busy
    finished.set_result(SimpleNamespace(status=5, result=SimpleNamespace(error_code=0)))
    assert not backend.is_busy and backend.result is False
