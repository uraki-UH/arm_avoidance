"""局所QPの距離・関節制約と、出力失敗時の停止経路の検証。"""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from local_qp import local_qp
from dual_arm_avoidance_geometry import robot_geometry
from dual_arm_avoidance_demo import avoidance_demo
from dual_arm_gng_lidar_demo import gng_lidar_demo


@pytest.fixture
def setup():
    geometry = SimpleNamespace(joint_names=['x', 'y', 'other'], limits=np.array([[-1., 1.]]*3),
        radii=np.array([.05]), is_arm=np.array([True]), self_pairs=np.empty((0, 2), dtype=int),
        centers=lambda q: np.array([[q[0], q[1], .5]]))
    geometry.internal_clearances = lambda centers: robot_geometry.internal_clearances(geometry, centers)
    config = {'control_period_sec': .2, 'max_joint_velocity': .4,
              'min_cloud_clearance_th': .015, 'min_clearance_th': .035,
              'min_planning_clearance_th': .01, 'target_clearance': .12,
              'local_qp': {'max_joint_acceleration': 3., 'max_solve_sec': .02}}
    solver = local_qp(geometry, [.5]*3, config)
    tree = cKDTree([[.1, 0, .5]])
    return solver, tree


def project(setup, target, **changes):
    solver, tree = setup
    args = dict(positions=np.zeros(3), target=np.array(target, dtype=float), active=[0, 1],
                cloud_tree=tree, cell_radius=0., can_bridge=lambda *_: True)
    args.update(changes)
    return solver.project(**args)


def test_approach_limited_but_tangent_motion_preserved(setup):
    result = project(setup, [.02, .01, .8])
    np.testing.assert_allclose(result, [.0075, .01, 0.], atol=1e-6)
    assert setup[0].report['status'] == 'solved'


def test_clear_space_preserves_nominal_target(setup):
    result = project(setup, [.01, .005, 0.], cloud_tree=cKDTree([[4, 0, .5]]))
    np.testing.assert_allclose(result, [.01, .005, 0.], atol=1e-6)


def test_fallback_retreat_increases_clearance(setup):
    result = project(setup, [0., 0., 0.])
    assert result[0] < -.001 and abs(result[1]) < 1e-6 and result[2] == 0


def test_quintic_velocity_and_acceleration_limits(setup):
    solver, _ = setup
    result = project(setup, [-.9, .9, 0.])
    duration = solver.config['control_period_sec']
    assert np.max(abs(result))*1.875/duration <= .4+1e-6
    assert np.max(abs(result))*(10/np.sqrt(3))/duration**2 <= 3.+1e-6


def test_joint_limit_and_inactive_joint_preserved(setup):
    setup[0].geometry.limits[0, 0] = -.003
    result = project(setup, [-.1, 0., .7])
    assert -.003-1e-7 <= result[0] < 0 and result[2] == 0


def test_inactive_joint_is_measured_fixed_even_outside_limit(setup):
    result = project(setup, [-.01, 0., 2.], positions=np.array([0., 0., 2.]))
    assert result is not None and result[2] == 2.


def test_active_joint_outside_limit_is_rejected(setup):
    assert project(setup, [2., 0., 0.], positions=np.array([2., 0., 0.])) is None
    assert setup[0].report['status'] == 'joint_limit'


@pytest.mark.parametrize('kind', ['nan', 'empty_cloud', 'collision', 'floor', 'duplicate', 'out_of_range'])
def test_invalid_input_or_clearance_rejected(setup, kind):
    changes = {}
    if kind == 'nan':
        changes['positions'] = np.array([np.nan, 0., 0.])
    elif kind == 'empty_cloud':
        changes['cloud_tree'] = cKDTree(np.empty((0, 3)))
    elif kind == 'collision':
        changes['cloud_tree'] = cKDTree([[.05, 0, .5]])
    elif kind == 'floor':
        setup[0].geometry.centers = lambda _: np.array([[0., 0., .04]])
    elif kind == 'duplicate':
        changes['active'] = [0, 0]
    else:
        changes['active'] = [3]
    assert project(setup, [0., 0., 0.], **changes) is None


def test_nonlinear_path_rejection_has_no_candidate(setup):
    assert project(setup, [-.01, 0., 0.], can_bridge=lambda *_: False) is None
    assert setup[0].report['status'] == 'path_rejected'


@pytest.mark.parametrize('kind,expected', [('floor', -.005), ('table', .0075), ('self', .005)])
def test_internal_surfaces_limit_approach(setup, kind, expected):
    geometry = setup[0].geometry
    if kind == 'floor':
        geometry.centers = lambda q: np.array([[0., -.8, .07+q[0]]])
    elif kind == 'table':
        geometry.centers = lambda q: np.array([[.45+q[0], 0., .2]])
    else:
        geometry.centers = lambda q: np.array([[q[0], 0., .5], [.12, 0., .5]])
        geometry.radii = np.array([.05, .05])
        geometry.is_arm = np.array([True, False])
        geometry.self_pairs = np.array([[0, 1]])
    result = project(setup, [np.sign(expected)*.02, 0., 0.], cloud_tree=cKDTree([[4, 0, .5]]))
    assert result is not None
    assert result[0] == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize('status', [2, 3, 7, 8])
def test_inaccurate_infeasible_or_timeout_has_no_candidate(setup, status):
    with patch.object(setup[0].osqp, 'OSQP') as factory:
        factory.return_value.solve.return_value = SimpleNamespace(
            x=np.zeros(2), info=SimpleNamespace(status='rejected', status_val=status, run_time=.001))
        assert project(setup, [-.01, 0., 0.]) is None


@pytest.mark.parametrize('target,expected_hessian,expected_linear', [
    ([.02, .01, .8], [1., 1.], [-.02, -.01]),
    ([0., 0., 0.], [201., 1.], [14., 0.]),
])
def test_solver_boundary_preserves_objective_constraints_and_limits(
        setup, target, expected_hessian, expected_linear):
    solver, _ = setup
    delta = np.array([-.007, .002])
    can_bridge = Mock(return_value=True)
    with patch.object(solver.osqp, 'OSQP') as factory:
        factory.return_value.solve.return_value = SimpleNamespace(
            x=delta, info=SimpleNamespace(status='solved', status_val=1, run_time=.000125))
        result = project(setup, target, can_bridge=can_bridge)
        arguments = factory.return_value.setup.call_args.kwargs
        factory.return_value.solve.assert_called_once_with(raise_error=False)
    np.testing.assert_allclose(arguments['P'].toarray(), np.diag(expected_hessian))
    np.testing.assert_allclose(arguments['q'], expected_linear)
    np.testing.assert_allclose(arguments['A'].toarray(), [[1., 0.], [0., 1.], [-1., 0.]])
    max_delta = 3.*.2**2/(10/np.sqrt(3))
    np.testing.assert_allclose(arguments['l'], [-max_delta, -max_delta, -.0075])
    np.testing.assert_allclose(arguments['u'], [max_delta, max_delta, np.inf])
    assert {key: arguments[key] for key in (
        'verbose', 'eps_abs', 'eps_rel', 'max_iter', 'time_limit', 'polishing')} == {
            'verbose': False, 'eps_abs': 1e-8, 'eps_rel': 1e-8, 'max_iter': 2000,
            'time_limit': .02, 'polishing': False}
    np.testing.assert_allclose(result, [-.007, .002, 0.])
    assert can_bridge.call_count == 1
    np.testing.assert_array_equal(can_bridge.call_args.args[0], np.zeros(3))
    np.testing.assert_array_equal(can_bridge.call_args.args[1], result)
    assert can_bridge.call_args.args[2] == .035
    assert solver.report['status'] == 'solved'
    assert solver.report['num_constraints'] == 3
    assert solver.report['num_calls'] == 1
    assert solver.report['solve_ms'] == .125
    assert solver.report['total_ms'] >= 0.


@pytest.mark.parametrize('delta,expected_status', [
    (None, 'solved'),
    (np.array([np.nan, 0.]), 'solved'),
    (np.array([np.inf, 0.]), 'solved'),
    (np.array([-.1, 0.]), 'constraint_violation'),
    (np.array([0., .1]), 'constraint_violation'),
    (np.array([.01, 0.]), 'constraint_violation'),
])
def test_invalid_solver_result_never_reaches_path_check(setup, delta, expected_status):
    solver, _ = setup
    can_bridge = Mock(return_value=True)
    with patch.object(solver.osqp, 'OSQP') as factory:
        factory.return_value.solve.return_value = SimpleNamespace(
            x=delta, info=SimpleNamespace(status='solved', status_val=1, run_time=.001))
        assert project(setup, [-.01, 0., 0.], can_bridge=can_bridge) is None
    can_bridge.assert_not_called()
    assert solver.report['status'] == expected_status
    assert solver.report['solve_ms'] == 1.


@pytest.mark.parametrize('phase', ['setup', 'solve'])
@pytest.mark.parametrize('kind', ['value', 'osqp'])
def test_solver_exceptions_preserve_failure_report(setup, phase, kind):
    solver, _ = setup
    exception = ValueError('invalid problem') if kind == 'value' else solver.osqp.OSQPException(1)
    with patch.object(solver.osqp, 'OSQP') as factory:
        getattr(factory.return_value, phase).side_effect = exception
        assert project(setup, [-.01, 0., 0.]) is None
    assert solver.report['status'] == 'solver_error'
    assert solver.report['num_constraints'] == 3
    assert solver.report['num_calls'] == 1
    assert solver.report['total_ms'] >= 0.


def test_no_active_joint_keeps_hold_without_solver(setup):
    solver, _ = setup
    positions = np.array([0., 0., 2.])
    with patch.object(solver.osqp, 'OSQP') as factory:
        result = project(setup, [.3, .2, -.1], positions=positions, active=[])
        factory.assert_not_called()
    np.testing.assert_array_equal(result, positions)
    assert result is not positions
    assert solver.report['status'] == 'hold'
    assert solver.report['num_constraints'] == 0


@pytest.mark.parametrize('kind', ['no_solution', 'stale', 'overrun'])
def test_output_failure_never_forwards_nominal_target(setup, kind):
    node = gng_lidar_demo.__new__(gng_lidar_demo)
    node.state, node.is_stop_latched = 'running', False
    node.positions = np.zeros(3)
    node.arm_indices, node.active_angle_indices = [0, 1], [0, 1]
    node.cloud_tree, node.cell_radius = setup[1], 0.
    node.can_bridge = Mock()
    node.config = setup[0].config
    node.qp = Mock()
    node.qp.project.return_value = None if kind == 'no_solution' else np.ones(3)
    node.qp.report = {'total_ms': 500. if kind == 'overrun' else 1., 'status': kind}
    node.is_fresh = lambda: kind != 'stale'
    node.fail = Mock()
    with patch.object(avoidance_demo, 'publish_target') as publish:
        node.publish_target(np.ones(3)*.1)
        publish.assert_not_called()
    node.fail.assert_called_once()
