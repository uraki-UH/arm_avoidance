"""観測点群・自己形状の距離制約による関節変位QP。実機指令なし。"""
from dataclasses import dataclass
import time

import numpy as np
from scipy import sparse

from motion_smoothing import quintic_peak_velocity_factor


@dataclass(frozen=True)
class local_qp_problem:
    """関節変位QPの目的関数と線形制約。ソルバ境界の入出力契約。"""

    hessian: np.ndarray
    linear: np.ndarray
    matrix: sparse.csc_matrix
    min_values: np.ndarray
    max_values: np.ndarray


class local_qp:
    def __init__(self, geometry, max_velocity, config):
        import osqp

        self.osqp = osqp
        self.geometry = geometry
        self.config = config
        settings = config['local_qp']
        self.max_acceleration = settings['max_joint_acceleration']
        self.max_solve_sec = settings['max_solve_sec']
        for value in (self.max_acceleration, self.max_solve_sec):
            if not np.isfinite(value) or value <= 0:
                raise ValueError('QPの加速度・時間上限には有限の正数が必要です')
        self.max_velocity = np.minimum(np.asarray(max_velocity), config['max_joint_velocity'])
        if (self.max_velocity.shape != (len(geometry.joint_names),)
                or not np.all(np.isfinite(self.max_velocity)) or np.any(self.max_velocity <= 0)):
            raise ValueError('QPの関節速度上限が不正です')
        self.report = {'status': 'idle', 'total_ms': 0., 'solve_ms': 0., 'num_constraints': 0}
        self.num_calls = 0

    def project(self, positions, target, active, cloud_tree, cell_radius, can_bridge):
        """局所線形制約の求解と既存の非線形区間検査。失敗時の候補返却なし。"""
        began = time.perf_counter()
        self.num_calls += 1
        self.report = {'status': 'invalid_input', 'total_ms': 0., 'solve_ms': 0.,
                       'num_constraints': 0, 'num_calls': self.num_calls}
        try:
            return self._project(positions, target, active, cloud_tree, cell_radius, can_bridge)
        except (ValueError, self.osqp.OSQPException):
            self.report['status'] = 'solver_error'
            return None
        finally:
            self.report['total_ms'] = (time.perf_counter()-began)*1000

    def _project(self, positions, target, active, cloud_tree, cell_radius, can_bridge):
        positions, target = np.asarray(positions), np.asarray(target)
        active = np.asarray(active, dtype=int)
        if (positions.shape != self.max_velocity.shape or target.shape != positions.shape
                or not np.all(np.isfinite([positions, target]))
                or not np.isfinite(cell_radius) or cell_radius < 0
                or cloud_tree is None or cloud_tree.n == 0
                or active.ndim != 1 or len(set(active)) != len(active)
                or np.any(active < 0) or np.any(active >= len(positions))):
            return None
        geometry, config = self.geometry, self.config
        # 非制御関節は実測固定。制御対象だけの関節範囲制約と数値丸め余裕。
        if (np.any(positions[active] < geometry.limits[active, 0]-1e-7)
                or np.any(positions[active] > geometry.limits[active, 1]+1e-7)):
            self.report['status'] = 'joint_limit'
            return None
        min_cloud_gap = max(config['min_cloud_clearance_th'], config['min_clearance_th'])
        min_internal_gap = config.get('min_planning_clearance_th', .005)
        num_cloud = int(np.count_nonzero(geometry.is_arm))

        centers = geometry.centers(positions)
        distances, nearest = cloud_tree.query(centers[geometry.is_arm])
        cloud_gaps = distances-geometry.radii[geometry.is_arm]-cell_radius
        current_gaps = np.concatenate((cloud_gaps, geometry.internal_clearances(centers)))
        min_gaps = np.full(len(current_gaps), min_internal_gap)
        min_gaps[:num_cloud] = min_cloud_gap
        if not np.all(np.isfinite(current_gaps)) or np.any(current_gaps < min_gaps):
            self.report['status'] = 'insufficient_clearance'
            return None
        if not len(active):
            self.report['status'] = 'hold'
            return positions.copy()

        jacobian = self._clearance_jacobian(positions, active, centers, cloud_tree.data[nearest])
        if not np.all(np.isfinite(jacobian)):
            return None
        problem = self._build_problem(positions, target, active, current_gaps, min_gaps, jacobian)
        delta = self._solve_problem(problem)
        if delta is None:
            return None
        candidate = positions.copy()
        candidate[active] += delta
        if (np.any(candidate[active] < geometry.limits[active, 0]-1e-7)
                or np.any(candidate[active] > geometry.limits[active, 1]+1e-7)
                or not can_bridge(positions, candidate, min_cloud_gap)):
            self.report['status'] = 'path_rejected'
            return None
        self.report['status'] = 'solved'
        return candidate

    def _clearance_jacobian(self, positions, active, centers, nearest_points):
        """観測点群・自己干渉・環境形状の距離ヤコビアン。制約行の順序を維持。"""
        geometry = self.geometry
        # 球中心の前進差分と距離法線の合成。関節ごとの最近傍再探索なし。
        jacobian_step = 1e-5
        center_jacobian = np.empty((*centers.shape, len(active)))
        for column, idx in enumerate(active):
            offset = np.zeros(len(positions))
            offset[idx] = jacobian_step
            center_jacobian[:, :, column] = (geometry.centers(positions+offset)-centers)/jacobian_step

        def distance_gradient(delta, derivative):
            normal = delta/np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-12)
            return np.einsum('ni,nij->nj', normal, derivative)

        cloud_jacobian = distance_gradient(centers[geometry.is_arm]-nearest_points,
                                           center_jacobian[geometry.is_arm])
        first, second = geometry.self_pairs.T
        self_jacobian = distance_gradient(centers[first]-centers[second],
                                          center_jacobian[first]-center_jacobian[second])
        arm_centers = centers[geometry.is_arm]
        arm_jacobian = center_jacobian[geometry.is_arm]
        rows = [cloud_jacobian, self_jacobian]
        for kind, position, rotation_matrix, half_size in geometry.environment_shapes:
            if kind == 'plane':
                rows.append(np.einsum('i,nij->nj', rotation_matrix[:, 2], arm_jacobian))
            elif kind == 'box':
                local = (arm_centers-position) @ rotation_matrix
                delta = np.abs(local)-half_size
                outward = np.sign(local)*np.maximum(delta, 0.)
                normal = outward/np.maximum(np.linalg.norm(outward, axis=1, keepdims=True), 1e-12)
                # 箱内部の符号付き距離における最近接面の法線
                inside_idx = np.flatnonzero(np.all(delta <= 0., axis=1))
                face_idx = np.argmax(delta[inside_idx], axis=1)
                normal[inside_idx, face_idx] = np.where(local[inside_idx, face_idx] < 0., -1., 1.)
                rows.append(np.einsum('ni,nij->nj', normal @ rotation_matrix.T, arm_jacobian))
            else:
                raise ValueError('未対応の環境衝突形状: '+kind)
        return np.vstack(rows)

    def _build_problem(self, positions, target, active, current_gaps, min_gaps, jacobian):
        """追従・局所退避の目的関数と、関節範囲・速度・加速度・距離制約の組立て。"""
        geometry, config = self.geometry, self.config
        num_cloud = int(np.count_nonzero(geometry.is_arm))
        duration = config['control_period_sec']
        # 静止端点間の5次補間における速度係数1.875・加速度係数10/√3。
        max_delta = np.minimum(self.max_velocity[active]*duration/quintic_peak_velocity_factor,
                               self.max_acceleration*duration**2/(10/np.sqrt(3)))
        lower = np.maximum(-max_delta, geometry.limits[active, 0]-positions[active])
        upper = np.minimum(max_delta, geometry.limits[active, 1]-positions[active])
        nominal = target[active]-positions[active]
        hessian = np.eye(len(active))
        linear = -nominal
        # GNG候補待ち・疎なグラフでの局所退避。経路候補がある場合は追従を優先。
        if np.max(np.abs(nominal)) < 1e-6:
            is_close = current_gaps[:num_cloud] < config['target_clearance']
            gradient = jacobian[:num_cloud][is_close]
            desired_gap = config['target_clearance']-current_gaps[:num_cloud][is_close]
            hessian += 200.*np.einsum('ni,nj->ij', gradient, gradient)
            linear -= 200.*np.einsum('ni,n->i', gradient, desired_gap)
        # 1区間で安全余裕の半分を消費可能な接近制限。衝突制約の緩和なし。
        # 関節変位上限だけで充足する距離制約の除外。近接面の個数による打切りなし。
        margin = .5*(current_gaps-min_gaps)
        is_relevant = margin <= np.einsum('ni,i->n', np.abs(jacobian), max_delta)+1e-9
        matrix = sparse.csc_matrix(np.vstack((np.eye(len(active)), jacobian[is_relevant])))
        lower = np.concatenate((lower, -margin[is_relevant]))
        upper = np.concatenate((upper, np.full(np.count_nonzero(is_relevant), np.inf)))
        return local_qp_problem(hessian, linear, matrix, lower, upper)

    def _solve_problem(self, problem):
        """時間上限つき求解と線形制約の再検査。非線形区間検査前の関節変位。"""
        self.report['num_constraints'] = len(problem.min_values)
        solver = self.osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(problem.hessian), q=problem.linear,
                     A=problem.matrix, l=problem.min_values, u=problem.max_values,
                     verbose=False, eps_abs=1e-8, eps_rel=1e-8, max_iter=2000,
                     time_limit=self.max_solve_sec, polishing=False)
        result = solver.solve(raise_error=False)
        self.report.update(status=result.info.status, solve_ms=result.info.run_time*1000)
        if result.info.status_val != 1 or result.x is None or not np.all(np.isfinite(result.x)):
            return None
        constrained = problem.matrix@result.x
        if (np.any(constrained < problem.min_values-1e-7)
                or np.any(constrained > problem.max_values+1e-7)):
            self.report['status'] = 'constraint_violation'
            return None
        return result.x
