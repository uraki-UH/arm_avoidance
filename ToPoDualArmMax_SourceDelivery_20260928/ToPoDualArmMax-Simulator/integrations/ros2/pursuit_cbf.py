"""追跡障害物の共通線形CBF制約。関節空間QPと公式OSCBF目的関数の比較接続。"""
from pathlib import Path
import sys
import time

import numpy as np
from scipy import sparse


def angle_delta(goal, positions, is_periodic):
    delta = np.asarray(goal)-np.asarray(positions)
    return np.where(is_periodic, (delta+np.pi)%(2*np.pi)-np.pi, delta)


class pursuit_cbf:
    """観測距離・速度のみの速度CBF。数値失敗・実行不能時の無フィルタ代替なし。"""
    def __init__(self, geometry, active, scene, method='cbf_qp', margin=.08):
        if method not in ('cbf_qp', 'oscbf') or not 0 < margin <= .3:
            raise ValueError('CBF方式・余裕距離の不正')
        self.geometry, self.active, self.scene = geometry, np.asarray(active), scene
        self.method, self.margin = method, margin
        self.num_constraints = 128
        self.alpha = 3.
        self.previous_field = None
        self.phase = 'waiting'
        self.num_no_solution = 0
        self.num_consecutive_no_solution = 0
        self.max_num_consecutive_no_solution = 0
        self.num_recoveries = 0
        self.first_no_solution_sec = None
        self.diagnostics = {}
        self.num_constraints_used = 0
        self.warmup_ms = 0.
        self.is_periodic = np.asarray(getattr(geometry, 'is_periodic', np.zeros(len(geometry.joint_names), dtype=bool)))
        if method == 'cbf_qp':
            import osqp
            self.osqp = osqp
        else:
            root = Path(__file__).parent/'oscbf_avoidance'
            if not (root/'.deps/cbfpy').is_dir():
                raise ImportError('OSCBF依存はoscbf_avoidance/install.shでの導入が必要')
            for path in (root/'.deps', root/'vendor'):
                if str(path) not in sys.path:
                    sys.path.insert(0, str(path))
            import jax
            from cbfpy import CBF
            from oscbf_avoidance.filter import CollisionConfig
            begin = time.perf_counter()
            limits = geometry.limits[self.active]
            config = CollisionConfig(np.full(len(active), 2.), np.maximum(limits[:, 0], -1e6), np.minimum(limits[:, 1], 1e6), self.num_constraints)
            config.relax_cbf, config.solver_tol = False, 1e-6
            self.solve = jax.jit(CBF.from_config(config).safety_filter)
            num = len(active)
            self.solve(np.zeros(num), np.zeros(num), np.zeros(num), np.zeros((self.num_constraints, num)), np.ones(self.num_constraints), np.eye(num)).block_until_ready()
            self.warmup_ms = (time.perf_counter()-begin)*1000

    def constraints(self, positions, velocity, field, stamp_sec):
        num = len(self.active)
        probes = np.repeat(positions[None], num+1, axis=0)
        probes[1:, self.active] += np.eye(num)*.0001
        centers = self.geometry.centers(probes)
        gaps = field.query(centers)[0]-self.geometry.radii
        gradients = ((gaps[1:]-gaps[0])/.0001).T
        obstacle_rate = np.zeros(len(gaps[0]))
        if self.previous_field is not None:
            previous, previous_sec = self.previous_field
            period = stamp_sec-previous_sec
            if 1e-6 < period <= .3:
                # 同じ現姿勢への二観測照合。生成器の速度・将来位置の参照なし。
                obstacle_rate = (field.query(centers[0])[0]-previous.query(centers[0])[0])/period
        self.previous_field = (field, stamp_sec)
        rows, bounds = [], []
        for idx in np.flatnonzero((gaps[0] < .35) & (np.linalg.norm(gradients, axis=1) > 1e-8)):
            rows.append(gradients[idx])
            bounds.append(-self.alpha*(gaps[0, idx]-self.margin)-obstacle_rate[idx])
        pairs = self.geometry.self_pairs
        if len(pairs):
            pair_gaps = np.linalg.norm(centers[:, pairs[:, 0]]-centers[:, pairs[:, 1]], axis=-1)-self.geometry.radii[pairs[:, 0]]-self.geometry.radii[pairs[:, 1]]
            for idx in np.flatnonzero(pair_gaps[0] < .08):
                gradient = (pair_gaps[1:, idx]-pair_gaps[0, idx])/.0001
                if np.linalg.norm(gradient) > 1e-8:
                    rows.append(gradient)
                    bounds.append(-self.alpha*(pair_gaps[0, idx]-.005))
        min_velocity = np.maximum(-2., velocity[self.active]-.4)
        max_velocity = np.minimum(2., velocity[self.active]+.4)
        limits = self.geometry.limits[self.active]
        min_velocity = np.maximum(min_velocity, -self.alpha*(positions[self.active]-limits[:, 0]-.03))
        max_velocity = np.minimum(max_velocity, self.alpha*(limits[:, 1]-positions[self.active]-.03))
        rows.extend(np.eye(num));bounds.extend(min_velocity)
        rows.extend(-np.eye(num));bounds.extend(-max_velocity)
        self.num_constraints_used = len(rows)
        return np.asarray(rows), np.asarray(bounds)

    def task_weights(self):
        scene = self.scene
        body = scene.mujoco.mj_name2id(scene.model, scene.mujoco.mjtObj.mjOBJ_BODY, 'robot_L_tcp')
        dofs = [scene.model.jnt_dofadr[scene.robot.ids[self.geometry.joint_names[idx]]] for idx in self.active]
        linear = np.zeros((3, scene.model.nv));angular = np.zeros_like(linear)
        scene.mujoco.mj_jac(scene.model, scene.data, linear, angular, scene.data.xpos[body], body)
        jacobian = np.vstack((linear[:, dofs], angular[:, dofs]))
        mass = np.zeros((scene.model.nv, scene.model.nv))
        scene.mujoco.mj_fullM(scene.model, mass, scene.data.qM)
        inverse = np.linalg.inv(mass[np.ix_(dofs, dofs)])
        projected = inverse@jacobian.T@np.linalg.pinv(jacobian@inverse@jacobian.T, rcond=1e-5)
        null = np.eye(len(dofs))-projected@jacobian
        return null.T@null+jacobian.T@jacobian+np.eye(len(dofs))*.001

    def propose(self, positions, velocity, goal, field, stamp_sec):
        matrix, values = self.constraints(positions, velocity, field, stamp_sec)
        nominal = np.clip(3.*angle_delta(goal, positions, self.is_periodic)[self.active], -2., 2.)
        weights = self.task_weights() if self.method == 'oscbf' else np.eye(len(self.active))
        # 固定寸法QPへの制約生成。未選択行を含む全残差の採用前検査。
        selected = np.argsort(matrix@nominal-values)[:self.num_constraints]
        can_solve = False
        for _ in range(4):
            active_matrix = np.zeros((self.num_constraints,len(self.active)))
            active_values = np.full(self.num_constraints,-1.)
            active_matrix[:len(selected)],active_values[:len(selected)] = matrix[selected],values[selected]
            safe_velocity = self.solve_active(positions,nominal,active_matrix,active_values,weights)
            if safe_velocity is None or not np.isfinite(safe_velocity).all():
                break
            residual = matrix@safe_velocity-values
            self.diagnostics['min_residual'] = float(np.min(residual))
            if np.min(residual) >= -1e-3:
                can_solve = True
                break
            selected = np.argsort(residual)[:self.num_constraints]
        target = positions.copy()
        if not can_solve:
            self.phase = 'qp_no_solution'
            self.num_no_solution += 1
            self.num_consecutive_no_solution += 1
            self.max_num_consecutive_no_solution = max(self.max_num_consecutive_no_solution,self.num_consecutive_no_solution)
            if self.first_no_solution_sec is None:
                self.first_no_solution_sec = float(stamp_sec)
            # 呼出元の速度・加速度制限による減速保持。安全な代替解との混同禁止。
            return target
        self.num_recoveries += int(self.num_consecutive_no_solution>0)
        self.num_consecutive_no_solution = 0
        target[self.active] += safe_velocity*.1
        self.phase = 'task' if np.max(np.abs(safe_velocity-nominal)) < .01 else 'avoiding'
        return target

    def solve_active(self,positions,nominal,matrix,values,weights):
        if self.method == 'cbf_qp':
            solver = self.osqp.OSQP()
            solver.setup(P=sparse.eye(len(self.active), format='csc'), q=-nominal,
                A=sparse.csc_matrix(matrix), l=values, u=np.full(len(values), np.inf),
                verbose=False, eps_abs=1e-6, eps_rel=1e-6, max_iter=2000, polishing=False)
            result = solver.solve()
            self.diagnostics['solver_status'] = result.info.status
            safe_velocity = result.x if result.info.status_val in (1, 2) else None
        else:
            safe_velocity = np.asarray(self.solve(positions[self.active], nominal, positions[self.active], matrix, -values/self.alpha, weights))
            self.diagnostics['solver_status'] = 'cbfpy_result_validation'
        return safe_velocity
