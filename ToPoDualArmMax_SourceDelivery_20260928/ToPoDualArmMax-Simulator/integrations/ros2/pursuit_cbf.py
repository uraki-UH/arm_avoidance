"""追跡障害物の共通線形CBF制約。関節空間QPと公式OSCBF目的関数の比較接続。"""
import numpy as np
from scipy import sparse
from scipy.optimize import linprog, minimize
from gng_sphere_avoidance import joint_velocity_command


def angle_delta(goal, positions, is_periodic):
    delta = np.asarray(goal)-np.asarray(positions)
    return np.where(is_periodic, (delta+np.pi)%(2*np.pi)-np.pi, delta)


class pursuit_cbf:
    """観測距離・速度のみの速度CBF。数値失敗・実行不能時の無フィルタ代替なし。"""
    def __init__(self, geometry, active, scene, method='cbf_qp', margin=.08, task=None, internal_checker=None):
        if method not in ('cbf_qp', 'oscbf') or not 0 < margin <= .3:
            raise ValueError('CBF方式・余裕距離の不正')
        self.geometry, self.active, self.scene = geometry, np.asarray(active), scene
        self.method, self.margin = method, margin
        self.task = task
        self.internal_checker = internal_checker
        self.alpha = 3.
        self.previous_field = None
        self.phase = 'waiting'
        self.num_no_solution = 0
        self.num_relaxed_solutions = 0
        self.num_consecutive_no_solution = 0
        self.max_num_consecutive_no_solution = 0
        self.num_recoveries = 0
        self.first_no_solution_sec = None
        self.diagnostics = {}
        self.num_constraints_used = 0
        self.warmup_ms = 0.
        self.is_periodic = np.asarray(getattr(geometry, 'is_periodic', np.zeros(len(geometry.joint_names), dtype=bool)))
        # 作業空間目的関数は維持。固定行数への切捨て・JAX経路との二重検証を廃止。
        import osqp
        self.osqp = osqp
        self.diagnostics['solver_backend'] = 'osqp'
        self.diagnostics['num_no_solution_by_reason'] = dict(solver=0,residual=0,internal_guard=0)
        self.diagnostics['num_numerical_recoveries'] = 0

    def constraints(self, positions, velocity, field, stamp_sec):
        num = len(self.active)
        probes = np.repeat(positions[None], num+1, axis=0)
        probes[1:, self.active] += np.eye(num)*.0001
        frames = None
        if self.internal_checker is not None:
            frames = self.geometry.link_transforms(probes)
            selected = frames[:,self.geometry.sphere_links]
            centers = np.einsum('pnij,nj->pni',selected[:,:,:3,:3],self.geometry.sphere_points)+selected[:,:,:3,3]
        else:
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
        self.num_external_constraints = len(rows)
        internal_margin = .005 if self.internal_checker is None else self.internal_checker.min_clearance_th
        if self.internal_checker is not None:
            internal_gaps = self.internal_checker.linear_samples(probes,centers,frames)
        else:
            pairs = self.geometry.self_pairs
            parts = [np.empty((len(probes),0))]
            if len(pairs):
                parts.append(np.linalg.norm(centers[:,pairs[:,0]]-centers[:,pairs[:,1]],axis=-1)-self.geometry.radii[pairs[:,0]]-self.geometry.radii[pairs[:,1]])
            if getattr(self.geometry,'environment_shapes',()):
                parts.append(self.geometry.environment_clearances(centers))
            internal_gaps = np.concatenate(parts,axis=1)
        for idx in np.flatnonzero(internal_gaps[0]<.12):
            gradient = (internal_gaps[1:,idx]-internal_gaps[0,idx])/.0001
            if np.linalg.norm(gradient)>1e-8:
                rows.append(gradient)
                bounds.append(-self.alpha*(internal_gaps[0,idx]-internal_margin))
        limits = self.geometry.limits[self.active]
        min_velocity = joint_velocity_command(positions[self.active],velocity[self.active],np.full(num,-2.),limits)
        max_velocity = joint_velocity_command(positions[self.active],velocity[self.active],np.full(num,2.),limits)
        rows.extend(np.eye(num));bounds.extend(min_velocity)
        rows.extend(-np.eye(num));bounds.extend(-max_velocity)
        self.num_constraints_used = len(rows)
        return np.asarray(rows), np.asarray(bounds)

    def task_state(self):
        scene = self.scene
        body = scene.mujoco.mj_name2id(scene.model, scene.mujoco.mjtObj.mjOBJ_BODY, 'robot_L_tcp')
        dofs = [scene.model.jnt_dofadr[scene.robot.ids[self.geometry.joint_names[idx]]] for idx in self.active]
        linear = np.zeros((3, scene.model.nv));angular = np.zeros_like(linear)
        scene.mujoco.mj_jac(scene.model, scene.data, linear, angular, scene.data.xpos[body], body)
        jacobian = np.vstack((linear[:, dofs], angular[:, dofs]))
        frame = np.eye(4);frame[:3,3] = scene.data.xpos[body];frame[:3,:3] = scene.data.xmat[body].reshape(3,3)
        return frame,jacobian,dofs

    def task_weights(self, state=None):
        scene = self.scene
        _,jacobian,dofs = self.task_state() if state is None else state
        mass = np.zeros((scene.model.nv, scene.model.nv))
        scene.mujoco.mj_fullM(scene.model, mass, scene.data.qM)
        inverse = np.linalg.inv(mass[np.ix_(dofs, dofs)])
        projected = inverse@jacobian.T@np.linalg.pinv(jacobian@inverse@jacobian.T, rcond=1e-5)
        null = np.eye(len(dofs))-projected@jacobian
        return null.T@null+jacobian.T@jacobian+np.eye(len(dofs))*.001

    def propose_velocity(self, positions, velocity, goal, field, stamp_sec):
        """制御対象軸の速度解 [rad/s]。仮の角度目標への変換なし。"""
        self.diagnostics['external_slack_mps'] = 0.
        matrix, values = self.constraints(positions, velocity, field, stamp_sec)
        nominal = np.clip(3.*angle_delta(goal, positions, self.is_periodic)[self.active], -2., 2.)
        task_state = self.task_state() if self.task is not None else None
        if self.task is not None:
            nominal,_ = self.task.control(positions,task_state[0],task_state[1])
        weights = (self.task_weights(task_state) if task_state is not None else self.task_weights()) if self.method == 'oscbf' else np.eye(len(self.active))
        safe_velocity = self.solve_active(positions,nominal,matrix,values,weights)
        can_solve = safe_velocity is not None and np.isfinite(safe_velocity).all()
        failure_reason = 'solver' if not can_solve else ''
        self.diagnostics['strict_solver_status'] = self.diagnostics.get('solver_status','')
        has_relaxed_solution = False
        if can_solve:
            self.diagnostics['min_residual'] = float(np.min(matrix@safe_velocity-values))
            can_solve = self.diagnostics['min_residual']>=-1e-3
            if not can_solve:
                failure_reason = 'residual'
        if not can_solve and str(self.diagnostics.get('solver_status','')).startswith('primal infeasible'):
            safe_velocity = self.solve_recovery(positions,nominal,matrix,values,weights)
            can_solve = safe_velocity is not None
            has_relaxed_solution = can_solve
        if can_solve and self.internal_checker is not None:
            safe_velocity = self.guard_velocity(positions,velocity,safe_velocity,matrix,values)
            can_solve = safe_velocity is not None
            if not can_solve:
                failure_reason = 'internal_guard'
        if not can_solve:
            self.diagnostics['last_failure_reason'] = failure_reason
            self.diagnostics['num_no_solution_by_reason'][failure_reason] += 1
            self.phase = 'qp_no_solution'
            self.num_no_solution += 1
            self.num_consecutive_no_solution += 1
            self.max_num_consecutive_no_solution = max(self.max_num_consecutive_no_solution,self.num_consecutive_no_solution)
            if self.first_no_solution_sec is None:
                self.first_no_solution_sec = float(stamp_sec)
            # 制限付き速度指令による減速保持。安全な代替解との混同禁止。
            return joint_velocity_command(positions[self.active],velocity[self.active],
                np.zeros(len(self.active)),self.geometry.limits[self.active])
        self.num_recoveries += int(self.num_consecutive_no_solution>0)
        self.num_consecutive_no_solution = 0
        self.num_relaxed_solutions += int(has_relaxed_solution)
        self.phase = ('constraint_recovery' if has_relaxed_solution else
                      'task' if np.max(np.abs(safe_velocity-nominal)) < .01 else 'avoiding')
        return safe_velocity.copy()

    def guard_velocity(self,positions,velocity,desired,matrix,values):
        """線形化誤差時の有限バックトラック。全制約と同じ内部離隔の再確認。"""
        next_pose = positions.copy();next_pose[self.active] += desired*.05
        if self.internal_checker.check(next_pose):
            return desired
        brake = joint_velocity_command(positions[self.active],velocity[self.active],np.zeros(len(self.active)),
                                       self.geometry.limits[self.active])
        required = values.copy()
        required[:self.num_external_constraints] -= self.diagnostics['external_slack_mps']
        for fraction in (.5,.25,.125,.0625,0.):
            candidate = brake+fraction*(desired-brake)
            if np.min(matrix@candidate-required)<-1e-3:
                continue
            next_pose[self.active] = positions[self.active]+candidate*.05
            if self.internal_checker.check(next_pose):
                return candidate
        return None

    def solve_recovery(self,positions,nominal,matrix,values,weights):
        """外部CBFの実行不能時だけの最小違反回復。内部形状・関節制限は非緩和。"""
        num = len(self.active)
        augmented = np.column_stack((matrix,np.r_[np.ones(self.num_external_constraints),np.zeros(len(matrix)-self.num_external_constraints)]))
        # 最小違反量は線形計画、タスク目的は元のQP。微小な正則化との重み比の廃止。
        objective = np.r_[np.zeros(num),1.]
        result = linprog(objective,A_ub=-augmented,b_ub=-values,
            bounds=[(None,None)]*num+[(0.,None)],method='highs',options={'time_limit':.02})
        if not result.success or not np.isfinite(result.x).all() or np.min(augmented@result.x-values)<-1e-6:
            return None
        slack = max(0.,float(np.max(values[:self.num_external_constraints]-matrix[:self.num_external_constraints]@result.x[:num],initial=0.)))+1e-5
        relaxed = values.copy();relaxed[:self.num_external_constraints] -= slack
        velocity = self.solve_active(positions,nominal,matrix,relaxed,weights)
        if velocity is None and np.min(matrix@result.x[:num]-relaxed)>=-1e-6:
            # 実行可能点のある同一QPの数値再求解。追加緩和・LP極端点の直接駆動なし。
            velocity = self.solve_numerical(nominal,matrix,relaxed,weights,result.x[:num])
        if velocity is None or not np.isfinite(velocity).all() or np.min(matrix@velocity-relaxed)<-1e-3:
            return None
        self.diagnostics['external_slack_mps'] = slack
        self.diagnostics['min_residual'] = float(np.min(matrix@velocity-values))
        return velocity

    def solve_active(self,positions,nominal,matrix,values,weights):
        weights = (weights+weights.T)/2
        solver = self.osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(np.triu(weights)), q=-weights@nominal,
            A=sparse.csc_matrix(matrix), l=values, u=np.full(len(values),np.inf),
            verbose=False,eps_abs=1e-6,eps_rel=1e-6,max_iter=4000,polishing=False)
        result = solver.solve()
        self.diagnostics['solver_status'] = result.info.status
        if result.info.status_val in (1,2):
            return result.x
        if result.info.status_val != 7:
            return None
        # 反復上限時だけの同一凸QP再求解。制約削除・余裕距離の緩和なし。
        initial = result.x if result.x is not None and np.isfinite(result.x).all() else nominal
        return self.solve_numerical(nominal,matrix,values,weights,initial)

    def solve_numerical(self,nominal,matrix,values,weights,initial):
        """同一目的・全制約の数値再求解。元の単位での残差確認。"""
        scale = np.maximum(np.linalg.norm(matrix,axis=1),1e-12)
        normalized = matrix/scale[:,None]
        required = values/scale
        objective = weights/max(float(np.linalg.norm(weights,ord=2)),1e-12)
        fallback = minimize(lambda x: .5*(x-nominal)@objective@(x-nominal),initial,
            jac=lambda x: objective@(x-nominal),method='SLSQP',
            constraints={'type':'ineq','fun':lambda x: normalized@x-required,'jac':lambda _:normalized},
            options={'maxiter':80,'ftol':1e-10,'disp':False})
        if not fallback.success or not np.isfinite(fallback.x).all() or np.min(matrix@fallback.x-values)<-1e-6:
            return None
        self.diagnostics['solver_status'] = 'solved same qp (slsqp)'
        self.diagnostics['num_numerical_recoveries'] += 1
        return fallback.x
