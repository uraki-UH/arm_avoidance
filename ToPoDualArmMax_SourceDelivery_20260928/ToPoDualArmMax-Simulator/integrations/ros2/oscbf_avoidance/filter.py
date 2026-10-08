"""外接球・箱間距離の局所線形CBFと、OSCBF作業空間目的関数の速度制御への接続。"""
import time
import numpy as np
import jax
import jax.numpy as jnp
from cbfpy import CBF
from oscbf.core.manipulator import Manipulator
from oscbf.core.oscbf_configs import OSCBFVelocityConfig


class MujocoManipulator(Manipulator):
    """公式設定クラス用の関節情報。運動学・慣性行列はMuJoCoで評価。"""
    def __init__(self, velocities):
        self.num_joints = len(velocities)
        self.joint_max_velocities = tuple(velocities)


@jax.tree_util.register_static
class CollisionConfig(OSCBFVelocityConfig):
    def __init__(self, velocities, lower, upper, num_constraints):
        self.lower, self.upper = jnp.asarray(lower), jnp.asarray(upper)
        num = len(velocities)
        super().__init__(MujocoManipulator(velocities), init_args=(np.zeros(num), np.zeros((num_constraints, num)), np.ones(num_constraints), np.eye(num)))

    def h_1(self, q, origin, gradients, margins, weights):
        return jnp.concatenate((margins + gradients @ (q-origin), q-self.lower, self.upper-q))

    def alpha(self, h):
        return 3.0*h

    def P(self, q, desired, origin, gradients, margins, weights):
        return weights

    def q(self, q, desired, origin, gradients, margins, weights):
        return -weights @ desired


class CollisionFilter:
    def __init__(self, scene, settings):
        self.scene, self.model, self.data, self.mj = scene, scene.model, scene.data, scene.mujoco
        self.clearance = float(settings.get('clearance', .06))
        if not np.isfinite(self.clearance) or not .005 <= self.clearance <= .3:
            raise ValueError('OSCBFの余裕距離は0.005～0.3 mです')
        self.names = list(scene.robot.actuator_ids)
        if not self.names:
            raise ValueError('OSCBFの制御対象関節がありません')
        ids = [scene.robot.ids[name] for name in self.names]
        self.dofs = self.model.jnt_dofadr[ids]
        self.positions = self.model.jnt_qposadr[ids]
        self.velocities = np.array([float(scene.robot.joints[name].find('limit').get('velocity')) for name in self.names])
        max_velocity = float(settings.get('max_joint_velocity', .3))
        if not np.isfinite(max_velocity) or not 0 < max_velocity <= 1.:
            raise ValueError('OSCBFの速度上限は0より大きく1以下です')
        self.velocities = np.minimum(self.velocities, max_velocity)
        self.num_constraints = settings.get('max_constraints', 64)
        if type(self.num_constraints) is not int or not 1 <= self.num_constraints <= 128:
            raise ValueError('OSCBFの形状対上限は1～128です')
        self.config = CollisionConfig(self.velocities, self.model.jnt_range[ids, 0], self.model.jnt_range[ids, 1], self.num_constraints)
        self.config.relax_cbf = False
        self.config.solver_tol = 1e-6
        self.cbf = CBF.from_config(self.config)
        self.solve = jax.jit(self.cbf.safety_filter)
        self.robot_geoms, self.other_geoms = [], []
        for idx in range(self.model.ngeom):
            name = self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[idx])) or ''
            (self.robot_geoms if name.startswith('robot_') else self.other_geoms).append(idx)
        self.safe_velocity = np.zeros(len(ids))
        self.status = dict(mode='oscbf', min_dist=None, num_constraints=0, solve_ms=0.)
        self.update()

    def jacobian(self, point, body):
        linear = np.zeros((3, self.model.nv)); angular = np.zeros_like(linear)
        self.mj.mj_jac(self.model, self.data, linear, angular, point, body)
        return linear[:, self.dofs], angular[:, self.dofs], linear

    def update(self):
        start = time.perf_counter()
        q = self.data.qpos[self.positions].copy()
        rows = []
        for a in self.robot_geoms:
            for b in self.other_geoms:
                center = self.data.geom_xpos[a]
                radius = self.model.geom_rbound[a]
                rotation = self.data.geom_xmat[b].reshape(3, 3)
                local = rotation.T @ (center-self.data.geom_xpos[b])
                if self.model.geom_type[b] == self.mj.mjtGeom.mjGEOM_PLANE:
                    direction = rotation[:, 2]
                    dist = local[2]-radius
                    nearest = center-direction*local[2]
                elif self.model.geom_type[b] == self.mj.mjtGeom.mjGEOM_BOX:
                    closest = np.clip(local, -self.model.geom_size[b], self.model.geom_size[b])
                    delta = local-closest
                    norm = np.linalg.norm(delta)
                    if norm > 1e-9:
                        direction = rotation @ (delta/norm)
                        dist = norm-radius
                    else:
                        gap = self.model.geom_size[b]-np.abs(local)
                        axis = int(np.argmin(gap)); normal = np.zeros(3)
                        normal[axis] = 1. if local[axis] >= 0 else -1.
                        direction = rotation @ normal
                        closest[axis] = normal[axis]*self.model.geom_size[b, axis]
                        dist = -gap[axis]-radius
                    nearest = self.data.geom_xpos[b]+rotation@closest
                else:
                    raise ValueError('OSCBFの環境形状は箱と平面のみ対応')
                if dist >= self.clearance+.25:
                    continue
                ja, _, _ = self.jacobian(center, int(self.model.geom_bodyid[a]))
                if np.linalg.norm(ja) < 1e-9:
                    continue
                _, _, jb = self.jacobian(nearest, int(self.model.geom_bodyid[b]))
                # 障害物速度の一次予測。姿勢指定物体の瞬間移動は保証対象外
                velocity = jb @ self.data.qvel
                margin = dist-self.clearance-float(direction @ velocity)/3.
                rows.append((dist, direction @ ja, margin))
        rows.sort(key=lambda row: row[0])
        if len(rows) > self.num_constraints:
            raise RuntimeError('OSCBF: 近接形状対が設定上限を超えたため物理計算を停止')
        gradients = np.zeros((self.num_constraints, len(q))); margins = np.ones(self.num_constraints)
        for idx, (_, gradient, margin) in enumerate(rows):
            gradients[idx], margins[idx] = gradient, margin
        # 両手先の作業空間を優先するOSCBF目的関数。特異姿勢は正則化
        task = []
        for side in ('L', 'R'):
            body = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, 'robot_'+side+'_tcp')
            if body >= 0:
                linear, angular, _ = self.jacobian(self.data.xpos[body], body)
                task.extend((linear, angular))
        weights = np.eye(len(q))
        if task:
            jacobian = np.vstack(task)
            mass = np.zeros((self.model.nv, self.model.nv))
            self.mj.mj_fullM(self.model, mass, self.data.qM)
            inverse = np.linalg.inv(mass[np.ix_(self.dofs, self.dofs)])
            projected = inverse @ jacobian.T @ np.linalg.pinv(jacobian @ inverse @ jacobian.T, rcond=1e-5)
            null = np.eye(len(q))-projected@jacobian
            weights = null.T@null+jacobian.T@jacobian+np.eye(len(q))*1e-3
        desired = np.clip(2.*(np.array([self.scene.robot.targets[name] for name in self.names])-q), -self.velocities, self.velocities)
        result = np.asarray(self.solve(q, desired, q, gradients, margins, weights))
        # ソルバの制約違反時は無フィルタ制御へ戻さずセッション停止
        if not np.all(np.isfinite(result)) or np.any(np.abs(result)>self.velocities+1e-4) or np.any(gradients@result+3*margins < -1e-3) or np.any(result+3*(q-np.asarray(self.config.lower)) < -1e-3) or np.any(-result+3*(np.asarray(self.config.upper)-q) < -1e-3):
            raise RuntimeError(f'OSCBF: 解が不正または制約違反のため停止（速度超過={np.max(np.abs(result)-self.velocities):.4g}, CBF残差={np.min(gradients@result+3*margins):.4g}）')
        self.safe_velocity = result
        self.status = dict(mode='oscbf', min_dist=float(rows[0][0]) if rows else None, num_constraints=len(rows), solve_ms=(time.perf_counter()-start)*1000)

    def apply(self):
        # 慣性行列による速度追従。アクチュエータのURDFトルク上限はMuJoCoで適用
        mass = np.zeros((self.model.nv, self.model.nv))
        self.mj.mj_fullM(self.model, mass, self.data.qM)
        acceleration = 40.*(self.safe_velocity-self.data.qvel[self.dofs])
        torques = mass[np.ix_(self.dofs, self.dofs)] @ acceleration + self.data.qfrc_bias[self.dofs]
        for idx, name in enumerate(self.names):
            dof, position = self.dofs[idx], self.positions[idx]
            actuator = self.scene.robot.actuator_ids[name]
            self.data.ctrl[actuator] = self.data.qpos[position]+(torques[idx]+3.*self.data.qvel[dof])/40.
