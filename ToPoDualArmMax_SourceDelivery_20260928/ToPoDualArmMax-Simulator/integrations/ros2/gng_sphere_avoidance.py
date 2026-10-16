"""球観測に対する有限時間の回避。疎な時系列記憶・候補ロールアウト・現観測ガード。"""
from collections import OrderedDict
import ctypes
import os
from pathlib import Path
import struct
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def unit_vector(value):
    value = np.asarray(value, dtype=float)
    return value/max(float(np.linalg.norm(value)), 1e-9)


def rotation_error(target, current):
    """世界座標の回転ベクトル誤差。単一姿勢・一括軌道・180度付近への対応。"""
    delta = np.asarray(target)@np.swapaxes(current,-1,-2)
    return Rotation.from_matrix(delta.reshape(-1,3,3)).as_rotvec().reshape(delta.shape[:-2]+(3,))


class cartesian_task:
    """手先6次元の往復タスク。関節姿勢の復帰は零空間内の低優先度動作。"""
    def __init__(self, geometry, active, poses, eef_link='L_tcp', max_position_error_th=.015,
                 max_orientation_error_rad_th=np.deg2rad(3.)):
        self.geometry,self.active,self.poses = geometry,np.asarray(active),poses
        self.eef_link = eef_link
        self.eef_idx = geometry.link_indices[eef_link]
        self.goal = 'b'
        self.goals = dict(zip(poses,geometry.link_transforms(np.stack(list(poses.values())))[:,self.eef_idx]))
        self.max_position_error_th = float(max_position_error_th)
        self.max_orientation_error_rad_th = float(max_orientation_error_rad_th)
        if not 0 < self.max_position_error_th <= .1 or not 0 < self.max_orientation_error_rad_th <= .5:
            raise ValueError('手先到達許容誤差の範囲外')
        self.position_scale = max(float(np.linalg.norm(self.goals['b'][:3,3]-self.goals['a'][:3,3])),.05)
        self.orientation_scale = max(float(np.linalg.norm(rotation_error(self.goals['b'][:3,:3],self.goals['a'][:3,:3]))),.25)
        # A/Bの全幅コストを従来の平均二乗関節誤差へ整合。回避との重み比の保持。
        self.cost_scale = .5*max(float(np.mean((poses['b'][self.active]-poses['a'][self.active])**2)),1e-6)

    def errors(self, frames, goal=None):
        target = self.goals[self.goal if goal is None else goal]
        return target[:3,3]-frames[...,:3,3],rotation_error(target[:3,:3],frames[...,:3,:3])

    def pose_cost(self, frames):
        position,orientation = self.errors(frames)
        return self.cost_scale*(np.sum(position**2,axis=-1)/self.position_scale**2+np.sum(orientation**2,axis=-1)/self.orientation_scale**2)

    def jacobian(self, probes, step=.002):
        linear = (probes[1:,:3,3]-probes[0,:3,3])/step
        angular = rotation_error(probes[1:,:3,:3],probes[0,:3,:3])/step
        return np.concatenate((linear,angular),axis=-1).T

    def control(self, positions, frame, jacobian):
        position,orientation = self.errors(frame)
        linear = 3.*position;linear *= min(1.,1./max(float(np.linalg.norm(linear)),1e-9))
        angular = 3.*orientation;angular *= min(1.,2./max(float(np.linalg.norm(angular)),1e-9))
        scale = np.array([1.,1.,1.,.2,.2,.2])
        scaled = scale[:,None]*jacobian
        left,values,right = np.linalg.svd(scaled,full_matrices=True)
        # 減衰逆行列とSVD零空間への姿勢復帰の分離。
        inverse = (right[:len(values)].T*(values/(values**2+.02**2)))@left[:,:len(values)].T
        rank = int(np.count_nonzero(values>max(float(values[0]),1e-9)*1e-3))
        basis = right[rank:]
        recovery = .2*(self.poses[self.goal][self.active]-positions[self.active])
        nominal = inverse@(scale*np.r_[linear,angular])+basis.T@(basis@recovery)
        nominal *= min(1.,2./max(float(np.max(np.abs(nominal))),1e-9))
        return nominal,basis[:2]

    def measure(self, frame, linear_velocity, angular_velocity, goal=None):
        position,orientation = self.errors(frame,goal)
        position_error = float(np.linalg.norm(position));orientation_error = float(np.linalg.norm(orientation))
        can_arrive = position_error<self.max_position_error_th and orientation_error<self.max_orientation_error_rad_th
        can_arrive &= np.linalg.norm(linear_velocity)<.12 and np.linalg.norm(angular_velocity)<.3
        return position_error,orientation_error,bool(can_arrive)

    def metadata(self, mode):
        goals = {name:dict(position=frame[:3,3].tolist(),quaternion=Rotation.from_matrix(frame[:3,:3]).as_quat().tolist()) for name,frame in self.goals.items()}
        return dict(mode=mode,eef_link=self.eef_link,frame='world',goals=goals,reference_kind='goal_line',
                    reference_path=[goals[name]['position'] for name in ('a','b')],
                    max_position_error_th=self.max_position_error_th,max_orientation_error_rad_th=self.max_orientation_error_rad_th)


class sphere_field:
    """既知の観測球に対する符号付き表面距離。ボクセル化を要しない共通観測。"""
    n = 1
    def __init__(self, center, radius):
        self.center, self.radius = np.asarray(center, dtype=float), float(radius)
        if self.center.shape != (3,) or not np.isfinite(self.center).all() or not np.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('観測球の中心・半径の不正')
        self.data = self.center[None]
    def query(self, points, **kwargs):
        gap = np.linalg.norm(np.asarray(points)-self.center, axis=-1)-self.radius
        return gap, np.zeros_like(gap, dtype=int)


class motion_sequence_predictor:
    """相対方向の疎な二時刻文脈と追従係数の同定。完全なHTMとは異なる小容量の時系列記憶。"""
    def __init__(self, enable_learning=True):
        self.enable_learning = enable_learning
        self.memory = OrderedDict()
        self.numerator, self.denominator, self.num_learning = 0., 1e-6, 0
        self.reset_observation()

    def reset_observation(self):
        self.center = self.velocity = self.eef = self.last_sec = self.previous_symbol = None
        self.last_error_m = self.error_m = 0.
        self.num_updates = 0

    @staticmethod
    def basis(center, eef, velocity):
        radial = unit_vector(eef-center)
        tangent = velocity-radial*np.dot(velocity, radial)
        if np.linalg.norm(tangent) < 1e-7:
            tangent = np.cross(radial, np.eye(3)[np.argmin(np.abs(radial))])
        tangent = unit_vector(tangent)
        return np.stack((radial, tangent, np.cross(radial, tangent)))

    @staticmethod
    def symbol(center, eef, velocity):
        bearing = unit_vector(eef-center)
        speed = np.linalg.norm(velocity)
        closing = float(np.dot(unit_vector(velocity), bearing)) if speed > 1e-6 else 0.
        return int(np.clip(np.round(closing*2), -2, 2)), int(np.clip(speed/.12, 0, 4))

    def context(self):
        symbol = self.symbol(self.center, self.eef, self.velocity)
        return (*self.previous_symbol, *symbol) if self.previous_symbol is not None else (*symbol, *symbol)

    @property
    def gain(self):
        return float(np.clip(self.numerator/self.denominator, 0., .8)) if self.num_learning >= 12 else 0.

    def update(self, center, eef, stamp_sec):
        center, eef = np.asarray(center, dtype=float), np.asarray(eef, dtype=float)
        if center.shape != (3,) or eef.shape != (3,) or not np.isfinite([*center, *eef, stamp_sec]).all():
            raise ValueError('球観測の形式不正')
        if self.last_sec is not None and stamp_sec == self.last_sec:
            return
        if self.last_sec is not None and not 0 < stamp_sec-self.last_sec <= .3:
            self.reset_observation()
        if self.last_sec is None:
            self.center, self.eef, self.last_sec = center.copy(), eef.copy(), float(stamp_sec)
            self.velocity = np.zeros(3)
            self.num_updates = 1
            return
        period = stamp_sec-self.last_sec
        velocity = .65*(center-self.center)/period+.35*self.velocity
        basis = self.basis(self.center, self.eef, self.velocity)
        speed = max(np.linalg.norm(velocity), np.linalg.norm(self.velocity))
        direction = speed*unit_vector(self.eef-self.center)-self.velocity
        change = velocity-self.velocity
        expected = self.gain*direction
        self.last_error_m = float(np.linalg.norm(change-expected)*period)
        self.error_m = .9*self.error_m+.1*self.last_error_m
        key = self.context()
        if self.enable_learning and self.num_updates > 2:
            residual = basis@(change-expected)
            count, mean = self.memory.pop(key, (0., np.zeros(3)))
            weight = min(count, 20.)
            self.memory[key] = (weight+1., (weight*mean+residual)/(weight+1.))
            while len(self.memory) > 256:
                self.memory.popitem(last=False)
            self.numerator = .98*self.numerator+float(np.dot(direction, change))
            self.denominator = .98*self.denominator+float(np.dot(direction, direction))
            self.num_learning += 1
        self.previous_symbol = self.symbol(self.center, self.eef, self.velocity)
        self.center, self.eef, self.velocity, self.last_sec = center.copy(), eef.copy(), velocity, float(stamp_sec)
        self.num_updates += 1

    def forecast(self, eef_poses, times, mode):
        eef_poses, times = np.asarray(eef_poses), np.asarray(times)
        if mode not in ('reactive','constant_velocity','sequence') or times.ndim != 1 or len(times) < 1 or np.any(np.diff(np.r_[0.,times]) <= 0) or not np.isfinite(times).all():
            raise ValueError('予測方式・時刻列の不正')
        if eef_poses.ndim != 3 or eef_poses.shape[1:] != (len(times),3) or not np.isfinite(eef_poses).all():
            raise ValueError('予測候補姿勢の不正')
        num_candidates = len(eef_poses)
        if self.center is None:
            raise ValueError('球観測なし')
        center = np.repeat(self.center[None], num_candidates, axis=0)
        velocity = np.repeat(self.velocity[None], num_candidates, axis=0)
        output = np.empty((num_candidates, len(times), 3))
        if mode == 'reactive':
            output[:] = self.center
            return output
        confidence = min(1., self.num_learning/80.)*np.exp(-self.error_m/.02) if mode == 'sequence' else 0.
        count, residual = self.memory.get(self.context(), (0., np.zeros(3)))
        residual = residual*min(1., count/6.)
        last_sec = 0.
        speed = float(np.linalg.norm(self.velocity))
        for idx, now in enumerate(times):
            period = now-last_sec
            if confidence:
                delta = eef_poses[:, idx]-center
                direction = delta/np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-8)
                velocity += confidence*min(1., self.gain*period/.05)*(speed*direction-velocity)
                correction = self.basis(self.center, self.eef, self.velocity).T@residual
                velocity += confidence*correction*(period/.05)*np.exp(-now/.2)
                norm = np.linalg.norm(velocity, axis=1, keepdims=True)
                velocity *= np.minimum(1., max(speed*1.15, .01)/np.maximum(norm, 1e-8))
            center = center+velocity*period
            output[:, idx] = center
            last_sec = now
        return output

    def export(self):
        return dict(numerator=self.numerator, denominator=self.denominator, num_learning=self.num_learning,
                    memory=[[list(key), count, value.tolist()] for key, (count, value) in self.memory.items()])

    def restore(self, values):
        numerator, denominator = float(values['numerator']), float(values['denominator'])
        num_learning = int(values['num_learning'])
        memory = OrderedDict()
        if not np.isfinite([numerator,denominator]).all() or denominator <= 0 or num_learning < 0:
            raise ValueError('時系列記憶の係数不正')
        if len(values['memory']) > 256:
            raise ValueError('時系列記憶の容量超過')
        for key,count,value in values['memory']:
            vector = np.asarray(value,dtype=float)
            if len(key) != 4 or tuple(key) in memory or vector.shape != (3,) or not np.isfinite(vector).all() or not 0 < count <= 21:
                raise ValueError('時系列記憶の文脈不正')
            memory[tuple(key)] = (float(count),vector)
        self.numerator, self.denominator, self.num_learning = numerator,denominator,num_learning
        self.memory = memory
        self.reset_observation()


def load_graph(filename):
    """保存GNG v10の角度・接続だけの読込み。学習済みグラフの変更なし。"""
    def read_exact(stream, size):
        value = stream.read(size)
        if len(value) != size:
            raise ValueError('GNGの欠損')
        return value
    def matrix(stream):
        rows, cols = struct.unpack('<qq', read_exact(stream, 16))
        if not 0 <= rows*cols <= 4096 or rows < 0 or cols < 0:
            raise ValueError('GNG配列の不正')
        return np.frombuffer(read_exact(stream, rows*cols*4), dtype='<f4').astype(float)
    with Path(filename).open('rb') as stream:
        version, num_layers, num_nodes = struct.unpack('<Iii', read_exact(stream, 12))
        if version != 10 or not 1 <= num_layers <= 16 or not 0 < num_nodes <= 100000:
            raise ValueError('GNG形式の不正')
        angles = {}
        for _ in range(num_nodes):
            idx, _, _ = struct.unpack('<iff', read_exact(stream, 12))
            if idx in angles:
                raise ValueError('GNGノードの重複')
            angles[idx] = matrix(stream)
            matrix(stream)
            num_coords, = struct.unpack('<i', read_exact(stream, 4))
            if not 0 <= num_coords <= num_layers:
                raise ValueError('GNG座標層の不正')
            for _ in range(num_coords):
                matrix(stream)
            read_exact(stream, 5)
            matrix(stream)
            read_exact(stream, 30)
        adjacency = {idx:[] for idx in angles}
        for layer in range(num_layers+1):
            num_edges, = struct.unpack('<i', read_exact(stream, 4))
            if not 0 <= num_edges <= 10000000:
                raise ValueError('GNG辺数の不正')
            for _ in range(num_edges):
                first, second, _, is_active = struct.unpack('<iii?', read_exact(stream, 13))
                if is_active and (first not in angles or second not in angles):
                    raise ValueError('GNG接続先の不正')
                if layer == 0 and is_active:
                    adjacency[first].append(second)
                    adjacency[second].append(first)
        if stream.read(1):
            raise ValueError('GNG末尾の不一致')
    ids = tuple(angles)
    return dict(angles=angles, adjacency=adjacency, angle_node_ids=ids,
                angle_tree=cKDTree(np.asarray(list(angles.values()))), labels=dict.fromkeys(ids, 1))


class representative_index:
    """代表姿勢の連続球包絡索引。AABB粗照合後だけのC++詳細照合。"""
    def __init__(self, geometry, active, positions, graph):
        paths = [Path(prefix)/'lib/gng_vlut_system/libgng_dynamic_kernel.so'
                 for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if prefix]
        filename = next((path for path in paths if path.is_file()), None)
        if filename is None:
            raise RuntimeError('gng_dynamic_kernelのビルド・installが必要')
        self.library = ctypes.CDLL(str(filename))
        self.kernel_path = filename
        if self.library.gng_dynamic_kernel_version() != 1:
            raise RuntimeError('動的照会カーネルのABI不一致')
        floats = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        flags = np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')
        self.library.gng_dynamic_node_gaps.argtypes = [ctypes.c_size_t,ctypes.c_size_t]+[floats]*5+[ctypes.c_double]*2+[floats,flags]
        self.library.gng_dynamic_node_gaps.restype = ctypes.c_int
        ids = graph['angle_node_ids']
        poses = np.repeat(positions[None], len(ids), axis=0)
        poses[:, active] = [graph['angles'][idx] for idx in ids]
        self.centers = np.ascontiguousarray(geometry.centers(poses)[:, geometry.is_arm])
        self.radii = np.ascontiguousarray(geometry.radii[geometry.is_arm])
        self.lower = np.ascontiguousarray(np.min(self.centers-self.radii[None,:,None], axis=1))
        self.upper = np.ascontiguousarray(np.max(self.centers+self.radii[None,:,None], axis=1))
        self.ids, self.graph = ids, graph
        self.gaps, self.labels = np.empty(len(ids)), np.empty(len(ids),dtype=np.uint8)
        self.graph['arm_centers'] = self.centers
        self.graph['node_positions'] = {idx:pos for pos,idx in enumerate(ids)}
        integers = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
        self.library.gng_dynamic_internal_clear.argtypes = [ctypes.c_size_t]*3+[integers,floats,floats,ctypes.c_double,flags]
        self.library.gng_dynamic_internal_clear.restype = ctypes.c_int

    def internal_clear(self, geometry, centers, margin=.005):
        shape = centers.shape[:-2]
        values = np.ascontiguousarray(centers.reshape(-1, len(geometry.radii),3))
        can_pass = np.empty(len(values),dtype=np.uint8)
        code = self.library.gng_dynamic_internal_clear(len(values),len(geometry.radii),len(geometry.self_pairs),
            np.ascontiguousarray(geometry.self_pairs,dtype=np.int64),values,geometry.radii,margin,can_pass)
        if code:
            raise ValueError('自己干渉照会の入力不正')
        is_clear = can_pass.reshape(shape).astype(bool)
        if geometry.environment_shapes:
            is_clear &= np.all(geometry.environment_clearances(centers)>=margin,axis=-1)
        return is_clear

    def update(self, center, radius, margin=.10, enable_graph_labels=True):
        code = self.library.gng_dynamic_node_gaps(len(self.ids), len(self.radii), self.centers, self.radii,
            self.lower, self.upper, np.ascontiguousarray(center,dtype=float), radius, margin, self.gaps, self.labels)
        if code:
            raise ValueError('動的代表姿勢照会の入力不正')
        if enable_graph_labels:
            self.graph['labels'] = dict(zip(self.ids,map(int,self.labels)))

    def update_field(self, field, margin=.10):
        if not hasattr(field, 'covering_spheres'):
            self.update(field.center, field.radius, margin)
            return
        # カプセル被覆球のC++照合。索引は保守的な候補除外、採用検査は元カプセル。
        gaps = np.full(len(self.ids), np.inf)
        has_collision = np.zeros(len(self.ids), dtype=bool)
        has_danger = np.zeros(len(self.ids), dtype=bool)
        for center, radius in field.covering_spheres():
            self.update(center, radius, margin, enable_graph_labels=False)
            np.minimum(gaps, self.gaps, out=gaps)
            has_collision |= self.labels == 2
            has_danger |= self.labels == 3
        self.gaps[:] = gaps
        self.labels[:] = np.where(has_collision, 2, np.where(has_danger, 3, 1))
        self.graph['labels'] = dict(zip(self.ids, map(int, self.labels)))


class sphere_rollout_policy:
    """速度・加速度制約付き候補の有限ロールアウト。予測は選好、現観測検査は共通の採用条件。"""
    def __init__(self, geometry, active, eef_link, graph, mode='constant_velocity', candidate_metric='joint',
                 horizon_sec=.4, max_velocity=2., max_acceleration=8., margin=.08, memory=None, node_index=None,
                 task_weight=.12, target_offset=None, task=None):
        if mode not in ('reactive','constant_velocity','sequence') or candidate_metric not in ('joint','occupancy'):
            raise ValueError('候補・予測方式の不正')
        if not .1 <= horizon_sec <= 1. or not 0 < max_velocity <= 10. or not 0 < max_acceleration <= 100. or not 0 <= margin <= .5:
            raise ValueError('有限ロールアウトの制約不正')
        if not 0 <= task_weight <= 10.:
            raise ValueError('タスク選好係数の不正')
        self.geometry, self.active, self.graph = geometry, np.asarray(active), graph
        self.angle_values = None if graph is None else np.asarray([graph['angles'][idx] for idx in graph['angle_node_ids']])
        self.eef_idx = geometry.link_indices[eef_link]
        self.target_offset = np.zeros(3) if target_offset is None else np.asarray(target_offset)
        self.is_periodic = np.asarray(getattr(geometry,'is_periodic',np.zeros(len(geometry.joint_names),dtype=bool)))
        self.mode, self.candidate_metric = mode, candidate_metric
        self.horizon_sec, self.max_velocity, self.max_acceleration, self.margin = horizon_sec,max_velocity,max_acceleration,margin
        self.task_weight = task_weight
        self.task = task
        self.predictor = memory or motion_sequence_predictor()
        self.node_index = node_index
        self.previous_velocity = np.zeros(len(active))
        self.num_proposals = self.num_candidates = self.num_guard_rejections = self.num_node_rejections = 0
        self.samples_ms = []
        self.phase = 'waiting'

    def kinematics(self, poses, enable_task_frames=False):
        transforms = self.geometry.link_transforms(poses)
        selected = transforms[...,self.geometry.sphere_links,:,:]
        centers = np.einsum('...nij,nj->...ni',selected[...,:3,:3],self.geometry.sphere_points)+selected[...,:3,3]
        target = transforms[...,self.eef_idx,:3,3]+np.einsum('...ij,j->...i',transforms[...,self.eef_idx,:3,:3],self.target_offset)
        if enable_task_frames:
            return centers,target,transforms[...,self.task.eef_idx,:,:]
        return centers,target

    def angle_delta(self, goal, positions):
        delta = np.asarray(goal)-np.asarray(positions)
        return np.where(self.is_periodic[self.active],(delta+np.pi)%(2*np.pi)-np.pi,delta)

    def proposals(self, positions, velocity, goal, field):
        num_active = len(self.active)
        nominal = np.clip(3.*self.angle_delta(goal[self.active],positions[self.active]), -self.max_velocity, self.max_velocity)
        # 有限差分姿勢の一括FK。反復ボクセル登録なし。
        probes = np.repeat(positions[None], num_active+1, axis=0)
        probes[1:, self.active] += np.eye(num_active)*.002
        result = self.kinematics(probes,enable_task_frames=True) if self.task is not None else self.kinematics(probes)
        centers,eef = result[:2]
        basis = []
        if self.task is not None:
            nominal,basis = self.task.control(positions,result[2][0],self.task.jacobian(result[2]))
        self.nominal_velocity = nominal
        candidates = [nominal, np.zeros(num_active), self.previous_velocity]
        if self.task is not None:
            # 遠い手先目標への低速追従候補。既存の自己干渉・観測ガードの共用。
            candidates.extend(scale*nominal for scale in (.25,.5))
        for idx in range(num_active):
            for sign in (-1.,1.):
                delta = np.zeros(num_active); delta[idx] = sign*self.max_velocity
                candidates.append(delta)
        # 最大2零空間方向の正負候補。手先への移動と内部姿勢変更の同時選択。
        for direction in basis:
            candidates.extend(nominal+sign*.5*self.max_velocity*direction for sign in (-1.,1.))
        gaps = field.query(centers[0])[0]-self.geometry.radii
        selected = int(np.argmin(gaps))
        gradient = (field.query(centers[1:,selected])[0]-field.query(centers[0,selected])[0])/.002
        escape = gradient/max(float(np.max(np.abs(gradient))),1e-8)*self.max_velocity
        candidates.extend((escape, nominal+.6*escape, nominal+escape))
        eef_jacobian = (eef[1:]-eef[0]).T/.002
        inverse = eef_jacobian.T@np.linalg.inv(eef_jacobian@eef_jacobian.T+.002*np.eye(3))
        bearing = unit_vector(eef[0]-field.center)
        for axis in np.eye(3):
            tangent = np.cross(bearing,axis)
            for sign in (-1.,1.):
                cartesian = .22*(bearing+sign*unit_vector(tangent))
                candidates.append(inverse@cartesian+.2*nominal)
        if self.graph is not None:
            if np.any(self.is_periodic[self.active]):
                squared = np.sum(self.angle_delta(self.angle_values,positions[self.active])**2,axis=1)
                num = min(32,len(squared))
                nearest = np.argpartition(squared,num-1)[:num]
                nearest = nearest[np.argsort(squared[nearest])]
            else:
                _, nearest = self.graph['angle_tree'].query(positions[self.active], k=min(32,len(self.graph['angles'])))
            ids = [self.graph['angle_node_ids'][int(idx)] for idx in np.atleast_1d(nearest)]
            ids = [idx for idx in ids if self.graph['labels'].get(idx)==1]
            self.num_node_rejections += len(np.atleast_1d(nearest))-len(ids)
            targets = np.asarray([self.graph['angles'][idx] for idx in ids])
            if len(targets):
                if self.candidate_metric == 'occupancy':
                    if 'arm_centers' in self.graph:
                        lookup = self.graph['node_positions']
                        shapes = self.graph['arm_centers'][[lookup[idx] for idx in ids]]
                    else:
                        poses = np.repeat(positions[None],len(targets),axis=0); poses[:,self.active] = targets
                        shapes = self.geometry.centers(poses)[:,self.geometry.is_arm]
                    change = np.sum(np.linalg.norm(shapes-centers[0,self.geometry.is_arm],axis=-1)
                                    *self.geometry.radii[self.geometry.is_arm]**2,axis=1)
                    targets = targets[np.argsort(change)[:8]]
                else:
                    targets = targets[:8]
                for target in targets:
                    delta = self.angle_delta(target,positions[self.active])
                    candidates.append(delta/max(float(np.max(np.abs(delta))),1e-8)*self.max_velocity)
        values = np.asarray(candidates)
        values *= np.minimum(1., self.max_velocity/np.maximum(np.max(np.abs(values),axis=1,keepdims=True),1e-8))
        return np.unique(np.round(values,7),axis=0)

    def propose(self, positions, velocity, goal, field, stamp_sec):
        begin = time.perf_counter()
        positions,velocity,goal = map(np.asarray,(positions,velocity,goal))
        eef = self.geometry.link_transforms(positions)[self.eef_idx,:3,3]
        self.predictor.update(field.center,getattr(field,'eef_position',eef),stamp_sec)
        candidates = self.proposals(positions,velocity,goal,field)
        times = np.linspace(.05,self.horizon_sec,max(2,int(round(self.horizon_sec/.05))))
        poses = np.repeat(positions[None],len(candidates),axis=0)
        speeds = np.repeat(velocity[self.active][None],len(candidates),axis=0)
        trajectory = []
        for period in np.diff(np.r_[0.,times]):
            # 可動域端での制動距離と追従誤差用の角度余裕 [rad]。
            lower = np.maximum(poses[:,self.active]-self.geometry.limits[self.active,0]-.03,0.)
            upper = np.maximum(self.geometry.limits[self.active,1]-poses[:,self.active]-.03,0.)
            desired = np.clip(candidates,-np.sqrt(2*self.max_acceleration*lower),np.sqrt(2*self.max_acceleration*upper))
            speeds += np.clip(desired-speeds,-self.max_acceleration*period,self.max_acceleration*period)
            poses[:,self.active] += speeds*period
            poses = np.clip(poses,self.geometry.limits[:,0],self.geometry.limits[:,1])
            trajectory.append(poses.copy())
        trajectory = np.stack(trajectory,axis=1)
        result = self.kinematics(trajectory,enable_task_frames=True) if self.task is not None else self.kinematics(trajectory)
        shape,eef_poses = result[:2]
        if hasattr(field,'trajectory_gaps'):
            gaps = (field.query(shape)[0] if self.mode=='reactive' else field.trajectory_gaps(shape,times))-self.geometry.radii
        else:
            obstacle = self.predictor.forecast(eef_poses,times,self.mode)
            gaps = np.linalg.norm(shape-obstacle[:,:,None,:],axis=-1)-self.geometry.radii-field.radius
        min_gaps = np.min(gaps,axis=-1)
        deficits = np.maximum(0.,self.margin-min_gaps)
        task_error = np.mean(self.angle_delta(goal[self.active],trajectory[:,-1,self.active])**2,axis=1)
        posture_cost = np.zeros(len(candidates))
        if self.task is not None:
            posture_cost = .0002*task_error
            task_error = self.task.pose_cost(result[2][:,-1])
        smooth = np.mean((candidates-self.previous_velocity)**2,axis=1)
        costs = 150.*np.mean(deficits**2,axis=1)+self.task_weight*task_error+posture_cost+.002*smooth-.03*np.minimum(np.min(min_gaps,axis=1),.2)
        # 現観測での短区間検査。衝突開始時も障害物の消去なし、離隔の単調回復だけを許容。
        current_centers = self.geometry.centers(positions)
        current_gap = float(np.min(field.query(current_centers)[0]-self.geometry.radii))
        min_gap = min(.01,current_gap-.0002)
        target_idx = min(1,len(times)-1)
        guard_gaps = field.query(shape[:,:target_idx+1])[0]-self.geometry.radii
        can_pass = np.min(guard_gaps,axis=(1,2)) >= min_gap
        # 将来自己干渉姿勢への誘引も抑制。外部予測の安全保証とは別の内部形状検査。
        is_internal_clear = (self.node_index.internal_clear(self.geometry,shape) if self.node_index is not None
                             else self.geometry.has_internal_clearance(shape,.005))
        can_pass &= np.all(is_internal_clear,axis=1)
        self.num_guard_rejections += int(np.count_nonzero(~can_pass))
        costs[~can_pass] = np.inf
        best = int(np.argmin(costs))
        if not np.isfinite(costs[best]):
            self.phase = 'holding'
            self.previous_velocity[:] = 0.
            target = positions.copy()
        else:
            target = trajectory[best,target_idx].copy()
            self.previous_velocity = candidates[best].copy()
            self.phase = 'task' if np.max(np.abs(candidates[best]-self.nominal_velocity)) < .01 else 'avoiding'
        self.num_candidates += len(candidates)
        self.num_proposals += 1
        self.samples_ms.append((time.perf_counter()-begin)*1000)
        return target

