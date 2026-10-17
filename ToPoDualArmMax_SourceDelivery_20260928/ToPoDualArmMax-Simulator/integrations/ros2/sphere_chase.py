"""指定リンク追跡障害物の閉ループMuJoCo実験。制御への真値・将来軌道の受渡しなし。"""
import argparse
from collections import deque
import copy
from itertools import count, product
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import xml.etree.ElementTree as et

import numpy as np

from gng_inputs import workspace, load_models, load_robot_profiles
from gng_avoidance import independent_policy, check_kinematics
from gng_sphere_avoidance import (load_graph, motion_sequence_predictor, representative_index,
                                  sphere_field, sphere_rollout_policy, cartesian_task, joint_velocity_command,
                                  vlut_index, vlut_graph_policy)
from physics_scene import PhysicsScene, compiled_model
from physics_robot import origin as physics_origin
from dual_arm_avoidance_geometry import robot_geometry, resolve_mesh_path, shape_vertices, origin as geometry_origin
from pursuit_obstacle import pursuit_obstacle, capsule_field, shoulder_anchor_candidates
from pursuit_cbf import pursuit_cbf, angle_delta
from shuttle_trajectory import shuttle_planner


class experiment_resources:
    """静的資源だけの保持。入力更新時の破棄、実験状態の保持なし。"""
    def __init__(self):
        self.key, self.values = None, {}

    def prepare(self, enable_gng=True):
        loader = load_models if enable_gng else load_robot_profiles
        params,models = loader(workspace/'gng_vlut_system/config/topo_dual_arm_max_long.yaml')
        urdf = Path(params['urdf_path'])
        files = {urdf, workspace/'benchmarks/shuttle_bag/task.json', Path(__file__).resolve().parents[2]/'app/source.urdf'}
        files.update(resolve_mesh_path(mesh.get('filename'),urdf.parent) for mesh in et.parse(urdf).getroot().findall('.//mesh'))
        directory = Path(params['gng']['data_directory'])/params['gng']['experiment_id']/'left_arm'
        if enable_gng:
            files.update(directory/models['left_arm'][name] for name in ('gng_file','vlut_file'))
        files.update(Path(prefix)/'lib/gng_vlut_system/libgng_dynamic_kernel.so'
                     for prefix in os.environ.get('AMENT_PREFIX_PATH','').split(os.pathsep)
                     if prefix and (Path(prefix)/'lib/gng_vlut_system/libgng_dynamic_kernel.so').is_file())
        key = (json.dumps((params,models),sort_keys=True),
               tuple((str(path),path.stat().st_mtime_ns,path.stat().st_size) for path in sorted(files)))
        if self.key != key:
            self.close()
            self.key = key
        return params,models

    def get(self, name, factory):
        if name not in self.values:
            self.values[name] = factory()
        return self.values[name]

    def close(self):
        for value in self.values.values():
            if hasattr(value,'close'):
                value.close()
        self.values.clear()

    def __del__(self):
        self.close()


def robot_assets(params, models):
    """再実行で共用する機体形状と内部離隔形状。動的状態を持たない初期形状。"""
    check_kinematics(params['urdf_path'],'long')
    geometry = robot_geometry(params['urdf_path'],environment_shapes=(
        ('plane',np.array([0.,0.,-.14]),np.eye(3),np.zeros(3)),
        ('box',np.array([0.,0.,-.07]),np.eye(3),np.array([.085,.085,.069]))))
    arm = independent_policy(geometry,{'left_arm':models['left_arm']}).policies['left_arm']
    geometry.is_arm = arm.geometry.is_arm.copy()
    geometry.active_joint_names = tuple(arm.arm_names)
    return geometry,internal_clearance_checker(geometry,params['urdf_path'])


class internal_clearance_checker:
    """計画・実測共通の内部離隔。球による絞込み後だけの凸形状照合。"""

    def __init__(self, geometry, urdf_path, min_clearance_th=.005):
        import mujoco
        self.geometry, self.mujoco = geometry, mujoco
        self.min_clearance_th = min_clearance_th
        self.coarse_check = None
        self.num_refined_poses = self.num_shape_checks = 0
        self.num_distance_repairs = 0
        self.last_violations = []
        root = et.Element('mujoco', model='internal_clearance')
        et.SubElement(root, 'compiler', angle='radian')
        asset, world = et.SubElement(root, 'asset'), et.SubElement(root, 'worldbody')
        self.link_geoms = {}
        num_links = len(geometry.link_indices)+len(geometry.environment_shapes)
        self.box_centers = np.zeros((num_links,3))
        self.box_halves = np.zeros((num_links,3))
        geom_links = []
        path = Path(urdf_path)
        for link in et.parse(path).getroot().findall('link'):
            link_idx = geometry.link_indices[link.get('name')]
            self.link_geoms[link_idx] = []
            link_vertices = []
            for shape_idx, shape in enumerate(link.findall('collision') or link.findall('visual')):
                vertices = shape_vertices(shape,path.parent,link.get('name'))
                placement = geometry_origin(shape.find('origin'))
                link_vertices.append(vertices@placement[:3,:3].T+placement[:3,3])
                form = shape.find('geometry')
                mesh = form.find('mesh')
                if mesh is not None:
                    mesh_name = f'link_{link_idx}_{shape_idx}'
                    et.SubElement(asset, 'mesh', name=mesh_name,
                        file=str(resolve_mesh_path(mesh.get('filename'), path.parent)), scale=mesh.get('scale', '1 1 1'))
                    attrs = dict(type='mesh', mesh=mesh_name)
                elif form.find('box') is not None:
                    attrs = dict(type='box', size=' '.join(map(str, np.fromstring(form.find('box').get('size'), sep=' ')/2)))
                elif form.find('sphere') is not None:
                    attrs = dict(type='sphere', size=form.find('sphere').get('radius'))
                elif form.find('cylinder') is not None:
                    cylinder = form.find('cylinder')
                    attrs = dict(type='cylinder', size=f"{cylinder.get('radius')} {float(cylinder.get('length'))/2}")
                else:
                    raise ValueError('内部離隔の未対応形状: '+link.get('name'))
                self.link_geoms[link_idx].append(len(geom_links))
                geom_links.append(link_idx)
                et.SubElement(world, 'geom', **attrs, **physics_origin(shape, mujoco), contype='0', conaffinity='0')
            if link_vertices:
                vertices = np.concatenate(link_vertices)
                lower,upper = vertices.min(axis=0),vertices.max(axis=0)
                self.box_centers[link_idx],self.box_halves[link_idx] = (lower+upper)/2,(upper-lower)/2
        self.num_robot_geoms = len(geom_links)
        self.environment_names = {}
        for env_idx, (kind, position, rotation, half_size) in enumerate(geometry.environment_shapes):
            link_idx = len(geometry.link_indices)+env_idx
            quat = np.zeros(4);mujoco.mju_mat2Quat(quat, rotation.reshape(9))
            et.SubElement(world, 'geom', type=kind, pos=' '.join(map(str, position)),
                quat=' '.join(map(str, quat)), size='10 10 .1' if kind=='plane' else ' '.join(map(str, half_size)),
                contype='0', conaffinity='0')
            self.link_geoms[link_idx] = [len(geom_links)]
            geom_links.append(-1)
            self.environment_names[link_idx] = 'environment_'+kind
            self.box_halves[link_idx] = half_size
        self.model = compiled_model(mujoco, root, workspace/'docker/runtime/pursuit-clearance')
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.polytope_vertices = {}
        box_signs = np.array([[x,y,z] for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])
        for idx in range(self.model.ngeom):
            if self.model.geom_type[idx] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_idx = self.model.geom_dataid[idx]
                begin = self.model.mesh_vertadr[mesh_idx]
                self.polytope_vertices[idx] = self.model.mesh_vert[begin:begin+self.model.mesh_vertnum[mesh_idx]]
            elif self.model.geom_type[idx] == mujoco.mjtGeom.mjGEOM_BOX:
                self.polytope_vertices[idx] = box_signs*self.model.geom_size[idx]
        self.geom_links = np.asarray(geom_links[:self.num_robot_geoms])
        self.local_frames = np.repeat(np.eye(4)[None], self.num_robot_geoms, axis=0)
        for idx in range(self.num_robot_geoms):
            self.local_frames[idx,:3,3] = self.model.geom_pos[idx]
            rotation = np.empty(9);mujoco.mju_quat2Mat(rotation, self.model.geom_quat[idx])
            self.local_frames[idx,:3,:3] = rotation.reshape(3,3)
        pairs = geometry.sphere_links[geometry.self_pairs]
        arm_links = geometry.sphere_links[geometry.is_arm]
        env_pairs = [np.column_stack((arm_links, np.full(len(arm_links), len(geometry.link_indices)+idx)))
                     for idx in range(len(geometry.environment_shapes))]
        self.gap_pairs = np.concatenate([pairs, *env_pairs])
        self.link_names = {idx:name for name,idx in geometry.link_indices.items()}
        self.link_names.update(self.environment_names)
        self.can_control_links = np.zeros(num_links,dtype=bool)
        active = {geometry.joint_names.index(name) for name in getattr(geometry,'active_joint_names',geometry.joint_names)}
        for operation in geometry.operations:
            parent,child,_,_,_,idx = operation[:6]
            self.can_control_links[child] = self.can_control_links[parent] or idx in active

    def set_shape_frames(self, transforms):
        """照合専用データの形状配置。既存FKの共用。"""
        frames = transforms[self.geom_links]@self.local_frames
        self.data.geom_xpos[:self.num_robot_geoms] = frames[:,:3,3]
        self.data.geom_xmat[:self.num_robot_geoms] = frames[:,:3,:3].reshape(-1,9)
        self.world_boxes = np.repeat(np.eye(4)[None],len(self.box_centers),axis=0)
        self.world_boxes[:len(transforms)] = transforms
        for idx,(_,position,rotation,_) in enumerate(self.geometry.environment_shapes):
            self.world_boxes[len(transforms)+idx,:3,:3] = rotation
            self.world_boxes[len(transforms)+idx,:3,3] = position
        self.world_boxes[:,:3,3] += np.einsum('nij,nj->ni',self.world_boxes[:,:3,:3],self.box_centers)

    def contact_points(self, link_indices, radius):
        """実衝突形状の支持点に接する球中心の局所座標。初期化時だけの形状参照。"""
        directions = np.array([value for value in product((-1.,0.,1.),repeat=3) if any(value)])
        directions /= np.linalg.norm(directions,axis=1)[:,None]
        links,points = [],[]
        for link_idx in link_indices:
            for geom_idx in self.link_geoms[link_idx]:
                if geom_idx in self.polytope_vertices:
                    vertices = self.polytope_vertices[geom_idx]
                    support = vertices[np.argmax(vertices@directions.T,axis=0)]
                elif self.model.geom_type[geom_idx] == self.mujoco.mjtGeom.mjGEOM_SPHERE:
                    support = directions*self.model.geom_size[geom_idx,0]
                else:
                    # URDFの円柱面・端面上の支持点。外接箱の頂点による代用なし。
                    size = self.model.geom_size[geom_idx]
                    radial = directions[:,:2]/np.maximum(np.linalg.norm(directions[:,:2],axis=1)[:,None],1e-12)
                    support = np.column_stack((radial*size[0],np.sign(directions[:,2])*size[1]))
                frame = self.local_frames[geom_idx]
                # 接触境界の丸め誤差用の1µm。禁止形状の追加膨張なし。
                centers = support+directions*(radius-1e-6)
                points.extend(centers@frame[:3,:3].T+frame[:3,3])
                links.extend([link_idx]*len(centers))
        return np.asarray(links,dtype=int),np.asarray(points).reshape(-1,3)

    def box_gap(self, first, second):
        """リンク包絡箱の分離軸による距離下界。近接対だけの凸形状照合。"""
        a,b = self.world_boxes[first],self.world_boxes[second]
        if self.environment_names.get(second)=='environment_plane':
            normal = b[:3,2]
            return normal@(a[:3,3]-b[:3,3])-np.abs(normal@a[:3,:3])@self.box_halves[first]
        delta = b[:3,3]-a[:3,3]
        rotation = np.abs(a[:3,:3].T@b[:3,:3])
        first_gaps = np.abs(a[:3,:3].T@delta)-self.box_halves[first]-rotation@self.box_halves[second]
        second_gaps = np.abs(b[:3,:3].T@delta)-self.box_halves[second]-rotation.T@self.box_halves[first]
        return max(float(np.max(first_gaps)),float(np.max(second_gaps)))

    def separation_proof(self, first, second, points):
        """異常距離値用の分離平面証明。凸多面体の支持頂点による保守距離。"""
        if first not in self.polytope_vertices or second not in self.polytope_vertices:
            return None
        rotations = self.data.geom_xmat[[first,second]].reshape(2,3,3)
        positions = self.data.geom_xpos[[first,second]]
        axes = np.vstack((points[3:]-points[:3],positions[1]-positions[0],rotations[0].T,rotations[1].T))
        axes = np.vstack((axes,-axes))
        lengths = np.linalg.norm(axes,axis=1)
        axes = axes[lengths>1e-10]/lengths[lengths>1e-10,None]
        if not len(axes):
            return None
        projected = [self.polytope_vertices[idx]@(rotation.T@axes.T) for idx,rotation in zip((first,second),rotations)]
        first_idx,second_idx = np.argmax(projected[0],axis=0),np.argmin(projected[1],axis=0)
        gaps = (positions[1]-positions[0])@axes.T+projected[1][second_idx,np.arange(len(axes))]-projected[0][first_idx,np.arange(len(axes))]
        idx = int(np.argmax(gaps))
        if gaps[idx]<=1e-8:
            return None
        anchors = np.concatenate((positions[0]+rotations[0]@self.polytope_vertices[first][first_idx[idx]],
                                  positions[1]+rotations[1]@self.polytope_vertices[second][second_idx[idx]]))
        return float(gaps[idx]),anchors,axes[idx]

    def pair_gap(self, first, second, max_dist, closest_points=None, separating_normal=None):
        """指定リンク対の凸形状間距離 [m]。複数形状の最小値。"""
        gap = max_dist
        if self.box_gap(first,second)>=max_dist:
            return gap
        for first_geom in self.link_geoms[first]:
            for second_geom in self.link_geoms[second]:
                self.num_shape_checks += 1
                points = np.zeros(6)
                has_swapped_order = self.model.geom_type[first_geom]>self.model.geom_type[second_geom]
                args = (second_geom,first_geom) if has_swapped_order else (first_geom,second_geom)
                value = self.mujoco.mj_geomDistance(self.model,self.data,*args,max_dist,points)
                if has_swapped_order:
                    points = np.r_[points[3:],points[:3]]
                normal = points[3:]-points[:3]
                length = float(np.linalg.norm(normal))
                normal = normal/max(length,1e-9)*(-1. if value<0 else 1.)
                # 接触値・証人点不整合時の支持平面照合。接触値の無条件無視なし。
                if value<max_dist and (value<=0. or abs(abs(value)-length)>1e-6):
                    repair = self.separation_proof(first_geom,second_geom,points)
                    if repair is None and value<=0.:
                        # 独立な証人方向の取得だけ。別解法の距離値への置換なし。
                        saved_flags = self.model.opt.disableflags
                        fallback_points = np.zeros(6)
                        try:
                            self.model.opt.disableflags |= int(self.mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
                            self.mujoco.mj_geomDistance(self.model,self.data,*args,max_dist,fallback_points)
                        finally:
                            self.model.opt.disableflags = saved_flags
                        if has_swapped_order:
                            fallback_points = np.r_[fallback_points[3:],fallback_points[:3]]
                        repair = self.separation_proof(first_geom,second_geom,fallback_points)
                    if repair is not None and repair[0]>value+1e-6:
                        value,points,normal = repair
                        self.num_distance_repairs += 1
                if value<gap:
                    gap = value
                    if closest_points is not None:
                        closest_points[:] = points
                    if separating_normal is not None:
                        separating_normal[:] = normal
        return gap

    def linear_samples(self, positions, centers, link_frames, max_dist_th=.08):
        """現姿勢の最近点と既存FKによる距離の一次標本。差分姿勢のメッシュ再照合なし。"""
        gaps = self.geometry.internal_clearances(centers[0])
        pairs = np.unique(self.gap_pairs[gaps<max_dist_th],axis=0)
        pairs = pairs[np.any(self.can_control_links[pairs],axis=1)]
        self.set_shape_frames(link_frames[0])
        values = []
        for first,second in pairs:
            points = np.zeros(6)
            normal = np.zeros(3)
            gap = self.pair_gap(first,second,max_dist_th,points,separating_normal=normal)
            if gap>=max_dist_th:
                continue
            sampled = []
            for idx,point in ((first,points[:3]),(second,points[3:])):
                if idx>=len(self.geometry.link_indices):
                    sampled.append(np.broadcast_to(point,(len(positions),3)))
                    continue
                base = link_frames[0,idx]
                local = base[:3,:3].T@(point-base[:3,3])
                sampled.append(np.einsum('nij,j->ni',link_frames[:,idx,:3,:3],local)+link_frames[:,idx,:3,3])
            delta = sampled[1]-sampled[0]
            values.append(gap+(delta-delta[0])@normal)
        return np.stack(values,axis=1) if values else np.empty((len(positions),0))

    def check(self, positions, centers=None, link_frames=None):
        """単一姿勢・姿勢列の共通距離基準。物理シーンへの書込みなし。"""
        positions = np.asarray(positions)
        centers = self.geometry.centers(positions) if centers is None else centers
        is_clear = (self.coarse_check(centers, self.min_clearance_th) if self.coarse_check is not None else
                    self.geometry.has_internal_clearance(centers, self.min_clearance_th))
        flags = np.asarray(is_clear).reshape(-1).copy()
        rows = np.flatnonzero(~flags)
        if positions.ndim==1:
            self.last_violations = []
        if not len(rows):
            return bool(flags[0]) if positions.ndim==1 else flags.reshape(positions.shape[:-1])
        poses = positions.reshape(-1, positions.shape[-1])
        frames = (self.geometry.link_transforms(poses[rows]) if link_frames is None else
                  np.asarray(link_frames).reshape(-1, len(self.geometry.link_indices),4,4)[rows])
        values = np.asarray(centers).reshape(-1, len(self.geometry.radii),3)[rows]
        for row, transforms, sphere_centers in zip(rows, frames, values):
            self.num_refined_poses += 1
            self.set_shape_frames(transforms)
            gaps = self.geometry.internal_clearances(sphere_centers)
            pairs = np.unique(self.gap_pairs[gaps<self.min_clearance_th], axis=0)
            flags[row] = True
            for first, second in pairs:
                gap = self.pair_gap(first,second,self.min_clearance_th)
                if gap<self.min_clearance_th:
                    flags[row] = False
                    if positions.ndim==1:
                        self.last_violations.append(dict(links=[self.link_names[first],self.link_names[second]],gap_m=float(gap)))
                    else:
                        break
        return bool(flags[0]) if positions.ndim==1 else flags.reshape(positions.shape[:-1])


def make_scene(pose, names, active, bodies=None, position_limits=None, drive_mode='position'):
    locked = [name for idx,name in enumerate(names) if idx not in active]
    body = dict(id='pursuer',mode='kinematic',mass=.2,position=[3.,0.,3.],quaternion=[0,0,0,1],
                geoms=[dict(type='sphere',radius=.06,position=[0,0,0],quaternion=[0,0,0,1])])
    pedestal = dict(id='experiment_pedestal',mode='static',mass=1.,position=[0,0,-.07],quaternion=[0,0,0,1],
        geoms=[dict(type='box',size=[.085,.085,.069],position=[0,0,0],quaternion=[0,0,0,1])])
    scene = PhysicsScene([*([body] if bodies is None else bodies),pedestal],dict(model='long',pose=dict(zip(names,map(float,pose))),
        position=[0,0,0],quaternion=[0,0,0,1],locked_joints=locked,control_mode=drive_mode),dict(mode='none'),model_cache_dir=workspace/'docker/runtime/pursuit-model')
    scene.robot.control_entries = [(name,idx,dof,min(limit,2.)) for name,idx,dof,limit in scene.robot.control_entries]
    if position_limits is None:
        position_limits = [(-np.pi,np.pi) if scene.robot.joints[name].get('type')=='continuous' else
                           tuple(float(scene.robot.joints[name].find('limit').get(key)) for key in ('lower','upper')) for name in names]
    for name,bounds in zip(names,position_limits):
        idx = scene.robot.ids[name]
        scene.model.jnt_limited[idx] = True
        scene.model.jnt_range[idx] = bounds
    scene.mujoco.mj_forward(scene.model,scene.data)
    return scene


def state(scene, names):
    pose, motion = scene.robot.state(), scene.robot.motion_state()
    return np.asarray([pose[name] for name in names]), np.asarray([motion['velocity'][name] for name in names])


def body_velocity_at_origin(scene, body, twist):
    """世界座標の角速度・リンク原点速度。重心速度からの低コスト補正。"""
    scene.mujoco.mj_objectVelocity(scene.model,scene.data,scene.mujoco.mjtObj.mjOBJ_BODY,body,twist,0)
    twist[3:] += np.cross(twist[:3],scene.data.xpos[body]-scene.data.xipos[body])
    return twist


def configure_position_limits(geometry, config):
    """全関節の有限可動域。URDF制限の維持と実機校正値の明示指定。"""
    limits = geometry.limits.copy()
    kinds = {joint[0]:joint[3] for joint in geometry.joints}
    sources = {name:'simulation_fallback' if kinds[name]=='continuous' else 'urdf' for name in geometry.joint_names}
    overrides = config.get('joint_limits_deg',{})
    if not isinstance(overrides,dict) or set(overrides)-set(geometry.joint_names):
        raise ValueError('関節可動域の対象名・形式の不正')
    for name,bounds in overrides.items():
        if not isinstance(bounds,dict) or set(bounds)!={'min_position_deg','max_position_deg'}:
            raise ValueError('関節可動域にはmin_position_deg・max_position_degの指定が必要')
        values = [bounds['min_position_deg'],bounds['max_position_deg']]
        if any(type(value) not in (float,int) or not math.isfinite(value) for value in values) or values[0]>=values[1]:
            raise ValueError('関節可動域の有限性・大小関係の不正')
        idx = geometry.joint_names.index(name)
        values = np.radians(values)
        if kinds[name]!='continuous' and (values[0]<limits[idx,0]-1e-9 or values[1]>limits[idx,1]+1e-9):
            raise ValueError('URDF制限を広げる関節可動域の指定不可')
        limits[idx] = values
        sources[name] = 'configuration'
    if not np.isfinite(limits).all() or np.any(limits[:,0]>=limits[:,1]):
        raise ValueError('全関節に有限な試験可動域が必要')
    geometry.limits = limits
    # 実機可動域をまたぐ角度ラップ・無限回転の禁止。旧設定のtrue指定でも制限維持。
    geometry.is_periodic = np.zeros(len(geometry.joint_names),dtype=bool)
    return sources


def position_limit_status(positions, names, limits):
    """実測可動域超過と該当関節・上下限。接触判定とは独立の評価。"""
    violated = np.flatnonzero((positions<limits[:,0]-.05)|(positions>limits[:,1]+.05))
    joints = [dict(name=names[idx],position_rad=float(positions[idx]),min_position_rad=float(limits[idx,0]),
                   max_position_rad=float(limits[idx,1])) for idx in violated]
    return ('physical_joint_limit' if len(violated) else ''),joints


def apply_velocity(scene, names, active, command_velocity):
    """速度解の直接送信 [rad/s]。角度目標・換算時間・位置駆動への代替なし。"""
    values = np.asarray(command_velocity,dtype=float)
    if scene.robot.control_mode.value!='velocity' or values.shape!=(len(active),) or not np.isfinite(values).all():
        raise ValueError('速度駆動と制御対象軸の有限速度解が必要')
    scene.robot.set_command(dict(mode='velocity',velocity={names[idx]:float(value) for idx,value in zip(active,values)}))


def apply_position(scene, names, target):
    """旧位置経由点方式の絶対姿勢。固定角度刻み・速度解との往復変換なし。"""
    values = np.asarray(target,dtype=float)
    if scene.robot.control_mode.value!='position' or values.shape!=(len(names),) or not np.isfinite(values).all():
        raise ValueError('位置駆動と全軸の有限な絶対姿勢が必要')
    limits = scene.model.jnt_range[[scene.robot.ids[name] for name in names]]
    scene.robot.set_targets(dict(zip(names,map(float,np.clip(values,limits[:,0],limits[:,1])))))


def calibrate(geometry, active, poses, duration_sec=8., cache_dir=None, drive_mode='position'):
    """同一物理モデル・姿勢・有限可動域の速度校正値の再利用。"""
    source = Path(__file__).parent
    urdf = source.parents[1]/'app/source.urdf'
    files = [source/name for name in ('sphere_chase.py','physics_scene.py','physics_robot.py')]+[urdf]
    files.extend(sorted({urdf.parent/node.get('filename') for node in et.parse(urdf).findall('.//collision/geometry/mesh')}))
    import mujoco
    signature = dict(mujoco_version=mujoco.__version__,numpy_version=np.__version__,duration_sec=duration_sec,drive_mode=drive_mode,
        names=geometry.joint_names,active=np.asarray(active).tolist(),limits=geometry.limits.tolist(),poses={name:value.tolist() for name,value in poses.items()},
        files={str(path.relative_to(source.parents[1])):hashlib.sha256(path.read_bytes()).hexdigest() for path in files})
    key = hashlib.sha256(json.dumps(signature,sort_keys=True).encode()).hexdigest()
    cache_dir = Path(cache_dir) if cache_dir is not None else workspace/'docker/runtime/pursuit-calibration'
    cache = cache_dir/(key+'.json')
    try:
        record = json.loads(cache.read_text())
        value = record['reference_eef_speed_mps']
        if record.get('key')==key and type(value) in (int,float) and math.isfinite(value) and .01<=value<=100:
            geometry.calibration_source = 'cache'
            return float(value)
    except (OSError,ValueError,KeyError,TypeError):
        pass
    value = calibrate_reference(geometry,active,poses,duration_sec,drive_mode)
    geometry.calibration_source = 'computed'
    temporary = None
    try:
        cache_dir.mkdir(parents=True,exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w',dir=cache_dir,prefix='calibration-',suffix='.tmp',delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(dict(key=key,reference_eef_speed_mps=value),stream,allow_nan=False)
        temporary.chmod(0o644)
        temporary.replace(cache)
    except OSError:
        # 書込み不可時の校正結果の継続使用。キャッシュ成立との混同なし。
        geometry.calibration_source = 'computed_uncached'
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return value


def calibrate_reference(geometry, active, poses, duration_sec=8., drive_mode='position'):
    scene = make_scene(poses['a'],geometry.joint_names,active,position_limits=geometry.limits,drive_mode=drive_mode)
    body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_L_tcp')
    previous = scene.data.xpos[body].copy()
    max_speed = 0.
    for frame in range(int(duration_sec/.01)):
        positions, velocity = state(scene,geometry.joint_names)
        target = poses['b' if int(frame*.01/2)%2==0 else 'a']
        if frame%5==0:
            if drive_mode=='velocity':
                desired = 3.*angle_delta(target,positions,geometry.is_periodic)[active]
                command_velocity = joint_velocity_command(positions[active],velocity[active],desired,geometry.limits[active])
                apply_velocity(scene,geometry.joint_names,active,command_velocity)
            else:
                apply_position(scene,geometry.joint_names,target)
        scene.step()
        current = scene.data.xpos[body].copy()
        if frame > 20:
            max_speed = max(max_speed,float(np.linalg.norm(current-previous)/.01))
        previous = current
    if not np.isfinite(max_speed) or max_speed < .01:
        raise ValueError('手先速度の無障害校正の失敗')
    return max_speed


def run_case(config, output_dir=None, emit=None, stop_event=None, resources=None):
    startup_begin = time.perf_counter()
    startup_stages_ms = {}
    def progress(stage):
        elapsed_ms = (time.perf_counter()-startup_begin)*1000
        startup_stages_ms[stage] = elapsed_ms
        if emit is not None:
            emit(dict(type='experiment_progress',stage=stage,elapsed_ms=elapsed_ms))
    execution_mode = config.get('execution_mode','recorded')
    if execution_mode not in ('recorded','continuous'):
        raise ValueError('未対応の実行方式')
    is_continuous = execution_mode=='continuous'
    if is_continuous and output_dir is not None:
        raise ValueError('連続実験は記録なし。結果保存先の併用不可')
    if output_dir is not None and any((Path(output_dir)/name).exists() for name in ('metrics.json','details.json','trajectory.jsonl')):
        raise ValueError('既存の試験結果への上書き禁止。新しい出力先の指定')
    policy_name = config.get('policy','rollout_cv')
    if policy_name not in ('vlut_graph','avoidance_first','vrm_predictive','rollout_reactive','rollout_cv','rollout_sequence','cbf_qp','oscbf'):
        raise ValueError('未対応の球回避方式')
    is_cbf = policy_name in ('cbf_qp','oscbf')
    duration = float(config.get('duration_sec',0. if is_continuous else 20.))
    if is_continuous and duration!=0.:
        raise ValueError('連続実験の時間指定不可')
    if not is_continuous and not 1 <= duration <= 180:
        raise ValueError('試験時間は1〜180 sec')
    ramp_sec = float(config.get('speed_ramp_sec',18. if is_continuous else duration))
    if not math.isfinite(ramp_sec) or not 1 <= ramp_sec <= 180:
        raise ValueError('速度上昇時間は1〜180 sec')
    seed = int(config.get('seed',11))
    radius = float(config.get('radius_m',.06))
    delay = float(config.get('observation_delay_sec',0.))
    noise = float(config.get('observation_noise_m',0.))
    if not .015 <= radius <= .15 or not 0 <= delay <= .2 or not 0 <= noise <= .01:
        raise ValueError('球・観測条件の範囲外')
    min_ratio, max_ratio = config.get('speed_ratios',[.2,.8])
    if not 0 < min_ratio <= max_ratio <= 3:
        raise ValueError('追跡速度比は0〜3の範囲内')
    if not 0 < float(config.get('max_ball_acceleration',.8)) <= 30. or not 0 <= float(config.get('pursuit_delay_sec',0.)) <= .5 or not .25 <= float(config.get('start_dist_m',.45)) <= 2.:
        raise ValueError('追跡球の加速度・追従遅延・開始距離の範囲外')
    progress('robot_geometry')
    resources = experiment_resources() if resources is None else resources
    params,models = resources.prepare(enable_gng=not is_cbf or config.get('trajectory_planner')=='gng_chomp')
    base_geometry,base_checker = resources.get('robot',lambda:robot_assets(params,models))
    geometry = copy.deepcopy(base_geometry)
    profiles = independent_policy(geometry,{'left_arm':models['left_arm']})
    arm = profiles.policies['left_arm']
    geometry.is_arm = arm.geometry.is_arm.copy()
    active = np.asarray(arm.arm_indices)
    geometry.active_joint_names = tuple(arm.arm_names)
    if type(config.get('enable_periodic_angles',False)) is not bool:
        raise ValueError('連続関節の周期扱いは真偽値')
    joint_limit_sources = configure_position_limits(geometry,config)
    internal_checker = copy.copy(base_checker)
    internal_checker.geometry = geometry
    internal_checker.data = copy.copy(base_checker.data)
    internal_checker.last_violations = []
    target_link = config.get('target_link','L_tcp')
    if target_link not in geometry.link_indices:
        raise ValueError(f'追跡対象リンク名の不正: {target_link}')
    target_offset = np.asarray(config.get('target_offset_m',[0.,0.,0.]),dtype=float)
    if target_offset.shape != (3,) or not np.isfinite(target_offset).all() or np.linalg.norm(target_offset)>.5:
        raise ValueError('追跡対象のリンク内オフセットの不正')
    if config.get('pursuit_mode','link_point') not in ('link_point','distal_sweep','random_walk'):
        raise ValueError('球の動きの不正')
    task = json.loads((workspace/'benchmarks/shuttle_bag/task.json').read_text())
    poses = {key:np.asarray([math.radians(value.get(name,0.)) for name in geometry.joint_names])
             for key,value in task['poses_deg'].items()}
    task_mode = config.get('task_mode','cartesian' if policy_name.startswith('rollout_') or policy_name in ('vlut_graph','cbf_qp','oscbf') else 'joint')
    if task_mode not in ('cartesian','joint') or (task_mode=='cartesian' and policy_name in ('avoidance_first','vrm_predictive')):
        raise ValueError('未対応のタスク方式。旧経由点方式はjointのみ')
    offset_deg = config.get('goal_offset_deg',{})
    for name,value in offset_deg.items():
        if name not in arm.arm_names or not np.isfinite(value) or abs(value)>15:
            raise ValueError('目標変動の指定不正')
        poses['b'][geometry.joint_names.index(name)] += math.radians(value)
    for pose in poses.values():
        if np.any(pose<geometry.limits[:,0]) or np.any(pose>geometry.limits[:,1]) or not internal_checker.check(pose):
            raise ValueError('開始・目標姿勢の位置制限または自己干渉')
    progress('reference_trajectory')
    trajectory_planner = config.get('trajectory_planner','rrt_chomp' if is_cbf else 'gng_chomp')
    if trajectory_planner not in ('gng_chomp','rrt_chomp','direct'):
        raise ValueError('未対応の往復軌道生成方式')
    directory = Path(params['gng']['data_directory'])/params['gng']['experiment_id']/'left_arm'
    def seed_graph():
        graph = resources.get('shuttle_graph',lambda:load_graph(directory/models['left_arm']['gng_file']))
        seed_index = resources.get('vlut_index',lambda:vlut_index(geometry,active,poses['a'],graph,directory/models['left_arm']['vlut_file']))
        internal_checker.coarse_check = lambda centers,margin:seed_index.internal_clear(geometry,centers,margin)
        return graph
    # 動的障害物の真値・将来軌跡を参照しない、自己形状・床・台座だけの事前計画。
    def full_poses(values):
        result = np.repeat(poses['a'][None],len(values),axis=0)
        result[:,active] = values
        return result
    def obstacle_cost(values):
        gaps = geometry.internal_clearances(geometry.centers(full_poses(values)))
        # 形状近似の離隔ポテンシャル。採否は共通の実形状照合、目的値は実電力量ではない。
        return .03*np.sum(np.maximum(0.,.03-gaps)**2,axis=-1)
    reference_key = ('shuttle',trajectory_planner,poses['a'].tobytes(),poses['b'].tobytes(),geometry.limits.tobytes())
    has_cached_reference = reference_key in resources.values
    reference_started = time.perf_counter()
    def build_reference():
        planner = shuttle_planner(geometry.limits[active],lambda values:internal_checker.check(full_poses(values)),obstacle_cost)
        values,stats = planner.plan(poses['a'][active],poses['b'][active],trajectory_planner,
                                   seed_graph if trajectory_planner=='gng_chomp' else None)
        return full_poses(values),stats
    reference_path,reference_stats = resources.get(reference_key,build_reference)
    for key in [key for key in resources.values if isinstance(key,tuple) and key[0]=='shuttle'][:-2]:
        resources.values.pop(key)
    reference_stats = dict(reference_stats,has_cached_reference=has_cached_reference,
                          prepare_ms=(time.perf_counter()-reference_started)*1000)
    task_object = cartesian_task(geometry,active,poses,task.get('eef_link','L_tcp'),
        float(task.get('max_eef_position_error_th',.015)),math.radians(float(task.get('max_eef_orientation_error_deg_th',3.))),
        reference_path=reference_path,reference_stats=reference_stats)
    control_task = task_object if task_mode=='cartesian' else None
    task_metadata = task_object.metadata(task_mode)
    progress('drive_calibration')
    # 連続速度提案と駆動の整合。旧位置経由点方式の位置駆動は保持。
    has_velocity_proposal = policy_name.startswith('rollout_') or policy_name in ('vlut_graph','cbf_qp','oscbf')
    required_drive_mode = 'velocity' if has_velocity_proposal else 'position'
    drive_mode = config.get('drive_mode',required_drive_mode)
    if drive_mode!=required_drive_mode:
        raise ValueError('方式と駆動の不一致。速度解方式はvelocity、旧位置経由点方式はposition専用')
    calibrated_reference_speed = calibrate(geometry,active,poses,drive_mode=drive_mode)
    reference_speed = float(config.get('reference_eef_speed_mps',calibrated_reference_speed))
    if not math.isfinite(reference_speed) or not .01<=reference_speed<=10.:
        raise ValueError('障害物の基準速度の範囲外')
    progress('obstacle_setup')
    transform = geometry.link_transforms(poses['a'])[geometry.link_indices[target_link]]
    followed = transform[:3,3]+transform[:3,:3]@target_offset
    generator = np.random.default_rng(seed)
    initial = config.get('initial_direction')
    if initial is not None and (np.asarray(initial).shape!=(3,) or not np.isfinite(initial).all() or np.linalg.norm(initial)<1e-8):
        raise ValueError('開始方向は非ゼロの有限3次元ベクトル')
    start_dist = float(config.get('start_dist_m',.45))
    target_frames = geometry.link_transforms(np.stack(list(poses.values())))[:,geometry.link_indices[target_link]]
    target_positions = target_frames[:,:3,3]+np.einsum('nij,j->ni',target_frames[:,:3,:3],target_offset)
    if config.get('obstacle_shape')=='linked_arm' and 'shoulder_position_m' in config:
        anchor = np.asarray(config['shoulder_position_m'],dtype=float)
        if anchor.shape!=(3,) or not np.isfinite(anchor).all():
            raise ValueError('リンク原点の位置不正（world座標・m）')
        if np.max(np.linalg.norm(target_positions-anchor,axis=1))>=sum(config.get('link_lengths_m',[.35,.35]))-.005:
            raise ValueError('指定したリンク原点から往復対象に届きません。原点位置の変更が必要です')
    if config.get('pursuit_mode')=='random_walk':
        extent = config.get('random_walk_extent_m',.6)
        change_sec = config.get('random_walk_change_sec',.6)
        if type(extent) not in (int,float) or not .3<=extent<=1.5 or type(change_sec) not in (int,float) or not .1<=change_sec<=5.:
            raise ValueError('ランダムウォークの範囲・更新間隔の不正')
        start_dist = min(start_dist,.8*extent)
    for attempt in range(64):
        direction = np.asarray(initial,dtype=float) if initial is not None and attempt==0 else generator.normal(size=3)
        direction[2] = abs(direction[2])*.5
        direction /= max(float(np.linalg.norm(direction)),1e-9)
        center = followed+direction*start_dist
        gap = np.min(np.linalg.norm(geometry.centers(poses['a'])-center,axis=1)-geometry.radii-radius)
        if gap>.12 and center[2]>radius:
            try:
                obstacle_config = {**config,'target_link':target_link}
                if config.get('obstacle_shape')=='linked_arm' and 'shoulder_position_m' not in config:
                    obstacle_config['shoulder_candidates_m'] = shoulder_anchor_candidates(target_positions,center,direction,
                        np.asarray(config.get('link_lengths_m',[.35,.35]))).tolist()
                obstacle = pursuit_obstacle(obstacle_config,center,direction,geometry,poses['a'],guard_geometry=internal_checker)
            except ValueError as error:
                if '開始位置' not in str(error):
                    raise
                continue
            if np.min(obstacle.field().query(geometry.centers(poses['a']))[0]-geometry.radii)>.12:
                break
    else:
        raise ValueError('非接触の障害物開始位置なし。リンク型では原点位置の変更が必要です')
    initial_center = obstacle.center.copy()
    motion_metadata = dict(random_walk=dict(min_position_m=obstacle.min_random_walk_position_m.tolist(),
        max_position_m=obstacle.max_random_walk_position_m.tolist(),change_sec=obstacle.random_walk_change_sec)) if obstacle.pursuit_mode=='random_walk' else {}
    scene = make_scene(poses['a'],geometry.joint_names,active,obstacle.bodies(),geometry.limits,drive_mode)
    obstacle.move_scene(scene)
    obstacle_bodies = {value for name,value in scene.body_ids.items() if name.startswith('pursuer_')}
    obstacle_geoms = {idx for idx in range(scene.model.ngeom) if scene.model.geom_bodyid[idx] in obstacle_bodies}
    obstacle_indices = {}
    for idx in obstacle_geoms:
        body_id = int(scene.model.geom_bodyid[idx])
        obstacle_indices[idx] = next(int(name.removeprefix('pursuer_')) for name,value in scene.body_ids.items() if value==body_id)
    eef_body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_L_tcp')
    target_body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_'+target_link)
    anchor_bodies = [scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_'+name) for name in obstacle.guard_anchor_links]
    link_bodies = [scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_'+internal_checker.link_names[idx])
                   for idx in range(len(geometry.link_indices))]
    if any(body<0 for body in link_bodies):
        raise ValueError('リンクと物理モデルの対応不一致')
    protected_links = {geometry.spheres[idx][0] for idx in np.flatnonzero(obstacle.protected_mask)}
    active_links = {geometry.spheres[idx][0] for idx in np.flatnonzero(geometry.is_arm)}
    geom_links = [scene.mujoco.mj_id2name(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,int(body)) or 'world' for body in scene.model.geom_bodyid]
    eef = scene.data.xpos[eef_body].copy()
    metadata = models['left_arm']
    directory = Path(params['gng']['data_directory'])/params['gng']['experiment_id']/'left_arm'
    graph = index = rollout = legacy = None
    memory = motion_sequence_predictor(enable_learning=config.get('enable_learning',True))
    if config.get('memory_file'):
        memory.restore(json.loads(Path(config['memory_file']).read_text()))
    progress('cbf_solver' if is_cbf else 'gng_index')
    if not is_cbf:
        def build_index():
            graph = resources.get('shuttle_graph',lambda:load_graph(directory/metadata['gng_file']))
            return (vlut_index(geometry,active,poses['a'],graph,directory/metadata['vlut_file']) if policy_name=='vlut_graph' else
                    representative_index(geometry,active,poses['a'],graph))
        index = resources.get('vlut_index' if policy_name=='vlut_graph' else 'representative_index',build_index)
        index.reset_observation()
        graph = index.graph
        internal_checker.coarse_check = lambda centers, margin:index.internal_clear(geometry,centers,margin)
        if policy_name=='vlut_graph':
            rollout = vlut_graph_policy(geometry,active,target_link,graph,node_index=index,memory=memory,
                margin=float(config.get('margin_m',.08)),task_weight=float(config.get('task_weight',.12)),
                target_offset=target_offset,task=control_task,internal_checker=internal_checker)
        elif policy_name.startswith('rollout_'):
            rollout = sphere_rollout_policy(geometry,active,target_link,graph,
                mode=dict(rollout_reactive='reactive',rollout_cv='constant_velocity',rollout_sequence='sequence')[policy_name],
                candidate_metric=config.get('candidate_metric','joint'),memory=memory,node_index=index,
                horizon_sec=float(config.get('horizon_sec',.4)),margin=float(config.get('margin_m',.08)),
                task_weight=float(config.get('task_weight',.12)),target_offset=target_offset,task=control_task,
                internal_checker=internal_checker)
        else:
            legacy = independent_policy(geometry,{'left_arm':metadata},strategy='vrm_predictive' if policy_name=='vrm_predictive' else 'avoidance_first').policies['left_arm']
            legacy.clock_sec = lambda:float(scene.data.time)
            legacy.positions = poses['a'].copy()
            legacy.config.update(max_bridge_step=.025,min_clearance_th=.01,min_planning_clearance_th=.005,
                trigger_clearance_th=.10,release_clearance_th=.14,return_clear_sec=.5,max_state_age_sec=.3,max_local_step=.12)
        if rollout is not None and is_continuous:
            rollout.samples_ms = deque(maxlen=1200)
    cbf = pursuit_cbf(geometry,active,scene,policy_name,float(config.get('margin_m',.08)),task=control_task,
                      internal_checker=internal_checker) if is_cbf else None
    progress('ready')
    pending, eef_history = deque(),deque()
    goal, arrivals, first_contact = 'b',0,None
    arrival_events = deque(maxlen=120) if is_continuous else []
    num_arrivals_with_posture_dev = 0
    position_error_squared = orientation_error_squared = 0.
    max_task_position_error_m = max_task_orientation_error_rad = 0.
    task_position_error_m = task_orientation_error_rad = 0.
    task_twist = np.zeros(6)
    contact_sec = proxy_contact_sec = internal_failure_sec = stopped_sec = 0.
    min_gap = 1e6
    max_ball_speed = max_eef_speed = max_joint_speed = 0.
    costs = deque(maxlen=1200) if is_continuous else []
    num_proposals = 0
    reference_dev_squared = max_reference_dev_rad = 0.
    total_cost_ms = 0.
    node_query_total_ms = 0.
    rows = []
    observed = None
    target = poses['a'].copy()
    phase = 'waiting'
    last_eef = eef.copy()
    phase_sec = {}
    failure_reason = ''
    num_internal_frames = 0
    evaluated_sec = 0.
    previous_segments = None
    previous_stamp = None
    max_ball_acceleration = 0.
    previous_ball_velocity = np.zeros(3)
    min_actual_fixed_gap = 10.
    min_protected_gap = 10.
    num_protected_contacts = 0
    failure_details = {}
    outcome = 'unfinished'
    actual_centers = geometry.centers(poses['a'])
    joint_position_limits = {name:dict(min_position_rad=float(bounds[0]),max_position_rad=float(bounds[1]),source=joint_limit_sources[name])
                             for name,bounds in zip(geometry.joint_names,geometry.limits)}
    if emit is not None:
        from sphere_replay import recording_metadata
        emit(dict(type='experiment_ready',recording=recording_metadata(dict(
            config={**config,'policy':policy_name,'seed':seed},enable_periodic_angles=False,joint_position_limits=joint_position_limits,
            task=task_metadata,contact_rule='target_tip',target_contact_links=sorted(obstacle.target_links),outcome=outcome,**motion_metadata))))
    num_frames = None if is_continuous else int(round(duration/.01))
    for frame in count():
        if (num_frames is not None and frame>=num_frames) or (stop_event is not None and stop_event.is_set()):
            break
        frame_started_sec = time.monotonic() if is_continuous else None
        now = frame*.01
        task_object.goal = goal
        positions,velocity = state(scene,geometry.joint_names)
        actual_eef = scene.data.xpos[eef_body].copy()
        target_position = scene.data.xpos[target_body]+scene.data.xmat[target_body].reshape(3,3)@target_offset
        eef_history.append((now,target_position.copy()))
        follow_delay = float(config.get('pursuit_delay_sec',0.))
        while len(eef_history)>1 and eef_history[1][0] <= now-follow_delay:
            eef_history.popleft()
        followed = eef_history[0][1]
        speed = reference_speed*(min_ratio+(max_ratio-min_ratio)*min(now/ramp_sec,1.))
        obstacle.update_guard(actual_centers,[scene.data.xpos[body] for body in anchor_bodies])
        obstacle.update_link_frames(scene.data.xpos[link_bodies],scene.data.xmat[link_bodies].reshape(-1,3,3))
        achieved_speed = obstacle.advance(followed,speed,float(config.get('max_ball_acceleration',.8)),.01,config.get('enable_sidestep',False),now)
        center = obstacle.center.copy()
        max_ball_speed = max(max_ball_speed,achieved_speed)
        max_ball_acceleration = max(max_ball_acceleration,float(np.linalg.norm(obstacle.velocity-previous_ball_velocity)/.01))
        previous_ball_velocity = obstacle.velocity.copy()
        obstacle.move_scene(scene)
        if frame%5==0:
            offset = generator.normal(0.,noise,3)
            packet = dict(center=center+offset,stamp=now,eef=target_position.copy(),segments=obstacle.segments+offset)
            pending.append((now+delay,packet))
        while pending and pending[0][0] <= now+1e-9:
            observed = pending.popleft()[1]
        if frame%5==0 and observed is not None:
            begin = time.perf_counter()
            motion_goal = task_object.update_reference(positions)
            reference_dev_squared += task_object.reference.dev_rad**2
            max_reference_dev_rad = max(max_reference_dev_rad,task_object.reference.dev_rad)
            # 遅延による不確かさの解析的な球半径への集約。方式間で共通の現観測。
            inflated_radius = radius+reference_speed*max_ratio*(now-observed['stamp'])+3*noise
            if obstacle.shape == 'sphere':
                field = sphere_field(observed['center'],inflated_radius)
            else:
                endpoint_velocity = np.zeros((2,2,3))
                if previous_segments is not None and observed['stamp']>previous_stamp:
                    endpoint_velocity = (observed['segments']-previous_segments)/(observed['stamp']-previous_stamp)
                field = capsule_field(observed['segments'],inflated_radius,endpoint_velocity)
                previous_segments,previous_stamp = observed['segments'].copy(),observed['stamp']
            field.stamp_sec,field.eef_position = observed['stamp'],observed['eef']
            if cbf is not None:
                command_velocity = cbf.propose_velocity(positions,velocity,motion_goal,field,observed['stamp'])
                phase = cbf.phase
            elif rollout is not None:
                query_begin = time.perf_counter()
                if policy_name=='vlut_graph':
                    index.update_field(field)
                else:
                    # 連続候補方式で参照する近傍32ノードだけの照合。グラフ探索方式は全ノード更新。
                    _,nearest = graph['angle_tree'].query(positions[active],k=min(32,len(graph['angles'])))
                    index.update_field(field,node_indices=np.atleast_1d(nearest))
                node_query_total_ms += (time.perf_counter()-query_begin)*1000
                command_velocity = rollout.propose_velocity(positions,velocity,motion_goal,field,observed['stamp'])
                phase = rollout.phase
            else:
                index.update_field(field)
                legacy.positions,legacy.home = positions.copy(),motion_goal.copy()
                legacy.cloud_tree,legacy.cell_radius = field,0.
                for name in ('angles','adjacency','angle_node_ids','angle_tree','labels'):
                    setattr(legacy,name,graph[name])
                if policy_name=='vrm_predictive':
                    legacy.observe((field,0.,observed['stamp']),graph)
                    target,can_move = legacy.select_waypoint()
                elif legacy.can_follow_task():
                    target,can_move = motion_goal.copy(),True
                    legacy.motion_phase = 'task'
                else:
                    target,can_move = legacy.select_retreat()
                phase = legacy.motion_phase if can_move else 'holding'
            if has_velocity_proposal:
                apply_velocity(scene,geometry.joint_names,active,command_velocity)
            else:
                apply_position(scene,geometry.joint_names,target)
            cost_ms = (time.perf_counter()-begin)*1000
            costs.append(cost_ms)
            total_cost_ms += cost_ms
            num_proposals += 1
        scene.step()
        positions,velocity = state(scene,geometry.joint_names)
        actual_eef = scene.data.xpos[eef_body].copy()
        max_eef_speed = max(max_eef_speed,float(np.linalg.norm(actual_eef-last_eef)/.01))
        last_eef = actual_eef
        max_joint_speed = max(max_joint_speed,float(np.max(np.abs(velocity[active]))))
        actual_centers = geometry.centers(positions)
        actual_gaps = obstacle.field().query(actual_centers)[0]-geometry.radii
        gap = float(np.min(actual_gaps))
        min_actual_fixed_gap = min(min_actual_fixed_gap,float(np.min(actual_gaps[~geometry.is_arm])))
        min_protected_gap = min(min_protected_gap,float(np.min(actual_gaps[obstacle.protected_mask])))
        min_gap = min(min_gap,gap)
        contacts = [contact for contact in scene.data.contact[:scene.data.ncon]
                    if (int(contact.geom1) in obstacle_geoms) != (int(contact.geom2) in obstacle_geoms)]
        has_contact = bool(contacts)
        contact_links = []
        contact_details = []
        for contact in contacts:
            other = int(contact.geom2) if int(contact.geom1) in obstacle_geoms else int(contact.geom1)
            obstacle_geom = int(contact.geom1) if int(contact.geom1) in obstacle_geoms else int(contact.geom2)
            body_name = scene.mujoco.mj_id2name(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,int(scene.model.geom_bodyid[other])) or 'ground'
            link = body_name.removeprefix('robot_')
            contact_links.append(link)
            contact_details.append(dict(link=link,is_protected=link in protected_links,is_environment=link in ('world','ground','experiment_pedestal'),
                obstacle_idx=obstacle_indices[obstacle_geom],position=contact.pos.tolist()))
        has_protected_contact = any(name in protected_links for name in contact_links)
        num_protected_contacts += int(has_protected_contact)
        contact_sec += .01*has_contact
        proxy_contact_sec += .01*(gap<0)
        is_internal_clear = internal_checker.check(positions, actual_centers)
        internal_failure_sec += .01*(not is_internal_clear)
        num_internal_frames = 0 if is_internal_clear else num_internal_frames+1
        if has_contact and first_contact is None:
            first_contact = now+.01
        stopped_sec += .01*(np.max(np.abs(velocity[active]))<.03)
        phase_sec[phase] = phase_sec.get(phase,0.)+.01
        goal_error = float(np.max(np.abs(angle_delta(poses[goal],positions,geometry.is_periodic)[active])))
        # 接触後の押し出し・内部干渉・異常運動を成功に含めない共通打切り条件。
        limit_reason,limit_joints = position_limit_status(positions,geometry.joint_names,geometry.limits)
        environment_contacts = []
        for contact in scene.data.contact[:scene.data.ncon]:
            names = [geom_links[geom] for geom in (contact.geom1,contact.geom2)]
            if any(name in ('world','experiment_pedestal') for name in names):
                environment_contacts.extend(name.removeprefix('robot_') for name in names if name.removeprefix('robot_') in active_links)
        if environment_contacts and not has_contact:
            contact_sec += .01
            if first_contact is None:
                first_contact = now+.01
        if has_contact:
            failure_reason = obstacle.contact_result(contact_details)
        elif environment_contacts:
            failure_reason = 'robot_environment_contact'
        elif num_internal_frames >= 3:
            failure_reason = 'internal_interference'
        elif np.max(np.abs(velocity[active])) > 6.:
            failure_reason = 'abnormal_joint_speed'
        elif limit_reason:
            failure_reason = limit_reason
        if failure_reason:
            outcome = 'attacker_loss' if failure_reason.startswith('attacker_') else 'attacker_win' if failure_reason=='target_touch' else 'robot_failure'
            failure_details = dict(contact_links=sorted(set(contact_links+environment_contacts)),has_protected_contact=has_protected_contact,contacts=contact_details)
            if failure_reason=='internal_interference':
                failure_details['internal_clearance'] = internal_checker.last_violations
            if failure_reason in ('physical_joint_limit','planning_domain_exit'):
                failure_details['joints'] = limit_joints
        evaluated_sec = now+.01
        task_frame = np.eye(4);task_frame[:3,3] = actual_eef;task_frame[:3,:3] = scene.data.xmat[eef_body].reshape(3,3)
        body_velocity_at_origin(scene,eef_body,task_twist)
        task_position_error_m,task_orientation_error_rad,can_arrive = task_object.measure(task_frame,task_twist[3:],task_twist[:3])
        position_error_squared += task_position_error_m**2;orientation_error_squared += task_orientation_error_rad**2
        max_task_position_error_m = max(max_task_position_error_m,task_position_error_m)
        max_task_orientation_error_rad = max(max_task_orientation_error_rad,task_orientation_error_rad)
        evaluated_goal = goal
        if task_mode=='joint':
            can_arrive = goal_error<.035 and np.max(np.abs(velocity[active]))<.12
        if not failure_reason and can_arrive:
            arrivals += 1
            posture_dev = float(np.max(np.abs(positions[active]-poses[goal][active])))
            num_arrivals_with_posture_dev += int(posture_dev>=.035 or np.max(np.abs(velocity[active]))>=.12)
            arrival_events.append(dict(time_sec=evaluated_sec,goal=goal,position_error_m=task_position_error_m,
                orientation_error_rad=task_orientation_error_rad,max_posture_dev_rad=posture_dev,
                max_joint_speed_radps=float(np.max(np.abs(velocity[active]))),linear_speed_mps=float(np.linalg.norm(task_twist[3:])),angular_speed_radps=float(np.linalg.norm(task_twist[:3]))))
            goal = 'a' if goal=='b' else 'b'
        if frame%5==0 or failure_reason or (num_frames is not None and frame==num_frames-1):
            motion = scene.robot.motion_state()
            # ロールアウト・予測の時間幅と同じ長さの直近実軌跡。時間幅のない方式では軌跡なし。
            predictor = rollout if rollout is not None else getattr(legacy,'predictor',None)
            obstacle_trail_sec = float(predictor.horizon_sec) if predictor is not None else 0.
            row = dict(time_sec=evaluated_sec,joints=dict(zip(geometry.joint_names,map(float,positions))),
                obstacle_trail_sec=obstacle_trail_sec,
                motion={field:{name:values[name] for name in geometry.joint_names if name in values} for field,values in motion.items()},
                task_goal=evaluated_goal,num_arrivals=arrivals,task_position_error_m=task_position_error_m,task_orientation_error_rad=task_orientation_error_rad,
                eef=actual_eef.tolist(),sphere=center.tolist(),sphere_radius_m=radius,phase=phase,
                target=(obstacle.sweep_aim if obstacle.sweep_aim is not None else scene.data.xpos[target_body]+scene.data.xmat[target_body].reshape(3,3)@target_offset).tolist(),
                has_contact=bool(has_contact or environment_contacts),proxy_gap_m=gap,obstacle_phase=obstacle.phase,obstacle_speed_mps=achieved_speed)
            if obstacle.shape == 'linked_arm':
                row['obstacle_segments'] = obstacle.segments.tolist()
            if not is_continuous:
                rows.append(row)
            if emit is not None:
                # 保存用の実測フレームの共用。表示用の追加物理積分・FK・衝突照合なし。
                emit(dict(type='experiment_frame',frame=row))
        if not np.isfinite(positions).all():
            raise RuntimeError('非有限の実測関節状態')
        if failure_reason:
            break
        if is_continuous:
            # 実時間ペーシング。遅延蓄積による追付き連打なし。
            wait_sec = max(0.,.01-(time.monotonic()-frame_started_sec))
            if stop_event is not None:
                stop_event.wait(wait_sec)
            else:
                time.sleep(wait_sec)
    metrics = dict(num_arrivals=arrivals,num_round_trips=arrivals//2,contact_sec=contact_sec,
        safe_sec=max(0.,evaluated_sec-.01) if failure_reason else evaluated_sec,
        evaluated_sec=evaluated_sec,failure_penalty_sec=0. if is_continuous else max(0.,duration-evaluated_sec),
        is_failure=int(outcome in ('attacker_win','robot_failure')),is_attacker_loss=int(outcome=='attacker_loss'),
        first_failure_speed_ratio=min_ratio+(max_ratio-min_ratio)*min(evaluated_sec/ramp_sec,1.),
        proxy_contact_sec=proxy_contact_sec,internal_failure_sec=internal_failure_sec,stopped_sec=stopped_sec,
        min_proxy_gap_m=min_gap,first_contact_sec=(None if is_continuous else duration) if first_contact is None else first_contact,
        reference_eef_speed_mps=reference_speed,max_ball_speed_mps=max_ball_speed,max_eef_speed_mps=max_eef_speed,
        max_joint_speed_radps=max_joint_speed,propose_mean_ms=total_cost_ms/num_proposals if num_proposals else None,
        propose_p95_ms=float(np.percentile(costs,95)) if len(costs) else None,num_proposals=num_proposals,
        num_memory_contexts=len(memory.memory),num_memory_learning=memory.num_learning,
        learned_pursuit_gain=memory.gain,num_gng_nodes=0 if cbf is not None else len(graph['angles']),
        obstacle_blocked_sec=obstacle.blocked_sec,min_fixed_gap_m=min_actual_fixed_gap,
        num_obstacle_route_plans=obstacle.num_route_plans,obstacle_route_total_ms=obstacle.route_cost_ms,
        min_protected_gap_m=min_protected_gap,num_protected_contacts=num_protected_contacts,
        max_obstacle_acceleration_m_per_sec2=max_ball_acceleration,
        cbf_warmup_ms=cbf.warmup_ms if cbf is not None else 0.,num_qp_no_solution=cbf.num_no_solution if cbf is not None else 0,
        qp_no_solution_sec=phase_sec.get('qp_no_solution',0.),
        first_qp_no_solution_sec=cbf.first_no_solution_sec if cbf is not None and cbf.first_no_solution_sec is not None else (None if is_continuous else duration),
        max_num_consecutive_qp_no_solution=cbf.max_num_consecutive_no_solution if cbf is not None else 0,
        num_qp_recoveries=cbf.num_recoveries if cbf is not None else 0,
        num_qp_relaxed_solutions=cbf.num_relaxed_solutions if cbf is not None else 0,
        qp_constraint_recovery_sec=phase_sec.get('constraint_recovery',0.),
        num_coordinated_selections=rollout.num_coordinated_selections if rollout is not None else 0)
    metrics.update(startup_ms=startup_stages_ms['ready'],num_internal_refined_poses=internal_checker.num_refined_poses,
                   num_internal_shape_checks=internal_checker.num_shape_checks,
                   num_internal_distance_repairs=internal_checker.num_distance_repairs,
                   calibrated_reference_eef_speed_mps=calibrated_reference_speed,num_obstacle_route_recoveries=obstacle.num_route_recoveries)
    metrics.update(reference_prepare_ms=reference_stats['prepare_ms'],reference_planning_ms=reference_stats['planning_ms'],
        reference_path_length_rad=reference_stats['path_length_rad'],reference_num_checked_poses=reference_stats['num_checked_poses'],
        reference_dev_rms_rad=math.sqrt(reference_dev_squared/num_proposals) if num_proposals else None,
        max_reference_dev_rad=max_reference_dev_rad)
    metrics['obstacle_tip_blocked_sec'] = obstacle.tip_blocked_sec
    metrics.update(num_obstacle_configuration_plans=obstacle.num_configuration_plans,
        obstacle_configuration_plan_total_ms=obstacle.configuration_plan_ms)
    if cbf is not None:
        metrics.update({'num_qp_'+key+'_rejections':value for key,value in cbf.diagnostics['num_no_solution_by_reason'].items()})
        metrics['num_qp_numerical_recoveries'] = cbf.diagnostics['num_numerical_recoveries']
    if rollout is not None:
        metrics['vlut_query_mean_ms' if policy_name=='vlut_graph' else 'representative_query_mean_ms'] = node_query_total_ms/num_proposals if num_proposals else None
        metrics.update({name+'_mean_ms':value/num_proposals if num_proposals else None for name,value in getattr(rollout,'timing_totals_ms',{}).items()})
    if policy_name=='vlut_graph':
        metrics.update(num_vlut_obstacle_voxels=int(index.stats[0]),num_vlut_danger_voxels=int(index.stats[1]),
            num_vlut_obstacle_changes=int(index.stats[2]),num_vlut_danger_changes=int(index.stats[3]))
    num_evaluated = int(round(evaluated_sec/.01))
    metrics.update(num_arrivals_with_posture_dev=num_arrivals_with_posture_dev,
        task_position_rms_error_m=math.sqrt(position_error_squared/num_evaluated) if num_evaluated else None,
        task_orientation_rms_error_deg=math.degrees(math.sqrt(orientation_error_squared/num_evaluated)) if num_evaluated else None,
        max_task_position_error_m=max_task_position_error_m if num_evaluated else None,
        max_task_orientation_error_deg=math.degrees(max_task_orientation_error_rad) if num_evaluated else None)
    if obstacle.pursuit_mode == 'distal_sweep':
        metrics.update(num_obstacle_sweeps=obstacle.num_sweeps,num_completed_obstacle_sweeps=obstacle.num_completed_sweeps,
            num_sweep_links=len(obstacle.sweep_links),obstacle_sweep_total_ms=obstacle.sweep_cost_ms)
    if obstacle.pursuit_mode == 'random_walk':
        metrics['num_random_walk_goals'] = obstacle.num_random_walk_goals
    if not max_ball_speed <= reference_speed*max_ratio+1e-8:
        raise RuntimeError('障害物速度上限の超過')
    result = dict(config=config,metrics=metrics,phase_sec=phase_sec,failure_reason=failure_reason,failure_details=failure_details,
        drive_mode=drive_mode,
        collision_backend='continuous_cbf' if is_cbf else 'vlut' if policy_name=='vlut_graph' else 'representative_geometry',
        execution_mode=execution_mode,has_recording=not is_continuous,cost_window_size=len(costs),
        is_valid_scenario=outcome!='attacker_loss',outcome=outcome,contact_rule='target_tip',
        target_contact_links=sorted(obstacle.target_links),task=task_metadata,arrival_events=list(arrival_events),**motion_metadata,
        startup_stages_ms=startup_stages_ms,
        calibration_source=geometry.calibration_source,
        enable_periodic_angles=False,
        joint_position_limits=joint_position_limits,
        has_hardware_output=False,
        initial_center_m=initial_center.tolist(),target_link=target_link,target_offset_m=target_offset.tolist(),
        obstacle_shape=obstacle.shape,obstacle_joint_types=list(obstacle.joint_types),
        shoulder_position_m=obstacle.anchor.tolist() if obstacle.shape=='linked_arm' else None,
        cbf_diagnostics=cbf.diagnostics if cbf is not None else None,
        is_sphere_fully_observed=not(delay or noise),memory=memory.export(),
        source_sha256={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                       for name in ('sphere_chase.py','shuttle_trajectory.py','pursuit_obstacle.py','pursuit_cbf.py','gng_sphere_avoidance.py','gng_prediction.py','physics_scene.py','physics_robot.py','gng_avoidance.py')},
        input_sha256={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest()
                      for path in ([Path(params['urdf_path'])] if is_cbf else [Path(params['urdf_path']),directory/metadata['gng_file'],index.kernel_path]+
                                   ([directory/metadata['vlut_file']] if policy_name=='vlut_graph' else []))})
    if output_dir is not None:
        output_dir = Path(output_dir);output_dir.mkdir(parents=True,exist_ok=True)
        for name,value in (('metrics.json',metrics),('details.json',result),('memory.json',memory.export())):
            (output_dir/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        with (output_dir/'trajectory.jsonl').open('w') as stream:
            for row in rows:
                stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--case-dir',type=Path,required=True)
    parser.add_argument('--seed',type=int)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if args.seed is not None:
        config['seed'] = args.seed
    result = run_case(config,args.case_dir)
    print(json.dumps(result['metrics'],ensure_ascii=False,allow_nan=False),flush=True)


if __name__=='__main__':
    main()

