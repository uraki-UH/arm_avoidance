"""指定リンク追跡障害物の閉ループMuJoCo実験。制御への真値・将来軌道の受渡しなし。"""
import argparse
from collections import deque
from itertools import count
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from gng_inputs import workspace, load_models
from gng_avoidance import independent_policy, check_kinematics
from gng_sphere_avoidance import (load_graph, motion_sequence_predictor, representative_index,
                                  sphere_field, sphere_rollout_policy)
from physics_scene import PhysicsScene
from dual_arm_avoidance_geometry import robot_geometry
from pursuit_obstacle import pursuit_obstacle, capsule_field
from pursuit_cbf import pursuit_cbf, angle_delta


def make_scene(pose, names, active, bodies=None, position_limits=None):
    locked = [name for idx,name in enumerate(names) if idx not in active]
    body = dict(id='pursuer',mode='kinematic',mass=.2,position=[3.,0.,3.],quaternion=[0,0,0,1],
                geoms=[dict(type='sphere',radius=.06,position=[0,0,0],quaternion=[0,0,0,1])])
    scene = PhysicsScene([body] if bodies is None else bodies,dict(model='long',pose=dict(zip(names,map(float,pose))),
        position=[0,0,0],quaternion=[0,0,0,1],locked_joints=locked),dict(mode='none'))
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


def apply_target(scene, names, active, positions, velocity, target):
    desired = np.clip((target[active]-positions[active])/.1,-2.,2.)
    limits = scene.model.jnt_range[[scene.robot.ids[names[idx]] for idx in active]]
    # 可動域手前の減速と目標飽和。物理関節ストッパーとは独立の指令制限。
    lower = np.maximum(positions[active]-limits[:,0]-.05,0.)
    upper = np.maximum(limits[:,1]-positions[active]-.05,0.)
    desired = np.clip(desired,-np.minimum(3*lower,np.sqrt(16*lower)),np.minimum(3*upper,np.sqrt(16*upper)))
    desired = np.clip(desired,velocity[active]-.4,velocity[active]+.4)
    command = positions.copy()
    command[active] = np.clip(positions[active]+desired*.1,limits[:,0],limits[:,1])
    scene.robot.set_targets(dict(zip(names,map(float,command))))


def calibrate(geometry, active, poses, duration_sec=8.):
    scene = make_scene(poses['a'],geometry.joint_names,active,position_limits=geometry.limits)
    body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_L_tcp')
    previous = scene.data.xpos[body].copy()
    max_speed = 0.
    for frame in range(int(duration_sec/.01)):
        positions, velocity = state(scene,geometry.joint_names)
        target = poses['b' if int(frame*.01/2)%2==0 else 'a']
        if frame%5==0:
            apply_target(scene,geometry.joint_names,active,positions,velocity,target)
        scene.step()
        current = scene.data.xpos[body].copy()
        if frame > 20:
            max_speed = max(max_speed,float(np.linalg.norm(current-previous)/.01))
        previous = current
    if not np.isfinite(max_speed) or max_speed < .01:
        raise ValueError('手先速度の無障害校正の失敗')
    return max_speed


def run_case(config, output_dir=None, emit=None, stop_event=None):
    execution_mode = config.get('execution_mode','recorded')
    if execution_mode not in ('recorded','continuous'):
        raise ValueError('未対応の実行方式')
    is_continuous = execution_mode=='continuous'
    if is_continuous and output_dir is not None:
        raise ValueError('連続実験は記録なし。結果保存先の併用不可')
    if output_dir is not None and any((Path(output_dir)/name).exists() for name in ('metrics.json','details.json','trajectory.jsonl')):
        raise ValueError('既存の試験結果への上書き禁止。新しい出力先の指定')
    policy_name = config.get('policy','rollout_cv')
    if policy_name not in ('avoidance_first','vrm_predictive','rollout_reactive','rollout_cv','rollout_sequence','cbf_qp','oscbf'):
        raise ValueError('未対応の球回避方式')
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
    params, models = load_models(workspace/'gng_vlut_system/config/topo_dual_arm_max_long.yaml')
    check_kinematics(params['urdf_path'],'long')
    geometry = robot_geometry(params['urdf_path'],environment_shapes=())
    profiles = independent_policy(geometry,models)
    arm = profiles.policies['left_arm']
    geometry.is_arm = arm.geometry.is_arm.copy()
    active = np.asarray(arm.arm_indices)
    geometry.active_joint_names = tuple(arm.arm_names)
    if type(config.get('enable_periodic_angles',False)) is not bool:
        raise ValueError('連続関節の周期扱いは真偽値')
    joint_limit_sources = configure_position_limits(geometry,config)
    target_link = config.get('target_link','L_tcp')
    if target_link not in geometry.link_indices:
        raise ValueError(f'追跡対象リンク名の不正: {target_link}')
    target_offset = np.asarray(config.get('target_offset_m',[0.,0.,0.]),dtype=float)
    if target_offset.shape != (3,) or not np.isfinite(target_offset).all() or np.linalg.norm(target_offset)>.5:
        raise ValueError('追跡対象のリンク内オフセットの不正')
    task = json.loads((workspace/'benchmarks/shuttle_bag/task.json').read_text())
    poses = {key:np.asarray([math.radians(value.get(name,0.)) for name in geometry.joint_names])
             for key,value in task['poses_deg'].items()}
    offset_deg = config.get('goal_offset_deg',{})
    for name,value in offset_deg.items():
        if name not in arm.arm_names or not np.isfinite(value) or abs(value)>15:
            raise ValueError('目標変動の指定不正')
        poses['b'][geometry.joint_names.index(name)] += math.radians(value)
    for pose in poses.values():
        if np.any(pose<geometry.limits[:,0]) or np.any(pose>geometry.limits[:,1]) or not geometry.has_internal_clearance(geometry.centers(pose),.005):
            raise ValueError('開始・目標姿勢の位置制限または自己干渉')
    reference_speed = calibrate(geometry,active,poses)
    transform = geometry.link_transforms(poses['a'])[geometry.link_indices[target_link]]
    followed = transform[:3,3]+transform[:3,:3]@target_offset
    generator = np.random.default_rng(seed)
    initial = config.get('initial_direction')
    if initial is not None and (np.asarray(initial).shape!=(3,) or not np.isfinite(initial).all() or np.linalg.norm(initial)<1e-8):
        raise ValueError('開始方向は非ゼロの有限3次元ベクトル')
    for attempt in range(64):
        direction = np.asarray(initial,dtype=float) if initial is not None and attempt==0 else generator.normal(size=3)
        direction[2] = abs(direction[2])*.5
        direction /= max(float(np.linalg.norm(direction)),1e-9)
        center = followed+direction*float(config.get('start_dist_m',.45))
        gap = np.min(np.linalg.norm(geometry.centers(poses['a'])-center,axis=1)-geometry.radii-radius)
        if gap>.12 and center[2]>radius:
            try:
                obstacle = pursuit_obstacle({**config,'target_link':target_link},center,direction,geometry,poses['a'])
            except ValueError as error:
                if '開始位置' not in str(error):
                    raise
                continue
            if np.min(obstacle.field().query(geometry.centers(poses['a']))[0]-geometry.radii)>.12:
                break
    else:
        raise ValueError('非接触の球開始位置なし')
    initial_center = obstacle.center.copy()
    scene = make_scene(poses['a'],geometry.joint_names,active,obstacle.bodies(),geometry.limits)
    obstacle.move_scene(scene)
    obstacle_geoms = {idx for idx in range(scene.model.ngeom) if scene.model.geom_bodyid[idx] in set(scene.body_ids.values())}
    obstacle_indices = {idx:int(scene.mujoco.mj_id2name(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,int(scene.model.geom_bodyid[idx])).removeprefix('body_pursuer_'))
                        for idx in ()}
    for idx in obstacle_geoms:
        body_id = int(scene.model.geom_bodyid[idx])
        obstacle_indices[idx] = next(int(name.removeprefix('pursuer_')) for name,value in scene.body_ids.items() if value==body_id)
    eef_body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_L_tcp')
    target_body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_'+target_link)
    anchor_bodies = [scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_'+name) for name in obstacle.guard_anchor_links]
    protected_links = {geometry.spheres[idx][0] for idx in np.flatnonzero(obstacle.protected_mask)}
    eef = scene.data.xpos[eef_body].copy()
    metadata = models['left_arm']
    directory = Path(params['gng']['data_directory'])/params['gng']['experiment_id']/'left_arm'
    graph = load_graph(directory/metadata['gng_file'])
    index = representative_index(geometry,active,poses['a'],graph)
    memory = motion_sequence_predictor(enable_learning=config.get('enable_learning',True))
    if config.get('memory_file'):
        memory.restore(json.loads(Path(config['memory_file']).read_text()))
    rollout = sphere_rollout_policy(geometry,active,target_link,graph,
        mode=dict(rollout_reactive='reactive',rollout_cv='constant_velocity',rollout_sequence='sequence').get(policy_name,'reactive'),
        candidate_metric=config.get('candidate_metric','joint'),memory=memory,node_index=index,
        horizon_sec=float(config.get('horizon_sec',.4)),margin=float(config.get('margin_m',.08)),
        task_weight=float(config.get('task_weight',.12)),target_offset=target_offset)
    if is_continuous:
        rollout.samples_ms = deque(maxlen=1200)
    cbf = pursuit_cbf(geometry,active,scene,policy_name,float(config.get('margin_m',.08))) if policy_name in ('cbf_qp','oscbf') else None
    legacy = independent_policy(geometry,{'left_arm':metadata},strategy='vrm_predictive' if policy_name=='vrm_predictive' else 'avoidance_first').policies['left_arm']
    legacy.clock_sec = lambda:float(scene.data.time)
    legacy.positions = poses['a'].copy()
    legacy.config.update(max_bridge_step=.025,min_clearance_th=.01,min_planning_clearance_th=.005,
        trigger_clearance_th=.10,release_clearance_th=.14,return_clear_sec=.5,max_state_age_sec=.3,max_local_step=.12)
    pending, eef_history = deque(),deque()
    goal, arrivals, first_contact = 'b',0,None
    contact_sec = proxy_contact_sec = internal_failure_sec = stopped_sec = 0.
    min_gap = 1e6
    max_ball_speed = max_eef_speed = max_joint_speed = 0.
    costs = deque(maxlen=1200) if is_continuous else []
    num_proposals = 0
    total_cost_ms = 0.
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
            contact_rule='target_tip',target_contact_links=sorted(obstacle.target_links),outcome=outcome))))
    num_frames = None if is_continuous else int(round(duration/.01))
    for frame in count():
        if (num_frames is not None and frame>=num_frames) or (stop_event is not None and stop_event.is_set()):
            break
        frame_started_sec = time.monotonic() if is_continuous else None
        now = frame*.01
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
                target = cbf.propose(positions,velocity,poses[goal],field,observed['stamp'])
                phase = cbf.phase
            elif policy_name.startswith('rollout_'):
                index.update_field(field)
                target = rollout.propose(positions,velocity,poses[goal],field,observed['stamp'])
                phase = rollout.phase
            else:
                index.update_field(field)
                legacy.positions,legacy.home = positions.copy(),poses[goal].copy()
                legacy.cloud_tree,legacy.cell_radius = field,0.
                for name in ('angles','adjacency','angle_node_ids','angle_tree','labels'):
                    setattr(legacy,name,graph[name])
                if policy_name=='vrm_predictive':
                    legacy.observe((field,0.,observed['stamp']),graph)
                    target,can_move = legacy.select_waypoint()
                elif legacy.can_follow_task():
                    target,can_move = poses[goal].copy(),True
                    legacy.motion_phase = 'task'
                else:
                    target,can_move = legacy.select_retreat()
                phase = legacy.motion_phase if can_move else 'holding'
            apply_target(scene,geometry.joint_names,active,positions,velocity,target)
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
            contact_details.append(dict(link=link,is_protected=link in protected_links,
                obstacle_idx=obstacle_indices[obstacle_geom],position=contact.pos.tolist()))
        has_protected_contact = any(name in protected_links for name in contact_links)
        num_protected_contacts += int(has_protected_contact)
        contact_sec += .01*has_contact
        proxy_contact_sec += .01*(gap<0)
        is_internal_clear = bool(geometry.has_internal_clearance(actual_centers,.005))
        internal_failure_sec += .01*(not is_internal_clear)
        num_internal_frames = 0 if is_internal_clear else num_internal_frames+1
        if has_contact and first_contact is None:
            first_contact = now+.01
        stopped_sec += .01*(np.max(np.abs(velocity[active]))<.03)
        phase_sec[phase] = phase_sec.get(phase,0.)+.01
        goal_error = float(np.max(np.abs(angle_delta(poses[goal],positions,geometry.is_periodic)[active])))
        # 接触後の押し出し・内部干渉・異常運動を成功に含めない共通打切り条件。
        limit_reason,limit_joints = position_limit_status(positions,geometry.joint_names,geometry.limits)
        if has_contact:
            failure_reason = obstacle.contact_result(contact_details)
        elif num_internal_frames >= 3:
            failure_reason = 'internal_interference'
        elif np.max(np.abs(velocity[active])) > 6.:
            failure_reason = 'abnormal_joint_speed'
        elif limit_reason:
            failure_reason = limit_reason
        if failure_reason:
            outcome = 'attacker_loss' if failure_reason.startswith('attacker_') else 'attacker_win' if failure_reason=='target_touch' else 'robot_failure'
            failure_details = dict(contact_links=sorted(set(contact_links)),has_protected_contact=has_protected_contact,contacts=contact_details)
            if failure_reason in ('physical_joint_limit','planning_domain_exit'):
                failure_details['joints'] = limit_joints
        evaluated_sec = now+.01
        if not failure_reason and goal_error<.035 and np.max(np.abs(velocity[active]))<.12:
            arrivals += 1
            goal = 'a' if goal=='b' else 'b'
        if frame%5==0 or failure_reason or (num_frames is not None and frame==num_frames-1):
            row = dict(time_sec=evaluated_sec,joints=dict(zip(geometry.joint_names,map(float,positions))),
                eef=actual_eef.tolist(),sphere=center.tolist(),sphere_radius_m=radius,phase=phase,
                target=(scene.data.xpos[target_body]+scene.data.xmat[target_body].reshape(3,3)@target_offset).tolist(),
                has_contact=bool(has_contact),proxy_gap_m=gap,obstacle_phase=obstacle.phase,obstacle_speed_mps=achieved_speed)
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
        num_qp_recoveries=cbf.num_recoveries if cbf is not None else 0)
    if not max_ball_speed <= reference_speed*max_ratio+1e-8:
        raise RuntimeError('障害物速度上限の超過')
    result = dict(config=config,metrics=metrics,phase_sec=phase_sec,failure_reason=failure_reason,failure_details=failure_details,
        execution_mode=execution_mode,has_recording=not is_continuous,cost_window_size=len(costs),
        is_valid_scenario=outcome!='attacker_loss',outcome=outcome,contact_rule='target_tip',
        target_contact_links=sorted(obstacle.target_links),
        enable_periodic_angles=False,
        joint_position_limits=joint_position_limits,
        has_hardware_output=False,
        initial_center_m=initial_center.tolist(),target_link=target_link,target_offset_m=target_offset.tolist(),
        obstacle_shape=obstacle.shape,obstacle_joint_types=list(obstacle.joint_types),
        shoulder_position_m=obstacle.anchor.tolist() if obstacle.shape=='linked_arm' else None,
        cbf_diagnostics=cbf.diagnostics if cbf is not None else None,
        is_sphere_fully_observed=not(delay or noise),memory=memory.export(),
        source_sha256={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                       for name in ('sphere_chase.py','pursuit_obstacle.py','pursuit_cbf.py','gng_sphere_avoidance.py','gng_prediction.py','physics_scene.py','physics_robot.py','gng_avoidance.py')},
        input_sha256={str(path):hashlib.sha256(Path(path).read_bytes()).hexdigest()
                      for path in (Path(params['urdf_path']),directory/metadata['gng_file'],index.kernel_path)})
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

