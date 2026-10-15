"""手先追跡球の閉ループMuJoCo実験。制御への真値・将来軌道の受渡しなし。"""
import argparse
from collections import deque
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


def make_scene(pose, names, active):
    locked = [name for idx,name in enumerate(names) if idx not in active]
    body = dict(id='pursuer',mode='kinematic',mass=.2,position=[3.,0.,3.],quaternion=[0,0,0,1],
                geoms=[dict(type='sphere',radius=.06,position=[0,0,0],quaternion=[0,0,0,1])])
    scene = PhysicsScene([body],dict(model='long',pose=dict(zip(names,map(float,pose))),
        position=[0,0,0],quaternion=[0,0,0,1],locked_joints=locked),dict(mode='none'))
    scene.robot.control_entries = [(name,idx,dof,min(limit,2.)) for name,idx,dof,limit in scene.robot.control_entries]
    return scene


def state(scene, names):
    pose, motion = scene.robot.state(), scene.robot.motion_state()
    return np.asarray([pose[name] for name in names]), np.asarray([motion['velocity'][name] for name in names])


def apply_target(scene, names, active, positions, velocity, target):
    desired = np.clip((target[active]-positions[active])/.1,-2.,2.)
    desired = np.clip(desired,velocity[active]-.4,velocity[active]+.4)
    command = positions.copy()
    command[active] += desired*.1
    scene.robot.set_targets(dict(zip(names,map(float,command))))


def calibrate(geometry, active, poses, duration_sec=8.):
    scene = make_scene(poses['a'],geometry.joint_names,active)
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


def run_case(config, output_dir=None):
    if output_dir is not None and any((Path(output_dir)/name).exists() for name in ('metrics.json','details.json','trajectory.jsonl')):
        raise ValueError('既存の試験結果への上書き禁止。新しい出力先の指定')
    policy_name = config.get('policy','rollout_cv')
    if policy_name not in ('avoidance_first','vrm_predictive','rollout_reactive','rollout_cv','rollout_sequence'):
        raise ValueError('未対応の球回避方式')
    duration = float(config.get('duration_sec',20.))
    if not 1 <= duration <= 180:
        raise ValueError('試験時間は1〜180 sec')
    seed = int(config.get('seed',11))
    radius = float(config.get('radius_m',.06))
    delay = float(config.get('observation_delay_sec',0.))
    noise = float(config.get('observation_noise_m',0.))
    if not .015 <= radius <= .15 or not 0 <= delay <= .2 or not 0 <= noise <= .01:
        raise ValueError('球・観測条件の範囲外')
    min_ratio, max_ratio = config.get('speed_ratios',[.2,.8])
    if not 0 < min_ratio <= max_ratio < 1:
        raise ValueError('球速度比は0〜1の範囲内')
    if not 0 < float(config.get('max_ball_acceleration',.8)) <= 5. or not 0 <= float(config.get('pursuit_delay_sec',0.)) <= .5 or not .25 <= float(config.get('start_dist_m',.45)) <= 2.:
        raise ValueError('追跡球の加速度・追従遅延・開始距離の範囲外')
    params, models = load_models(workspace/'gng_vlut_system/config/topo_dual_arm_max_long.yaml')
    check_kinematics(params['urdf_path'],'long')
    geometry = robot_geometry(params['urdf_path'],environment_shapes=())
    profiles = independent_policy(geometry,models)
    arm = profiles.policies['left_arm']
    geometry.is_arm = arm.geometry.is_arm.copy()
    active = np.asarray(arm.arm_indices)
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
    scene = make_scene(poses['a'],geometry.joint_names,active)
    sphere_geom = next(idx for idx in range(scene.model.ngeom)
        if scene.model.geom_bodyid[idx]==scene.body_ids['pursuer'])
    scene.model.geom_size[sphere_geom,0] = radius
    eef_body = scene.mujoco.mj_name2id(scene.model,scene.mujoco.mjtObj.mjOBJ_BODY,'robot_L_tcp')
    eef = scene.data.xpos[eef_body].copy()
    generator = np.random.default_rng(seed)
    initial = config.get('initial_direction')
    for attempt in range(64):
        direction = np.asarray(initial,dtype=float) if initial is not None and attempt==0 else generator.normal(size=3)
        direction[2] = abs(direction[2])*.5
        direction /= max(float(np.linalg.norm(direction)),1e-9)
        center = eef+direction*float(config.get('start_dist_m',.45))
        gap = np.min(np.linalg.norm(geometry.centers(poses['a'])-center,axis=1)-geometry.radii-radius)
        if gap>.12 and center[2]>radius:
            break
    else:
        raise ValueError('非接触の球開始位置なし')
    initial_center = center.copy()
    metadata = models['left_arm']
    directory = Path(params['gng']['data_directory'])/params['gng']['experiment_id']/'left_arm'
    graph = load_graph(directory/metadata['gng_file'])
    index = representative_index(geometry,active,poses['a'],graph)
    memory = motion_sequence_predictor(enable_learning=config.get('enable_learning',True))
    if config.get('memory_file'):
        memory.restore(json.loads(Path(config['memory_file']).read_text()))
    rollout = sphere_rollout_policy(geometry,active,'L_tcp',graph,
        mode=dict(rollout_reactive='reactive',rollout_cv='constant_velocity',rollout_sequence='sequence').get(policy_name,'reactive'),
        candidate_metric=config.get('candidate_metric','joint'),memory=memory,node_index=index,
        horizon_sec=float(config.get('horizon_sec',.4)),margin=float(config.get('margin_m',.08)),
        task_weight=float(config.get('task_weight',.12)))
    legacy = independent_policy(geometry,{'left_arm':metadata},strategy='vrm_predictive' if policy_name=='vrm_predictive' else 'avoidance_first').policies['left_arm']
    legacy.clock_sec = lambda:float(scene.data.time)
    legacy.positions = poses['a'].copy()
    legacy.config.update(max_bridge_step=.025,min_clearance_th=.01,min_planning_clearance_th=.005,
        trigger_clearance_th=.10,release_clearance_th=.14,return_clear_sec=.5,max_state_age_sec=.3,max_local_step=.12)
    pending, eef_history = deque(),deque()
    ball_velocity = np.zeros(3)
    goal, arrivals, first_contact = 'b',0,None
    contact_sec = proxy_contact_sec = internal_failure_sec = stopped_sec = 0.
    min_gap = 1e6
    max_ball_speed = max_eef_speed = max_joint_speed = 0.
    costs = []
    rows = []
    observed = None
    target = poses['a'].copy()
    phase = 'waiting'
    last_eef = eef.copy()
    phase_sec = {}
    failure_reason = ''
    num_internal_frames = 0
    evaluated_sec = 0.
    for frame in range(int(round(duration/.01))):
        now = frame*.01
        positions,velocity = state(scene,geometry.joint_names)
        actual_eef = scene.data.xpos[eef_body].copy()
        eef_history.append((now,actual_eef.copy()))
        follow_delay = float(config.get('pursuit_delay_sec',0.))
        while len(eef_history)>1 and eef_history[1][0] <= now-follow_delay:
            eef_history.popleft()
        followed = eef_history[0][1]
        direction = followed-center
        direction /= max(float(np.linalg.norm(direction)),1e-9)
        if config.get('enable_sidestep',False):
            direction += .35*np.sin(2.1*now)*np.cross(direction,[0.,0.,1.])
            direction /= max(float(np.linalg.norm(direction)),1e-9)
        speed = reference_speed*(min_ratio+(max_ratio-min_ratio)*min(now/duration,1.))
        desired = direction*speed
        change = desired-ball_velocity
        ball_velocity += change*min(1.,float(config.get('max_ball_acceleration',.8))*.01/max(float(np.linalg.norm(change)),1e-9))
        ball_velocity *= min(1.,speed/max(float(np.linalg.norm(ball_velocity)),1e-9))
        center += ball_velocity*.01
        max_ball_speed = max(max_ball_speed,float(np.linalg.norm(ball_velocity)))
        scene.move([dict(id='pursuer',position=center.tolist(),quaternion=[0,0,0,1])])
        if frame%5==0:
            packet = dict(center=center+generator.normal(0.,noise,3),stamp=now,eef=actual_eef.copy())
            pending.append((now+delay,packet))
        while pending and pending[0][0] <= now+1e-9:
            observed = pending.popleft()[1]
        if frame%5==0 and observed is not None:
            begin = time.perf_counter()
            # 遅延による不確かさの解析的な球半径への集約。方式間で共通の現観測。
            field = sphere_field(observed['center'],radius+reference_speed*max_ratio*(now-observed['stamp'])+3*noise)
            field.stamp_sec,field.eef_position = observed['stamp'],observed['eef']
            index.update(field.center,field.radius)
            if policy_name.startswith('rollout_'):
                target = rollout.propose(positions,velocity,poses[goal],field,observed['stamp'])
                phase = rollout.phase
            else:
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
            costs.append((time.perf_counter()-begin)*1000)
        scene.step()
        positions,velocity = state(scene,geometry.joint_names)
        actual_eef = scene.data.xpos[eef_body].copy()
        max_eef_speed = max(max_eef_speed,float(np.linalg.norm(actual_eef-last_eef)/.01))
        last_eef = actual_eef
        max_joint_speed = max(max_joint_speed,float(np.max(np.abs(velocity[active]))))
        actual_centers = geometry.centers(positions)
        gap = float(np.min(np.linalg.norm(actual_centers-center,axis=1)-geometry.radii-radius))
        min_gap = min(min_gap,gap)
        has_contact = any((contact.geom1==sphere_geom and contact.geom2!=sphere_geom) or
                          (contact.geom2==sphere_geom and contact.geom1!=sphere_geom)
                          for contact in scene.data.contact[:scene.data.ncon])
        contact_sec += .01*has_contact
        proxy_contact_sec += .01*(gap<0)
        is_internal_clear = bool(geometry.has_internal_clearance(actual_centers,.005))
        internal_failure_sec += .01*(not is_internal_clear)
        num_internal_frames = 0 if is_internal_clear else num_internal_frames+1
        if has_contact and first_contact is None:
            first_contact = now+.01
        stopped_sec += .01*(np.max(np.abs(velocity[active]))<.03)
        phase_sec[phase] = phase_sec.get(phase,0.)+.01
        goal_error = float(np.max(np.abs(positions[active]-poses[goal][active])))
        # 接触後の押し出し・内部干渉・異常運動を成功に含めない共通打切り条件。
        if has_contact:
            failure_reason = 'physical_contact'
        elif num_internal_frames >= 3:
            failure_reason = 'internal_interference'
        elif np.max(np.abs(velocity[active])) > 6.:
            failure_reason = 'abnormal_joint_speed'
        elif np.any(positions<geometry.limits[:,0]-.05) or np.any(positions>geometry.limits[:,1]+.05):
            failure_reason = 'joint_limit'
        evaluated_sec = now+.01
        if not failure_reason and goal_error<.035 and np.max(np.abs(velocity[active]))<.12:
            arrivals += 1
            goal = 'a' if goal=='b' else 'b'
        if frame%5==0 or failure_reason:
            rows.append(dict(time_sec=evaluated_sec,joints=dict(zip(geometry.joint_names,map(float,positions))),
                eef=actual_eef.tolist(),sphere=center.tolist(),sphere_radius_m=radius,phase=phase,
                has_contact=bool(has_contact),proxy_gap_m=gap))
        if not np.isfinite(positions).all():
            raise RuntimeError('非有限の実測関節状態')
        if failure_reason:
            break
    metrics = dict(num_arrivals=arrivals,num_round_trips=arrivals//2,contact_sec=contact_sec,
        safe_sec=max(0.,evaluated_sec-.01) if failure_reason else duration,
        evaluated_sec=min(duration,evaluated_sec),failure_penalty_sec=max(0.,duration-evaluated_sec),
        is_failure=int(bool(failure_reason)),
        first_failure_speed_ratio=min_ratio+(max_ratio-min_ratio)*min(evaluated_sec/duration,1.),
        proxy_contact_sec=proxy_contact_sec,internal_failure_sec=internal_failure_sec,stopped_sec=stopped_sec,
        min_proxy_gap_m=min_gap,first_contact_sec=duration if first_contact is None else first_contact,
        reference_eef_speed_mps=reference_speed,max_ball_speed_mps=max_ball_speed,max_eef_speed_mps=max_eef_speed,
        max_joint_speed_radps=max_joint_speed,propose_mean_ms=float(np.mean(costs)),
        propose_p95_ms=float(np.percentile(costs,95)),num_proposals=len(costs),
        num_memory_contexts=len(memory.memory),num_memory_learning=memory.num_learning,
        learned_pursuit_gain=memory.gain,num_gng_nodes=len(graph['angles']))
    if not max_ball_speed <= reference_speed*max_ratio+1e-8:
        raise RuntimeError('障害物速度上限の超過')
    result = dict(config=config,metrics=metrics,phase_sec=phase_sec,failure_reason=failure_reason,
        has_hardware_output=False,
        initial_center_m=initial_center.tolist(),
        is_sphere_fully_observed=not(delay or noise),memory=memory.export(),
        source_sha256={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                       for name in ('sphere_chase.py','gng_sphere_avoidance.py','gng_prediction.py','physics_scene.py','physics_robot.py','gng_avoidance.py')},
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

