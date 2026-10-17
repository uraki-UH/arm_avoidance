"""球・二節カプセルの追跡生成と共通観測。固定部回避は障害物側だけの経路条件。"""
from itertools import product
import time

import numpy as np
from scipy.optimize import minimize

from gng_sphere_avoidance import sphere_field, unit_vector


def shoulder_anchor_candidates(targets, wrist, direction, lengths):
    """初期手首・往復対象点を含む固定肩位置。試験中の肩移動なし。"""
    points = np.vstack((targets, wrist))
    center = np.mean(np.asarray(targets),axis=0)
    outward = unit_vector(np.r_[np.asarray(direction)[:2],0.])
    if np.linalg.norm(outward)<.5:
        outward = np.array([1.,0.,0.])
    lateral = np.cross([0.,0.,1.],outward)
    candidates = np.array([center+(np.cos(angle)*outward+np.sin(angle)*lateral)*offset+[0.,0.,height]
                           for angle in (0.,np.pi/3,-np.pi/3,2*np.pi/3,-2*np.pi/3,np.pi)
                           for offset in (.35,.45,.25) for height in (.12,.24,0.)])
    radii = np.max(np.linalg.norm(candidates[:,None]-points,axis=-1),axis=1)
    costs = radii+.1*np.linalg.norm(candidates-(center+outward*.4+[0.,0.,.12]),axis=1)
    can_reach = radii<np.sum(lengths)-.04
    order = np.argsort(costs)
    if np.any(can_reach):
        order = order[can_reach[order]]
    return candidates[order[:24]]


def shoulder_anchor(targets, wrist, direction, lengths):
    """往復対象を含む肩候補の先頭。既存CLIとの共通入口。"""
    return shoulder_anchor_candidates(targets,wrist,direction,lengths)[0]


def segment_nearest(points, segments):
    points, segments = np.asarray(points), np.asarray(segments)
    start, delta = segments[:, 0], segments[:, 1]-segments[:, 0]
    fraction = np.sum((points[..., None, :]-start)*delta, axis=-1)/np.maximum(np.sum(delta*delta, axis=-1), 1e-12)
    return start+np.clip(fraction, 0., 1.)[..., None]*delta


class capsule_field:
    """二節の観測カプセル。最近点照合・観測速度による端点の一次予測。"""
    def __init__(self, segments, radius, endpoint_velocity=None):
        self.segments, self.radius = np.asarray(segments, dtype=float), float(radius)
        self.endpoint_velocity = np.zeros((2, 2, 3)) if endpoint_velocity is None else np.asarray(endpoint_velocity, dtype=float)
        if self.segments.shape != (2, 2, 3) or self.endpoint_velocity.shape != (2, 2, 3) or not np.isfinite([self.segments, self.endpoint_velocity]).all() or not np.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('観測カプセルの形式不正')
        self.center = self.segments[-1, -1].copy()
        self.data = self.segments.reshape(-1, 3)
        self.n = len(self.data)

    def query(self, points, **kwargs):
        nearest = segment_nearest(points, self.segments)
        gaps = np.linalg.norm(np.asarray(points)[..., None, :]-nearest, axis=-1)-self.radius
        return np.min(gaps, axis=-1), np.argmin(gaps, axis=-1)

    def trajectory_gaps(self, points, times):
        # 時刻軸と端点軸だけの一括照合。候補ごとのボクセル登録なし。
        result = []
        for idx, period in enumerate(times):
            segments = self.segments+self.endpoint_velocity*period
            nearest = segment_nearest(points[:, idx], segments)
            result.append(np.min(np.linalg.norm(points[:, idx, :, None]-nearest, axis=-1), axis=-1)-self.radius)
        return np.stack(result, axis=1)

    def covering_spheres(self):
        centers, radii = [], []
        for start, end in self.segments:
            length = float(np.linalg.norm(end-start))
            num = max(2, int(np.ceil(length/.08))+1)
            centers.extend(start+np.linspace(0., 1., num)[:, None]*(end-start))
            radii.extend([np.hypot(self.radius, length/(2*(num-1)))]*num)
        return zip(centers, radii)


class pursuit_obstacle:
    """移動リンクを速度制限内で追跡する球・球関節二節腕。固定部全体の幾何ガード。"""
    def __init__(self, config, center, direction, geometry, positions, guard_geometry=None):
        self.guard_box_links = None
        self.shape = config.get('obstacle_shape', 'sphere')
        if self.shape not in ('sphere', 'linked_arm'):
            raise ValueError('障害物形状はsphereまたはlinked_arm')
        self.pursuit_mode = config.get('pursuit_mode', 'link_point')
        if self.pursuit_mode not in ('link_point', 'distal_sweep', 'random_walk'):
            raise ValueError('追跡方式の不正')
        if self.pursuit_mode == 'distal_sweep' and (self.shape != 'sphere' or config.get('pursuit_delay_sec', 0.) != 0. or np.any(config.get('target_offset_m', [0., 0., 0.]))):
            raise ValueError('横切り方式は球・追従遅延なし・追跡点オフセットなし')
        if self.pursuit_mode == 'random_walk' and (self.shape != 'sphere' or config.get('pursuit_delay_sec',0.) != 0. or np.any(config.get('target_offset_m',[0.,0.,0.]))):
            raise ValueError('ランダムウォークは球・追従遅延なし・追跡点オフセットなし')
        self.random_walk_extent_m = config.get('random_walk_extent_m',.6)
        self.random_walk_change_sec = config.get('random_walk_change_sec',.6)
        if self.pursuit_mode == 'random_walk' and (type(self.random_walk_extent_m) not in (int,float) or not .3 <= self.random_walk_extent_m <= 1.5 or type(self.random_walk_change_sec) not in (int,float) or not .1 <= self.random_walk_change_sec <= 5.):
            raise ValueError('ランダムウォークの範囲・更新間隔の不正')
        self.joint_types = ('ball','ball') if self.shape=='linked_arm' else ()
        self.radius = float(config.get('radius_m', .06))
        self.max_deceleration = float(config.get('max_ball_deceleration', config.get('max_ball_acceleration', .8)))
        self.max_turn_acceleration = float(config.get('max_ball_turn_acceleration', config.get('max_ball_acceleration', .8)))
        self.enable_separate_response = 'max_ball_deceleration' in config or 'max_ball_turn_acceleration' in config
        if not np.isfinite([self.max_deceleration, self.max_turn_acceleration]).all() or not 0 < self.max_deceleration <= 30 or not 0 < self.max_turn_acceleration <= 30:
            raise ValueError('減速・方向転換加速度の範囲外')
        # 迂回条件の単一スイッチ。旧設定名は入力時だけの互換受付。
        self.enable_proximal_guard = config.get('enable_proximal_guard', config.get('enable_fixed_guard', True))
        if type(self.enable_proximal_guard) is not bool:
            raise ValueError('近位部の迂回条件は真偽値')
        self.margin = 0.
        self.lengths = np.asarray(config.get('link_lengths_m', [.35, .35]), dtype=float)
        self.anchor = np.asarray(config.get('shoulder_position_m', np.asarray(center)-direction*.12+[0., 0., .15]), dtype=float)
        if self.anchor.shape != (3,) or not np.isfinite(self.anchor).all() or self.lengths.shape != (2,) or not np.isfinite(self.lengths).all() or np.any(self.lengths < .15) or np.any(self.lengths > .6):
            raise ValueError('追跡腕の肩位置・リンク寸法の不正')
        self.fixed_centers = geometry.centers(positions)[~geometry.is_arm]
        self.fixed_radii = geometry.radii[~geometry.is_arm]
        self.fixed_shape_radii = self.fixed_radii.copy()
        self.fixed_mask = ~geometry.is_arm.copy()
        self.protected_mask = self.fixed_mask.copy()
        self.guard_anchor_links, self.guard_anchor_radii = [], []
        self.target_links = {config.get('target_link','L_tcp')}
        self.target_mask = np.zeros(len(geometry.radii),dtype=bool)
        self.target_centers = np.empty((0,3))
        self.target_contact_centers = None
        if hasattr(geometry, 'joints'):
            active_names = set(getattr(geometry, 'active_joint_names', ()))
            parents = {joint[2]: joint for joint in geometry.joints}
            def motion_root(name):
                while name in parents and parents[name][0] not in active_names:
                    name = parents[name][1]
                return name
            target_root = motion_root(config.get('target_link','L_tcp'))
            self.target_links = {name for name in geometry.link_indices if motion_root(name)==target_root}
            if self.pursuit_mode in ('distal_sweep','random_walk'):
                active_roots = {joint[2] for joint in geometry.joints if joint[0] in active_names}
                if target_root not in active_roots:
                    raise ValueError('通過開始リンクは制御対象腕の可動リンク')
                # 可動剛体を境界とする末端側の部分木。肩側カバーの誤包含なし。
                def is_distal(name):
                    while name != target_root and name in parents:
                        name = parents[name][1]
                    return name == target_root
                self.target_links = {name for name in geometry.link_indices if is_distal(name)}
            self.target_mask = np.array([name in self.target_links for name,_,_ in geometry.spheres])
            proximal_roots, link = set(), target_root
            while link in parents:
                link = parents[link][1]
                proximal_roots.add(motion_root(link))
            if 'target_link' in config:
                # 指定リンクより近位の運動剛体群。固定結合のカバーも同一群。
                self.protected_mask |= np.array([motion_root(name) in proximal_roots for name,_,_ in geometry.spheres])
                self.protected_mask &= np.array([name not in self.target_links for name,_,_ in geometry.spheres])
                # RRT・局所移動の禁止形状は保護リンクだけ。許可リンクの実接触判定とは独立。
                self.fixed_mask = self.protected_mask.copy()
                self.fixed_shape_radii = geometry.radii[self.fixed_mask].copy()
            radii_by_anchor = {}
            for idx,(name,point,radius) in enumerate(geometry.spheres):
                if 'target_link' in config:
                    continue
                chain, link = [], name
                while link in parents:
                    joint = parents[link];chain.append(joint);link = joint[1]
                active_joints = [joint for joint in chain if joint[0] in active_names]
                if not 1 <= len(active_joints) <= 2:
                    continue
                self.protected_mask[idx] = True
                anchor = active_joints[-1][2]
                # 近位2関節の全回転を含む外接球。姿勢ごとの肩形状への置換なし。
                bound = float(np.linalg.norm(point))+radius
                for joint in chain:
                    if joint[2] == anchor:
                        break
                    bound += float(np.linalg.norm(joint[4][:3,3]))
                radii_by_anchor[anchor] = max(radii_by_anchor.get(anchor, 0.),bound)
            self.guard_anchor_links = list(radii_by_anchor)
            self.guard_anchor_radii = np.asarray(list(radii_by_anchor.values()))
            transforms = geometry.link_transforms(positions)
            if guard_geometry is not None:
                self.guard_box_links = np.unique(geometry.sphere_links[self.fixed_mask])
                self.guard_box_local_centers = guard_geometry.box_centers[self.guard_box_links]
                self.guard_box_halves = np.vstack((guard_geometry.box_halves[self.guard_box_links],[.085,.085,.069]))
                self.target_point_links,self.target_points = guard_geometry.contact_points(np.unique(geometry.sphere_links[self.target_mask]),self.radius)
                self.update_link_frames(transforms[:,:3,3],transforms[:,:3,:3])
            anchors = [transforms[geometry.link_indices[name],:3,3] for name in self.guard_anchor_links]
            self.update_guard(geometry.centers(positions),anchors)
            self.target_groups = [np.array([idx for idx,(name,_,_) in enumerate(geometry.spheres) if name in self.target_links and motion_root(name)==root],dtype=int)
                                  for root in sorted({motion_root(name) for name in self.target_links})]
            self.target_groups = [group for group in self.target_groups if len(group)]
            self.target_group_names = [motion_root(geometry.spheres[group[0]][0]) for group in self.target_groups]
        if self.pursuit_mode == 'distal_sweep' and not getattr(self,'target_groups',None):
            raise ValueError('横切り対象の衝突形状なし')
        self.center = np.asarray(center, dtype=float).copy()
        self.segments = self.links(self.center)
        if self.shape == 'linked_arm':
            self.select_linked_start(config.get('shoulder_candidates_m',[self.anchor]))
        self.center = self.segments[-1,-1].copy()
        self.velocity = np.zeros(3)
        self.max_endpoint_speed = 0.
        self.blocked_sec = 0.
        self.tip_blocked_sec = self.tip_stalled_sec = 0.
        self.configuration_velocities = []
        self.next_configuration_sec = 0.
        self.num_configuration_plans = 0
        self.configuration_plan_ms = 0.
        self.phase = 'pursuing'
        self.route_waypoint = None
        self.route_points = np.empty((0,3))
        self.route_target = None
        self.route_rng = np.random.default_rng(int(config.get('seed',401)))
        self.route_history = []
        self.num_route_recoveries = 0
        self.next_route_sec = 0.
        self.num_route_plans = 0
        self.route_cost_ms = 0.
        self.sweep_rng = np.random.default_rng(int(config.get('seed',401)))
        self.sweep_entry = self.sweep_exit = self.sweep_aim = None
        self.is_crossing = False
        self.next_sweep_sec = 0.
        self.sweep_expiry_sec = 0.
        self.num_sweeps = self.num_completed_sweeps = 0
        self.sweep_links = set()
        self.sweep_cost_ms = 0.
        self.random_walk_rng = np.random.default_rng(int(config.get('seed',401)))
        self.random_walk_goal = self.center.copy()
        self.next_random_walk_sec = 0.
        self.num_random_walk_goals = 0
        if self.pursuit_mode == 'random_walk':
            # 初期リンク位置を中心とする世界座標の固定範囲。現在の手先への追従なし。
            origin = geometry.link_transforms(positions)[geometry.link_indices[config.get('target_link','L_tcp')],:3,3]
            self.min_random_walk_position_m = origin-self.random_walk_extent_m
            self.max_random_walk_position_m = origin+self.random_walk_extent_m
            self.min_random_walk_position_m[2] = max(self.min_random_walk_position_m[2],self.radius-.10+.005)
            if np.any(self.center<self.min_random_walk_position_m) or np.any(self.center>self.max_random_walk_position_m):
                raise ValueError('障害物の開始位置がランダムウォーク範囲外')
        self.min_fixed_gap = self.fixed_gap(self.segments)
        if self.enable_proximal_guard and (self.min_fixed_gap < self.margin or np.min(self.segments[...,2])-self.radius<-.10):
            raise ValueError('追跡障害物の開始位置が固定部の禁止領域内')

    def links(self, wrist, swivel_rad=0.,reference_segments=None):
        if self.shape == 'sphere':
            return np.asarray([[wrist, wrist]])
        delta = np.asarray(wrist)-self.anchor
        length = float(np.linalg.norm(delta))
        min_length = abs(self.lengths[0]-self.lengths[1])+.005
        max_length = float(np.sum(self.lengths))-.005
        length = np.clip(length, min_length, max_length)
        axis = unit_vector(delta)
        wrist = self.anchor+length*axis
        along = (self.lengths[0]**2-self.lengths[1]**2+length**2)/(2*length)
        previous = getattr(self,'segments',None) if reference_segments is None else reference_segments
        reference = np.array([0.,0.,1.]) if previous is None else previous[0,1]-self.anchor
        bend = unit_vector(reference-axis*np.dot(reference,axis))
        if np.linalg.norm(bend) < .5:
            bend = unit_vector(np.cross(axis, [0., 1., 0.]))
        if np.linalg.norm(bend) < .5:
            bend = unit_vector(np.cross(axis, [1., 0., 0.]))
        # 球関節×2の手首3D位置と肘旋回。世界Z軸に固定した曲げ平面なし。
        bend = np.cos(swivel_rad)*bend+np.sin(swivel_rad)*np.cross(axis,bend)
        elbow = self.anchor+along*axis+np.sqrt(max(0., self.lengths[0]**2-along**2))*bend
        return np.asarray([[self.anchor, elbow], [elbow, wrist]])

    def select_linked_start(self, anchors):
        """固定肩候補と肘旋回の初期配置。周辺への可動余地と全端点の禁止域照合。"""
        anchors = np.asarray(anchors,dtype=float)
        if anchors.ndim!=2 or anchors.shape[1]!=3 or not 1<=len(anchors)<=24 or not np.isfinite(anchors).all():
            raise ValueError('リンク原点候補の形式不正')
        best = None
        for anchor in anchors:
            self.anchor = anchor
            for angle in np.linspace(-np.pi,np.pi,8,endpoint=False):
                candidate = self.links(self.center,angle)
                if np.linalg.norm(candidate[-1,-1]-self.center)>1e-6 or np.min(candidate[...,2])-self.radius<-.10:
                    continue
                gap = self.fixed_gap(candidate) if self.enable_proximal_guard else 1.
                if self.enable_proximal_guard and gap<self.margin:
                    continue
                probes = [self.links(self.center+axis*.06,reference_segments=candidate) for axis in np.vstack((np.eye(3),-np.eye(3)))]
                num_directions = sum(not self.enable_proximal_guard or self.can_traverse(candidate,probe) for probe in probes)
                score = .02*num_directions+min(gap,.15)
                if best is None or score>best[0]:
                    best = score,anchor.copy(),candidate.copy()
        if best is None:
            raise ValueError('リンク原点から追跡可能な非接触の開始位置なし。原点位置の変更が必要')
        _,self.anchor,self.segments = best

    def fixed_gap(self, segments):
        if self.guard_box_links is not None:
            return float(np.min(self.path_gaps(segments[:,0],segments[:,1])))
        if not len(self.fixed_centers):
            return 10.
        nearest = segment_nearest(self.fixed_centers, segments)
        return float(np.min(np.linalg.norm(self.fixed_centers[:, None]-nearest, axis=-1)-self.fixed_radii[:, None]-self.radius))

    def update_guard(self, centers, anchors):
        self.robot_centers = np.asarray(centers)
        self.fixed_centers = np.concatenate((np.asarray(centers)[self.fixed_mask],[[0.,0.,-.07]],np.asarray(anchors).reshape(-1,3)))
        self.target_centers = np.asarray(centers)[self.target_mask]
        self.fixed_radii = np.r_[self.fixed_shape_radii,np.linalg.norm([.085,.085,.069]),self.guard_anchor_radii]

    def update_link_frames(self, positions, rotations):
        """禁止形状と接触目標で共通の実測リンク変換。追加FK・形状再生成なし。"""
        links = self.guard_box_links
        self.guard_box_centers = np.vstack((positions[links]+np.einsum('nij,nj->ni',rotations[links],self.guard_box_local_centers),[0.,0.,-.07]))
        self.guard_box_axes = np.concatenate((rotations[links],np.eye(3)[None]))
        self.target_contact_centers = positions[self.target_point_links]+np.einsum('nij,nj->ni',rotations[self.target_point_links],self.target_points)

    def can_traverse(self, previous, candidate):
        if self.guard_box_links is not None:
            # 端点4点の投影区間による掃引包絡。箱面に沿う移動への過大な変位控除なし。
            gap = float(np.min(self.box_path_gaps(np.concatenate((previous,candidate),axis=1))))
        else:
            # 旧球包絡用のHausdorff変位上界。端点だけの通過判定なし。
            displacement = float(np.max(np.linalg.norm(candidate-previous,axis=-1)))
            gap = self.fixed_gap((previous+candidate)/2)-displacement/2
        if np.min(candidate[...,2])-self.radius < -.10:
            return False
        if gap>=self.margin:
            return True
        current_gap = self.fixed_gap(previous)
        # 移動する近位部に包まれた場合だけの離隔回復。禁止形状への追加進入なし。
        return (current_gap<self.margin and gap>=current_gap-1e-9 and
                self.fixed_gap(candidate)>current_gap+1e-9)

    def slew_velocity(self, desired, acceleration, period):
        change = desired-self.velocity
        if self.enable_separate_response and np.linalg.norm(self.velocity) > 1e-8:
            axis = unit_vector(self.velocity)
            longitudinal = float(change@axis)
            lateral = change-longitudinal*axis
            requested = self.velocity+axis*np.clip(longitudinal, -self.max_deceleration*period, acceleration*period)
            requested += lateral*min(1., self.max_turn_acceleration*period/max(float(np.linalg.norm(lateral)), 1e-9))
        else:
            requested = self.velocity+change*min(1., acceleration*period/max(float(np.linalg.norm(change)), 1e-9))
        return requested

    def path_gaps(self, starts, ends):
        """球中心経路の禁止形状への離隔。一括照合、ボクセル再登録なし。"""
        if self.guard_box_links is not None:
            return self.box_path_gaps(np.stack((starts,ends),axis=1))
        if not len(self.fixed_centers):
            return np.full(len(starts),10.)
        segments = np.stack((starts,ends),axis=1)
        nearest = segment_nearest(self.fixed_centers,segments)
        return np.min(np.linalg.norm(nearest-self.fixed_centers[:,None],axis=-1)-self.fixed_radii[:,None]-self.radius,axis=0)

    def box_path_gaps(self, points,enable_pair_bounds=False):
        """箱の面法線による線分・掃引包絡の距離下界 [m]。"""
        local = np.einsum('pvmi,mij->pvmj',np.asarray(points)[:,:,None]-self.guard_box_centers,self.guard_box_axes)
        lower,upper = np.min(local,axis=1),np.max(local,axis=1)
        gaps = np.max(np.maximum(lower-self.guard_box_halves,-upper-self.guard_box_halves),axis=-1)
        # 角部の斜め分離方向。任意軸の支持区間による保守下界、経路点の間引きなし。
        paths,boxes = np.nonzero(gaps<self.margin+self.radius)
        if not len(paths):
            return (gaps if enable_pair_bounds else np.min(gaps,axis=1))-self.radius
        near = np.transpose(local,(0,2,1,3))[paths,boxes]
        halves = self.guard_box_halves[boxes]
        directions = [np.mean(near,axis=1)]
        directions.extend(near[:,idx]-np.clip(near[:,idx],-halves,halves) for idx in range(near.shape[1]))
        for first in range(0,near.shape[1],2):
            start,delta = near[:,first],near[:,first+1]-near[:,first]
            fraction = np.clip(-np.sum(start*delta,axis=-1)/np.maximum(np.sum(delta*delta,axis=-1),1e-12),0.,1.)
            directions.append(start+fraction[...,None]*delta)
        for direction in directions:
            lengths = np.linalg.norm(direction,axis=-1)
            direction = direction/np.maximum(lengths[:,None],1e-12)
            bound = np.min(np.einsum('kvi,ki->kv',near,direction),axis=1)-np.sum(halves*np.abs(direction),axis=-1)
            bound[lengths<1e-12] = -np.inf
            gaps[paths,boxes] = np.maximum(gaps[paths,boxes],bound)
        return (gaps if enable_pair_bounds else np.min(gaps,axis=1))-self.radius

    def sweep_goal(self, now, speed):
        """末端側剛体の横切り目標。世界座標で保持する進入点・通過点と固定上限の再計画。"""
        if self.sweep_exit is not None:
            axis = unit_vector(self.sweep_exit-self.sweep_entry)
            has_finished = self.is_crossing and (np.linalg.norm(self.center-self.sweep_exit)<.035 or (self.center-self.sweep_exit)@axis>=0.)
            if has_finished or now >= self.sweep_expiry_sec:
                self.num_completed_sweeps += int(has_finished)
                self.sweep_entry = self.sweep_exit = None
                self.sweep_aim = None
                self.is_crossing = False
            elif not self.is_crossing and np.linalg.norm(self.center-self.sweep_entry)<.04:
                self.is_crossing = True
        if self.sweep_exit is None and now >= self.next_sweep_sec:
            begin = time.perf_counter()
            self.next_sweep_sec = now+.25
            # 最大8剛体標本×9方向。既存FKの球中心の参照のみ、再FK・ボクセル登録なし。
            for _ in range(8):
                group_idx = int(self.sweep_rng.integers(len(self.target_groups)))
                group = self.target_groups[group_idx]
                first,second = self.sweep_rng.choice(group,size=2)
                fraction = self.sweep_rng.random()
                aim = self.robot_centers[first]*(1.-fraction)+self.robot_centers[second]*fraction
                forward = unit_vector(aim-self.center)
                directions = np.vstack((forward,unit_vector(self.sweep_rng.normal(size=3)),np.eye(3),-np.eye(3),-forward))
                entries,exits = aim-directions*.16,aim+directions*.22
                is_valid = np.minimum(entries[:,2],exits[:,2])-self.radius>=-.10
                if self.enable_proximal_guard:
                    is_valid &= self.path_gaps(entries,exits)>=self.margin
                if not np.any(is_valid):
                    continue
                costs = np.linalg.norm(entries-self.center,axis=1)+.08*(1.-directions@forward)
                idx = int(np.argmin(np.where(is_valid,costs,np.inf)))
                self.sweep_entry,self.sweep_exit,self.sweep_aim = entries[idx].copy(),exits[idx].copy(),aim.copy()
                travel_sec = (np.linalg.norm(self.center-self.sweep_entry)+.38)/max(speed,.01)
                self.sweep_expiry_sec = now+np.clip(2.*travel_sec,1.5,15.)
                self.route_waypoint = None
                self.next_route_sec = 0.
                self.num_sweeps += 1
                self.sweep_links.add(self.target_group_names[group_idx])
                break
            self.sweep_cost_ms += (time.perf_counter()-begin)*1000.
        if self.sweep_exit is None:
            # 横切り候補の更新中も対象範囲への追跡継続。近位部の経路制約は後段で共通適用。
            centers = self.target_centers
            return centers[int(np.argmin(np.linalg.norm(centers-self.center,axis=1)))].copy()
        return self.sweep_exit if self.is_crossing else self.sweep_entry

    def plan_route(self, goal):
        """固定上限の双方向RRT-Connect。禁止形状全体の照合と可視ショートカット。"""
        start = self.center.copy()
        min_position_m = np.minimum(start,goal)-.6
        max_position_m = np.maximum(start,goal)+.6
        waypoints = np.empty((0,3))
        if self.guard_box_links is not None:
            extents = np.einsum('mij,mj->mi',np.abs(self.guard_box_axes),self.guard_box_halves)
            is_near = self.box_path_gaps(np.array([[start,goal]]),True)[0]<self.margin+.15
            if np.any(is_near):
                min_position_m = np.minimum(min_position_m,np.min((self.guard_box_centers-extents)[is_near],axis=0)-.2)
                max_position_m = np.maximum(max_position_m,np.max((self.guard_box_centers+extents)[is_near],axis=0)+.2)
                signs = np.array([value for value in product((-1.,0.,1.),repeat=3) if any(value)])
                local = signs[None]*(self.guard_box_halves[is_near,None]+self.radius+self.margin+.04)
                waypoints = (self.guard_box_centers[is_near,None]+np.einsum('mvi,mji->mvj',local,self.guard_box_axes[is_near])).reshape(-1,3)
                can_sample = waypoints[:,2]>=self.radius-.10+.005
                can_sample &= self.path_gaps(waypoints,waypoints)>=self.margin
                waypoints = waypoints[can_sample]
        min_position_m[2] = max(min_position_m[2],self.radius-.10+.005)
        initial_gap = self.path_gaps([start],[start])[0]
        trees = [[start],[np.asarray(goal).copy()]]
        parents = [[-1],[-1]]

        def extend(side, target):
            points = np.asarray(trees[side])
            idx = int(np.argmin(np.linalg.norm(points-target,axis=1)))
            delta = target-points[idx]
            candidate = points[idx]+delta*min(1.,.28/max(float(np.linalg.norm(delta)),1e-9))
            if candidate[2]<min_position_m[2]:
                return None
            if self.shape=='linked_arm' and np.linalg.norm(candidate-self.anchor)>=np.sum(self.lengths)-.005:
                return None
            gap = self.path_gaps([points[idx]],[candidate])[0]
            can_escape = side==0 and idx==0 and initial_gap<self.margin and gap>=initial_gap-1e-9
            if gap<self.margin and not can_escape:
                return None
            if np.linalg.norm(candidate-points[idx])<1e-8:
                return idx
            trees[side].append(candidate);parents[side].append(idx)
            return len(trees[side])-1

        def branch(side, idx):
            result = []
            while idx>=0:
                result.append(trees[side][idx]);idx=parents[side][idx]
            return result[::-1]

        path = None
        for turn in range(128):
            side = turn%2
            if len(waypoints) and turn%4<2:
                sample = waypoints[int(self.route_rng.integers(len(waypoints)))]
            else:
                sample = self.route_rng.uniform(min_position_m,max_position_m) if turn%5 else trees[1-side][0]
            idx = extend(side,sample)
            if idx is None:
                continue
            target = trees[side][idx]
            for _ in range(16):
                other = extend(1-side,target)
                if other is None:
                    break
                if np.linalg.norm(trees[1-side][other]-target)<1e-8:
                    first,second = (idx,other) if side==0 else (other,idx)
                    path = branch(0,first)+branch(1,second)[-2::-1]
                    break
            if path is not None:
                break
        if path is None:
            # 未到達時も探索済みの安全な枝への移動。遮断された目標への直進反復なし。
            points = np.asarray(trees[0])
            can_select = np.linalg.norm(points-start,axis=1)>.06
            if self.route_history:
                recent = np.min(np.linalg.norm(points[:,None]-self.route_history,axis=-1),axis=1)
                if np.any(can_select&(recent>.08)):
                    can_select &= recent>.08
            if not np.any(can_select):
                return np.empty((0,3))
            idx = int(np.argmin(np.where(can_select,np.linalg.norm(points-goal,axis=1),np.inf)))
            path = branch(0,idx)
            self.num_route_recoveries += 1
        # 線分全体の照合による短絡。未照合の曲線補間なし、旋回平滑化は速度応答側。
        path = np.asarray(path)
        result,idx = [],0
        while idx<len(path)-1:
            gaps = self.path_gaps(np.broadcast_to(path[idx],path[idx+1:].shape),path[idx+1:])
            valid = np.flatnonzero(gaps>=self.margin)
            idx += int(valid[-1])+1 if len(valid) else 1
            result.append(path[idx])
        return np.asarray(result).reshape(-1,3)

    def route_goal(self, followed, now):
        """直進追跡と保持済み迂回経路。目標・禁止域の変化または停滞時だけの再探索。"""
        centers = self.target_centers if self.target_contact_centers is None else self.target_contact_centers
        if self.enable_proximal_guard and len(centers) and self.path_gaps([followed],[followed])[0]<self.margin:
            # 近位部内の関節原点に代わる実衝突形状への接触目標。外接球周囲への膨張なし。
            can_approach = self.path_gaps(centers,centers)>=self.margin
            if np.any(can_approach):
                candidates = centers[can_approach]
                followed = candidates[np.argmin(np.linalg.norm(candidates-self.center,axis=1))]
        if not self.enable_proximal_guard or self.path_gaps([self.center],[followed])[0]>=self.margin:
            self.route_waypoint = None
            self.route_points = np.empty((0,3))
            self.phase = 'pursuing'
            return followed
        while len(self.route_points) and np.linalg.norm(self.route_points[0]-self.center)<.04:
            self.route_history = (self.route_history+[self.route_points[0].copy()])[-8:]
            self.route_points = self.route_points[1:]
        has_path = bool(len(self.route_points))
        can_follow = has_path and self.path_gaps([self.center],[self.route_points[0]])[0]>=self.margin
        has_changed = self.route_target is None or np.linalg.norm(self.route_target-followed)>.1
        if now>=self.next_route_sec and (not can_follow or has_changed or self.tip_stalled_sec>.12):
            begin = time.perf_counter()
            self.route_points = self.plan_route(followed)
            self.route_target = np.asarray(followed).copy()
            self.next_route_sec = now+.2
            self.num_route_plans += 1
            self.route_cost_ms += (time.perf_counter()-begin)*1000
        self.route_waypoint = self.route_points[0].copy() if len(self.route_points) else None
        self.phase = 'detouring' if self.route_waypoint is not None else 'route_blocked'
        return self.center.copy() if self.route_waypoint is None else self.route_waypoint

    def random_walk_target(self, now):
        """固定間隔のランダム目標点。最大8候補、追加FK・ボクセル登録・経路探索なし。"""
        if now < self.next_random_walk_sec:
            return self.random_walk_goal
        self.next_random_walk_sec = now+self.random_walk_change_sec
        padding = min(.1,self.random_walk_extent_m*.2)
        points = self.random_walk_rng.uniform(self.min_random_walk_position_m+padding,self.max_random_walk_position_m-padding,size=(8,3))
        can_reach = np.linalg.norm(points-self.center,axis=1)>.04
        if self.enable_proximal_guard:
            can_reach &= self.path_gaps(np.broadcast_to(self.center,points.shape),points)>=self.margin
        indices = np.flatnonzero(can_reach)
        self.random_walk_goal = points[indices[0]].copy() if len(indices) else self.center.copy()
        self.num_random_walk_goals += int(bool(len(indices)))
        return self.random_walk_goal

    def can_step(self, previous, candidate):
        """移動範囲と既存固定部ガードの共通採用条件。"""
        if self.pursuit_mode == 'random_walk' and (np.any(candidate<self.min_random_walk_position_m) or np.any(candidate>self.max_random_walk_position_m)):
            return False
        return not self.enable_proximal_guard or self.can_traverse(previous,candidate)

    def can_steps(self,previous,candidates,current_gap):
        """縮小候補の一括掃引照合。既存の余裕・床・単調回復条件の保持。"""
        if self.guard_box_links is None:
            parents=np.broadcast_to(previous,candidates.shape)
            return np.array([self.can_step(parent,candidate) for parent,candidate in zip(parents,candidates)])
        is_valid = np.ones(len(candidates),dtype=bool)
        if self.pursuit_mode=='random_walk':
            is_valid &= np.all((candidates>=self.min_random_walk_position_m)&(candidates<=self.max_random_walk_position_m),axis=(1,2,3))
        if not self.enable_proximal_guard:
            return is_valid
        previous = np.broadcast_to(previous,candidates.shape)
        swept = np.concatenate((previous,candidates),axis=2).reshape(-1,4,3)
        gaps = np.min(self.box_path_gaps(swept).reshape(len(candidates),-1),axis=1)
        is_valid &= np.min(candidates[...,2],axis=(1,2))-self.radius>=-.10
        can_pass = gaps>=self.margin
        if current_gap<self.margin:
            candidate_gaps = np.min(self.path_gaps(candidates[:,:,0].reshape(-1,3),candidates[:,:,1].reshape(-1,3)).reshape(len(candidates),-1),axis=1)
            can_pass |= (gaps>=current_gap-1e-9)&(candidate_gaps>current_gap+1e-9)
        return is_valid&can_pass

    def contact_result(self, contacts):
        """実接触の勝敗。禁止部・対象外・先端以外の接触を回避失敗と分離。"""
        if not contacts:
            return ''
        if any(contact.get('is_environment') for contact in contacts):
            return 'attacker_environment_contact'
        if any(contact['is_protected'] for contact in contacts):
            return 'attacker_protected_contact'
        if any(contact['link'] not in self.target_links for contact in contacts):
            return 'attacker_off_target_contact'
        if self.shape=='linked_arm' and any(contact['obstacle_idx']!=1 or np.linalg.norm(np.asarray(contact['position'])-self.center)>self.radius+.01 for contact in contacts):
            return 'attacker_shaft_contact'
        return 'target_touch'

    def configuration_options(self,goal,speed,period,now):
        """境界付近の手首・肘旋回の連動候補。9姿勢の距離差分と4変数の有限最適化。"""
        if self.shape!='linked_arm' or self.guard_box_links is None or not self.enable_proximal_guard:
            return []
        if self.tip_stalled_sec<.05 and self.fixed_gap(self.segments)>self.margin+.04:
            return []
        lever=max(float(np.min(self.lengths))*.5,.05)
        if now>=self.next_configuration_sec:
            self.next_configuration_sec=now+.05
            self.num_configuration_plans+=1
            started=time.perf_counter();step=.001
            probes=[self.segments]
            for axis in np.eye(4):
                for sign in (-1.,1.):
                    probes.append(self.links(self.center+sign*step*axis[:3],sign*step*axis[3]/lever))
            probes=np.asarray(probes)
            bounds=self.box_path_gaps(np.stack((probes[:,:,0],probes[:,:,1]),axis=2).reshape(-1,2,3),True).reshape(9,-1)
            gradients=((bounds[2::2]-bounds[1::2])/(2*step)).T
            selected=(bounds[0]<self.margin+.06)&(np.linalg.norm(gradients,axis=1)>1e-8)
            matrix=gradients[selected];required=2.*(self.margin-bounds[0,selected])
            self.configuration_velocities=[]
            desired=np.r_[unit_vector(goal-self.center)*speed,0.]
            constraints=[{'type':'ineq','fun':lambda v:speed*speed-v@v,'jac':lambda v:-2*v}]
            if len(matrix):
                constraints.append({'type':'ineq','fun':lambda v:matrix@v-required,'jac':lambda _:matrix})
            # 余裕距離の追加要求による狭路での相互矛盾なし。実掃引ガードは従来のまま。
            solution=minimize(lambda v:.5*np.sum((v-desired)**2),np.zeros(4),jac=lambda v:v-desired,
                method='SLSQP',constraints=constraints,options={'maxiter':32,'ftol':1e-10,'disp':False})
            if solution.success and np.isfinite(solution.x).all() and np.all(matrix@solution.x>=required-1e-6):
                self.configuration_velocities.append(solution.x)
            self.configuration_plan_ms+=(time.perf_counter()-started)*1000
        return [(value[:3],float(value[3]/lever*period)) for value in self.configuration_velocities]

    def advance(self, followed, speed, acceleration, period, enable_sidestep=False, now=0.):
        if self.pursuit_mode == 'random_walk':
            route_goal = self.random_walk_target(now)
            self.phase = 'random_walking'
        else:
            if self.pursuit_mode == 'distal_sweep':
                followed = self.sweep_goal(now,speed)
            route_goal = self.route_goal(followed,now)
        if self.pursuit_mode == 'distal_sweep' and self.phase == 'pursuing':
            self.phase = 'sweeping' if self.is_crossing else 'sweep_approach'
        direction = unit_vector(route_goal-self.center)
        if enable_sidestep:
            direction = unit_vector(direction+.35*np.sin(2.1*now)*np.cross(direction, [0., 0., 1.]))
        desired_speed = min(speed,np.sqrt(2.*self.max_deceleration*np.linalg.norm(route_goal-self.center))) if self.pursuit_mode=='random_walk' else speed
        requested = self.slew_velocity(direction*desired_speed, acceleration, period)
        requested *= min(1., speed/max(float(np.linalg.norm(requested)), 1e-9))
        options = [(requested,0.)]
        coupled=self.configuration_options(route_goal,speed,period,now)
        options=[(self.slew_velocity(value,acceleration,period),swivel) for value,swivel in coupled]+options
        current_fixed_gap = self.fixed_gap(self.segments)
        is_guard_recovery = self.enable_proximal_guard and current_fixed_gap<self.margin
        is_route_recovery = self.enable_proximal_guard and self.phase=='route_blocked'
        if self.enable_proximal_guard:
            for axis in np.eye(3):
                tangent = unit_vector(np.cross(direction, axis))
                options.extend((self.slew_velocity(unit_vector(direction+sign*2*tangent)*speed, acceleration, period),0.) for sign in (-1., 1.))
            if is_guard_recovery or self.tip_stalled_sec>.08 or self.route_waypoint is None:
                # 行き止まりから離れる並進。追跡方向に限定しない、既存加減速制限内の候補。
                options.extend((self.slew_velocity(axis*speed,acceleration,period),0.) for axis in np.vstack((np.eye(3),-np.eye(3),-direction)))
        if self.shape=='linked_arm':
            swivel_step = min(.08,speed*period/max(float(np.min(self.lengths))*.25,1e-9))
            options.extend((option,sign*swivel_step) for option in (requested,np.zeros(3)) for sign in (-1.,1.))
        old = self.segments.copy()
        best = None
        for idx,(option,swivel_rad) in enumerate(options):
            option = option*min(1.,speed/max(float(np.linalg.norm(option)),1e-9))
            segments = self.links(self.center+option*period,swivel_rad)
            delta = segments-old
            # 手首だけでなく肘を含む全端点の速度上限。
            fraction = min(1., speed*period/max(float(np.max(np.linalg.norm(delta, axis=-1))), 1e-9))
            if fraction < 1.:
                segments = self.links(self.center+option*period*fraction*.98,swivel_rad*fraction*.98)
            # 初回不成立時だけの縮小候補一括照合。同じ7段階、再選択順序の保持。
            is_speed_valid = np.max(np.linalg.norm(segments-old,axis=-1))<=speed*period+1e-10
            if not is_speed_valid or not self.can_step(old,segments):
                fractions = fraction*.5**np.arange(1,8)
                reduced = np.array([self.links(self.center+option*period*value,swivel_rad*value) for value in fractions])
                is_valid = np.max(np.linalg.norm(reduced-old,axis=-1),axis=(1,2))<=speed*period+1e-10
                is_valid &= self.can_steps(old,reduced,current_fixed_gap)
                accepted = np.flatnonzero(is_valid)
                if not len(accepted):
                    continue
                fraction = fractions[accepted[0]]
                segments = reduced[accepted[0]]
            min_gap = self.fixed_gap(segments)
            cost = np.linalg.norm(segments[-1, -1]-route_goal)
            if is_route_recovery:
                # 探索予算内の未発見と停止の分離。安全な移動候補の継続選択。
                cost = -np.linalg.norm(segments[-1,-1]-old[-1,-1])+.01*np.linalg.norm(segments[-1,-1]-followed)
            if is_guard_recovery:
                cost = -min_gap+.01*cost
            if self.shape=='linked_arm':
                cost -= .05*min(min_gap,.2)
            if best is None or cost < best[0]:
                best = (cost, segments, min_gap)
            if idx == len(coupled) and fraction>=.99 and np.max(np.linalg.norm(segments-old,axis=-1))>speed*period*.5:
                break
        if best is None:
            segments, min_gap = old, self.fixed_gap(old)
            self.blocked_sec += period
        else:
            _, segments, min_gap = best
        self.segments = segments
        self.center = segments[-1, -1].copy()
        self.velocity = (self.center-old[-1, -1])/period
        achieved = float(np.max(np.linalg.norm(segments-old, axis=-1))/period)
        if np.linalg.norm(self.velocity)<speed*.05 and np.linalg.norm(route_goal-self.center)>.03:
            self.tip_blocked_sec += period
            self.tip_stalled_sec += period
        else:
            # 回復中の微小進行による肘目標の頻繁な解除防止。
            self.tip_stalled_sec = max(0.,self.tip_stalled_sec-period*.2)
        if achieved < speed*.05:
            if best is not None:
                self.blocked_sec += period
            self.phase = 'random_waiting' if self.pursuit_mode=='random_walk' else 'target_out_of_reach' if self.shape=='linked_arm' and np.linalg.norm(followed-self.anchor)>np.sum(self.lengths)-.005 else 'route_blocked'
        self.max_endpoint_speed = max(self.max_endpoint_speed, achieved)
        self.min_fixed_gap = min(self.min_fixed_gap, min_gap)
        return achieved

    def field(self, radius=None):
        radius = self.radius if radius is None else radius
        return sphere_field(self.center, radius) if self.shape == 'sphere' else capsule_field(self.segments, radius)

    def bodies(self):
        kind = self.shape == 'sphere'
        return [dict(id=f'pursuer_{idx}', mode='kinematic', mass=.2, position=[3., 0., 3.], quaternion=[0, 0, 0, 1],
                     geoms=[dict(type='sphere' if kind else 'capsule', radius=self.radius,
                                 half_length=0. if kind else float(self.lengths[idx])/2,
                                 position=[0, 0, 0], quaternion=[0, 0, 0, 1])])
                for idx in range(1 if kind else 2)]

    def move_scene(self, scene):
        updates = []
        for idx, (start, end) in enumerate(self.segments):
            axis = unit_vector(end-start)
            rotation = np.r_[np.cross([0., 0., 1.], axis), 1.+axis[2]]
            if np.linalg.norm(rotation) < 1e-8:
                rotation = np.array([1., 0., 0., 0.])
            rotation /= np.linalg.norm(rotation)
            if self.shape == 'sphere':
                rotation = np.array([0., 0., 0., 1.])
            updates.append(dict(id=f'pursuer_{idx}', position=((start+end)/2).tolist(), quaternion=rotation.tolist()))
        scene.move(updates)
