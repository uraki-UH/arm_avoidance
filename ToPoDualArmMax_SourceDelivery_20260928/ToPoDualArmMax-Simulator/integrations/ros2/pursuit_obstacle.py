"""球・二節カプセルの追跡生成と共通観測。固定部回避は障害物側だけの経路条件。"""
from itertools import product
import time

import numpy as np

from gng_sphere_avoidance import sphere_field, unit_vector


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
    def __init__(self, config, center, direction, geometry, positions):
        self.shape = config.get('obstacle_shape', 'sphere')
        if self.shape not in ('sphere', 'linked_arm'):
            raise ValueError('障害物形状はsphereまたはlinked_arm')
        self.joint_types = ('ball','ball') if self.shape=='linked_arm' else ()
        self.radius = float(config.get('radius_m', .06))
        self.max_deceleration = float(config.get('max_ball_deceleration', config.get('max_ball_acceleration', .8)))
        self.max_turn_acceleration = float(config.get('max_ball_turn_acceleration', config.get('max_ball_acceleration', .8)))
        self.enable_separate_response = 'max_ball_deceleration' in config or 'max_ball_turn_acceleration' in config
        if not np.isfinite([self.max_deceleration, self.max_turn_acceleration]).all() or not 0 < self.max_deceleration <= 30 or not 0 < self.max_turn_acceleration <= 30:
            raise ValueError('減速・方向転換加速度の範囲外')
        self.enable_fixed_guard = config.get('enable_fixed_guard', True)
        if type(self.enable_fixed_guard) is not bool:
            raise ValueError('固定部ガードは真偽値')
        self.enable_proximal_guard = config.get('enable_proximal_guard', True)
        if type(self.enable_proximal_guard) is not bool:
            raise ValueError('近位部の迂回条件は真偽値')
        self.enable_fixed_guard = self.enable_fixed_guard and self.enable_proximal_guard
        self.margin = float(config.get('fixed_clearance_m', .025))
        self.lengths = np.asarray(config.get('link_lengths_m', [.35, .35]), dtype=float)
        self.anchor = np.asarray(config.get('shoulder_position_m', np.asarray(center)-direction*.12+[0., 0., .15]), dtype=float)
        if self.anchor.shape != (3,) or not np.isfinite(self.anchor).all() or self.lengths.shape != (2,) or not np.isfinite(self.lengths).all() or np.any(self.lengths < .15) or np.any(self.lengths > .6) or not .005 <= self.margin <= .1:
            raise ValueError('追跡腕の肩位置・リンク寸法・固定部余裕の不正')
        self.fixed_centers = geometry.centers(positions)[~geometry.is_arm]
        self.fixed_radii = geometry.radii[~geometry.is_arm]
        self.fixed_shape_radii = self.fixed_radii.copy()
        self.fixed_mask = ~geometry.is_arm.copy()
        self.protected_mask = self.fixed_mask.copy()
        self.guard_anchor_links, self.guard_anchor_radii = [], []
        self.target_links = {config.get('target_link','L_tcp')}
        if hasattr(geometry, 'joints'):
            active_names = set(getattr(geometry, 'active_joint_names', ()))
            parents = {joint[2]: joint for joint in geometry.joints}
            def motion_root(name):
                while name in parents and parents[name][0] not in active_names:
                    name = parents[name][1]
                return name
            target_root = motion_root(config.get('target_link','L_tcp'))
            self.target_links = {name for name in geometry.link_indices if motion_root(name)==target_root}
            proximal_roots, link = set(), target_root
            while link in parents:
                link = parents[link][1]
                proximal_roots.add(motion_root(link))
            if 'target_link' in config:
                # 指定リンクより近位の運動剛体群。固定結合のカバーも同一群。
                self.protected_mask |= np.array([motion_root(name) in proximal_roots for name,_,_ in geometry.spheres])
                self.protected_mask &= np.array([name not in self.target_links for name,_,_ in geometry.spheres])
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
            anchors = [transforms[geometry.link_indices[name],:3,3] for name in self.guard_anchor_links]
            self.update_guard(geometry.centers(positions),anchors)
        self.center = np.asarray(center, dtype=float).copy()
        self.segments = self.links(self.center)
        self.center = self.segments[-1,-1].copy()
        self.velocity = np.zeros(3)
        self.max_endpoint_speed = 0.
        self.blocked_sec = 0.
        self.phase = 'pursuing'
        self.route_waypoint = None
        self.next_route_sec = 0.
        self.num_route_plans = 0
        self.route_cost_ms = 0.
        self.min_fixed_gap = self.fixed_gap(self.segments)
        if self.enable_fixed_guard and self.min_fixed_gap < self.margin:
            raise ValueError('追跡障害物の開始位置が固定部の禁止領域内')

    def links(self, wrist, swivel_rad=0.):
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
        previous = getattr(self,'segments',None)
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

    def fixed_gap(self, segments):
        if not len(self.fixed_centers):
            return 10.
        nearest = segment_nearest(self.fixed_centers, segments)
        return float(np.min(np.linalg.norm(self.fixed_centers[:, None]-nearest, axis=-1)-self.fixed_radii[:, None]-self.radius))

    def update_guard(self, centers, anchors):
        self.fixed_centers = np.concatenate((np.asarray(centers)[self.fixed_mask],np.asarray(anchors).reshape(-1,3)))
        # 固定拘束の微小な追従ずれ用余裕 [m]。接触評価形状とは独立の禁止領域。
        self.fixed_radii = np.r_[self.fixed_shape_radii,self.guard_anchor_radii]+.005

    def can_traverse(self, previous, candidate):
        displacement = float(np.max(np.linalg.norm(candidate-previous,axis=-1)))
        # 線分全体のHausdorff変位上界による経路証明。端点だけの通過判定なし。
        gap = self.fixed_gap((previous+candidate)/2)-displacement/2
        return gap >= self.margin and np.min(candidate[...,2])-self.radius >= -.10

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
        if not len(self.fixed_centers):
            return np.full(len(starts),10.)
        segments = np.stack((starts,ends),axis=1)
        nearest = segment_nearest(self.fixed_centers,segments)
        return np.min(np.linalg.norm(nearest-self.fixed_centers[:,None],axis=-1)-self.fixed_radii[:,None]-self.radius,axis=0)

    def route_goal(self, followed, now):
        """直進可能時の即時追跡、遮蔽時だけの小規模3D可視グラフ。"""
        if not self.enable_fixed_guard or self.path_gaps([self.center],[followed])[0]>=self.margin:
            self.route_waypoint = None
            self.phase = 'pursuing'
            return followed
        if now < self.next_route_sec and self.route_waypoint is not None and np.linalg.norm(self.route_waypoint-self.center)>.025:
            self.phase = 'detouring'
            return self.route_waypoint
        begin = time.perf_counter()
        self.next_route_sec = now+.2
        self.num_route_plans += 1
        # 近接する禁止球3個×26方向。探索量の固定上限、同一経路の側替わり抑制。
        nearest = segment_nearest(self.fixed_centers,np.array([[self.center,followed]]))[:,0]
        gaps = np.linalg.norm(nearest-self.fixed_centers,axis=1)-self.fixed_radii-self.radius
        indices = np.argsort(gaps)[:3]
        directions = np.array([value for value in product((-1.,0.,1.),repeat=3) if any(value)])
        directions /= np.linalg.norm(directions,axis=1)[:,None]
        nodes = [self.center,followed]
        if self.route_waypoint is not None:
            nodes.append(self.route_waypoint)
        for idx in indices:
            extent = (self.fixed_radii[idx]+self.radius+self.margin+.025)*1.3
            nodes.extend(self.fixed_centers[idx]+directions*extent)
        nodes = np.asarray(nodes)
        is_valid = nodes[:,2]-self.radius>=-.10
        is_valid &= self.path_gaps(nodes,nodes)>=self.margin
        is_valid[0] = True
        # 追跡点自体が近位球内の場合も、禁止域手前までの到達経路を探索。
        nodes = nodes[np.r_[True,True,is_valid[2:]]]
        first,second = np.triu_indices(len(nodes),1)
        distances = np.linalg.norm(nodes[first]-nodes[second],axis=1)
        can_connect = self.path_gaps(nodes[first],nodes[second])>=self.margin+.001
        weights = np.full((len(nodes),len(nodes)),np.inf)
        weights[first[can_connect],second[can_connect]] = distances[can_connect]
        weights = np.minimum(weights,weights.T)
        costs = np.full(len(nodes),np.inf);costs[0] = 0.
        parents = np.full(len(nodes),-1,dtype=int)
        visited = np.zeros(len(nodes),dtype=bool)
        for _ in range(len(nodes)):
            idx = int(np.argmin(np.where(visited,np.inf,costs)))
            if visited[idx] or not np.isfinite(costs[idx]):
                break
            visited[idx] = True
            candidate = costs[idx]+weights[idx]
            improved = candidate<costs
            parents[improved] = idx;costs[improved] = candidate[improved]
        if np.isfinite(costs[1]):
            chosen = 1
        else:
            # 未到達点への直進反復の回避。到達可能な境界候補への接近。
            remaining = np.linalg.norm(nodes-followed,axis=1)
            chosen = int(np.argmin(np.where(visited,remaining+.12*costs,np.inf)))
        while parents[chosen]>0:
            chosen = int(parents[chosen])
        self.route_waypoint = nodes[chosen].copy() if chosen else None
        self.route_cost_ms += (time.perf_counter()-begin)*1000
        self.phase = 'detouring' if self.route_waypoint is not None else 'route_blocked'
        return followed if self.route_waypoint is None else self.route_waypoint

    def contact_result(self, contacts):
        """実接触の勝敗。禁止部・対象外・先端以外の接触を回避失敗と分離。"""
        if not contacts:
            return ''
        if any(contact['is_protected'] for contact in contacts):
            return 'attacker_protected_contact'
        if any(contact['link'] not in self.target_links for contact in contacts):
            return 'attacker_off_target_contact'
        if self.shape=='linked_arm' and any(contact['obstacle_idx']!=1 or np.linalg.norm(np.asarray(contact['position'])-self.center)>self.radius+.01 for contact in contacts):
            return 'attacker_shaft_contact'
        return 'target_touch'

    def advance(self, followed, speed, acceleration, period, enable_sidestep=False, now=0.):
        route_goal = self.route_goal(followed,now)
        direction = unit_vector(route_goal-self.center)
        if enable_sidestep:
            direction = unit_vector(direction+.35*np.sin(2.1*now)*np.cross(direction, [0., 0., 1.]))
        requested = self.slew_velocity(direction*speed, acceleration, period)
        requested *= min(1., speed/max(float(np.linalg.norm(requested)), 1e-9))
        options = [(requested,0.)]
        if self.enable_fixed_guard:
            for axis in np.eye(3):
                tangent = unit_vector(np.cross(direction, axis))
                options.extend((self.slew_velocity(unit_vector(direction+sign*2*tangent)*speed, acceleration, period),0.) for sign in (-1., 1.))
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
            # IKの非線形性・禁止領域の経路余裕によるステップ縮小。
            for _ in range(8):
                is_speed_valid = np.max(np.linalg.norm(segments-old, axis=-1)) <= speed*period+1e-10
                if is_speed_valid and (not self.enable_fixed_guard or self.can_traverse(old,segments)):
                    break
                fraction *= .5
                segments = self.links(self.center+option*period*fraction,swivel_rad*fraction)
            else:
                continue
            min_gap = self.fixed_gap(segments)
            cost = np.linalg.norm(segments[-1, -1]-route_goal)
            if self.shape=='linked_arm':
                cost -= .05*min(min_gap,.2)
            if best is None or cost < best[0]:
                best = (cost, segments, min_gap)
            if idx == 0 and fraction>=.99:
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
        if achieved < speed*.05:
            if best is not None:
                self.blocked_sec += period
            self.phase = 'route_blocked'
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
