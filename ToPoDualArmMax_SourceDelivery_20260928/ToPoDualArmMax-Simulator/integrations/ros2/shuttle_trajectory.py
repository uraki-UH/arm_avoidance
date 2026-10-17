"""往復基準軌道の生成・離散CHOMP改善・幾何学的追従。動的回避とは独立した静的計画。"""
import heapq
import time

import numpy as np


def path_lengths(path):
    """関節空間の累積弧長 [rad]。周期折返しなし。"""
    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]


def interpolate_path(path, lengths, samples):
    return np.stack([np.interp(samples,lengths,path[:,idx]) for idx in range(path.shape[1])],axis=-1)


def nearest_path_points(path, positions):
    """姿勢候補の基準折れ線への直交投影。端点間直線への代用なし。"""
    path,positions = np.asarray(path),np.asarray(positions)
    delta = np.diff(path,axis=0)
    ratios = np.clip(np.sum((positions[...,None,:]-path[:-1])*delta,axis=-1)/
                     np.maximum(np.sum(delta**2,axis=-1),1e-12),0.,1.)
    points = path[:-1]+ratios[...,None]*delta
    indices = np.argmin(np.sum((points-positions[...,None,:])**2,axis=-1),axis=-1)
    return np.take_along_axis(points,indices[...,None,None],axis=-2)[...,0,:]


class shuttle_planner:
    """GNG接続または双方向RRTによる初期経路と、固定端点の共変勾配改善。"""
    def __init__(self, limits, is_clear, obstacle_cost=None):
        self.limits = np.asarray(limits,dtype=float)
        if self.limits.ndim!=2 or self.limits.shape[1]!=2 or not np.isfinite(self.limits).all() or np.any(self.limits[:,0]>=self.limits[:,1]):
            raise ValueError('軌道生成の有限可動域の不正')
        self.is_clear,self.obstacle_cost = is_clear,obstacle_cost
        self.num_checks = 0

    def check(self, poses):
        values = np.atleast_2d(np.asarray(poses,dtype=float))
        if values.shape[1]!=len(self.limits) or not np.isfinite(values).all():
            return False
        if np.any(values<self.limits[:,0]) or np.any(values>self.limits[:,1]):
            return False
        self.num_checks += len(values)
        return bool(np.all(self.is_clear(values)))

    def dense_path(self, path):
        parts = [np.linspace(first,second,max(1,int(np.ceil(np.max(np.abs(second-first))/.04)))+1)[:-1]
                 for first,second in zip(path[:-1],path[1:])]
        return np.vstack([*parts,path[-1:]])

    def can_connect(self, first, second):
        return self.check(self.dense_path(np.stack((first,second))))

    def graph_route(self, start, goal, graph):
        """保存GNGの接続探索。端点接続・採用辺の共通衝突照合。"""
        graph = graph() if callable(graph) else graph
        if graph is None:
            return None
        ids = graph['angle_node_ids']
        angles = np.array([graph['angles'][idx] for idx in ids])
        if angles.ndim!=2 or angles.shape[1]!=len(start):
            raise ValueError('GNGと往復制御軸の不一致')
        lookup = dict(zip(ids,range(len(ids))))
        order = np.argsort(np.linalg.norm(angles-start,axis=1),kind='stable')[:16]
        goal_ids = set(np.asarray(ids)[np.argsort(np.linalg.norm(angles-goal,axis=1),kind='stable')[:16]])
        queue,costs,parents = [],{},{}
        for idx in order:
            if self.can_connect(start,angles[idx]):
                node = ids[idx]
                costs[node] = float(np.linalg.norm(angles[idx]-start))
                parents[node] = None
                heapq.heappush(queue,(costs[node]+float(np.linalg.norm(angles[idx]-goal)),node))
        visited = set()
        while queue and len(visited)<512:
            _,node = heapq.heappop(queue)
            if node in visited:
                continue
            visited.add(node)
            current = angles[lookup[node]]
            if node in goal_ids and self.can_connect(current,goal):
                route = [goal,current]
                while parents[node] is not None:
                    node = parents[node]
                    route.append(angles[lookup[node]])
                return np.array([start,*route[::-1]])
            for neighbor in graph['adjacency'].get(node,()):
                if neighbor not in lookup or neighbor in visited:
                    continue
                target = angles[lookup[neighbor]]
                cost = costs[node]+float(np.linalg.norm(target-current))
                if cost>=costs.get(neighbor,np.inf) or not self.can_connect(current,target):
                    continue
                costs[neighbor],parents[neighbor] = cost,node
                heapq.heappush(queue,(cost+float(np.linalg.norm(target-goal)),neighbor))
        return None

    def rrt_connect(self, start, goal):
        """固定乱数の双方向RRT-Connect。各延長区間の照合、有限回の探索。"""
        generator = np.random.default_rng(0)
        trees = [([start],[None]),([goal],[None])]

        def extend(tree, target):
            nodes,parents = tree
            idx = int(np.argmin(np.linalg.norm(np.asarray(nodes)-target,axis=1)))
            delta = target-nodes[idx]
            candidate = nodes[idx]+delta*min(1.,.3/max(float(np.linalg.norm(delta)),1e-12))
            if not self.can_connect(nodes[idx],candidate):
                return None
            nodes.append(candidate);parents.append(idx)
            return len(nodes)-1

        def branch(tree, idx):
            result = []
            while idx is not None:
                result.append(tree[0][idx]);idx = tree[1][idx]
            return result[::-1]

        for iter_idx in range(1024):
            first,second = iter_idx%2,1-iter_idx%2
            target = trees[second][0][0] if iter_idx%5==0 else generator.uniform(self.limits[:,0],self.limits[:,1])
            idx = extend(trees[first],target)
            if idx is None:
                continue
            target = trees[first][0][idx]
            for _ in range(64):
                other = extend(trees[second],target)
                if other is None:
                    break
                if np.linalg.norm(trees[second][0][other]-target)<1e-8:
                    left,right = branch(trees[first],idx),branch(trees[second],other)
                    return np.array(left+right[-2::-1] if first==0 else right+left[-2::-1])
        raise ValueError('往復の衝突なし初期経路が見つかりません')

    def improve(self, seed):
        """離散CHOMP。差分計量の逆作用、固定端点、離隔ポテンシャル、降下線探索。"""
        lengths = path_lengths(seed)
        samples = np.unique(np.r_[lengths,np.linspace(0.,lengths[-1],33)])
        path = interpolate_path(seed,lengths,samples)
        if len(path)>257:
            raise ValueError('往復初期経路の節点数の範囲外')
        num = len(path)
        first = np.diff(np.eye(num),axis=0)
        second = np.diff(np.eye(num),n=2,axis=0)
        metric = first.T@first+.2*second.T@second
        inverse = np.linalg.inv(metric[1:-1,1:-1]+np.eye(num-2)*1e-9)

        def cost(values):
            value = .5*float(np.sum(values*(metric@values)))
            if self.obstacle_cost is not None:
                value += float(np.sum(self.obstacle_cost(values)))
            return value

        if not self.check(self.dense_path(path)):
            raise ValueError('往復初期経路の区間衝突')
        initial_cost = current_cost = cost(path)
        num_updates = 0
        for _ in range(24):
            gradient = (metric@path)[1:-1]
            if self.obstacle_cost is not None:
                # 全関節差分の一括評価。軌道外の追加ボクセル登録なし。
                probes = np.repeat(path[1:-1,None,:],len(self.limits)+1,axis=1)
                probes[:,1:,:] += np.eye(len(self.limits))*.001
                values = np.asarray(self.obstacle_cost(probes.reshape(-1,len(self.limits)))).reshape(len(path)-2,-1)
                gradient += (values[:,1:]-values[:,:1])/.001
            update = inverse@gradient
            norm = float(np.max(np.abs(update)))
            if norm<1e-6:
                break
            update *= min(1.,.12/norm)
            has_update = False
            for step in (1.,.5,.25,.125,.0625):
                candidate = path.copy()
                candidate[1:-1] = np.clip(path[1:-1]-step*update,self.limits[:,0],self.limits[:,1])
                candidate_cost = cost(candidate)
                if candidate_cost<current_cost-1e-10 and self.check(self.dense_path(candidate)):
                    path,current_cost,has_update = candidate,candidate_cost,True
                    num_updates += 1
                    break
            if not has_update:
                break
        return path,dict(initial_cost=initial_cost,final_cost=current_cost,num_updates=num_updates)

    def plan(self, start, goal, method='gng_chomp', graph=None):
        started = time.perf_counter()
        if method not in ('gng_chomp','rrt_chomp','direct'):
            raise ValueError('未対応の往復軌道生成方式')
        start,goal = np.asarray(start,dtype=float),np.asarray(goal,dtype=float)
        if start.shape!=(len(self.limits),) or goal.shape!=start.shape or not self.check(np.stack((start,goal))):
            raise ValueError('往復軌道の端点または可動域の不正')
        if np.linalg.norm(goal-start)<1e-8:
            raise ValueError('往復目標の重複')
        seed_kind,seed = 'direct',np.stack((start,goal))
        if method=='rrt_chomp':
            seed_kind,seed = 'rrt_connect',self.rrt_connect(start,goal)
        elif not self.can_connect(start,goal):
            if method=='direct':
                raise ValueError('直線基準軌道の区間衝突')
            seed = self.graph_route(start,goal,graph)
            seed_kind = 'gng'
            if seed is None:
                seed_kind,seed = 'rrt_connect',self.rrt_connect(start,goal)
        if method=='direct':
            path,stats = seed,{}
        else:
            path,stats = self.improve(seed)
        if not self.check(self.dense_path(path)):
            raise ValueError('改善後の往復軌道の区間衝突')
        return path,dict(method=method,seed_kind=seed_kind,optimizer='none' if method=='direct' else 'chomp_discrete',
            planning_ms=(time.perf_counter()-started)*1000,num_checked_poses=self.num_checks,
            seed_length_rad=float(path_lengths(seed)[-1]),path_length_rad=float(path_lengths(path)[-1]),
            max_validation_step_rad=.04,**stats)


class joint_path_reference:
    """実姿勢への投影による往復追従。回避中の時計進行・強制的な位相飛越しなし。"""
    def __init__(self, path):
        self.path = np.asarray(path,dtype=float)
        if self.path.ndim!=2 or len(self.path)<2 or not np.isfinite(self.path).all():
            raise ValueError('往復基準軌道の形式不正')
        self.lengths = path_lengths(self.path)
        if self.lengths[-1]<1e-8:
            raise ValueError('往復基準軌道の重複')
        self.goal = None
        self.progress = 0.
        self.target = self.path[0].copy()
        self.velocity = np.zeros(self.path.shape[1])
        self.dev_rad = 0.

    def update(self, positions, goal):
        path = self.path if goal=='b' else self.path[::-1]
        lengths = path_lengths(path)
        if self.goal!=goal:
            self.goal,self.progress = goal,0.
        delta = np.diff(path,axis=0)
        norm = np.sum(delta**2,axis=1)
        ratios = np.clip(np.sum((positions-path[:-1])*delta,axis=1)/np.maximum(norm,1e-12),0.,1.)
        points = path[:-1]+ratios[:,None]*delta
        distances = np.linalg.norm(points-positions,axis=1)
        projected = lengths[:-1]+ratios*np.diff(lengths)
        idx = int(np.argmin(distances**2+1e-4*(projected-self.progress)**2))
        self.dev_rad = float(distances[idx])
        self.progress = float(lengths[idx]+ratios[idx]*(lengths[idx+1]-lengths[idx]))
        # 経路からの離隔に応じた復帰優先。動的回避による関節姿勢変更の許容。
        lookahead = max(.12,.65-self.dev_rad)
        self.target = interpolate_path(path,lengths,min(lengths[-1],self.progress+lookahead))
        self.velocity = 3.*(self.target-positions)
        self.velocity *= min(1.,2./max(float(np.max(np.abs(self.velocity))),1e-9))
        return self.velocity
