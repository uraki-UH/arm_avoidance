"""vrm_predictive方式の因果的な短期予測。ロボット形状・ボクセル登録からの独立部品。"""
from collections import deque
import time

import numpy as np


class streaming_predictor:
    """8観測と一次隣接による16入力・24中間素子のMLP。未来ラベルの事前参照なし。"""
    def __init__(self, horizon_sec=.5, max_age_sec=.3):
        if (not np.isfinite(horizon_sec) or not 0 < horizon_sec <= 2.
                or not np.isfinite(max_age_sec) or not 0 < max_age_sec <= 2.):
            raise ValueError('予測時間の不正')
        self.horizon_sec, self.max_age_sec = float(horizon_sec), float(max_age_sec)
        self.reset()

    def reset(self):
        generator = np.random.default_rng(7)
        self.first_weights = generator.normal(0., .15, (16, 24))
        self.first_bias = np.zeros(24)
        self.last_weights, self.last_bias = np.zeros(24), -2.2
        self.history, self.pending = deque(maxlen=8), deque(maxlen=32)
        self.node_ids, self.last_sec = (), None
        self.adjacency_source = None
        self.sample_generator = np.random.default_rng(11)
        self.risk, self.risk_th = np.empty(0), .55
        self.num_inferred_nodes = 0
        self.num_updates = self.num_batches = self.num_positive = self.num_negative = 0
        self.num_missed = self.num_events = self.num_false_alarms = 0
        self.miss_rate = self.false_alarm_rate = 0.
        self.update_ms = self.max_update_ms = self.total_update_ms = 0.

    def features(self):
        """同一ノード順の履歴8点・近傍3点・差分・履歴最大値。"""
        values = list(self.history)
        values = [values[0]]*(8-len(values))+values
        close = np.stack([value[1] for value in values[::-1]], axis=1)
        neighbors = np.stack([value[2] for value in values[::-1]], axis=1)
        return np.column_stack((np.ones(len(close)), close, neighbors[:, [0, 3, 7]],
                                close[:, 0]-close[:, 3], close[:, 0]-close[:, 7],
                                np.max(close, axis=1), neighbors[:, 0]-neighbors[:, 3]))

    def infer(self, features):
        # 狭い行列のCループ一括演算。BLASの多スレッド起動コストへの非依存。
        hidden = np.tanh(np.einsum('ni,ih->nh', features, self.first_weights)+self.first_bias)
        values = np.clip(np.einsum('nh,h->n', hidden, self.last_weights)+self.last_bias, -30., 30.)
        return hidden, 1./(1.+np.exp(-values))

    def mature(self, now, is_unavailable):
        """予測発行後に実際に到着した観測だけによる教師・不一致集計。"""
        for item in self.pending:
            if now <= item['due_sec']+1e-9:
                item['became_unavailable'] |= is_unavailable
        while self.pending and now >= self.pending[0]['due_sec']:
            item = self.pending.popleft()
            # 観測欠落を含む教師の破棄。欠測の安全ラベル化なし。
            if item['due_sec']-item['last_sec'] > self.max_age_sec:
                continue
            is_eligible = item['is_clear']
            target = item['became_unavailable'][is_eligible].astype(float)
            features = item['features'][is_eligible]
            if not len(target):
                continue
            is_predicted = item['risk'][is_eligible] >= item['risk_th']
            num_events = int(np.count_nonzero(target))
            num_missed = int(np.count_nonzero((target == 1) & ~is_predicted))
            num_false = int(np.count_nonzero((target == 0) & is_predicted))
            self.num_events += num_events
            self.num_missed += num_missed
            self.num_false_alarms += num_false
            self.false_alarm_rate = .9*self.false_alarm_rate+.1*num_false/max(1, len(target)-num_events)
            if num_events:
                self.miss_rate = .9*self.miss_rate+.1*num_missed/num_events
                # EMAと変化幅制限による閾値振動の抑制。確率校正や安全保証とは別の調整値。
                change = .04*(.1-self.miss_rate)+.01*self.false_alarm_rate
                self.risk_th = float(np.clip(self.risk_th+np.clip(change, -.015, .015), .2, .85))
            self.num_positive += num_events
            self.num_negative += len(target)-num_events
            # 教師だけの正負別上限サンプリング。全ノードでの予測・不一致集計は維持。
            groups = [np.flatnonzero(target == value) for value in (0, 1)]
            selected = np.concatenate([self.sample_generator.choice(group, min(256, len(group)), replace=False)
                                       for group in groups])
            features, target = features[selected], target[selected]
            hidden, probability = self.infer(features)
            # 希少な進入イベントの消失防止。校正済み確率ではない選好リスク。
            weight = np.where(target > 0, len(target)/max(1, 2*np.count_nonzero(target)),
                              len(target)/max(1, 2*np.count_nonzero(target == 0)))
            gradient = (probability-target)*weight/np.sum(weight)
            first_gradient = gradient[:, None]*self.last_weights*(1.-hidden*hidden)
            self.last_weights -= .08*(np.einsum('nh,n->h', hidden, gradient)+.001*self.last_weights)
            self.last_bias -= .08*float(np.sum(gradient))
            self.first_weights -= .08*(np.einsum('ni,nh->ih', features, first_gradient)+.001*self.first_weights)
            self.first_bias -= .08*np.sum(first_gradient, axis=0)
            self.num_batches += 1
        for item in self.pending:
            if now <= item['due_sec']:
                item['last_sec'] = now

    def update(self, node_ids, adjacency, gaps, labels, observation_sec):
        """新規観測単位の更新。同一観測の重複学習・長時間欠測の差分利用なし。"""
        begin = time.perf_counter()
        ids, now = tuple(node_ids), float(observation_sec)
        gaps = np.asarray(gaps, dtype=float)
        if (not ids or len(ids) > 16384 or len(set(ids)) != len(ids) or gaps.shape != (len(ids),)
                or np.any(np.isnan(gaps)) or not np.isfinite(now)
                or any(labels.get(idx) not in (1, 2, 3) for idx in ids)):
            raise ValueError('予測入力の不正')
        if ids != self.node_ids or (self.last_sec is not None and now < self.last_sec):
            self.reset()
            self.node_ids = ids
        if self.last_sec == now:
            return self.risk.copy()
        if self.last_sec is not None and now-self.last_sec > self.max_age_sec:
            self.history.clear()
            self.pending.clear()
        states = np.fromiter((labels[idx] for idx in ids), dtype=np.uint8, count=len(ids))
        is_unavailable = states != 1
        close = np.clip(1.-np.maximum(gaps, 0.)/.12, 0., 1.)
        label_close = np.where(states == 1, 0., np.where(states == 3, .65, 1.))
        close = np.maximum(close, label_close)
        if adjacency is not self.adjacency_source:
            lookup = {idx:pos for pos, idx in enumerate(ids)}
            pairs = [(pos, lookup[other]) for pos, idx in enumerate(ids)
                     for other in adjacency.get(idx, ()) if other in lookup]
            self.edge_sources, self.edge_targets = np.asarray(pairs, dtype=int).reshape(-1, 2).T
            self.degrees = np.bincount(self.edge_sources, minlength=len(ids))
            self.adjacency_source = adjacency
        total = np.bincount(self.edge_sources, weights=close[self.edge_targets], minlength=len(ids))
        neighbors = np.divide(total, self.degrees, out=close.copy(), where=self.degrees > 0)
        self.mature(now, is_unavailable)
        # 実時間差分による冷間予測。8点窓でも観測周期の固定仮定なし。
        trend = np.zeros(len(ids))
        if self.history:
            previous = self.history[max(0, len(self.history)-3)]
            elapsed = now-previous[0]
            has_distance_pair = np.isfinite(gaps) & previous[3]
            difference = np.where(has_distance_pair, close-previous[1], label_close-previous[4])
            speed = np.clip(difference/elapsed, -4., 4.)
            trend = np.where(speed > .02, np.clip(close+speed*self.horizon_sec, 0., 1.), 0.)
        self.history.append((now, close, neighbors, np.isfinite(gaps), label_close))
        features = self.features()
        has_training = self.num_batches >= 8 and self.num_positive >= 16 and self.num_negative >= 16
        # 根拠のない一様な危険視の抑制。現在の不安全判定は既存VLUTの管轄。
        context = np.maximum(close, neighbors)
        selected = np.flatnonzero((~is_unavailable) & (context > .02)) if has_training else np.empty(0, dtype=int)
        if len(selected) > 1024:
            selected = selected[np.argpartition(context[selected], -1024)[-1024:]]
        learned = np.zeros(len(ids))
        if len(selected):
            _, probability = self.infer(features[selected])
            learned[selected] = probability*np.minimum(1., 2.*context[selected])
        self.num_inferred_nodes = len(selected)
        self.risk = np.maximum(trend, learned)
        self.pending.append(dict(due_sec=now+self.horizon_sec, last_sec=now, features=features,
                                 is_clear=~is_unavailable, became_unavailable=np.zeros(len(ids), dtype=bool),
                                 risk=self.risk.copy(), risk_th=self.risk_th))
        self.last_sec = now
        self.num_updates += 1
        self.update_ms = (time.perf_counter()-begin)*1000
        self.max_update_ms = max(self.max_update_ms, self.update_ms)
        self.total_update_ms += self.update_ms
        return self.risk.copy()

    def status(self):
        return dict(horizon_sec=self.horizon_sec, num_updates=self.num_updates, num_batches=self.num_batches,
                    num_positive=self.num_positive, num_events=self.num_events, num_missed=self.num_missed,
                    num_false_alarms=self.num_false_alarms, miss_rate=self.miss_rate, risk_th=self.risk_th,
                    num_inferred_nodes=self.num_inferred_nodes,
                    num_predicted_nodes=int(np.count_nonzero(self.risk >= self.risk_th)),
                    has_trained_model=self.num_batches >= 8 and self.num_positive >= 16 and self.num_negative >= 16,
                    update_ms=self.update_ms, mean_update_ms=self.total_update_ms/max(1, self.num_updates),
                    max_update_ms=self.max_update_ms)
