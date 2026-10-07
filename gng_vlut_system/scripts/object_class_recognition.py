"""テンプレート照合結果からのファジークラス・属性評価。ROS非依存。"""

from copy import deepcopy
import math
import re


def finite_value(value, name, min_value=0.0, max_value=1.0):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or not min_value <= value <= max_value):
        raise ValueError(f'{name}: 数値範囲が不正です。')
    return float(value)


def mapping(value, name, allowed=None):
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f'{name}: 文字列キーのマッピングが必要です。')
    if allowed is not None and set(value) - set(allowed):
        raise ValueError(f'{name}: 未対応の項目 {sorted(set(value) - set(allowed))}')
    return value


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', value):
        raise ValueError(f'識別子が不正です: {value!r}')
    return value


class class_recognizer:
    """候補単位の適合度とシーン内の存在根拠。物体間の対応付けは対象外。"""

    def __init__(self, config, template_ids):
        config = mapping(deepcopy(config), '設定', {
            'version', 'max_candidate_age_sec', 'defaults', 'classes', 'attributes', 'templates'})
        if type(config.get('version')) is not int or config['version'] != 1:
            raise ValueError('versionは1が必要です。')
        self.max_candidate_age_sec = finite_value(
            config.get('max_candidate_age_sec', 5.0), 'max_candidate_age_sec', 0.001, 3600.0)
        default_values = {'min_score': 0.35, 'max_score': 0.85,
                          'min_visible_ratio': 0.15, 'membership_th': 0.55}
        default_values.update(mapping(config.get('defaults', {}), 'defaults', default_values))
        self.rules = {}
        for kind in ('classes', 'attributes'):
            self.rules[kind] = {}
            for name, raw in mapping(config.get(kind, {}), kind).items():
                identifier(name)
                allowed = {*default_values, 'label'}
                if kind == 'classes':
                    allowed.add('parents')
                rule = {**default_values, **mapping(raw, name, allowed)}
                if not isinstance(rule.get('label'), str) or not rule['label'].strip():
                    raise ValueError(f'{name}: labelが必要です。')
                for key in default_values:
                    rule[key] = finite_value(rule[key], f'{name}.{key}')
                if rule['min_score'] >= rule['max_score']:
                    raise ValueError(f'{name}: scoreの範囲が不正です。')
                parents = rule.setdefault('parents', [])
                if (not isinstance(parents, list) or
                        any(not isinstance(parent, str) for parent in parents) or
                        len(parents) != len(set(parents))):
                    raise ValueError(f'{name}: parentsが不正です。')
                self.rules[kind][name] = rule
        self.ancestors = {}

        def visit(name, path):
            if name in path:
                raise ValueError(f'クラス階層の循環: {name}')
            if name not in self.rules['classes']:
                raise ValueError(f'未定義の親クラス: {name}')
            if name in self.ancestors:
                return self.ancestors[name]
            result = set()
            for parent in self.rules['classes'][name]['parents']:
                result.add(parent)
                result.update(visit(parent, path | {name}))
                if (self.rules['classes'][parent]['min_visible_ratio'] >
                        self.rules['classes'][name]['min_visible_ratio']):
                    raise ValueError(f'{parent}: 親の可視率条件が子より厳しくなっています。')
            self.ancestors[name] = result
            return result

        for name in self.rules['classes']:
            visit(name, set())
        self.templates = {}
        for template_id, raw in mapping(config.get('templates', {}), 'templates').items():
            identifier(template_id)
            raw = mapping(raw, template_id, self.rules)
            entries = {}
            for kind in self.rules:
                entries[kind] = dict(mapping(raw.get(kind, {}), f'{template_id}.{kind}'))
                for name, affinity in list(entries[kind].items()):
                    if name not in self.rules[kind]:
                        raise ValueError(f'{template_id}: 未定義の{kind}: {name}')
                    entries[kind][name] = finite_value(affinity, f'{template_id}.{name}')
                if kind == 'classes':
                    for name, affinity in list(entries[kind].items()):
                        for parent in self.ancestors[name]:
                            entries[kind][parent] = max(entries[kind].get(parent, 0.0), affinity)
            self.templates[template_id] = entries
        if (not isinstance(template_ids, (list, tuple)) or not template_ids or
                any(not isinstance(name, str) or not name for name in template_ids) or
                len(template_ids) != len(set(template_ids))):
            raise ValueError('template_idsには重複のないID一覧が必要です。')
        self.template_ids = tuple(template_ids)
        self.candidates = {}
        self.last_time_sec = None

    def update(self, template_id, candidate, now_sec):
        if template_id not in self.template_ids:
            raise ValueError(f'購読対象外のtemplate_id: {template_id}')
        candidate = mapping(candidate, 'candidate')
        if candidate.get('template_id') != template_id:
            raise ValueError('候補と購読topicのtemplate_idが不一致です。')
        if candidate.get('state') not in ('candidate', 'no_hypothesis'):
            raise ValueError('候補のstateが不正です。')
        if type(candidate.get('is_falsified')) is not bool:
            raise ValueError('is_falsifiedには真偽値が必要です。')
        for key in ('score', 'visible_ratio'):
            if candidate['state'] == 'candidate' or key in candidate:
                finite_value(candidate.get(key), key)
        self.check_time(now_sec)
        # 一つのtemplateにつき最新候補だけを保持。対応点群の複製なし。
        self.candidates[template_id] = (now_sec, {
            key: candidate[key] for key in ('state', 'is_falsified', 'score', 'visible_ratio')
            if key in candidate})

    def check_time(self, now_sec):
        finite_value(now_sec, 'now_sec', 0.0, math.inf)
        if self.last_time_sec is not None and now_sec < self.last_time_sec:
            raise ValueError('観測時刻の逆行です。')
        self.last_time_sec = now_sec

    def entry(self, kind, name, membership, state=None):
        rule = self.rules[kind][name]
        if state is None:
            state = 'supported' if membership >= rule['membership_th'] else 'weak'
        return {'label': rule['label'], 'membership': membership, 'state': state}

    def evaluate(self, template_id, now_sec):
        stored = self.candidates.get(template_id)
        age_sec = now_sec - stored[0] if stored else None
        candidate = stored[1] if stored else {}
        state = 'candidate'
        if stored is None:
            state = 'unobserved'
        elif age_sec > self.max_candidate_age_sec:
            state = 'stale'
        elif candidate['is_falsified'] or candidate['state'] == 'no_hypothesis':
            state = 'rejected'
        result = {'template_id': template_id, 'state': state, 'age_sec': age_sec,
                  'match_score': candidate.get('score'),
                  'visible_ratio': candidate.get('visible_ratio'),
                  'has_class_definition': template_id in self.templates}
        for kind in self.rules:
            result[kind] = {}
            for name, affinity in self.templates.get(template_id, {}).get(kind, {}).items():
                rule = self.rules[kind][name]
                if state != 'candidate':
                    result[kind][name] = self.entry(kind, name, None, state)
                    continue
                if candidate['visible_ratio'] < rule['min_visible_ratio']:
                    result[kind][name] = self.entry(kind, name, None, 'insufficient')
                    continue
                # 設定曲線による適合度。確率や認識精度への換算なし。
                membership = (candidate['score'] - rule['min_score']) / (
                    rule['max_score'] - rule['min_score'])
                result[kind][name] = self.entry(
                    kind, name, min(affinity, max(0.0, min(1.0, membership))))
        # 子クラスの根拠を祖先へ伝播。子の適合度を親より大きくしない集合包含。
        for name, value in list(result['classes'].items()):
            if value['membership'] is None:
                continue
            for parent in self.ancestors[name]:
                current = result['classes'][parent]['membership']
                result['classes'][parent] = self.entry(
                    'classes', parent, max(current or 0.0, value['membership']))
        return result

    def snapshot(self, now_sec):
        self.check_time(now_sec)
        hypotheses = [self.evaluate(template_id, now_sec) for template_id in self.template_ids]
        output = {'schema_version': 1, 'scope': 'scene_presence',
                  'is_probability': False, 'is_calibrated': False,
                  'attribute_basis': 'template_annotation', 'hypotheses': hypotheses,
                  'unconfigured_template_ids': [name for name in self.template_ids if name not in self.templates]}
        for kind in self.rules:
            output[kind] = {}
            for name in self.rules[kind]:
                entries = [(item['template_id'], item[kind][name]) for item in hypotheses if name in item[kind]]
                observed = [(template_id, entry) for template_id, entry in entries if entry['membership'] is not None]
                if observed:
                    best = max(observed, key=lambda item: item[1]['membership'])
                    output[kind][name] = {**best[1], 'support_template_id': best[0]}
                else:
                    states = {entry['state'] for _, entry in entries}
                    state = next((item for item in ('insufficient', 'rejected', 'stale') if item in states), 'unobserved')
                    output[kind][name] = self.entry(kind, name, None, state)
        return output
