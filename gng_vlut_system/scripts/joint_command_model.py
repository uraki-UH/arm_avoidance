"""共通関節指令のURDF制約・mimic整合・Dynamixel逆変換。"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as et


@dataclass(frozen=True)
class joint_spec:
    name: str
    min_position: float
    max_position: float
    max_velocity: float
    is_continuous: bool
    parent: str = ''
    multiplier: float = 1.0
    offset: float = 0.0


class joint_command_model:
    def __init__(self, urdf_path):
        self.joints = {}
        for item in et.parse(urdf_path).getroot().findall('joint'):
            if item.get('type') == 'fixed':
                continue
            name = item.get('name')
            limit = item.find('limit')
            mimic = item.find('mimic')
            is_continuous = item.get('type') == 'continuous'
            if not name or name in self.joints or limit is None:
                raise ValueError('URDFの可動関節名・limitが不正です')
            spec = joint_spec(name, -math.inf if is_continuous else float(limit.get('lower')),
                              math.inf if is_continuous else float(limit.get('upper')),
                              float(limit.get('velocity')), is_continuous,
                              '' if mimic is None else mimic.get('joint'),
                              1.0 if mimic is None else float(mimic.get('multiplier', '1')),
                              0.0 if mimic is None else float(mimic.get('offset', '0')))
            if (spec.min_position > spec.max_position or math.isnan(spec.min_position) or
                    math.isnan(spec.max_position) or not math.isfinite(spec.max_velocity) or
                    spec.max_velocity <= 0 or not math.isfinite(spec.multiplier) or
                    spec.multiplier == 0 or not math.isfinite(spec.offset)):
                raise ValueError('URDFの可動範囲・速度・mimicが不正です: '+name)
            self.joints[name] = spec
        if not self.joints:
            raise ValueError('URDFに可動関節がありません')
        self.independent_names = [name for name, spec in self.joints.items() if not spec.parent]
        self.aliases = {name: self.resolve_alias(name) for name in self.joints}
        self.bounds = {name: [self.joints[name].min_position, self.joints[name].max_position,
                              self.joints[name].max_velocity] for name in self.independent_names}
        for name, (parent, multiplier, offset) in self.aliases.items():
            spec = self.joints[name]
            limits = sorted(((spec.min_position-offset)/multiplier,
                             (spec.max_position-offset)/multiplier))
            bounds = self.bounds[parent]
            bounds[0], bounds[1] = max(bounds[0], limits[0]), min(bounds[1], limits[1])
            bounds[2] = min(bounds[2], spec.max_velocity/abs(multiplier))
            if bounds[0] > bounds[1]:
                raise ValueError('mimicと親の可動範囲に共通部分がありません: '+name)

    def resolve_alias(self, name, seen=None):
        seen = set() if seen is None else seen
        if name not in self.joints or name in seen:
            raise ValueError('mimic参照先の欠損または循環です: '+name)
        seen.add(name)
        spec = self.joints[name]
        if not spec.parent:
            return name, 1.0, 0.0
        parent, multiplier, offset = self.resolve_alias(spec.parent, seen)
        return parent, spec.multiplier*multiplier, spec.multiplier*offset+spec.offset

    def canonical_positions(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('関節名・角度の配列長不一致または名前重複です')
        result = {}
        for name, value in zip(names, positions):
            if name not in self.aliases or not math.isfinite(value):
                raise ValueError('未知関節または非有限角度です: '+name)
            parent, multiplier, offset = self.aliases[name]
            target = (value-offset)/multiplier
            if not math.isfinite(target):
                raise ValueError('mimic変換後の角度が非有限です')
            if parent in result and not math.isclose(target, result[parent], abs_tol=1e-6):
                raise ValueError('親とmimicの指令が矛盾しています: '+parent)
            result[parent] = target
        return result

    def feedback_positions(self, names, positions):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('実測関節名・角度の配列長不一致または名前重複です')
        observed = dict(zip(names, positions))
        if any(name not in self.joints or not math.isfinite(value) for name, value in observed.items()):
            raise ValueError('実測値に未知関節または非有限角度があります')
        # 物理シミュレータのmimic追従偏差と指令矛盾の区別。親の実測値を優先
        selected = {name: value for name, value in observed.items()
                    if not self.joints[name].parent or self.aliases[name][0] not in observed}
        return self.canonical_positions(list(selected), list(selected.values()))

    def initial_positions(self):
        return {name: max(bounds[0], min(bounds[1], 0.0)) for name, bounds in self.bounds.items()}

    def expand(self, values):
        return {name: values[parent]*multiplier+offset
                for name, (parent, multiplier, offset) in self.aliases.items() if parent in values}

    def step(self, current, targets, duration_sec, max_joint_velocity, enable_direct_tracking=False):
        if (not math.isfinite(duration_sec) or duration_sec < 0 or
                not math.isfinite(max_joint_velocity) or max_joint_velocity <= 0):
            raise ValueError('補間周期・速度が不正です')
        result = dict(current)
        for name, target in targets.items():
            if name not in current or name not in self.bounds or not math.isfinite(target):
                raise ValueError('指令対象の初期状態または角度が不正です: '+name)
            min_position, max_position, max_velocity = self.bounds[name]
            target = max(min_position, min(max_position, target))
            delta = target-current[name]
            if self.joints[name].is_continuous:
                delta = math.remainder(delta, 2*math.pi)
            max_step = min(max_velocity, max_joint_velocity)*duration_sec
            if not enable_direct_tracking:
                delta = max(-max_step, min(max_step, delta))
            result[name] = current[name]+delta
        return result


class dynamixel_mapping:
    def __init__(self, model, config):
        self.model = model
        self.entries = {}
        names, ids = config['joint_names'], config['joint_ids']
        scales = config.get('joint_scales', [1.0]*len(names))
        offsets = config.get('joint_offsets_deg', [0.0]*len(names))
        if not names or not (len(names) == len(ids) == len(scales) == len(offsets)):
            raise ValueError('Dynamixel対応表の配列長が不正です')
        motor_parents = {}
        for name, motor_id, scale, offset in zip(names, ids, scales, offsets):
            if (name not in model.joints or name in self.entries or not isinstance(motor_id, int) or
                    not 0 <= motor_id <= 252 or not math.isfinite(scale) or scale == 0 or
                    not math.isfinite(offset)):
                raise ValueError('Dynamixel対応表の関節名・ID・校正値が不正です')
            parent = model.aliases[name][0]
            if motor_id in motor_parents and motor_parents[motor_id] != parent:
                raise ValueError('独立した関節へのモータID重複です')
            motor_parents[motor_id] = parent
            self.entries[name] = (motor_id, scale, offset)
        self.covered_names = {model.aliases[name][0] for name in names}
        # 同じモータIDに対する校正式の整合性確認
        for position in (0.0, 0.123):
            self.convert({name: position for name in self.covered_names})

    def convert(self, positions):
        if set(positions)-self.covered_names:
            raise ValueError('Dynamixel ID未割当の関節指令です: '+','.join(sorted(set(positions)-self.covered_names)))
        expanded = self.model.expand(positions)
        result = {}
        for name, angle in expanded.items():
            if name not in self.entries:
                continue
            motor_id, scale, offset = self.entries[name]
            motor_deg = math.degrees(angle)/scale-offset
            if not math.isfinite(motor_deg):
                raise ValueError('Dynamixel逆変換後の角度が非有限です')
            if motor_id in result and not math.isclose(motor_deg, result[motor_id], abs_tol=1e-5):
                raise ValueError('同一モータIDへの指令が矛盾しています: '+str(motor_id))
            result[motor_id] = motor_deg
        return sorted(result), [result[motor_id] for motor_id in sorted(result)]
