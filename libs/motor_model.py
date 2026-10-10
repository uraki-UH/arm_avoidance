"""ROS・物理エンジン非依存のモータ計算API。Python標準ライブラリのみ。

設定: model_type / parameters / has_verified_calibration。
動作入力: 指令方式・指令値、供給電圧[V]、出力軸速度[rad/sec]、刻み[sec]。
出力: 駆動・正味トルク[N m]、電流[A]、駆動電圧[V]、電力・損失[W]。
単体実行: python3 -m libs.motor_model --config libs/motor_model.example.json

拡張: create_motor_model(config, factories={"別方式": factory})。
factoryの入力はparametersの辞書、戻り値はstep(motor_input)とreset()の実装。
未計算の出力はNone。モータごとの状態は各インスタンス内に保持。
機械側の角度・速度・負荷・接触は呼出し側で計算。実機への指令機能なし。

DC等価回路: V = R I + L dI/dt + Ke ω、τ = Kt I。
SI単位・同一モータ軸でKt=Keの理想電磁変換。減速比と効率は別パラメータ。
根拠: https://eecs6302.mit.edu/spring20/prelabs/prelab09/model
電力は刻み終端の瞬時値。Lを含む過渡状態では磁気エネルギー変化も別途必要。
駆動OFF・実現不能な電流制限時はコースト近似。電流の瞬時ゼロ化。
熱・電源電圧上昇・ドライバ損失・BLDC相電流・Dynamixel内部制御は対象外。
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
import math
from typing import Protocol


def _finite(name: str, value: float) -> None:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name}: 有限数が必要です')


def _clip(value: float, bound: float) -> float:
    return min(bound, max(-bound, value))


@dataclass(frozen=True, slots=True)
class motor_input:
    """共通の動作入力。出力軸を基準とする速度・トルク指令。"""

    command_mode: str
    command_value: float
    supply_voltage_v: float
    shaft_velocity_rad_sec: float
    dt_sec: float
    enable_drive: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.command_mode, str) or not self.command_mode:
            raise ValueError('command_mode: 計算方式に対応した指令名が必要です')
        for name in ('command_value', 'supply_voltage_v', 'shaft_velocity_rad_sec', 'dt_sec'):
            _finite(name, getattr(self, name))
        if self.supply_voltage_v < 0 or self.dt_sec <= 0:
            raise ValueError('供給電圧は非負、刻みは正の値が必要です')
        if type(self.enable_drive) is not bool:
            raise ValueError('enable_drive: 真偽値が必要です')


@dataclass(frozen=True, slots=True)
class motor_output:
    """共通の計算結果。摩擦を含む正味トルクと、摩擦を除く駆動トルク。"""

    drive_torque_nm: float
    shaft_torque_nm: float
    current_a: float | None = None
    drive_voltage_v: float | None = None
    back_emf_v: float | None = None
    electric_power_w: float | None = None
    shaft_power_w: float | None = None
    copper_loss_w: float | None = None
    gear_loss_w: float | None = None
    friction_loss_w: float | None = None
    is_voltage_limited: bool | None = None
    is_current_limited: bool | None = None
    is_driver_protected: bool | None = None

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if name.startswith('is_'):
                if value is not None and type(value) is not bool:
                    raise ValueError(f'{name}: 真偽値またはNoneが必要です')
            elif value is not None:
                _finite(name, value)
        if self.drive_torque_nm is None or self.shaft_torque_nm is None:
            raise ValueError('駆動・正味トルクは必須です')


class motor_calculator(Protocol):
    """計算方式の差替え契約。エンジン、ROS、ロボット名への依存なし。"""

    def step(self, value: motor_input) -> motor_output: ...
    def reset(self) -> None: ...


@dataclass(frozen=True, slots=True)
class dc_motor_parameters:
    """DC等価モデルの特性。電気定数はモータ軸、粘性抵抗は出力軸基準。"""

    resistance_ohm: float
    torque_nm_per_a: float
    max_current_a: float
    max_voltage_v: float
    inductance_h: float = 0.0
    back_emf_v_per_rad_sec: float | None = None
    gear_ratio: float = 1.0
    gear_efficiency: float = 1.0
    viscous_friction_nm_per_rad_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.back_emf_v_per_rad_sec is None:
            object.__setattr__(self, 'back_emf_v_per_rad_sec', self.torque_nm_per_a)
        for name in self.__dataclass_fields__:
            _finite(name, getattr(self, name))
        for name in ('resistance_ohm', 'torque_nm_per_a', 'max_current_a', 'max_voltage_v',
                     'back_emf_v_per_rad_sec', 'gear_ratio', 'gear_efficiency'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name}: 正の値が必要です')
        if self.inductance_h < 0 or self.viscous_friction_nm_per_rad_sec < 0 or self.gear_efficiency > 1:
            raise ValueError('インダクタンス・粘性抵抗は非負、減速効率は正で1以内です')
        if not math.isclose(self.torque_nm_per_a, self.back_emf_v_per_rad_sec, rel_tol=1e-9):
            raise ValueError('DC等価モデルのKtとKeは同一軸のSI単位で一致が必要です')


class dc_motor_model:
    """巻線RL・逆起電力・電流制限・減速機の軽量モデル。"""

    def __init__(self, parameters: Mapping) -> None:
        self.parameters = dc_motor_parameters(**parameters)
        self.current_a = 0.0

    def reset(self) -> None:
        self.current_a = 0.0

    def _requested_voltage(self, value: motor_input, back_emf_v: float,
                           alpha: float) -> tuple[float, bool]:
        parameters = self.parameters
        if value.command_mode == 'voltage':
            return value.command_value, False
        if value.command_mode == 'duty':
            return value.command_value * value.supply_voltage_v, False
        if value.command_mode not in ('current', 'torque'):
            raise ValueError('DC指令方式はvoltage / duty / current / torqueです')
        target = value.command_value
        if value.command_mode == 'torque':
            factor = (parameters.gear_efficiency if target * value.shaft_velocity_rad_sec >= 0
                      else 1 / parameters.gear_efficiency)
            target /= parameters.torque_nm_per_a * parameters.gear_ratio * factor
        limited = _clip(target, parameters.max_current_a)
        # 一刻みで目標電流に達する理想電流制御。必要電圧は供給電圧で別途制限
        voltage = back_emf_v + parameters.resistance_ohm * (self.current_a + (limited - self.current_a) / alpha)
        return voltage, target != limited

    def step(self, value: motor_input) -> motor_output:
        parameters = self.parameters
        motor_velocity = value.shaft_velocity_rad_sec * parameters.gear_ratio
        back_emf = parameters.back_emf_v_per_rad_sec * motor_velocity
        _finite('モータ軸速度', motor_velocity)
        _finite('逆起電力', back_emf)
        # 刻み内の電圧・速度一定の解析解。刻み幅による陽的積分の発散なし
        alpha = (-math.expm1(-value.dt_sec * parameters.resistance_ohm / parameters.inductance_h)
                 if parameters.inductance_h else 1.0)
        if not 0 < alpha <= 1:
            raise ValueError('電気時定数と刻みの組合せを計算できません')
        requested_voltage, is_current_limited = self._requested_voltage(value, back_emf, alpha)
        _finite('要求電圧', requested_voltage)
        rail = min(value.supply_voltage_v, parameters.max_voltage_v)
        voltage = _clip(requested_voltage, rail)
        is_voltage_limited = voltage != requested_voltage
        is_driver_protected = False
        if not value.enable_drive or rail == 0:
            current, voltage = 0.0, 0.0
        else:
            free_current = self.current_a + ((voltage - back_emf) / parameters.resistance_ohm - self.current_a) * alpha
            _finite('制限前電流', free_current)
            current = _clip(free_current, parameters.max_current_a)
            if free_current != current:
                is_current_limited = True
                # 電流制限に必要な平均電圧。電圧不足時は駆動切離しの近似
                limited_voltage = (back_emf + parameters.resistance_ohm
                                   * (self.current_a + (current - self.current_a) / alpha))
                if abs(limited_voltage) <= rail:
                    voltage = limited_voltage
                else:
                    current, voltage, is_driver_protected = 0.0, 0.0, True
        motor_torque = parameters.torque_nm_per_a * current
        motor_power = motor_torque * motor_velocity
        factor = parameters.gear_efficiency if motor_power >= 0 else 1 / parameters.gear_efficiency
        drive_torque = motor_torque * parameters.gear_ratio * factor
        friction_torque = parameters.viscous_friction_nm_per_rad_sec * value.shaft_velocity_rad_sec
        result = motor_output(
            drive_torque_nm=drive_torque,
            shaft_torque_nm=drive_torque - friction_torque,
            current_a=current, drive_voltage_v=voltage, back_emf_v=back_emf,
            electric_power_w=voltage * current,
            shaft_power_w=(drive_torque - friction_torque) * value.shaft_velocity_rad_sec,
            copper_loss_w=parameters.resistance_ohm * current * current,
            gear_loss_w=motor_power - drive_torque * value.shaft_velocity_rad_sec,
            friction_loss_w=friction_torque * value.shaft_velocity_rad_sec,
            is_voltage_limited=is_voltage_limited, is_current_limited=is_current_limited,
            is_driver_protected=is_driver_protected,
        )
        # 入出力検証完了後の状態更新。失敗時の前回電流の保持
        self.current_a = current
        return result


def create_motor_model(config: Mapping, *,
                       factories: Mapping[str, Callable[[Mapping], motor_calculator]] | None = None,
                       allow_unverified_parameters: bool = True) -> motor_calculator:
    """JSON互換設定からの方式選択。任意方式は呼出し側のfactoryで登録。"""
    if not isinstance(config, Mapping):
        raise ValueError('モータ設定は辞書形式が必要です')
    unknown = set(config) - {'model_type', 'parameters', 'name', 'parameter_source', 'has_verified_calibration'}
    if unknown:
        raise ValueError('未対応の設定項目: ' + ', '.join(sorted(unknown)))
    has_calibration = config.get('has_verified_calibration', False)
    if type(has_calibration) is not bool or type(allow_unverified_parameters) is not bool:
        raise ValueError('校正状態・未校正許可は真偽値が必要です')
    if not allow_unverified_parameters and not has_calibration:
        raise ValueError('未校正のモータ特性です')
    parameters = config.get('parameters')
    if not isinstance(parameters, Mapping):
        raise ValueError('parametersは計算方式ごとの辞書形式が必要です')
    model_type = config.get('model_type')
    if not isinstance(model_type, str) or not model_type:
        raise ValueError('model_typeは登録方式名が必要です')
    available = {'dc_equivalent': dc_motor_model, **(factories or {})}
    if model_type not in available:
        raise ValueError('未登録のモータ計算方式: ' + model_type)
    model = available[model_type](dict(parameters))
    if not callable(getattr(model, 'step', None)) or not callable(getattr(model, 'reset', None)):
        raise ValueError('計算方式にはstep・resetの実装が必要です')
    return model


def main() -> int:
    """モータ設定・動作入力JSONからの単体計算。ファイル書込み・実機通信なし。"""
    import argparse
    import json
    from pathlib import Path
    import sys

    parser = argparse.ArgumentParser(description='汎用モータ計算。出力は標準出力のJSON。')
    parser.add_argument('--config', type=Path, required=True, help='model / inputsを含むJSONのパス')
    parser.add_argument('--require-calibration', action='store_true', help='未校正の特性の拒否')
    args = parser.parse_args()
    try:
        request = json.loads(args.config.read_text(encoding='utf-8'))
        if not isinstance(request, dict) or set(request) != {'model', 'inputs'}:
            raise ValueError('JSONの項目はmodelとinputsです')
        inputs = request['inputs']
        if not isinstance(inputs, list) or not inputs:
            raise ValueError('inputsは空でない動作入力の配列が必要です')
        model = create_motor_model(request['model'], allow_unverified_parameters=not args.require_calibration)
        outputs = [asdict(model.step(motor_input(**value))) for value in inputs]
        print(json.dumps({'model_type': request['model']['model_type'],
                          'has_verified_calibration': request['model'].get('has_verified_calibration', False),
                          'outputs': outputs}, ensure_ascii=False, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError, KeyError, ArithmeticError) as error:
        print('計算エラー: ' + str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
