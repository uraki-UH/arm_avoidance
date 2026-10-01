"""実時間比・時刻跳躍による指令周期の半減とまとめ実行の防止。"""
import pytest
from pathlib import Path
import sys
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_avoidance_demo import avoidance_demo, next_control_time


def test_absolute_schedule_does_not_halve_rate():
    scheduled = 1.
    commands = []
    for idx in range(101):
        now = 1.+idx*.05*.88
        if now >= scheduled:
            commands.append(now)
            scheduled = next_control_time(now, scheduled, .05)
    assert 87 <= len(commands) <= 89
    assert all(.043 < b-a < .089 for a, b in zip(commands, commands[1:]))


def test_skips_missed_slots_and_keeps_future_slot():
    assert next_control_time(3.17, 1., .05) == pytest.approx(3.2)
    assert next_control_time(3.18, 3.2, .05) == pytest.approx(3.2)
    assert next_control_time(3.17, 0., .05) == pytest.approx(3.22)


def test_stop_latch_keeps_original_fault():
    state = SimpleNamespace(state='fault', error='joint_age_sec: 1.2')
    avoidance_demo.on_safety_stop(state, SimpleNamespace(data=True))
    assert state.state == 'stopped' and state.is_stop_latched
    assert state.error == 'joint_age_sec: 1.2'
    assert not state.enable_auto_start
