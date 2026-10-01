"""ROS起動不要のGazebo状態表示・操作案内の検証。"""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from dynamixel_sim_keyboard import gazebo_status_label, gazebo_key_help, gazebo_reset_feedback


def test_fault_with_unconfirmed_stop():
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {'is_stop_latched': True},
              'demo': {'state': 'fault'}}
    assert gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1) == (
        'gazebo | mode=avoid | 停止解除待ち(B) | 静止=未確認 | 回避=異常')


@pytest.mark.parametrize('mode, display', [
    ('hold', 'hold'), ('avoidance', 'avoid'), ('leader', 'follow'),
    ('switching', '切替中'), ('stopped', 'stop'),
])
def test_short_modes(mode, display):
    latest = {'sim': {'mode': mode}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'idle'}}
    label = gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1)
    assert f'mode={display} |' in label
    assert '回避=開始待ち' in label
    assert '静止=' not in label
    assert '停止ロック' not in label


@pytest.mark.parametrize('key', ['sim', 'safety', 'demo'])
def test_stale_status_is_not_retained(key):
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {
        'state': 'stopped', 'is_stop_latched': True, 'is_stop_applied': True,
        'is_stopped': True, 'state_age_sec': .1}, 'demo': {'state': 'idle'}}
    received = dict.fromkeys(latest, 10.)
    received[key] = 9.
    label = gazebo_status_label(latest, received, 10.1)
    field = {'sim': 'mode', 'safety': '停止状態', 'demo': '回避'}[key]
    assert f'{field}=未受信/失効' in label
    if key == 'safety':
        assert '確認済み' not in label


def test_stop_confirmation_requires_actual_fresh_stop():
    safety = {'state': 'stopped', 'is_stop_latched': True, 'is_stop_applied': True,
              'is_stopped': True, 'state_age_sec': .1}
    latest = {'safety': safety}
    received = {'safety': 10.}
    assert '静止=確認済み' in gazebo_status_label(latest, received, 10.1)
    safety['state_age_sec'] = 1.
    assert '静止=未確認' in gazebo_status_label(latest, received, 10.1)


def test_stopped_demo_after_reset_means_start_wait():
    latest = {'sim': {'mode': 'hold'}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'stopped'}}
    assert gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1) == (
        'gazebo | mode=hold | 回避=開始待ち')


def test_obstacle_wait_is_distinct_from_manual_stop():
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'running', 'phase': 'obstacle_wait'}}
    label = gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1)
    assert '回避=障害物待ち（離れたら自動再開）' in label
    assert '停止解除待ち' not in label
    assert '障害物待ち' not in gazebo_status_label(latest, dict.fromkeys(latest, 10.), 11.)


def test_waiting_for_reset_is_visible_even_if_demo_is_stopped():
    latest = {'sim': {'mode': 'stopped'}, 'safety': {'is_stop_latched': True},
              'demo': {'state': 'stopped'}}
    assert gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1) == (
        'gazebo | mode=stop | 停止解除待ち(B) | 静止=未確認 | 回避=開始待ち')


def test_missing_or_invalid_status():
    label = gazebo_status_label({}, {}, 10.)
    assert 'None' not in label
    assert '停止状態=未受信/失効' in label
    assert '停止状態=不正' in gazebo_status_label({'safety': {}}, {'safety': 10.}, 10.1)


def test_help_includes_toggle_reset_stop_and_exit():
    assert gazebo_key_help == 'A:回避/保持  B:停止解除→保持  Space:両方停止  Ctrl+C:停止して終了'


def test_stop_reason_stays_on_status_line_and_expires():
    latest = {'sim': {'mode': 'stopped',
                     'detail': '切替操作の拒否: switch_start_demo / 開始時の点群余裕不足:\n31.5 mm'}}
    label = gazebo_status_label(latest, {'sim': 1.}, 1.1)
    assert '理由=回避開始拒否: 開始時の点群余裕不足: 31.5 mm' in label
    assert '\n' not in label
    assert '理由=' not in gazebo_status_label(latest, {'sim': 1.}, 2.)


def test_reset_rejection_visible_without_overwriting_stop_reason():
    message = 'sim_reset: 拒否・応答不正 / Gazeboの実測停止未確認\n静止継続待ち'
    latest = {'sim': {'mode': 'stopped', 'detail': 'ソフト停止要求'},
              'sim_operation': gazebo_reset_feedback(message)}
    label = gazebo_status_label(latest, {'sim': 1.}, 1.1)
    assert '理由=ソフト停止要求' in label
    assert 'B=拒否・応答不正 / Gazeboの実測停止未確認 静止継続待ち' in label
    assert '\n' not in label
    latest['sim']['mode'] = 'hold'
    assert 'B=' not in gazebo_status_label(latest, {'sim': 1.}, 1.1)
    assert gazebo_reset_feedback('enable: 受付済み') is None
