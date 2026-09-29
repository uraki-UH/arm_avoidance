"""部分指令・mimic・校正の逆変換・未観測関節の検証。"""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from joint_command_model import joint_command_model, dynamixel_mapping


@pytest.fixture
def model(tmp_path):
    path = tmp_path/'robot.urdf'
    path.write_text('''<robot name="test">
      <joint name="arm" type="revolute"><limit lower="-2" upper="2" velocity="1"/></joint>
      <joint name="grip" type="revolute"><limit lower="0" upper="0.8" velocity="0.5"/></joint>
      <joint name="mimic" type="revolute"><limit lower="-0.8" upper="0" velocity="0.5"/>
        <mimic joint="grip" multiplier="-1"/></joint>
      <joint name="waist" type="continuous"><limit velocity="1"/></joint>
    </robot>''')
    return joint_command_model(path)


def test_partial_gripper_preserves_arm(model):
    current = {'arm': 0.7, 'grip': 0.1, 'waist': 0.2}
    out = model.step(current, {'grip': 0.4}, 0.1, 1.0)
    assert out['arm'] == 0.7 and out['waist'] == 0.2
    assert out['grip'] == pytest.approx(0.15)
    assert model.expand(out)['mimic'] == pytest.approx(-0.15)


def test_limits_apply_to_parent_and_mimic(model):
    out = model.step({'arm': 0.0, 'grip': 0.0, 'waist': 0.0}, {'grip': 2.0}, 1.0, 5.0, True)
    assert out['grip'] == 0.8
    assert model.canonical_positions(['mimic'], [-0.2]) == {'grip': 0.2}
    with pytest.raises(ValueError):
        model.canonical_positions(['grip', 'mimic'], [0.2, -0.3])


@pytest.mark.parametrize('names,positions', [(['arm'], []), (['arm', 'arm'], [0, 1]),
                                               (['unknown'], [0]), (['arm'], [math.nan])])
def test_invalid_input_rejected(model, names, positions):
    with pytest.raises(ValueError):
        model.canonical_positions(names, positions)


def test_mapping_inverse_and_duplicate_motor_id(model):
    mapping = dynamixel_mapping(model, {'joint_names': ['arm', 'grip', 'mimic'],
        'joint_ids': [1, 8, 8], 'joint_scales': [-1.0, -0.015, 0.015],
        'joint_offsets_deg': [-90.0, 0.0, 0.0]})
    ids, angles = mapping.convert({'arm': math.radians(30), 'grip': math.radians(0.6)})
    assert ids == [1, 8]
    assert angles == pytest.approx([60, -40])
    assert mapping.convert({'grip': math.radians(0.3)}) == ([8], [-20.0])
    with pytest.raises(ValueError):
        mapping.convert({'waist': 0.0})


def test_conflicting_shared_id_calibration_rejected(model):
    with pytest.raises(ValueError):
        dynamixel_mapping(model, {'joint_names': ['grip', 'mimic'], 'joint_ids': [8, 8],
                                  'joint_scales': [1.0, 1.0]})


def test_model_without_feedback_never_seeds_unknown_command(model):
    with pytest.raises(ValueError):
        model.step({}, {'grip': 0.3}, 0.1, 1.0)


def test_continuous_joint_uses_short_path(model):
    out = model.step({'waist': math.pi-0.01}, {'waist': -math.pi+0.01}, 0.1, 1.0)
    assert out['waist'] == pytest.approx(math.pi+0.01)


def test_feedback_uses_parent_when_mimic_has_tracking_error(model):
    assert model.feedback_positions(['grip', 'mimic'], [0.2, -0.195]) == {'grip': 0.2}
    assert model.feedback_positions(['mimic'], [-0.2]) == {'grip': 0.2}


def test_independent_joints_cannot_share_motor(model):
    with pytest.raises(ValueError):
        dynamixel_mapping(model, {'joint_names': ['arm', 'grip'], 'joint_ids': [1, 1]})
