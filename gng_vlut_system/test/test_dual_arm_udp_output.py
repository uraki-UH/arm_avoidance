"""実ネットワーク不使用でのUDP全関節対応・許可条件・失効停止の検証。"""

from array import array
import math
from pathlib import Path
import socket
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_udp_output import udp_output
from joint_command_model import joint_command_model


class fake_socket:
    def __init__(self, *args):
        assert args == (socket.AF_INET, socket.SOCK_DGRAM)
        self.incoming = []
        self.sent = []
        self.is_closed = False
        self.has_send_error = False
        self.has_receive_error = False
        self.has_short_send = False
        self.is_blocking = None
        self.address = None

    def setblocking(self, value):
        self.is_blocking = value

    def bind(self, address):
        self.address = address

    def recvfrom(self, max_size):
        assert max_size == 4096
        if self.has_receive_error:
            raise OSError('receive failure')
        if not self.incoming:
            raise BlockingIOError()
        return self.incoming.pop(0)

    def sendto(self, packet, address):
        if self.has_send_error:
            raise OSError('send failure')
        self.sent.append((packet, address))
        return len(packet)-1 if self.has_short_send else len(packet)

    def close(self):
        self.is_closed = True


@pytest.fixture
def urdf_path(tmp_path):
    path = tmp_path/'robot.urdf'
    joints = ''.join(
        '<joint name="joint'+str(idx)+'" type="revolute">'
        '<limit lower="-1" upper="1" velocity="'+('0.1' if idx == 0 else '1')+'"/>'
        '</joint>' for idx in range(19))
    path.write_text('<robot name="test">'+joints+
                    '<joint name="mimic" type="revolute">'
                    '<limit lower="-1" upper="1" velocity="1"/>'
                    '<mimic joint="joint0" multiplier="-1"/></joint></robot>')
    return path


@pytest.fixture
def config():
    return {
        'joint_names': ['joint'+str(idx) for idx in range(19)],
        'joint_scales': [1.0]*19, 'joint_offsets_deg': [0.0]*19,
        'robot_ip': '127.0.0.1', 'robot_port': 21001,
        'listen_ip': '127.0.0.1', 'listen_port': 21002, 'feedback_source_port': 21001,
        'enable_packet': 'ENABLE', 'stop_packet': 'STOP',
    }


@pytest.fixture
def output(urdf_path, config):
    instance = udp_output(urdf_path, config, socket_factory=fake_socket)
    yield instance
    instance.socket.has_send_error = False
    instance.close()


def feedback(output, values=None, now=0.0, source=None):
    values = [0]*19 if values is None else values
    data = ('agl,'+','.join(map(str, values))+',').encode('ascii')
    output.socket.incoming.append((data, output.feedback_source if source is None else source))
    output.poll(now)


def target(output, now=0.0, stamp_sec=None, value=0.0, idx=1):
    values = [0.0]*19
    values[idx] = value
    return output.update_target(output.joint_names, values, now if stamp_sec is None else stamp_sec, now)


def enable(output, now=0.0):
    feedback(output, now=now)
    target(output, now=now)
    output.enable(now)


def test_construct_and_off_input_never_send(output):
    assert output.socket.is_blocking is False
    assert output.socket.address == ('127.0.0.1', 21002)
    assert output.socket.sent == []
    feedback(output)
    target(output)
    assert output.tick(0.02) is False
    assert output.socket.sent == []
    assert output.status(0)['has_fresh_target']
    assert output.status(0)['has_fresh_feedback']
    assert not output.status(0)['enable_hardware_output']
    assert not output.status(0)['is_physical_stop_confirmed']


def test_ros_array_positions_supported(output):
    assert output.update_target(output.joint_names, array('d', [0.0]*19), 0.0, 0.0)
    assert output.status(0.0)['has_fresh_target']


@pytest.mark.parametrize('key,value', [
    ('joint_names', ['joint0']*19), ('joint_names', ['joint'+str(idx) for idx in range(18)]),
    ('joint_names', ['joint'+str(idx) for idx in range(18)]+['mimic']),
    ('joint_names', ['joint'+str(idx) for idx in range(18)]+['unknown']),
    ('joint_names', [None]*19), ('joint_scales', [1.0]*18), ('joint_scales', [0.0]*19),
    ('joint_scales', [math.nan]*19), ('joint_scales', [True]*19),
    ('joint_offsets_deg', None), ('joint_offsets_deg', [math.inf]*19),
    ('robot_ip', 'localhost'), ('robot_ip', '::1'), ('robot_ip', '0.0.0.0'),
    ('robot_ip', '255.255.255.255'), ('robot_ip', '239.0.0.1'), ('robot_ip', 2130706433),
    ('listen_ip', '0.0.0.0'), ('listen_ip', 'localhost'), ('listen_port', 0),
    ('robot_port', 65536), ('robot_port', True), ('feedback_source_port', '21001'),
    ('max_command_age_sec', 0), ('max_command_age_sec', 1.1), ('max_feedback_age_sec', math.nan),
    ('max_start_dev_rad', True), ('max_tracking_dev_rad', -1), ('max_joint_velocity', math.inf),
    ('update_hz', 101), ('update_hz', 1), ('stop_packet', '\x00'), ('enable_packet', '停止'),
])
def test_invalid_config_rejected_before_socket(urdf_path, config, key, value):
    config[key] = value
    def no_socket(*args):
        pytest.fail('設定拒否前のsocket作成')
    with pytest.raises(ValueError):
        udp_output(urdf_path, config, socket_factory=no_socket)


@pytest.mark.parametrize('key', ['robot_ip', 'listen_ip', 'robot_port', 'listen_port',
                                 'feedback_source_port', 'joint_names', 'joint_scales', 'joint_offsets_deg'])
def test_explicit_fields_required(urdf_path, config, key):
    config.pop(key)
    with pytest.raises(ValueError):
        udp_output(urdf_path, config, socket_factory=fake_socket)


def remote_config(config):
    config.update(robot_ip='192.0.2.10', listen_ip='0.0.0.0', is_mapping_verified=True,
                  is_receiver_stop_verified=True, is_receiver_watchdog_verified=True,
                  receiver_watchdog_sec=0.3)
    return config


@pytest.mark.parametrize('key,value', [
    ('is_mapping_verified', False), ('is_mapping_verified', 'true'),
    ('is_receiver_stop_verified', False), ('is_receiver_watchdog_verified', False),
    ('receiver_watchdog_sec', 0), ('receiver_watchdog_sec', 2),
    ('receiver_watchdog_sec', 0.01), ('enable_packet', ''), ('stop_packet', ''),
    ('stop_packet', 'ENABLE'),
])
def test_remote_interlock_required(urdf_path, config, key, value):
    remote_config(config)[key] = value
    with pytest.raises(ValueError):
        udp_output(urdf_path, config, allow_remote_udp=True, socket_factory=fake_socket)


def test_remote_also_requires_explicit_runtime_permission(urdf_path, config):
    with pytest.raises(ValueError):
        udp_output(urdf_path, remote_config(config), socket_factory=fake_socket)
    instance = udp_output(urdf_path, config, allow_remote_udp=True, socket_factory=fake_socket)
    assert instance.is_remote
    assert not instance.socket.sent
    instance.close()


def test_loopback_does_not_require_control_packets(urdf_path, config):
    config.pop('enable_packet')
    config.pop('stop_packet')
    instance = udp_output(urdf_path, config, socket_factory=fake_socket)
    enable(instance)
    assert instance.socket.sent == []
    instance.tick(0.02)
    assert len(instance.socket.sent) == 1
    instance.disable('OFF')
    assert len(instance.socket.sent) == 1
    instance.close()


def test_calibrated_order_roundtrip_and_trailing_comma(urdf_path, config):
    config['joint_names'].reverse()
    config['joint_scales'] = [-2.0]*19
    config['joint_offsets_deg'] = [3.0]*19
    instance = udp_output(urdf_path, config, socket_factory=fake_socket)
    values = {name: math.radians(idx*0.1) for idx, name in enumerate(instance.joint_names)}
    packet, positions = instance.encode(values)
    assert packet.startswith(b'30,28,26,24,')
    assert packet.endswith(b',')
    assert positions == pytest.approx(values)
    assert instance.decode(b'temp,35\nagl,'+packet+b'\n') == pytest.approx(values)
    instance.close()


@pytest.mark.parametrize('robot', ['max', 'max_long'])
def test_both_real_models_have_explicit_19_joint_mapping(robot, config):
    path = Path(__file__).resolve().parents[2]/'urdf'/('topo_dual_arm_'+robot)/'topo_dual_arm_max.urdf'
    config['joint_names'] = joint_command_model(path).independent_names
    instance = udp_output(path, config, socket_factory=fake_socket)
    packet, _ = instance.encode(dict.fromkeys(instance.joint_names, 0.0))
    assert packet == b'0,'*19
    assert set(instance.decode(b'agl,'+packet)) == set(config['joint_names'])
    instance.close()


@pytest.mark.parametrize('source', [('127.0.0.2', 21001), ('127.0.0.1', 21002)])
def test_feedback_requires_matching_source_ip_and_port(output, source):
    feedback(output, source=source)
    assert not output.status(0)['has_fresh_feedback']
    assert output.socket.sent == []


def test_receive_batch_bound(output):
    output.socket.incoming = [(b'agl,'+b'0,'*19, output.feedback_source)]*40
    output.poll(0)
    assert len(output.socket.incoming) == 8
    assert output.status(0)['has_fresh_feedback']


@pytest.mark.parametrize('packet', [
    b'agl,'+b'0,'*18, b'agl,'+b'0,'*20, b'agl,nan,'+b'0,'*18,
    b'agl,1.2,'+b'0,'*18, b'agl,inf,'+b'0,'*18, b'agl,,'+b'0,'*18,
    b'agl,2147483648,'+b'0,'*18, b'agl,-2147483649,'+b'0,'*18,
    b'agl,99999999999999999999,'+b'0,'*18, b'agl,\xff,'+b'0,'*18,
    b'agl,1000,'+b'0,'*18, b'temp,23', b'agl,'+b'0,'*19+b'\nagl,'+b'0,'*19,
])
def test_invalid_feedback_invalidates_without_output_when_off(output, packet):
    feedback(output)
    output.socket.incoming.append((packet, output.feedback_source))
    output.poll(0.1)
    assert not output.status(0.1)['has_fresh_feedback']
    assert not output.socket.sent


def test_invalid_feedback_during_output_sends_stop(output):
    enable(output)
    output.socket.incoming.append((b'agl,0,', output.feedback_source))
    with pytest.raises(ValueError):
        output.poll(0.02)
    assert not output.is_enabled
    assert output.socket.sent[-1][0] == b'STOP'
    assert not output.target


def test_feedback_only_has_small_physical_rounding_tolerance(output):
    feedback(output, [573]+[0]*18)
    assert output.status(0)['has_fresh_feedback']
    with pytest.raises(ValueError):
        target(output, value=1.00001, idx=0)


@pytest.mark.parametrize('value', [True, math.nan, math.inf, 1.001, 10**1000])
def test_invalid_target_value_clears_target(output, value):
    target(output)
    with pytest.raises(ValueError):
        target(output, now=0.1, value=value)
    assert not output.target
    assert not output.status(0.1)['has_fresh_target']


@pytest.mark.parametrize('names,positions', [(['joint0'], [0]),
    (['joint0']*19, [0]*19), (['joint'+str(idx) for idx in range(18)]+['mimic'], [0]*19),
    (['joint'+str(idx) for idx in range(19)], [0]*18)])
def test_partial_unknown_or_duplicate_target_rejected(output, names, positions):
    with pytest.raises(ValueError):
        output.update_target(names, positions, 0, 0)


def test_target_duplicate_stamp_does_not_refresh_or_replace(output):
    target(output)
    assert not target(output, now=0.4, stamp_sec=0, value=0.01)
    assert output.target['joint1'] == 0
    assert not output.status(0.51)['has_fresh_target']
    with pytest.raises(ValueError):
        target(output, now=0.5, stamp_sec=-0.1)
    assert not output.target


def test_invalid_target_during_output_sends_stop(output):
    enable(output)
    with pytest.raises(ValueError):
        target(output, now=0.1, value=math.nan)
    assert not output.is_enabled
    assert output.socket.sent[-1][0] == b'STOP'


def test_integer_encode_overflow_rejected_before_enable(urdf_path, config):
    config['joint_offsets_deg'] = [3e8]*19
    instance = udp_output(urdf_path, config, socket_factory=fake_socket)
    with pytest.raises(ValueError, match='32bit'):
        target(instance)
    assert not instance.socket.sent
    instance.close()


@pytest.mark.parametrize('has_target,has_feedback', [(False, False), (True, False), (False, True)])
def test_enable_requires_both_inputs(output, has_target, has_feedback):
    if has_target:
        target(output)
    if has_feedback:
        feedback(output)
    with pytest.raises(ValueError):
        output.enable(0)
    assert not output.socket.sent
    assert not output.is_enabled


def test_enable_requires_small_initial_position_dev(output):
    feedback(output)
    target(output, value=0.1)
    with pytest.raises(ValueError):
        output.enable(0)
    assert not output.socket.sent


def test_send_period_and_off_reenable_requires_new_stamp(output):
    enable(output)
    assert output.socket.sent == [(b'ENABLE', output.destination)]
    assert not output.tick(0)
    assert output.tick(0.02)
    assert output.socket.sent[-1] == (b'0,'*19, output.destination)
    assert not output.tick(0.03)
    output.disable('停止')
    assert output.socket.sent[-1][0] == b'STOP'
    assert not output.tick(0.04)
    assert not target(output, now=0.04, stamp_sec=0)
    with pytest.raises(ValueError):
        output.enable(0.04)
    target(output, now=0.04, stamp_sec=0.04)
    assert output.enable(0.04)


@pytest.mark.parametrize('has_new_target,has_new_feedback', [(False, True), (True, False), (False, False)])
def test_input_timeout_stops_without_angle_send(output, has_new_target, has_new_feedback):
    enable(output)
    if has_new_target:
        target(output, now=0.51)
    if has_new_feedback:
        feedback(output, now=0.51)
    with pytest.raises(ValueError):
        output.tick(0.51)
    assert [packet for packet, _ in output.socket.sent] == [b'ENABLE', b'STOP']


def test_long_tick_gap_cannot_be_hidden_by_new_inputs(output):
    enable(output)
    target(output, now=0.6)
    feedback(output, now=0.6)
    with pytest.raises(ValueError, match='周期'):
        output.tick(0.6)
    assert not output.is_enabled


@pytest.mark.parametrize('idx,value', [(1, 0.04), (0, 0.01)])
def test_velocity_jump_stops_instead_of_clamping(output, idx, value):
    enable(output)
    target(output, now=0.02, value=value, idx=idx)
    with pytest.raises(ValueError, match='速度'):
        output.tick(0.02)
    assert [packet for packet, _ in output.socket.sent] == [b'ENABLE', b'STOP']


def test_small_step_passes_velocity_limit(output):
    enable(output)
    target(output, now=0.02, value=0.005)
    assert output.tick(0.02)
    assert output.commanded['joint1'] == pytest.approx(math.radians(0.3))


def test_tracking_dev_stops(urdf_path, config):
    config['max_tracking_dev_rad'] = 0.004
    instance = udp_output(urdf_path, config, socket_factory=fake_socket)
    enable(instance)
    target(instance, now=0.02, value=0.006)
    with pytest.raises(ValueError, match='追従偏差'):
        instance.tick(0.02)
    assert not instance.is_enabled
    instance.close()


def test_send_and_stop_failure_preserves_cause_and_off_state(output):
    enable(output)
    output.socket.has_send_error = True
    with pytest.raises(ValueError, match='send failure.*停止パケット送信失敗'):
        output.tick(0.02)
    assert not output.is_enabled
    assert not output.target
    assert not output.commanded
    assert output.command_sent_sec is None


def test_direct_disable_failure_stays_off(output):
    enable(output)
    output.socket.has_send_error = True
    with pytest.raises(OSError):
        output.disable('手動停止')
    assert not output.is_enabled
    assert not output.target
    assert not output.commanded
    assert output.detail == '手動停止'


def test_close_failure_still_closes_socket(output):
    enable(output)
    output.socket.has_send_error = True
    with pytest.raises(OSError):
        output.close()
    assert output.is_closed
    assert output.socket.is_closed
    assert not output.is_enabled
    output.close()


def test_receive_error_stops(output):
    enable(output)
    output.socket.has_receive_error = True
    with pytest.raises(ValueError, match='UDP受信失敗'):
        output.poll(0.02)
    assert not output.is_enabled
    assert not output.status(0.02)['has_fresh_feedback']
    assert output.socket.sent[-1][0] == b'STOP'


def test_partial_send_stops(output):
    enable(output)
    output.socket.has_short_send = True
    with pytest.raises(ValueError, match='送信長'):
        output.tick(0.02)
    assert not output.is_enabled


@pytest.mark.parametrize('method', ['tick', 'poll', 'enable'])
def test_nonfinite_time_cannot_enable_output(output, method):
    if method != 'enable':
        enable(output)
    else:
        feedback(output)
        target(output)
    with pytest.raises(ValueError):
        getattr(output, method)(math.nan)
    assert not output.is_enabled


def test_status_never_claims_physical_stop(output):
    enable(output)
    output.disable('停止要求')
    assert not output.status(0)['is_physical_stop_confirmed']
    assert output.status(0)['num_sent'] == 2
