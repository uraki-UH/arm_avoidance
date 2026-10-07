"""限定起動引数・接続切断・所有プロセス終了の回帰検査。"""
from pathlib import Path
import socket
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import harmonic_launcher as launcher


@pytest.fixture
def asset_request(tmp_path):
    for relative in ('urdf/test/robot.urdf', 'gng_vlut_system/config/tasks.yaml'):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('test')
    return dict(urdf='urdf/test/robot.urdf', task_file='gng_vlut_system/config/tasks.yaml',
                namespace='sim_test', scenario='empty', gui='false', output_dir='',
                enable_autostart='false', ros_domain_id=96)


def test_command_and_request(asset_request, tmp_path):
    values = dict(asset_request)
    values.pop('ros_domain_id')
    values['urdf'] = str(tmp_path / values['urdf'])
    request = launcher.make_request(values, tmp_path, '96')
    assert request == asset_request
    command = launcher.launch_command(request, tmp_path, '/tmp/owned_output')
    assert command[:4] == ['ros2', 'launch', 'gng_vlut_system', 'dual_arm_tasks.launch.py']
    assert 'enable_autostart:=false' in command
    assert 'output_dir:=/tmp/owned_output' in command


@pytest.mark.parametrize('name,value', [
    ('namespace', 'real_robot'), ('namespace', 'sim_x;echo'),
    ('ros_domain_id', True), ('ros_domain_id', -1), ('ros_domain_id', 233),
    ('ros_domain_id', '96'), ('enable_autostart', '1'), ('gui', 'true'),
    ('output_dir', '/workspace'), ('urdf', '/etc/passwd'),
    ('urdf', '../../../etc/passwd'), ('task_file', 'urdf/missing'),
    ('task_file', None), ('scenario', '/etc/passwd'), ('scenario', 'x;echo'),
    ('command', 'sh'),
])
def test_reject_invalid_argument(asset_request, tmp_path, name, value):
    with pytest.raises(ValueError):
        launcher.launch_command(dict(asset_request, **{name: value}), tmp_path, tmp_path)


def test_reject_symlink_escape(asset_request, tmp_path):
    outside = tmp_path / 'outside.yaml'
    outside.write_text('outside')
    link = tmp_path / 'gng_vlut_system/config/link.yaml'
    link.symlink_to(outside)
    with pytest.raises(ValueError):
        launcher.shared_path(link, tmp_path)


def test_peer_partial_frames_and_size():
    left, right = socket.socketpair()
    with left, right:
        peer = launcher.json_peer(left)
        right.sendall(b'{"event":')
        assert peer.receive() == []
        right.sendall(b'"ping"}\n{"event":"stop"}\n')
        assert peer.receive() == [{'event': 'ping'}, {'event': 'stop'}]
        peer.buffer = b'x' * launcher.max_frame_bytes
        right.sendall(b'x')
        with pytest.raises(ValueError):
            peer.receive()


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail('状態待機の期限切れ')


@pytest.fixture
def service(tmp_path, monkeypatch):
    # ROS非依存の模擬launch。子プロセスも同一グループで生成。
    program = '''import os, subprocess, sys, time
child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
print(str(os.getpid()) + "," + str(child.pid), flush=True)
try:
    time.sleep(60)
except KeyboardInterrupt:
    child.wait(timeout=2)
'''
    monkeypatch.setattr(launcher, 'launch_command', lambda *_: [sys.executable, '-c', program])
    socket_path = tmp_path / 'launcher.sock'
    server = launcher.launch_server(socket_path, tmp_path)
    server.timeout = 0.05

    def listen():
        while not server.is_stopping.is_set():
            server.handle_request()

    thread = threading.Thread(target=listen)
    thread.start()
    yield server, socket_path
    server.is_stopping.set()
    thread.join(timeout=5)
    server.server_close()
    assert not thread.is_alive()


def open_session(socket_path):
    connection = socket.socket(socket.AF_UNIX)
    connection.settimeout(5)
    connection.connect(str(socket_path))
    peer = launcher.json_peer(connection)
    peer.send({'ros_domain_id': 96, 'namespace': 'sim_test'})
    text = ''
    while '\n' not in text:
        for message in peer.receive():
            if message['event'] == 'log':
                text += message['text']
    return peer, [int(value) for value in text.strip().split(',')]


def is_running(pid):
    try:
        # 終了済みゾンビを除いた実行中判定。
        return Path(f'/proc/{pid}/stat').read_text().split(') ')[1][0] != 'Z'
    except FileNotFoundError:
        return False


@pytest.mark.parametrize('mode', ['stop', 'disconnect', 'heartbeat', 'shutdown'])
def test_session_cleanup(service, monkeypatch, mode):
    server, socket_path = service
    peer, pids = open_session(socket_path)
    try:
        if mode == 'stop':
            peer.send({'event': 'stop'})
        elif mode == 'disconnect':
            peer.connection.close()
        elif mode == 'heartbeat':
            monkeypatch.setattr(launcher, 'max_heartbeat_age_sec', 0.1)
        else:
            server.is_stopping.set()
        wait_until(lambda: all(not is_running(pid) for pid in pids))
    finally:
        peer.connection.close()
    wait_until(lambda: not server.session_lock.locked(), timeout=18)


def test_busy_request_rejected(service):
    server, socket_path = service
    owner, pids = open_session(socket_path)
    try:
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(2)
            connection.connect(str(socket_path))
            response = launcher.json_peer(connection).receive()
            assert response[0]['event'] == 'error'
        assert all(is_running(pid) for pid in pids)
    finally:
        owner.connection.close()
    wait_until(lambda: not server.session_lock.locked(), timeout=18)


def test_peer_error_does_not_poison_service(service):
    server, socket_path = service
    with socket.socket(socket.AF_UNIX) as connection:
        connection.connect(str(socket_path))
        connection.sendall(b'invalid json\n')
        assert launcher.json_peer(connection).receive()[0]['event'] == 'error'
    wait_until(lambda: not server.session_lock.locked())
    peer, pids = open_session(socket_path)
    peer.connection.close()
    wait_until(lambda: all(not is_running(pid) for pid in pids))
