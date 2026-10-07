#!/usr/bin/env python3
"""Humbleからの限定起動依頼とHarmonicセッションの寿命管理。"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
import uuid


max_frame_bytes = 65536
max_heartbeat_age_sec = 5.0
path_roots = ('urdf', 'gng_vlut_system/config')


def shared_path(value, workspace):
    """公開済み読取専用資産内のパス検査。"""
    path = Path(value)
    path = (workspace / path).resolve() if not path.is_absolute() else path.resolve()
    if not any(path.is_relative_to(workspace / root) for root in path_roots):
        raise ValueError('URDF・設定はworkspaceのurdf/またはgng_vlut_system/config/内で指定してください')
    if not path.is_file():
        raise ValueError(f'資産ファイルがありません: {path}')
    return path


def make_request(values, workspace, domain):
    """コンテナ固有の絶対パスを含まない起動依頼。"""
    workspace = Path(workspace).resolve()
    request = dict(values)
    for name in ('urdf', 'task_file'):
        request[name] = str(shared_path(values[name], workspace).relative_to(workspace))
    if '/' in values['scenario'] or values['scenario'].endswith('.yaml'):
        request['scenario'] = str(shared_path(values['scenario'], workspace).relative_to(workspace))
    request['ros_domain_id'] = int(domain)
    return request


def launch_command(request, workspace, output_dir):
    """固定launchと許可済み引数だけのコマンド組立て。"""
    names = {'urdf', 'task_file', 'namespace', 'scenario', 'gui', 'output_dir',
             'enable_autostart', 'ros_domain_id'}
    if not isinstance(request, dict) or set(request) != names:
        raise ValueError('未対応の起動引数です')
    if any(not isinstance(request[name], str) for name in names - {'ros_domain_id'}):
        raise ValueError('起動引数は文字列が必要です')
    domain = request['ros_domain_id']
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_IDは0〜232で指定してください')
    if re.fullmatch(r'sim_[a-zA-Z0-9_]+', request['namespace']) is None:
        raise ValueError('名前空間はsim_で始まる英数字・下線が必要です')
    if request['gui'] not in ('true', 'false'):
        raise ValueError('guiはtrue/falseが必要です')
    if request['output_dir']:
        raise ValueError('Humble経由はoutput_dir未指定で使用してください')
    if request['enable_autostart'] not in ('true', 'false'):
        raise ValueError('enable_autostartはtrue/falseが必要です')
    args = {name: request[name] for name in ('namespace', 'enable_autostart')}
    for name in ('urdf', 'task_file'):
        if Path(request[name]).is_absolute():
            raise ValueError('起動サービスにはworkspace相対パスが必要です')
        args[name] = str(shared_path(request[name], workspace))
    scenario = request['scenario']
    if re.fullmatch(r'[a-z][a-z0-9_]*', scenario) is None:
        if Path(scenario).is_absolute():
            raise ValueError('シナリオにはworkspace相対パスが必要です')
        scenario = str(shared_path(scenario, workspace))
    args.update(scenario=scenario, gui=request['gui'], output_dir=str(output_dir))
    return ['ros2', 'launch', 'gng_vlut_system', 'dual_arm_tasks.launch.py'] + [
        f'{name}:={value}' for name, value in args.items()]


class json_peer:
    """サイズ制限付き改行区切りJSON通信。"""

    def __init__(self, connection):
        self.connection = connection
        self.buffer = b''

    def send(self, message):
        self.connection.sendall(json.dumps(message, ensure_ascii=False).encode() + b'\n')

    def receive(self):
        data = self.connection.recv(8192)
        if not data:
            raise EOFError('接続終了')
        self.buffer += data
        messages = []
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            if len(line) > max_frame_bytes:
                raise ValueError('通信サイズ超過')
            messages.append(json.loads(line))
        if len(self.buffer) > max_frame_bytes:
            raise ValueError('通信サイズ超過')
        return messages


def stop_process(process):
    """このサービスが作成したプロセスグループだけの終了。"""
    for stop_signal, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, stop_signal)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    process.wait(timeout=1)


def check_display(environment):
    """シミュレータ起動前のX11認証・OpenGL接続検査。"""
    if not environment.get('DISPLAY'):
        raise ValueError('GUI用DISPLAYが未設定です。ホストの画面設定で起動サービスを更新してください')
    try:
        result = subprocess.run(['glxinfo', '-B'], env=environment, capture_output=True,
                                text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValueError('GUI描画の接続確認に失敗しました。起動サービスのイメージと画面設定を確認してください') from error
    if result.returncode != 0 or 'OpenGL renderer string:' not in result.stdout:
        detail = (result.stderr or result.stdout).strip()[-1500:]
        raise ValueError('GUIへ接続できません。起動サービスのDISPLAY・XAUTHORITY・描画設定を確認してください: ' + detail)
    return next(line for line in result.stdout.splitlines() if 'OpenGL renderer string:' in line)


def run_session(server, peer, request):
    """接続・ハートビートと連動した一回分のシミュレーション。"""
    with tempfile.TemporaryDirectory(prefix='harmonic_session_') as output:
        command = launch_command(request, server.workspace, output)
        environment = dict(os.environ, ROS_DOMAIN_ID=str(request['ros_domain_id']),
                           GZ_PARTITION='harmonic_' + uuid.uuid4().hex,
                           ROS2CLI_NO_DAEMON='1', PYTHONUNBUFFERED='1')
        if request.get('gui') == 'true':
            renderer = check_display(environment)
            peer.send({'event': 'log', 'text': renderer + '\n'})
        process = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        print(f'開始: pid={process.pid} namespace={request["namespace"]}', flush=True)
        last_heartbeat = time.monotonic()
        try:
            peer.send({'event': 'started'})
            while not server.is_stopping.is_set():
                if time.monotonic() - last_heartbeat > max_heartbeat_age_sec:
                    raise TimeoutError('ハートビート期限切れ')
                readable, _, _ = select.select([peer.connection, process.stdout], [], [], 0.2)
                if peer.connection in readable:
                    for message in peer.receive():
                        if message == {'event': 'stop'}:
                            return 0
                        if message != {'event': 'ping'}:
                            raise ValueError('未対応の操作です')
                        last_heartbeat = time.monotonic()
                if process.stdout in readable:
                    data = os.read(process.stdout.fileno(), 8192)
                    if data:
                        peer.send({'event': 'log', 'text': data.decode(errors='replace')})
                if process.poll() is not None:
                    return process.returncode
            return 0
        finally:
            stop_process(process)
            process.stdout.close()
            print(f'終了: pid={process.pid}', flush=True)


class session_handler(socketserver.BaseRequestHandler):
    """同時起動の拒否とセッション単位の排他制御。"""

    def handle(self):
        self.request.settimeout(2)
        peer = json_peer(self.request)
        if not self.server.session_lock.acquire(blocking=False):
            try:
                peer.send({'event': 'error', 'text': '別のタスクlaunchが実行中です。先に終了してください'})
            except OSError:
                pass
            return
        try:
            deadline = time.monotonic() + 3
            messages = []
            while not messages:
                if time.monotonic() > deadline or self.server.is_stopping.is_set():
                    raise TimeoutError('起動依頼の期限切れ')
                messages = peer.receive()
            if len(messages) != 1:
                raise ValueError('起動依頼は1件だけ指定してください')
            code = run_session(self.server, peer, messages[0])
            peer.send({'event': 'exit', 'code': code})
        except (OSError, EOFError, ValueError) as error:
            print(f'接続終了: {error}', flush=True)
            try:
                peer.send({'event': 'error', 'text': str(error)})
            except OSError:
                pass
        finally:
            self.server.session_lock.release()


class launch_server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    """Docker API非公開のローカル起動サービス。"""

    def __init__(self, socket_path, workspace):
        self.workspace = Path(workspace).resolve()
        self.session_lock = threading.Lock()
        self.is_stopping = threading.Event()
        super().__init__(str(socket_path), session_handler)


def serve(socket_path, workspace):
    socket_path = Path(socket_path)
    # 生存中サービスのソケット削除防止用ロック。
    with socket_path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if socket_path.exists():
            if not socket_path.is_socket():
                raise ValueError('ソケット位置に既存ファイルがあります')
            socket_path.unlink()
        with launch_server(socket_path, workspace) as server:
            os.chmod(socket_path, 0o600)
            signal.signal(signal.SIGTERM, lambda *_: server.is_stopping.set())
            signal.signal(signal.SIGINT, lambda *_: server.is_stopping.set())
            server.timeout = 0.2
            print(f'Harmonic起動受付: {socket_path}', flush=True)
            try:
                while not server.is_stopping.is_set():
                    server.handle_request()
            finally:
                server.is_stopping.set()
                socket_path.unlink(missing_ok=True)


def connect(socket_path, request):
    is_stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: is_stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: is_stopping.set())
    with socket.socket(socket.AF_UNIX) as connection:
        connection.settimeout(2)
        try:
            connection.connect(str(socket_path))
        except OSError as error:
            raise RuntimeError('Harmonic起動サービスへ接続できません。ホストで '
                               'docker compose -f docker/compose.harmonic_launcher.yaml up -d '
                               'を実行してください') from error
        peer = json_peer(connection)
        peer.send(request)
        next_ping = time.monotonic() + 1
        stop_deadline = None
        while True:
            now = time.monotonic()
            if is_stopping.is_set() and stop_deadline is None:
                peer.send({'event': 'stop'})
                stop_deadline = now + 20
            if stop_deadline is not None and now > stop_deadline:
                raise TimeoutError('Harmonic終了確認の期限切れ')
            if stop_deadline is None and now >= next_ping:
                peer.send({'event': 'ping'})
                next_ping = now + 1
            if not select.select([connection], [], [], 0.2)[0]:
                continue
            for message in peer.receive():
                event = message['event']
                if event == 'log':
                    print(message['text'], end='', flush=True)
                elif event == 'error':
                    raise RuntimeError(message['text'])
                elif event == 'exit':
                    return message['code']
                elif event == 'started':
                    print('Harmonicへ接続済み。終了: Ctrl+C', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('serve', 'connect'))
    parser.add_argument('--socket', required=True)
    parser.add_argument('--workspace', default='/workspace')
    parser.add_argument('--request')
    args = parser.parse_args()
    try:
        if args.mode == 'serve':
            if os.environ.get('ROS_DISTRO') != 'jazzy':
                raise RuntimeError('起動サービスにはJazzy/Harmonic環境が必要です')
            serve(args.socket, args.workspace)
            return 0
        return connect(args.socket, json.loads(args.request))
    except (OSError, EOFError, ValueError, RuntimeError) as error:
        print(f'Harmonic起動エラー: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
