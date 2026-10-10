"""ブラウザサーバーとROSブリッジの一括起動。通常は既存サービスを維持、--restart指定時は同じ配置・ポートのサービスを再起動。"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen

root = Path(__file__).resolve().parents[2]


def check_service(port, endpoint, key, expected):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1):
            pass
    except ConnectionRefusedError:
        return False
    try:
        with urlopen(f'http://127.0.0.1:{port}{endpoint}', timeout=2) as response:
            value = json.load(response)
    except Exception as error:
        raise RuntimeError(f'{port}番は使用中ですが、対象サービスの応答がありません') from error
    if value.get(key) == expected:
        if expected == 'topo-pointcloud-bridge' and value.get('protocol_version', 1) < 4:
            raise RuntimeError(f'{port}番のROSブリッジは旧版です。bash start_ros.sh --restartで再起動してください')
        return True
    raise RuntimeError(f'{port}番は別サービスが使用中です')


def restart_existing(port, bridge_port):
    """同じ配置・ポートの起動元とブリッジのみを対象とした明示再起動。"""
    targets = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode().rstrip('\0').split('\0')
            cwd = (entry / 'cwd').resolve()
            if len(args) < 2 or not Path(args[0]).name.startswith('python'):
                continue
            script = (cwd / args[1]).resolve()
            def option(name, default):
                for idx, value in enumerate(args[2:], 2):
                    if value == name:
                        return int(args[idx + 1])
                    if value.startswith(name + '='):
                        return int(value.split('=', 1)[1])
                return default
            if script == Path(__file__).resolve():
                if option('--port', 8877) != port or option('--bridge-port', 8879) != bridge_port:
                    continue
            elif script == Path(__file__).with_name('pointcloud_bridge.py').resolve():
                if option('--port', 8879) != bridge_port:
                    continue
            else:
                continue
            targets.append((entry, (entry / 'stat').read_text().split()[21]))
        except (OSError, ValueError, IndexError):
            continue
    for entry, started in targets:
        try:
            if (entry / 'stat').read_text().split()[21] == started:
                print(f'再起動対象PID: {entry.name}', flush=True)
                os.kill(int(entry.name), signal.SIGINT)
        except ProcessLookupError:
            pass
        except FileNotFoundError:
            pass
    end = time.monotonic() + 15
    while targets:
        remaining = []
        for entry, started in targets:
            try:
                stat = (entry / 'stat').read_text().split()
                if stat[21] == started and stat[2] != 'Z':
                    remaining.append((entry, started))
            except FileNotFoundError:
                pass
        targets = remaining
        if targets and time.monotonic() >= end:
            raise RuntimeError('旧プロセスが停止しません。強制終了せず中断しました')
        time.sleep(.1)


def bridge_revision():
    names = ('pointcloud_bridge.py', 'lazy_output.py', 'follow_bridge.py', 'robot_exchange.py', 'depth_output.py', 'native_points.py', 'pointcloud_native.cpp', 'joint_stream.py', 'physics_scene.py', 'physics_stream.py', 'physics_robot.py', 'oscbf_avoidance/__init__.py', 'oscbf_avoidance/filter.py', 'oscbf_avoidance/defaults.json')
    return tuple((Path(__file__).parent / name).stat().st_mtime_ns for name in names)


def ensure_services(args):
    """起動済みサービスの再利用と、未起動サービスのバックグラウンド起動。"""
    import fcntl

    log_path = Path(f'/tmp/topo-simulator-{os.getuid()}-{args.port}-{args.bridge_port}.log')
    lock_path = log_path.with_suffix('.lock')

    def is_ready():
        has_server = check_service(args.port, '/api/health', 'app', 'topo-motion-studio')
        has_bridge = check_service(args.bridge_port, '/api/points/status', 'service', 'topo-pointcloud-bridge')
        return has_server and has_bridge

    # 複数ランチャーの同時起動による管理プロセス重複の防止
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if is_ready():
            print('サーバー・ROS/物理ブリッジ: 起動済みサービスを再利用', flush=True)
            return
        with log_path.open('w') as log:
            child = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve()), '--port', str(args.port),
                 '--bridge-port', str(args.bridge_port)],
                cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        try:
            end = time.monotonic() + 45
            while not is_ready():
                if child.poll() is not None:
                    raise RuntimeError(f'一括起動に失敗しました。ログ: {log_path}\n{log_path.read_text()}')
                if time.monotonic() >= end:
                    raise RuntimeError(f'一括起動がタイムアウトしました。ログ: {log_path}')
                time.sleep(.2)
        except BaseException:
            # 起動失敗時は今回の管理プロセスのみ終了。子サービスの停止は管理側へ委任
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=20)
            raise
        print(f'バックグラウンド起動完了: PID={child.pid} / ログ: {log_path}', flush=True)
        print(f'停止: docker compose exec gng_cpu kill -INT {child.pid}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--restart', action='store_true', help='同じ配置・ポートの旧起動を停止して再起動')
    parser.add_argument('--ensure', action='store_true', help='未起動サービスを背景起動し、応答確認後に終了')
    parser.add_argument('--port', type=int, default=8877)
    parser.add_argument('--bridge-port', type=int, default=8879)
    args = parser.parse_args()
    if args.port == args.bridge_port or any(not 1024 <= p <= 65535 for p in (args.port, args.bridge_port)):
        parser.error('異なる1024〜65535のポート番号が必要です')
    if args.ensure and args.restart:
        parser.error('--ensureと--restartの同時指定はできません')
    children = []

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        import rclpy
        if args.ensure:
            ensure_services(args)
            return 0
        if args.restart:
            restart_existing(args.port, args.bridge_port)
        bridge_child = None
        revision = bridge_revision()
        services = [
            (args.port, '/api/health', 'app', 'topo-motion-studio',
             ['node', 'app/server.mjs']),
            (args.bridge_port, '/api/points/status', 'service', 'topo-pointcloud-bridge',
             [sys.executable, 'integrations/ros2/pointcloud_bridge.py', '--port', str(args.bridge_port),
              '--allow-origin', f'http://127.0.0.1:{args.port}', '--allow-origin', f'http://localhost:{args.port}'])]
        for port, endpoint, key, expected, command in services:
            if check_service(port, endpoint, key, expected):
                print(f'{port}番: 起動済みサービスを再利用', flush=True)
                continue
            child = subprocess.Popen(command, cwd=root, env=dict(os.environ, PORT=str(args.port)), start_new_session=True)
            children.append(child)
            if port == args.bridge_port:
                bridge_child = child
                bridge_command = command
            end = time.monotonic() + 20
            while True:
                if child.poll() is not None:
                    raise RuntimeError(f'{port}番のサービス起動に失敗しました')
                if check_service(port, endpoint, key, expected):
                    break
                if time.monotonic() >= end:
                    raise RuntimeError(f'{port}番の起動がタイムアウトしました')
                time.sleep(.2)
        print(f'起動完了: http://127.0.0.1:{args.port}/?model=long\nROS送信先: http://127.0.0.1:{args.bridge_port}\nROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}\nCtrl+Cで今回起動したサービスのみ停止。既存ブリッジのROSドメインはその起動設定を使用。', flush=True)
        while True:
            if bridge_child is not None and bridge_revision() != revision:
                updated = bridge_revision()
                time.sleep(.5)
                if bridge_revision() != updated:
                    continue
                print('ブリッジ更新を検出。管理中のブリッジを再起動します', flush=True)
                bridge_child.send_signal(signal.SIGINT)
                bridge_child.wait(timeout=10)
                children.remove(bridge_child)
                bridge_child = subprocess.Popen(bridge_command, cwd=root, start_new_session=True)
                children.append(bridge_child)
                revision = updated
            if any(child.poll() is not None for child in children):
                raise RuntimeError('サービスが終了しました。上のログを確認してください')
            time.sleep(.5)
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f'起動エラー: {error}', file=sys.stderr)
        return 1
    finally:
        # この起動処理が所有するプロセスだけの終了
        for child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()


if __name__ == '__main__':
    sys.exit(main())
