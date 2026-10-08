"""点群のC++処理。初回のみコンパイル、ソースとPython ABIごとのキャッシュ。"""
from array import array
import fcntl
import hashlib
import importlib.util
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sysconfig
import tempfile
import threading

_backend = None
_backend_lock = threading.Lock()


def ensure_native():
    global _backend
    if _backend is not None:
        return _backend
    with _backend_lock:
        if _backend is not None:
            return _backend
        source = Path(__file__).with_name('pointcloud_native.cpp')
        compiler = shlex.split(os.environ.get('CXX', 'c++'))
        flags = ['-std=c++17', '-O3', '-fPIC', '-shared', '-ffp-contract=off']
        identity = repr((compiler, flags, sysconfig.get_config_var('SOABI'), platform.machine())).encode()
        source_bytes = source.read_bytes()
        digest = hashlib.sha256(source_bytes + identity).hexdigest()[:24]
        cache_root = Path(tempfile.gettempdir()) / f'topo-pointcloud-native-{os.getuid()}'
        cache_root.mkdir(mode=0o700, exist_ok=True)
        if cache_root.is_symlink() or cache_root.stat().st_uid != os.getuid() or cache_root.stat().st_mode & 0o077:
            raise RuntimeError('C++点群キャッシュの所有者・アクセス権が不正です')
        cache = cache_root / digest
        cache.mkdir(exist_ok=True)
        target = cache / ('_pointcloud_native' + sysconfig.get_config_var('EXT_SUFFIX'))
        with (cache / 'build.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if not target.exists():
                include = Path(sysconfig.get_path('include'))
                if not (include / 'Python.h').exists():
                    raise RuntimeError('C++点群処理にはPython開発ヘッダーが必要です。python3-devを導入してください')
                with tempfile.TemporaryDirectory(dir=cache) as build_dir:
                    output = Path(build_dir) / target.name
                    build_source = Path(build_dir) / source.name
                    build_source.write_bytes(source_bytes)
                    command = compiler + flags + ['-I' + str(include), str(build_source), '-o', str(output)]
                    try:
                        subprocess.run(command, check=True, capture_output=True, text=True, timeout=60)
                    except (OSError, subprocess.SubprocessError) as error:
                        detail = getattr(error, 'stderr', None) or str(error)
                        raise RuntimeError('C++点群処理のビルド失敗。g++とpython3-devを確認してください: ' + detail) from error
                    output.replace(target)
        spec = importlib.util.spec_from_file_location('_pointcloud_native', target)
        backend = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(backend)
        _backend = backend
        return backend


def byte_array(data):
    output = array('B')
    output.frombytes(data)
    return output


def build_depth_points(calibration, data, colors=None):
    width, height = calibration['width'], calibration['height']
    if type(width) is not int or type(height) is not int or not 1 <= width <= 1920 or not 1 <= height <= 1080:
        raise ValueError('深度画像の寸法が不正です')
    output = array('B', [0]) * (width * height * (20 if colors is not None else 12))
    ensure_native().build_depth(data, colors, width, height,
                               *(float(calibration[key]) for key in ('fx', 'fy', 'ppx', 'ppy')), output)
    return output


def colorize_points(xyz, colors, depth=None):
    num_bytes = memoryview(xyz).nbytes
    if num_bytes % 12 or num_bytes // 12 > 2073600:
        raise ValueError('点群のデータ長が不正です')
    output = array('B', [0]) * (num_bytes // 12 * 20)
    ensure_native().colorize(xyz, colors, depth, output)
    return output


def validate_payload(data, num_points, num_pixels, has_color):
    ensure_native().validate_payload(data, num_points, num_pixels, has_color)


if __name__ == '__main__':
    print(ensure_native().__file__)
