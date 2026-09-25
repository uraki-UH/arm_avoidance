"""実GNGの変更前・OFF・全機能ビルド比較。生成物はartifacts配下だけ。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import statistics
import subprocess
import tarfile

root = Path('/ros2_ws/src')
output = root / 'artifacts/voxel_framework_20260926'
core = Path('ais_gng_cpu/src/gng_cpu')
wrapper = Path('ais_gng_cpu/src/ais_gng/include')


def run(command, log, timeout=300):
    print('start:', shlex.join(map(str, command)), flush=True)
    with open(output / log, 'w') as stream:
        subprocess.run(list(map(str, command)), check=True, stdout=stream, stderr=subprocess.STDOUT,
                       timeout=timeout, env={**os.environ, 'CMAKE_BUILD_PARALLEL_LEVEL': '4'})
    print('finished:', log, flush=True)


def prepare():
    # 比較対象は開始時の保存ソースと今回のソース。既存生成先への上書きなし。
    for name in ('before', 'after'):
        target = output / (name + '_src')
        target.mkdir()
        if name == 'before':
            with tarfile.open(output / 'before/source.tar.gz') as archive:
                archive.extractall(target)
        else:
            shutil.copytree(root / core, target / core)
            shutil.copytree(root / wrapper, target / wrapper)
        for relative, old, new in (
            ('src/cpu/cugng.cpp', 'mt19937 mt(rnd());', 'mt19937 mt(20260926);'),
            ('src/utils/utils.hpp', 'dt = LIMIT(dt, 0.1, 0.5);', 'dt = 0.1;'),
        ):
            path = target / core / relative
            source = path.read_text()
            assert source.count(old) == 1, path
            path.write_text(source.replace(old, new))


def build():
    flags = Path('/ros2_ws/build/ais_gng/CMakeFiles/test_spatial_sampling.dir/flags.make').read_text()
    includes = shlex.split(re.search(r'^CXX_INCLUDES = (.*)$', flags, re.M)[1])
    for name in ('before', 'off', 'full'):
        source = output / ('before_src' if name == 'before' else 'after_src')
        target = output / ('build_' + name)
        enabled = 'ON' if name == 'full' else 'OFF'
        command = ['cmake', '-S', source / core, '-B', target, '-DCMAKE_BUILD_TYPE=Release',
                   '-DGNG_BUILD_BENCHMARKS=ON', '-DGNG_ENABLE_FRAME_LOG=OFF']
        if name != 'before':
            command += [f'-D{key}={enabled}' for key in
                        ('enable_voxel_framework', 'enable_voxel_fuzzy', 'enable_voxel_history')]
        run(command, name + '_configure.log')
        run(['cmake', '--build', target, '-j4'], name + '_build.log')
        run(['ctest', '--test-dir', target, '--output-on-failure'], name + '_test.log')
        run(['g++', '-std=c++20', '-O3', '-shared', '-fPIC', '-DGNG_VERSION=0',
             *(['-Dverification_enable_framework=1'] if name != 'before' else []),
             *[f'-D{key}_build={int(name == "full")}' for key in
               ('enable_voxel_framework', 'enable_voxel_fuzzy', 'enable_voxel_history')],
             '-I' + str(source / wrapper), '-I' + str(source / core / 'include'),
             '-I' + str(source / core / 'src'), *includes,
             root / 'benchmarks/voxel_framework_20260926/fixture.cpp', '-L' + str(target),
             '-Wl,-rpath,' + str(target), '-lgng_cpu', '-o', target / 'fixture.so'], name + '_fixture.log')


def measure(args):
    for trial in range(args.trials):
        names = ['before', 'off', 'full'] if trial % 2 == 0 else ['full', 'off', 'before']
        for enable_mixed in (False, True):
            for name in names:
                target = output / ('build_' + name)
                case = f'{name}_{"mixed" if enable_mixed else "plain"}_{trial}'
                command = ['taskset', '-c', str(args.cpu), 'python3',
                    root / 'benchmarks/gng_followup_efficiency_20260924/benchmark.py',
                    '--library', target / 'libgng_cpu.so', '--config',
                    root / 'ais_gng_cpu/src/ais_gng/config/gng_cpu/at128.yaml',
                    '--bag', args.bag, '--frames', str(args.frames), '--warmup', '20', '--voxel', '.5',
                    '--output', output / (case + '.json')]
                if enable_mixed:
                    command += ['--sampling-provider', target / 'fixture.so', '--features']
                run(command, case + '.log')


def summarize(args):
    summary = {}
    for kind in ('plain', 'mixed'):
        reference = json.loads((output / f'before_{kind}_0.json').read_text())
        for name in ('before', 'off', 'full'):
            results = [json.loads((output / f'{name}_{kind}_{trial}.json').read_text())
                       for trial in range(args.trials)]
            for result in results:
                assert len(result['records']) == len(reference['records'])
                for left, right in zip(reference['records'], result['records']):
                    # 実行時間以外の公開出力、順序、浮動小数点ビット列の全一致。
                    assert {k: v for k, v in left.items() if not k.endswith('_ms')} == {
                        k: v for k, v in right.items() if not k.endswith('_ms')}, (name, kind, left['frame'])
            summary[f'{name}_{kind}'] = {
                'frames': len(reference['records']) * args.trials,
                'mean_ms': statistics.mean(r['mean']['total_ms'] for r in results),
                'trials_ms': [r['mean']['total_ms'] for r in results],
                'peak_rss_kib': [r['peak_rss_kib'] for r in results], 'is_identical': True}
    for name in ('before', 'off', 'full'):
        # 共有ライブラリ同名衝突のない子プロセスでの構造体サイズ取得。
        command = ['python3', '-c', 'import ctypes as c,json,sys; l=c.CDLL(sys.argv[1]); '
                   'print(json.dumps([l.verification_gng_size(),l.verification_sampler_size()]))',
                   str(output / f'build_{name}/fixture.so')]
        summary[name + '_sizes'] = json.loads(subprocess.check_output(command, text=True))
    assert summary['before_sizes'] == summary['off_sizes'] == summary['full_sizes']
    for name in ('before', 'off', 'full'):
        section = output / (name + '_text.bin')
        subprocess.run(['objcopy', '--dump-section', '.text=' + str(section),
                        str(output / f'build_{name}/libgng_cpu.so')], check=True)
        summary[name + '_text_sha256'] = hashlib.sha256(section.read_bytes()).hexdigest()
    assert summary['before_text_sha256'] == summary['off_text_sha256'] == summary['full_text_sha256']
    disassembly = subprocess.check_output(['objdump', '-d', '--no-show-raw-insn',
                                          str(output / 'build_off/fixture.so')], text=True)
    def instructions(symbol):
        block = re.search(r'<' + symbol + r'>:\n(.*?)(?:\n\n|\Z)', disassembly, re.S)[1]
        return [line.split('\t')[-1].strip() for line in block.splitlines()
                if '\t' in line and not re.search(r'\b(nop|nopl|nopw|xchg|data16)\b', line)]
    direct, disabled = instructions('verification_baseline'), instructions('verification_disabled')
    # RIP相対オフセットだけは配置依存。参照先の定数アドレスは一致が必要。
    normalize = lambda items: [re.sub(r'[-0-9a-fx]+\(%rip\)', 'offset(%rip)', item) for item in items]
    assert normalize(direct) == normalize(disabled), (direct, disabled)
    summary['disabled_instructions'] = disabled
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


def matrix():
    for name, enable_fuzzy, enable_history in (('attributes', False, False), ('fuzzy', True, False), ('history', False, True)):
        target = output / ('matrix_' + name)
        run(['cmake', '-S', root / core, '-B', target, '-DCMAKE_BUILD_TYPE=Release',
             '-DGNG_BUILD_BENCHMARKS=ON', '-DGNG_ENABLE_FRAME_LOG=OFF', '-Denable_voxel_framework=ON',
             f'-Denable_voxel_fuzzy={"ON" if enable_fuzzy else "OFF"}',
             f'-Denable_voxel_history={"ON" if enable_history else "OFF"}'], name + '_configure.log')
        run(['cmake', '--build', target, '--target', 'voxel_framework_test', '-j4'], name + '_build.log')
        run(['ctest', '--test-dir', target, '-V', '-R', '^voxel_framework_test$'], name + '_test.log')
    command = ['cmake', '-S', root / core, '-B', output / 'invalid',
               '-Denable_voxel_framework=OFF', '-Denable_voxel_fuzzy=ON']
    result = subprocess.run(list(map(str, command)), capture_output=True, text=True, timeout=30)
    assert result.returncode != 0 and 'requires enable_voxel_framework' in result.stderr
    (output / 'invalid_config.log').write_text(result.stdout + result.stderr)
    # 利用側が未ビルド機能を要求した際の、静かなフォールバックの不在。
    for feature in ('true, false', 'false, true'):
        result = subprocess.run(['g++', '-std=c++17', '-fsyntax-only', '-x', 'c++', '-',
                                 '-I' + str(root / core / 'include')], input=
            '#include <fuzzrobo/libgng/voxel_framework.hpp>\n'
            f'fuzzrobo::voxel_framework::configured_features<{feature}> invalid;',
            capture_output=True, text=True, timeout=30)
        assert result.returncode != 0 and 'static assertion failed' in result.stderr
    print('matrix and expected compile failures: passed', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('prepare', 'build', 'measure', 'summarize', 'matrix'))
    parser.add_argument('--bag', default='/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3')
    parser.add_argument('--frames', type=int, default=80)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--cpu', type=int, default=4)
    args = parser.parse_args()
    if args.stage == 'prepare': prepare()
    elif args.stage == 'build': build()
    elif args.stage == 'measure': measure(args)
    elif args.stage == 'matrix': matrix()
    else: summarize(args)
