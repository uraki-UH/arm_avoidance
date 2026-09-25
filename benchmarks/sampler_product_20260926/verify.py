"""製品SDKの外部登録拒否と、組込みサンプリングの変更前後の実bag比較。"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import statistics
import subprocess

root = Path('/ros2_ws/src')
output = root / 'artifacts/sampler_product_20260926'
core = Path('ais_gng_cpu/src/gng_cpu')
wrapper = Path('ais_gng_cpu/src/ais_gng/include')
spec = importlib.util.spec_from_file_location('framework_verification',
    root / 'benchmarks/voxel_framework_20260926/verify.py')
framework = importlib.util.module_from_spec(spec)
spec.loader.exec_module(framework)
framework.output = output
run = framework.run


def build():
    flags = Path('/ros2_ws/build/ais_gng/CMakeFiles/test_spatial_sampling.dir/flags.make').read_text()
    includes = shlex.split(re.search(r'^CXX_INCLUDES = (.*)$', flags, re.M)[1])
    for name in ('before', 'development', 'product'):
        source = output / ('before_src' if name == 'before' else 'after_src')
        target = output / ('build_' + name)
        enabled = 'ON' if name == 'product' else 'OFF'
        command = ['cmake', '-S', source / core, '-B', target, '-DCMAKE_BUILD_TYPE=Release',
                   '-DGNG_ENABLE_FRAME_LOG=OFF']
        if name != 'before':
            command += [f'-Dallow_external_sampler={"OFF" if name == "product" else "ON"}']
        command += [f'-D{key}={enabled}' for key in
                    ('enable_voxel_framework', 'enable_voxel_fuzzy', 'enable_voxel_history')]
        run(command, name + '_deterministic_configure.log')
        run(['cmake', '--build', target, '--target', 'gng_cpu', '-j4'], name + '_deterministic_build.log')
        run(['g++', '-std=c++20', '-O3', '-shared', '-fPIC', '-DGNG_VERSION=0',
             *(['-Dverification_builtin=1'] if name != 'before' else []),
             f'-Dallow_external_sampler_build={int(name != "product")}',
             *[f'-D{key}_build={int(name == "product")}' for key in
               ('enable_voxel_framework', 'enable_voxel_fuzzy', 'enable_voxel_history')],
             '-I' + str(source / wrapper), '-I' + str(source / core / 'include'),
             '-I' + str(source / core / 'src'), *includes,
             root / 'benchmarks/voxel_framework_20260926/fixture.cpp', '-L' + str(target),
             '-Wl,-rpath,' + str(target), '-lgng_cpu', '-o', target / 'fixture.so'], name + '_fixture.log')


def measure(args):
    for trial in range(args.trials):
        names = ['before', 'development', 'product']
        if trial % 2:
            names.reverse()
        for kind in ('plain', 'mixed'):
            for name in names:
                target = output / ('build_' + name)
                case = f'{name}_{kind}_{trial}'
                command = ['taskset', '-c', str(args.cpu), 'python3',
                    root / 'benchmarks/gng_followup_efficiency_20260924/benchmark.py',
                    '--library', target / 'libgng_cpu.so', '--config',
                    root / 'ais_gng_cpu/src/ais_gng/config/gng_cpu/at128.yaml',
                    '--bag', args.bag, '--frames', str(args.frames), '--warmup', '20', '--voxel', '.5',
                    '--no-legacy-priority', '--output', output / (case + '.json')]
                if kind == 'mixed':
                    command += ['--sampling-provider', target / 'fixture.so', '--features']
                run(command, case + '.log')


def summarize(args):
    summary = {}
    for kind in ('plain', 'mixed'):
        reference = json.loads((output / f'before_{kind}_0.json').read_text())
        for name in ('before', 'development', 'product'):
            results = [json.loads((output / f'{name}_{kind}_{trial}.json').read_text())
                       for trial in range(args.trials)]
            for result in results:
                assert len(result['records']) == len(reference['records'])
                for left, right in zip(reference['records'], result['records']):
                    # 時間以外の全公開出力、候補順序、浮動小数点ビット列の一致。
                    assert {k: v for k, v in left.items() if not k.endswith('_ms')} == {
                        k: v for k, v in right.items() if not k.endswith('_ms')}, (name, kind, left['frame'])
            summary[f'{name}_{kind}'] = {
                'frames': len(reference['records']) * args.trials,
                'mean_ms': statistics.mean(r['mean']['total_ms'] for r in results),
                'trials_ms': [r['mean']['total_ms'] for r in results],
                'peak_rss_kib': [r['peak_rss_kib'] for r in results], 'is_identical': True}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


def audit():
    prefix = output / 'product_install'
    headers = prefix / 'include/fuzzrobo/libgng'
    forbidden = ('gng_set_sampling_rules', 'gng_set_priority_input', 'gng_set_weighted_priority_input')
    api = (headers / 'api.h').read_text()
    symbols = subprocess.check_output(['nm', '-D', '--defined-only', str(prefix / 'lib/libgng_cpu.so')], text=True)
    for symbol in forbidden:
        assert symbol not in api and symbol not in symbols, symbol
    assert 'gng_sampling_rule' not in api and 'gng_set_builtin_sampling' in symbols
    for name in ('voxel_framework.hpp', 'builtin_sampling.hpp'):
        assert not (headers / name).exists()
    exports = (prefix / 'share/gng_cpu/cmake/export_gng_cpuExport.cmake').read_text()
    assert 'gng_cpu::voxel_framework' not in exports
    include = ['-I' + str(prefix / 'include')]
    cases = {
        'approved': ('#include <fuzzrobo/libgng/api.h>\nint main(){return !gng_set_builtin_sampling(nullptr);}', True),
        'forbidden_type': ('#include <fuzzrobo/libgng/api.h>\ngng_sampling_rule rule; int main(){}', False),
        'forbidden_declaration': ('#include <fuzzrobo/libgng/api.h>\nint main(){return gng_set_sampling_rules(nullptr,0);}', False),
        # 独自宣言やマクロ再定義でも復活しない、実体シンボルの不在。
        'forged_link': ('#define allow_external_sampler_build 1\n#include <fuzzrobo/libgng/api.h>\n'
                       'extern "C" unsigned char gng_set_sampling_rules(const void*, unsigned);\n'
                       'int main(){return gng_set_sampling_rules(nullptr,0);}', False),
    }
    for name, (source, is_success) in cases.items():
        result = subprocess.run(['g++', '-std=c++20', '-x', 'c++', '-', *include,
            '-L' + str(prefix / 'lib'), '-Wl,-rpath,' + str(prefix / 'lib'), '-lgng_cpu',
            '-o', str(output / ('sdk_' + name))], input=source, capture_output=True, text=True, timeout=30)
        (output / ('sdk_' + name + '.log')).write_text(result.stdout + result.stderr)
        assert (result.returncode == 0) == is_success, (name, result.stderr)
        if name == 'forged_link':
            assert 'undefined reference' in result.stderr
    subprocess.run([str(output / 'sdk_approved')], check=True,
        env={**os.environ, 'LD_LIBRARY_PATH': str(prefix / 'lib')}, timeout=15)
    dirty = output / 'dirty_install_probe'
    (dirty / 'include/fuzzrobo/libgng').mkdir(parents=True, exist_ok=True)
    marker = dirty / 'include/fuzzrobo/libgng/voxel_framework.hpp'
    if not marker.is_symlink():
        marker.symlink_to('deliberately_absent.hpp')
    result = subprocess.run(['cmake', '--install', str(output / 'product'), '--prefix', str(dirty)],
        capture_output=True, text=True, timeout=30)
    assert result.returncode != 0 and '別のinstall先' in result.stderr
    assert not (dirty / 'lib/libgng_cpu.so').exists() and marker.is_symlink()
    (output / 'sdk_install_guard.log').write_text(result.stdout + result.stderr)
    print('SDK audit: symbols, headers, consumer compile/link, install guard passed', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('prepare', 'build', 'measure', 'summarize', 'audit'))
    parser.add_argument('--bag', default='/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3')
    parser.add_argument('--frames', type=int, default=80)
    parser.add_argument('--trials', type=int, default=2)
    parser.add_argument('--cpu', type=int, default=4)
    args = parser.parse_args()
    if args.stage == 'prepare': framework.prepare()
    elif args.stage == 'build': build()
    elif args.stage == 'measure': measure(args)
    elif args.stage == 'audit': audit()
    else: summarize(args)
