#!/usr/bin/env python3
"""保存済姿勢のボクセル占有圧縮・独立検証・完全一致辞書の有限再現実験。"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def append_record(path, record):
    with path.open('a') as stream:
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')


def stop_group(process):
    """自身の起動プロセス群だけを対象とする終了処理。"""
    # runner自身による子プロセス群回収のための猶予
    for signum, wait_sec in ((signal.SIGTERM, 10), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=wait_sec)
            break
        except subprocess.TimeoutExpired:
            continue
    process.wait(timeout=5)


def run_command(argv, output, name, timeout_sec, *, has_live_output=False):
    argv = [str(value) for value in argv]
    log_path = output / 'logs' / (name + '.log')
    started = time.monotonic()
    record = {'event': 'started', 'name': name, 'argv': argv, 'cwd': str(output),
              'timeout_sec': timeout_sec, 'time': datetime.now(timezone.utc).isoformat(),
              'log': None if has_live_output else str(log_path)}
    append_record(output / 'command.jsonl', record)
    print(json.dumps({'event': 'started', 'name': name, 'timeout_sec': timeout_sec}), flush=True)
    stream = None if has_live_output else log_path.open('w')
    process = None
    status = 'failed'
    try:
        process = subprocess.Popen(argv, cwd=output, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            returncode = process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            status = 'timeout'
            stop_group(process)
            raise
        except BaseException:
            status = 'cancelled'
            stop_group(process)
            raise
        if returncode != 0:
            raise subprocess.CalledProcessError(returncode, argv)
        status = 'completed'
    finally:
        if stream is not None:
            stream.close()
        finished = {'event': 'finished', 'name': name, 'status': status,
                    'returncode': None if process is None else process.poll(),
                    'elapsed_sec': time.monotonic() - started,
                    'time': datetime.now(timezone.utc).isoformat()}
        append_record(output / 'command.jsonl', finished)
        print(json.dumps(finished, ensure_ascii=False), flush=True)


def run(args):
    script_dir = Path(__file__).resolve().parent
    root = args.root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    runner = args.runner.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f'既存出力先への上書き拒否: {output}')
    if args.repeats < 1:
        raise ValueError('repeatsは正整数が必要')
    if not sys.platform.startswith('linux'):
        raise ValueError('Linux環境が必要')
    for filename in ('prepare.py', 'compress.cpp', 'run_case.py', 'dedup.py', 'verify.py'):
        if not (script_dir / filename).is_file():
            raise FileNotFoundError(script_dir / filename)
    if not runner.is_file():
        raise FileNotFoundError(runner)
    compiler = shutil.which('g++')
    if compiler is None:
        raise FileNotFoundError('g++')
    for model in ('topo_dual_arm_max', 'topo_dual_arm_max_long'):
        for filename in ('gng.bin', 'vlut.bin'):
            path = root / 'gng_vlut_system/gng_results' / model / filename
            if not path.is_file():
                raise FileNotFoundError(path)

    output.mkdir(parents=True, exist_ok=False)
    (output / 'logs').mkdir()
    (output / 'verification').mkdir()
    (output / 'dedup').mkdir()
    python = sys.executable
    data_dir = output / 'data'
    binary = output / 'compress'
    run_command([python, script_dir / 'prepare.py', '--root', root, '--output', data_dir],
                output, 'prepare', 180)
    run_command([compiler, '-O3', '-march=native', '-DNDEBUG', '-std=c++17', '-Wall', '-Wextra', '-Wpedantic',
                 script_dir / 'compress.cpp', '-o', binary], output, 'build', 120)

    cases = []
    case_settings = {}
    for model in ('max', 'long'):
        for radius_cells in (0, 1, 2, 4):
            name = f'{model}_radius_{radius_cells}'
            case_settings[name] = (model, radius_cells)
            cases.append({'name': name,
                          'argv': [python, str(script_dir / 'run_case.py'), str(binary),
                                   str(data_dir / (model + '.voxpose')), str(radius_cells),
                                   '@seed@', '@case_dir@/result'],
                          'metrics': 'result.numeric.json'})
    manifest = output / 'cases.json'
    write_json(manifest, {'cases': cases})
    # 各条件の実行上限と全反復数に基づく有限のバッチ上限
    max_batch_sec = len(cases) * args.repeats * 90 + 30
    run_command([python, runner, manifest, '--output', output / 'results',
                 '--repeats', args.repeats, '--start-seed', 20260930,
                 '--timeout-sec', 90, '--max-total-sec', max_batch_sec],
                output, 'benchmark_batch', max_batch_sec + 20, has_live_output=True)
    report = json.loads((output / 'results/report.json').read_text())
    if report['status'] != 'completed' or report['completed'] != len(cases) * args.repeats:
        raise RuntimeError('全反復の正常完了を確認できない状態')
    records = report['records']
    expected = {(name, trial) for name in case_settings for trial in range(1, args.repeats + 1)}
    actual = {(record['name'], record['trial']) for record in records}
    if actual != expected or len(records) != len(expected):
        raise RuntimeError('試行一覧の欠落または重複')

    trial_results = []
    verifications = {}
    for record in records:
        if record['status'] != 'completed' or not record['cleanup_ok']:
            raise RuntimeError(f"試行の失敗または終了処理の失敗: {record['name']}")
        append_record(output / 'command.jsonl', {'event': 'batch_child', **record})
        # runnerレポートに記録された出力ディレクトリからの結果取得
        case_dir = Path(record['log']).parent
        result = json.loads((case_dir / 'result.metrics.json').read_text())
        trial_results.append({'name': record['name'], 'trial': record['trial'],
                              'seed': record['seed'], 'metrics': result})
        if record['trial'] != 1:
            continue
        model, radius_cells = case_settings[record['name']]
        verification_path = output / 'verification' / (record['name'] + '.json')
        run_command([python, script_dir / 'verify.py', data_dir / (model + '.voxpose'),
                     case_dir / 'result.assignments.csv', radius_cells, verification_path],
                    output, 'verify_' + record['name'], 180)
        verifications[record['name']] = json.loads(verification_path.read_text())

    dictionaries = {}
    for model in ('max', 'long'):
        prefix = output / 'dedup' / model
        run_command([python, script_dir / 'dedup.py', data_dir / (model + '.voxpose'), prefix],
                    output, 'dedup_' + model, 180)
        dictionaries[model] = json.loads(Path(str(prefix) + '.metrics.json').read_text())
    summary = {'status': 'completed', 'repeats': args.repeats,
               'num_cases': len(cases), 'num_completed_trials': len(records),
               'root': str(root), 'script_dir': str(script_dir),
               'trial_results': trial_results, 'verification': verifications,
               'dedup': dictionaries}
    write_json(output / 'summary.json', summary)
    print(json.dumps({'event': 'completed', 'output': str(output),
                      'num_completed_trials': len(records),
                      'num_verified_conditions': len(verifications)}, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'),
                        help='gng_vlut_systemを含むワークスペース')
    parser.add_argument('--output', type=Path, required=True, help='新規出力ディレクトリ')
    parser.add_argument('--repeats', type=int, default=3, help='条件ごとの有限反復数')
    parser.add_argument('--runner', type=Path,
                        default=Path('~/.codex/skills/run-benchmark-batch/scripts/run_batch.py'),
                        help='run-benchmark-batchのrunner')
    args = parser.parse_args()

    def cancel(signum, frame):
        raise KeyboardInterrupt(f'中断シグナル: {signum}')

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, cancel)
    try:
        run(args)
    except KeyboardInterrupt:
        print('中断済み。起動済みプロセス群の終了処理を実施。', file=sys.stderr)
        raise SystemExit(130)


if __name__ == '__main__':
    main()
