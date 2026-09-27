#!/usr/bin/env python3
"""Linux向けの有限反復試験。結果保存・所要時間予測・プロセスグループの後片付け。"""
import argparse
import ctypes
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('有限の正数が必要')
    return result


def utc_time():
    return datetime.now(timezone.utc).isoformat()


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def expand(value, trial, seed, case_dir):
    for key, item in (('@trial@', trial), ('@seed@', seed), ('@case_dir@', case_dir)):
        value = value.replace(key, str(item))
    return value


def group_exists(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def reap_group(pid):
    while True:
        try:
            if os.waitpid(-pid, os.WNOHANG)[0] == 0:
                return
        except ChildProcessError:
            return


def cleanup(process):
    # 親の先行終了後も、起動した同一グループの子孫のみが対象。
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        process.poll()
        if process.returncode is not None:
            reap_group(process.pid)
        if not group_exists(process.pid):
            return True
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            process.poll()
            if process.returncode is not None:
                reap_group(process.pid)
            if not group_exists(process.pid):
                return True
            time.sleep(.02)
    return not group_exists(process.pid)


def load_cases(path):
    cases = json.loads(path.read_text())['cases']
    names = set()
    if not isinstance(cases, list) or not cases:
        raise ValueError('空でないcases配列が必要')
    for case in cases:
        name = case['name']
        if not isinstance(name, str) or not name or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_-' for c in name):
            raise ValueError('case名は英小文字・数字・_・-のみ')
        if name in names:
            raise ValueError('case名の重複')
        names.add(name)
        if not case.get('argv') or not isinstance(case['argv'], list) or not all(isinstance(v, str) for v in case['argv']):
            raise ValueError('argvは空でない文字列配列が必要')
        if not isinstance(case.get('env', {}), dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in case.get('env', {}).items()):
            raise ValueError('envは文字列辞書が必要')
        if 'timeout_sec' in case:
            positive(case['timeout_sec'])
        if 'metrics' in case and (Path(case['metrics']).is_absolute() or '..' in Path(case['metrics']).parts):
            raise ValueError('metricsは試行ディレクトリ内の相対パスが必要')
    return cases


def run(args):
    manifest = args.manifest.resolve()
    cases = load_cases(manifest)
    if args.repeats < 1:
        raise ValueError('repeatsは正整数が必要')
    # 二重fork等で分離されない子孫の回収。失敗時は試験開始前に停止。
    if not sys.platform.startswith('linux') or ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:
        raise RuntimeError('Linux child subreaperの設定失敗')
    destination = args.output.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    schedule = [(trial, case) for trial in range(args.repeats)
                for case in cases[trial % len(cases):]+cases[:trial % len(cases)]]
    report = dict(status='running', started_at=utc_time(), total=len(schedule), completed=0,
                  manifest=str(manifest), repeats=args.repeats, records=[])
    begin = time.monotonic()
    timings = {}
    is_cancelled = False

    def cancel(signum, _frame):
        nonlocal is_cancelled
        is_cancelled = True

    previous = {sig: signal.signal(sig, cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
    with (destination/'events.jsonl').open('w') as events:
        def emit(event, **fields):
            value = dict(event=event, time=utc_time(), **fields)
            line = json.dumps(value, ensure_ascii=False, allow_nan=False)
            events.write(line+'\n'); events.flush()
            print(line, flush=True)

        def remaining_estimate():
            all_times = [v for values in timings.values() for v in values]
            fallback = sum(all_times)/len(all_times) if all_times else args.estimate_sec
            if fallback is None:
                return None
            return sum(sum(timings[c['name']])/len(timings[c['name']]) if c['name'] in timings else fallback
                       for _, c in schedule[len(report['records']):])

        emit('started', total=len(schedule), estimated_remaining_sec=remaining_estimate(), report=str(destination/'report.json'))
        save(destination/'report.json', report)
        try:
            for trial, case in schedule:
                if is_cancelled or time.monotonic()-begin >= args.max_total_sec:
                    report['status'] = 'cancelled' if is_cancelled else 'timeout'
                    break
                case_dir = destination/f"{trial+1:03d}_{case['name']}"
                case_dir.mkdir()
                argv = [expand(v, trial+1, args.start_seed+trial, case_dir) for v in case['argv']]
                env = dict(os.environ, **case.get('env', {}))
                cwd = (manifest.parent/case.get('cwd', '.')).resolve()
                record = dict(name=case['name'], trial=trial+1, seed=args.start_seed+trial,
                              argv=argv, cwd=str(cwd), log=str(case_dir/'output.log'), status='running')
                started = time.monotonic()
                deadline = min(started+float(case.get('timeout_sec', args.timeout_sec)), begin+args.max_total_sec)
                process = None
                try:
                    with (case_dir/'output.log').open('w') as log:
                        process = subprocess.Popen(argv, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                        record['pid'] = process.pid
                        while process.poll() is None:
                            if is_cancelled or time.monotonic() >= deadline:
                                record['status'] = 'cancelled' if is_cancelled else 'timeout'
                                break
                            try:
                                process.wait(timeout=min(1., max(.001, deadline-time.monotonic())))
                            except subprocess.TimeoutExpired:
                                pass
                        if record['status'] == 'running':
                            record['status'] = 'completed' if process.returncode == 0 else 'failed'
                except Exception as error:
                    record.update(status='failed', error=str(error))
                finally:
                    if process:
                        record['cleanup_ok'] = cleanup(process)
                        record['returncode'] = process.poll()
                        if not record['cleanup_ok']:
                            record['status'] = 'cleanup_failed'
                    record['elapsed_sec'] = time.monotonic()-started
                if record['status'] == 'completed' and 'metrics' in case:
                    try:
                        metrics = json.loads((case_dir/case['metrics']).read_text())
                        if not isinstance(metrics, dict) or not metrics or not all(type(v) in (int, float) and math.isfinite(v) for v in metrics.values()):
                            raise ValueError('metricsは空でない有限数値辞書が必要')
                        record['metrics'] = metrics
                    except Exception as error:
                        record.update(status='invalid_metrics', error=str(error))
                report['records'].append(record)
                report['completed'] += record['status'] == 'completed'
                if record['status'] == 'completed':
                    timings.setdefault(case['name'], []).append(record['elapsed_sec'])
                report['elapsed_sec'] = time.monotonic()-begin
                report['estimated_remaining_sec'] = remaining_estimate()
                save(destination/'report.json', report)
                emit('case_finished', name=case['name'], trial=trial+1, status=record['status'],
                     finished=len(report['records']), total=len(schedule), elapsed_sec=round(report['elapsed_sec'], 2),
                     estimated_remaining_sec=report['estimated_remaining_sec'])
                if record['status'] != 'completed' and (not args.continue_on_error or record['status'] in ('cancelled', 'cleanup_failed')):
                    report['status'] = record['status']
                    break
            if report['status'] == 'running':
                report['status'] = 'completed' if report['completed'] == len(schedule) else 'failed'
        finally:
            report.update(finished_at=utc_time(), elapsed_sec=time.monotonic()-begin)
            save(destination/'report.json', report)
            emit('finished', status=report['status'], completed=report['completed'], total=len(schedule),
                 elapsed_sec=round(report['elapsed_sec'], 2), report=str(destination/'report.json'))
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return 0 if report['status'] == 'completed' else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--start-seed', type=int, default=1)
    parser.add_argument('--timeout-sec', type=positive, default=300.)
    parser.add_argument('--max-total-sec', type=positive, default=3600.)
    parser.add_argument('--estimate-sec', type=positive)
    parser.add_argument('--continue-on-error', action='store_true')
    args = parser.parse_args()
    try:
        return run(args)
    except (ValueError, KeyError, OSError, RuntimeError) as error:
        print(json.dumps(dict(event='error', error=str(error)), ensure_ascii=False), flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
