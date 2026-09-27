"""汎用runnerの回数・保存・失敗・終了処理の実プロセス検証。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

runner = Path(__file__).with_name('run_batch.py')


class batch_test(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='benchmark_batch_test_')
        self.root = Path(self.directory.name)
        self.addCleanup(self.directory.cleanup)

    def start(self, code, *args, **fields):
        manifest = self.root/'cases.json'
        manifest.write_text(json.dumps({'cases': [dict(name='case', argv=[sys.executable, '-c', code, '@seed@', '@case_dir@'], **fields)]}))
        command = [sys.executable, str(runner), str(manifest), '--output', str(self.root/'run'), '--repeats', '1', *args]
        return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def finish(self, process):
        stdout, stderr = process.communicate(timeout=15)
        report = json.loads((self.root/'run/report.json').read_text())
        self.assertFalse(stderr, stderr)
        return report, [json.loads(line) for line in stdout.splitlines()]

    def test_repeats_and_estimate(self):
        process = self.start('import json,sys; from pathlib import Path; (Path(sys.argv[2])/"metrics.json").write_text(json.dumps({"seed": int(sys.argv[1])}))',
                             '--repeats', '3', '--start-seed', '7', metrics='metrics.json')
        report, events = self.finish(process)
        self.assertEqual(process.returncode, 0)
        self.assertEqual([r['metrics']['seed'] for r in report['records']], [7,8,9])
        self.assertGreater(events[1]['estimated_remaining_sec'], 0)
        self.assertEqual(events[-1]['event'], 'finished')

    def test_failure_and_timeout(self):
        report, _ = self.finish(self.start('raise SystemExit(4)', '--repeats', '2'))
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(len(report['records']), 1)

    def test_timeout_and_orphan(self):
        process = self.start('import subprocess,sys,time; subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"]); time.sleep(30)', '--timeout-sec', '.15')
        report, _ = self.finish(process)
        self.assertEqual(report['status'], 'timeout')
        self.assertTrue(report['records'][0]['cleanup_ok'])
        with self.assertRaises(ProcessLookupError): os.killpg(report['records'][0]['pid'], 0)

    def test_parent_exit_cleanup(self):
        process = self.start('import subprocess,sys; subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"])')
        report, _ = self.finish(process)
        self.assertEqual(report['status'], 'completed')
        self.assertTrue(report['records'][0]['cleanup_ok'])

    def test_invalid_metrics(self):
        report, _ = self.finish(self.start('from pathlib import Path; import sys; (Path(sys.argv[2])/"metrics.json").write_text("{\\"x\\":NaN}")', metrics='metrics.json'))
        self.assertEqual(report['status'], 'invalid_metrics')

    def test_cancel(self):
        process = self.start('import time; time.sleep(30)')
        self.assertIn('started', process.stdout.readline())
        time.sleep(.15)
        process.send_signal(signal.SIGTERM)
        report, _ = self.finish(process)
        self.assertEqual(report['status'], 'cancelled')
        self.assertTrue(report['records'][0]['cleanup_ok'])

    def test_total_timeout(self):
        report, _ = self.finish(self.start('import time; time.sleep(30)',
            '--max-total-sec', '.15', '--timeout-sec', '10'))
        self.assertEqual(report['status'], 'timeout')
        self.assertTrue(report['records'][0]['cleanup_ok'])

    def test_continue_after_failure(self):
        report, _ = self.finish(self.start('raise SystemExit(4)', '--repeats', '3', '--continue-on-error'))
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(len(report['records']), 3)

    def test_existing_output_is_preserved(self):
        self.finish(self.start('pass'))
        previous = (self.root/'run/report.json').read_bytes()
        process = self.start('raise RuntimeError("実行禁止")')
        stdout, stderr = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 2)
        self.assertEqual(json.loads(stdout)['event'], 'error')
        self.assertFalse(stderr)
        self.assertEqual((self.root/'run/report.json').read_bytes(), previous)

    def test_rotating_order_and_environment(self):
        manifest = self.root/'cases.json'
        code = 'import os,sys,json; from pathlib import Path; (Path(sys.argv[1])/"metrics.json").write_text(json.dumps({"env": int(os.environ["BATCH_TEST_VALUE"]), "cwd": int(Path.cwd() == Path(sys.argv[2]))}))'
        cases = [dict(name=name, argv=[sys.executable, '-c', code, '@case_dir@', str(self.root)],
            cwd='.', env={'BATCH_TEST_VALUE':'42'}, metrics='metrics.json') for name in ('a','b')]
        manifest.write_text(json.dumps(dict(cases=cases)))
        process = subprocess.Popen([sys.executable, str(runner), str(manifest), '--output',
            str(self.root/'run'), '--repeats', '2'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        report, _ = self.finish(process)
        self.assertEqual(process.returncode, 0)
        self.assertEqual([r['name'] for r in report['records']], ['a','b','b','a'])
        self.assertTrue(all(r['metrics'] == {'env':42,'cwd':1} for r in report['records']))


if __name__ == '__main__':
    unittest.main()
