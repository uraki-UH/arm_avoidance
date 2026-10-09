"""初回UI準備・再利用・失敗伝播・同時起動の検証。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class test_startup_ui(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        scripts = self.root/'integrations/ros2'
        scripts.mkdir(parents=True)
        self.script = scripts/'prepare_ui.sh'
        shutil.copyfile(Path(__file__).resolve().parents[1]/'integrations/ros2/prepare_ui.sh', self.script)
        binaries = self.root/'bin'
        binaries.mkdir()
        npm = binaries/'npm'
        npm.write_text('''#!/usr/bin/env bash
set -eu
printf '%s\\n' "$*" >> calls
if [[ "${has_install_error:-0}" == 1 ]]; then exit 8; fi
if [[ "$1" == run ]]; then
    sleep .1
    mkdir -p app/generated app/vendor/three/build
    touch app/generated/ros-results.js app/vendor/three/build/three.module.js app/vendor/three/build/three.core.js
fi
''')
        npm.chmod(0o755)
        self.env = dict(os.environ, PATH=str(binaries)+os.pathsep+os.environ['PATH'])

    def run_prepare(self):
        return subprocess.run(['bash', str(self.script)], env=self.env,
                              capture_output=True, text=True, timeout=10)

    def test_first_start_builds_and_next_start_reuses(self):
        self.assertEqual(self.run_prepare().returncode, 0)
        self.assertEqual(self.run_prepare().returncode, 0)
        self.assertEqual((self.root/'calls').read_text().splitlines(),
                         ['ci --prefer-offline --no-audit --no-fund', 'run build'])

    def test_failed_install_stops_before_build(self):
        self.env['has_install_error'] = '1'
        self.assertEqual(self.run_prepare().returncode, 8)
        self.assertFalse((self.root/'app/generated/ros-results.js').exists())
        self.assertEqual(len((self.root/'calls').read_text().splitlines()), 1)

    def test_existing_ui_with_missing_vendor_is_rebuilt(self):
        generated = self.root/'app/generated'
        generated.mkdir(parents=True)
        (generated/'ros-results.js').touch()
        self.assertEqual(self.run_prepare().returncode, 0)
        self.assertTrue((self.root/'app/vendor/three/build/three.module.js').exists())
        self.assertTrue((self.root/'app/vendor/three/build/three.core.js').exists())

    def test_parallel_start_builds_once(self):
        processes = []
        try:
            for _ in range(2):
                processes.append(subprocess.Popen(['bash', str(self.script)], env=self.env,
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE))
            for process in processes:
                _, error = process.communicate(timeout=10)
                self.assertEqual(process.returncode, 0, error)
        finally:
            for process in processes:
                if process.poll() is None:
                    process.kill()
                process.communicate()
        self.assertEqual(len((self.root/'calls').read_text().splitlines()), 2)


if __name__ == '__main__':
    unittest.main()
