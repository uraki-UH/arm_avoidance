"""既存ノードに触れない、有限時間の追加照合ノード試験。"""
import os
from pathlib import Path
import signal
import subprocess
import time

root = Path('/ros2_ws/src/benchmarks/vehicle_registration_20260924')
stop = root / 'stop_rpc_test'
if stop.exists():
    stop.unlink()
command = ['ros2', 'run', 'topo_fuzzy_viewer', 'viewer_vehicle_registration_node.py']
with (root/'rpc_node.log').open('w') as log:
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (root/'rpc_test_pid.txt').write_text(str(process.pid))
    print('START', command, 'PID', process.pid, flush=True)
    try:
        end = time.monotonic()+180
        while time.monotonic()<end and process.poll() is None and not stop.exists():
            time.sleep(.2)
    finally:
        for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
            if process.poll() is not None:
                break
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                pass
        print('STOP', process.pid, process.returncode, flush=True)
        if stop.exists():
            stop.unlink()
