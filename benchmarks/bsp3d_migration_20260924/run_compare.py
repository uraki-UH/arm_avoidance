"""同一CPU・同一入力での範囲検索と目標選択の交互比較。"""
import json
from pathlib import Path
import subprocess
root = Path("/ros2_ws/src")
work = Path("/tmp/bsp3d_migration_20260924")
out = root/"artifacts/bsp3d_migration_20260924"
results=[]
for name in ("aabb","nearest","selection_test"):
    result=subprocess.run([str(work/name)],capture_output=True,text=True,timeout=60)
    (out/(name+"_test.log")).write_text(result.stdout+result.stderr)
    print(name,result.returncode,flush=True)
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
for fixture in ("points","planar"):
    for trial in range(3):
        methods=["before","bsp32","bsp16","bsp8","bsp32_bbox"]
        if trial % 2: methods.reverse()
        for method in methods:
            result=subprocess.run(["taskset","-c","0",str(work/("range_"+method)),str(out/(fixture+".txt"))],capture_output=True,text=True,timeout=120)
            (out/f"range_{fixture}_{method}_{trial}.log").write_text(result.stdout+result.stderr)
            if result.returncode: raise RuntimeError(result.stdout+result.stderr)
            rows=dict(item.split("=",1) for item in result.stdout.split() if "=" in item)
            results.append(dict(fixture=fixture,trial=trial,method=method,**rows))
            print(fixture,trial,method,rows,flush=True)
        for method in (["before","after"] if trial % 2 == 0 else ["after","before"]):
            result=subprocess.run(["taskset","-c","0",str(work/("selection_"+method)),str(out/(fixture+".txt"))],capture_output=True,text=True,timeout=120)
            (out/f"selection_{fixture}_{method}_{trial}.log").write_text(result.stdout+result.stderr)
            if result.returncode: raise RuntimeError(result.stdout+result.stderr)
            print("selection",fixture,trial,method,flush=True)
(out/"range_results.json").write_text(json.dumps(results,indent=2)+"\n")
