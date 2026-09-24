"""初期構築方式を分離した同一CPUの追加比較。"""
import json
from pathlib import Path
import subprocess
root = Path("/ros2_ws/src")
work = Path("/tmp/bsp3d_migration_20260924")
out = root/"artifacts/bsp3d_migration_20260924"
result = subprocess.run([str(work/"aabb_bulk")],capture_output=True,text=True,timeout=90)
(out/"aabb_bulk_test.log").write_text(result.stdout+result.stderr)
if result.returncode: raise RuntimeError(result.stdout+result.stderr)
print(result.stdout,flush=True)
rows=[]
for fixture in ("points","planar"):
 for trial in range(3):
  methods=["before","bsp32","bulk32","bulk16","bulk32_midpoint"]
  if trial % 2: methods.reverse()
  for method in methods:
   result=subprocess.run(["taskset","-c","0",str(work/("range_"+method)),str(out/(fixture+".txt"))],capture_output=True,text=True,timeout=90)
   (out/f"bulk_{fixture}_{method}_{trial}.log").write_text(result.stdout+result.stderr)
   if result.returncode: raise RuntimeError(result.stdout+result.stderr)
   values=dict(item.split("=",1) for item in result.stdout.split() if "=" in item)
   rows.append(dict(fixture=fixture,trial=trial,method=method,**values))
   print(fixture,trial,method,values,flush=True)
(out/"bulk_results.json").write_text(json.dumps(rows,indent=2)+"\n")
