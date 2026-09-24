"""既存ROS関連プロセスの読取専用スナップショット。"""
import json, os, pathlib; rows=[]
for p in pathlib.Path("/proc").iterdir():
 if p.name.isdigit() and int(p.name) != os.getpid():
  try:
   args=(p/"cmdline").read_bytes().replace(bytes([0]),b" ").decode(errors="replace").strip()
   if args and not any(x in args for x in ["python3 -c", "bash -lc", "docker exec"]):
    if any(x in args for x in ["ros2", "_node", "ais_gng", "vite", "component_container"]):
     stat=(p/"stat").read_text().split(); rows.append(dict(pid=int(p.name),ppid=int(stat[3]),args=args))
  except (OSError,ValueError): pass
print(json.dumps(rows))
