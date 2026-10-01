import argparse
import datetime
import json
from pathlib import Path
import subprocess

parser=argparse.ArgumentParser()
parser.add_argument('--phase', choices=('before','after'), required=True)
parser.add_argument('--output', type=Path, required=True)
args=parser.parse_args()
root=Path('/home/uraki/uraki_ws/artifacts/gng_nearest_index_20261001')
code=r"""import json,os
from pathlib import Path
skip=set(); pid=os.getpid()
while pid and pid not in skip:
    skip.add(pid)
    try: pid=int(Path(f'/proc/{pid}/stat').read_text().rsplit(') ',1)[1].split()[1])
    except (FileNotFoundError,ProcessLookupError): break
rows=[]
for path in Path('/proc').iterdir():
    if not path.name.isdigit() or int(path.name) in skip: continue
    try:
        command=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode('utf-8',errors='replace').strip()
        stat=(path/'stat').read_text().rsplit(') ',1)[1].split()
        if not any(value in command.lower() for value in ('ros2','roslaunch','rviz','gazebo','gzserver','gng_','safe_graph')): continue
        rows.append({'pid':int(path.name),'ppid':int(stat[1]),'pgid':int(stat[2]),'start_ticks':int(stat[19]),'command':command})
    except (FileNotFoundError,ProcessLookupError,PermissionError): pass
print(json.dumps(rows))
"""
def run(argv):
    result=subprocess.run(argv,text=True,capture_output=True,timeout=20,check=True)
    return result.stdout
snapshot={'phase':args.phase,'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'containers':json.loads(run(['docker','inspect','--format','{"Id":{{json .Id}},"Name":{{json .Name}},"State":{{json .State}}}','gng_cpu_container'])),
    'host_processes':json.loads(run(['python3','-B','-c',code])),
    'container_processes':json.loads(run(['docker','exec','gng_cpu_container','python3','-B','-c',code]))}
container=snapshot['containers']
snapshot['containers']={key:container[key] for key in ('Id','Name')}
snapshot['containers']['State']={key:container['State'][key] for key in ('Status','Running','Paused','Restarting','StartedAt')}
output=args.output
with output.open('x') as stream: json.dump(snapshot,stream,ensure_ascii=False,indent=2)
print(json.dumps({'output':str(output),'num_host':len(snapshot['host_processes']), 'num_container':len(snapshot['container_processes']),'container_running':snapshot['containers']['State']['Running']}))
