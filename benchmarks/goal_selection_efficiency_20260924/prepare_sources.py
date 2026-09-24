"""固定コミットと保存差分からの比較元再現。入力座標は別途保存物が必要。"""
from pathlib import Path
import io
import subprocess
import tarfile
root=Path(__file__).resolve().parents[2]
trial=Path(__file__).resolve().parent
out=root/'artifacts'/trial.name
out.mkdir(parents=True,exist_ok=True)
revision=(trial/'base_revision.txt').read_text().strip()
legacy=out/'legacy'
paths=['SpatialTree','gng_vlut_system/src/nodes/planning/goal_spatial_index.hpp']
archive=subprocess.check_output(['git','-c',f'safe.directory={root}','archive',revision,*paths],cwd=root)
with tarfile.open(fileobj=io.BytesIO(archive)) as entries:
    for entry in entries:
        if not entry.isfile(): continue
        relative=Path(entry.name)
        if relative.is_absolute() or '..' in relative.parts: raise ValueError('不正なアーカイブパス')
        target=legacy/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(entries.extractfile(entry).read())
for name in ['goal_spatial_index.hpp','goal_node_selection.hpp','topological_map_goal_selector_node.cpp']:
    content=subprocess.check_output(['git','-c',f'safe.directory={root}','show',revision+':gng_vlut_system/src/nodes/planning/'+name],cwd=root)
    (out/name).write_bytes(content)
subprocess.run(['patch','--batch','-p1','-d',str(out)],input=(trial/'before.patch').read_bytes(),check=True,timeout=30)
print('Comparison baseline prepared')
