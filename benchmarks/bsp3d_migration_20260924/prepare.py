"""移行前のGitスナップショットと不採用一括構築案の再現準備。"""
from pathlib import Path
import io
import shutil
import subprocess
import tarfile

root = Path(__file__).resolve().parents[2]
trial = Path(__file__).resolve().parent
out = root / "artifacts" / trial.name
out.mkdir(parents=True, exist_ok=True)
revision = (trial / "base_revision.txt").read_text().strip()
paths = ["SpatialTree", "bsp3d/include/bsp3d/bsp3d.hpp",
         "gng_vlut_system/src/nodes/planning", "gng_vlut_system/test/benchmark_static_spatial_index.cpp"]
archive = subprocess.check_output(["git", "-c", f"safe.directory={root}", "archive", revision, *paths], cwd=root)
with tarfile.open(fileobj=io.BytesIO(archive)) as entries:
    for entry in entries:
        if not entry.isfile():
            continue
        relative = Path(entry.name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Gitアーカイブ内の不正なパス")
        target = out / "before" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(entries.extractfile(entry).read())
bulk = out / "bulk_source"
header = bulk / "include/bsp3d/bsp3d.hpp"
header.parent.mkdir(parents=True, exist_ok=True)
shutil.copyfile(out / "before/bsp3d/include/bsp3d/bsp3d.hpp", header)
(bulk / "test_aabb.cpp").unlink(missing_ok=True)
subprocess.run(["patch", "--batch", "-p1", "-d", str(bulk)],
               input=(trial / "bulk_prototype.patch").read_bytes(), check=True, timeout=30)
print("移行前ソースと一括構築案の準備完了。入力座標はREADME記載のローカル保存物が必要。")
