"""移行前後のRelease比較バイナリと回帰試験の有限ビルド。"""
from pathlib import Path
import shlex
import argparse
import subprocess

root = Path("/ros2_ws/src")
work = Path("/tmp/bsp3d_migration_20260924")
out = root / "artifacts/bsp3d_migration_20260924"
work.mkdir(exist_ok=True)
source = root / "gng_vlut_system/test/benchmark_static_spatial_index.cpp"
flags_path = Path("/ros2_ws/build/gng_vlut_system/CMakeFiles/test_grasp_candidate_reachability.dir/flags.make")
flags = next(line.split(" = ",1)[1] for line in flags_path.read_text().splitlines() if line.startswith("CXX_INCLUDES = "))
ros_flags = shlex.split(flags)
base = ["c++","-std=c++17","-O3","-DNDEBUG"]
parser = argparse.ArgumentParser()
parser.add_argument("--only", nargs="*")
args = parser.parse_args()
selected = args.only
calls = []
def run(name, args):
    calls.append(args)
    if selected is not None and name not in selected: return
    with (out / (name + ".log")).open("w") as log:
        result = subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,timeout=240)
    if result.returncode:
        raise RuntimeError(name + " build failed: " + (out/(name+".log")).read_text()[-3000:])
    print(name, "built", flush=True)

for method, leaf, bbox in [("before",32,0),("bsp32",32,0),("bsp16",16,0),("bsp8",8,0),("bsp32_bbox",32,1),("bulk32",32,0),("bulk16",16,0),("bulk32_midpoint",32,0)]:
    flags = (["-DBSP3D_BENCH_REFERENCE","-I"+str(out/"before/SpatialTree/include")] if method=="before" else
             ["-I"+str(root/"bsp3d/include"),f"-DBSP3D_LEAF_NUM={leaf}",f"-DBSP3D_ENABLE_BBOX={bbox}"])
    if method.startswith("bulk"):
        flags = ["-I"+str(out/"bulk_source/include")]+flags+["-DBSP3D_BULK_BUILD"]
    if method.endswith("midpoint"): flags.append("-DBSP3D_SPLIT_RULE=1")
    run("range_"+method,base+flags+[str(source),"-o",str(work/("range_"+method))])
for method in ("before","after"):
    include = out/"before/gng_vlut_system/src" if method=="before" else root/"gng_vlut_system/src"
    flags = (["-DBSP3D_BENCH_REFERENCE","-I"+str(out/"before/SpatialTree/include")] if method=="before" else ["-I"+str(root/"bsp3d/include")])
    run("selection_"+method,base+["-DSPATIAL_BENCH_SELECTION","-I"+str(include)]+flags+ros_flags+[str(source),"-o",str(work/("selection_"+method))])
run("aabb",base+["-I"+str(root/"bsp3d/include"),str(root/"bsp3d/examples/test_aabb.cpp"),"-o",str(work/"aabb")])
run("aabb_bulk",base+["-I"+str(out/"bulk_source/include"),str(out/"bulk_source/test_aabb.cpp"),"-o",str(work/"aabb_bulk")])
run("nearest",base+["-UNDEBUG","-I"+str(root/"bsp3d/include"),str(root/"bsp3d/examples/test_bsp3d.cpp"),"-o",str(work/"nearest")])
test = root/"gng_vlut_system/test/test_grasp_candidate_reachability.cpp"
run("selection_test",base+["-I"+str(root/"bsp3d/include"),"-I"+str(root/"gng_vlut_system/src")]+ros_flags+[str(test),
    "/ros2_ws/build/gng_vlut_system/gtest/libgtest_main.a","/ros2_ws/build/gng_vlut_system/gtest/libgtest.a","-pthread","-o",str(work/"selection_test")])
(out/"build_commands.txt").write_text("\n".join(shlex.join(call) for call in calls)+"\n")
