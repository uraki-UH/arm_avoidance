import ctypes as ct, sqlite3, time
import numpy as np, yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
class vec(ct.Structure): _fields_=[("x",ct.c_float),("y",ct.c_float),("z",ct.c_float)]
class quat(ct.Structure): _fields_=[("x",ct.c_float),("y",ct.c_float),("z",ct.c_float),("w",ct.c_float)]
class lidar(ct.Structure): _fields_=[("pos",vec),("quat",quat),("point_step",ct.c_uint32)]
class node(ct.Structure): _fields_=[("id",ct.c_uint16),("pos",vec),("normal",vec),("rho",ct.c_float),("label",ct.c_uint8),("frame",ct.c_uint32),("inpcl_ids",ct.c_void_p),("inpcl_num",ct.c_uint32)]
class tmap(ct.Structure): _fields_=[("frame",ct.c_uint32),("num",ct.c_uint32),("clusters_num",ct.c_uint32),("edges_num",ct.c_uint32),("nodes",ct.POINTER(node)),("clusters",ct.c_void_p),("edges",ct.c_void_p),("dist",ct.c_void_p)]

import argparse
import hashlib
import json
import resource
import warnings
from pathlib import Path

# 数値計算ライブラリの導入済み構成。距離検証は下記の既知点で別途確認。
with warnings.catch_warnings():
    warnings.simplefilter("ignore", UserWarning)
    from scipy.spatial import cKDTree
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import connected_components

class sampling_statistics(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in
        ("voxel_ms", "attention_ms", "learn_ms", "label_ms", "maintenance_ms", "cluster_ms")] + [
        (name, ct.c_uint64) for name in ("num_nearest_queries", "num_tree_moves", "num_probe_points",
        "num_observed_nodes", "num_attention_candidates", "num_attention_hits", "num_added_nodes", "num_deleted_nodes", "num_zero_samples")]

parser = argparse.ArgumentParser()
parser.add_argument("--library", required=True)
parser.add_argument("--config", required=True)
parser.add_argument("--bag", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--frames", type=int, default=50)
parser.add_argument("--warmup", type=int, default=10)
parser.add_argument("--bag-frames", type=int, default=30)
parser.add_argument("--quality-every", type=int, default=10)
args = parser.parse_args()
assert args.frames > args.warmup >= 0
assert args.bag_frames > 0 and args.quality_every > 0
assert np.allclose(cKDTree([[0,0,0],[2,0,0]]).query([[0.5,0,0],[3,0,0]])[0], [0.5,1])
lib = ct.CDLL(args.library)
lib.gng_setParameter.argtypes = [ct.c_char_p, ct.c_uint32, ct.c_float]
lib.gng_setPointCloud.argtypes = [ct.c_void_p, ct.c_uint32, ct.POINTER(lidar)]
lib.gng_getTopologicalMap.restype = tmap
has_statistics = hasattr(lib, "gng_get_sampling_statistics")
if hasattr(lib, "gng_get_minimal_statistics"):
    class minimal_statistics(ct.Structure):
        _fields_ = sampling_statistics._fields_ + [
            ("input_prepare_ms", ct.c_double), ("num_input_points", ct.c_uint64)]
    sampling_statistics = minimal_statistics

if hasattr(lib, "gng_get_comparison_statistics"):
    class comparison_statistics(ct.Structure):
        _fields_ = sampling_statistics._fields_ + [("num_training_points", ct.c_uint64)]
    sampling_statistics = comparison_statistics

if has_statistics:
    lib.gng_get_sampling_statistics.restype = ct.POINTER(sampling_statistics)
params = yaml.safe_load(Path(args.config).read_text())["ais_gng_node"]["ros__parameters"]
accepted, ignored = {}, {}
for name, value in params.items():
    for idx, scalar in enumerate(value if isinstance(value, list) else [value]):
        if isinstance(scalar, (int, float, bool)):
            destination = accepted if lib.gng_setParameter(name.encode(), idx, float(scalar)) else ignored
            destination.setdefault(name, []).append(scalar)
start_init = time.perf_counter_ns()
assert lib.gng_init() == 0
init_ms = (time.perf_counter_ns()-start_init)/1e6

# ディスク読込・逆シリアル化の時間を除外した同一入力の比較。
with sqlite3.connect("file:" + args.bag + "?mode=ro", uri=True) as db:
    topic_id = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
    rows = db.execute("select data from messages where topic_id=? order by timestamp limit ?", (topic_id, args.bag_frames)).fetchall()
clouds = []
for row in rows:
    msg = deserialize_message(row[0], PointCloud2)
    num = msg.width * msg.height
    data = (ct.c_uint8 * len(msg.data)).from_buffer_copy(msg.data)
    positions = np.ndarray((num, 3), dtype=np.float32, buffer=msg.data, strides=(msg.point_step,4)).copy()
    is_valid = np.isfinite(positions).all(axis=1)
    for axis, name in enumerate("xyz"):
        is_valid &= positions[:,axis] >= params[f"input.{name}_min"]
        is_valid &= positions[:,axis] <= params[f"input.{name}_max"]
    clouds.append((data, num, lidar(vec(0,0,0), quat(0,0,0,1), msg.point_step), positions[is_valid]))
assert clouds
node_dtype = np.dtype({"names":["id","pos","normal","rho","label","frame"],
    "formats":[np.uint16,(np.float32,3),(np.float32,3),np.float32,np.uint8,np.uint32],
    "offsets":[getattr(node,name).offset for name in ["id","pos","normal","rho","label","frame"]],
    "itemsize":ct.sizeof(node)})
records, snapshots = [], []
previous_keys = set()
for frame_idx in range(args.frames):
    data, num, config, input_points = clouds[frame_idx % len(clouds)]
    start = time.perf_counter_ns()
    lib.gng_setPointCloud(data, num, ct.byref(config))
    after_input = time.perf_counter_ns()
    lib.gng_exec()
    after_exec = time.perf_counter_ns()
    result = lib.gng_getTopologicalMap()
    end = time.perf_counter_ns()
    nodes = np.frombuffer(ct.string_at(result.nodes, result.num * ct.sizeof(node)), dtype=node_dtype)
    edges = np.frombuffer(ct.string_at(result.edges, result.edges_num * 2), dtype=np.uint16).reshape(-1,2)
    digest = hashlib.sha256()
    for name in node_dtype.names:
        digest.update(nodes[name].tobytes())
    digest.update(edges.tobytes())
    keys = set(zip(nodes["id"].tolist(),nodes["frame"].tolist()))
    record = dict(frame=frame_idx, input_ms=(after_input-start)/1e6,
        exec_ms=(after_exec-after_input)/1e6, output_ms=(end-after_exec)/1e6,
        total_ms=(end-start)/1e6, nodes=result.num, edges=result.edges_num//2,
        added_output_nodes=len(keys-previous_keys), removed_output_nodes=len(previous_keys-keys),
        graph_sha256=digest.hexdigest())
    if has_statistics:
        statistics = lib.gng_get_sampling_statistics().contents
        record["statistics"] = {name:getattr(statistics,name) for name,_ in sampling_statistics._fields_}
    previous_keys = keys
    records.append(record)
    if frame_idx >= args.warmup and ((frame_idx-args.warmup) % args.quality_every == 0 or frame_idx == args.frames-1):
        snapshots.append((frame_idx, input_points, nodes["pos"].copy(), edges.copy()))

# 品質評価は全時間計測の終了後。距離はYAML範囲内の全元点から最近傍ノードまで。
quality = []
for frame_idx, input_points, positions, edges in snapshots:
    assert len(positions) > 0 and np.isfinite(positions).all()
    assert edges.size == 0 or (edges.max() < len(positions) and (edges[:,0] != edges[:,1]).all())
    distances, _ = cKDTree(positions).query(input_points, workers=1)
    graph = csr_matrix((np.ones(len(edges),dtype=np.uint8),(edges[:,0],edges[:,1])), shape=(len(positions),len(positions)))
    num_components, component_ids = connected_components(graph,directed=False)
    lengths = np.linalg.norm(positions[edges[:,0]]-positions[edges[:,1]],axis=1)
    quality.append(dict(frame=frame_idx, num_points=len(input_points),
        mean_dist_m=float(distances.mean()), p95_dist_m=float(np.percentile(distances,95)),
        rms_dist_m=float(np.sqrt(np.mean(distances*distances))),
        coverage_0_2m=float(np.mean(distances<0.2)), coverage_0_4m=float(np.mean(distances<0.4)),
        coverage_1_0m=float(np.mean(distances<1.0)), num_components=int(num_components),
        max_component_ratio=float(np.bincount(component_ids).max()/len(positions)),
        edge_p95_m=float(np.percentile(lengths,95)) if len(lengths) else None,
        input_min=input_points.min(axis=0).tolist(),input_max=input_points.max(axis=0).tolist(),
        node_min=positions.min(axis=0).tolist(),node_max=positions.max(axis=0).tolist()))
    # 原点への集中による評価の偏りを明示する別集計。GNGへの入力からの除外なし。
    is_nonzero = np.any(input_points != 0, axis=1)
    quality[-1]["num_zero_points"] = int(np.sum(~is_nonzero))
    for suffix, limit in [("0_2m",0.2),("0_4m",0.4),("1_0m",1.0)]:
        quality[-1]["nonzero_coverage_"+suffix] = float(np.mean(distances[is_nonzero]<limit)) if is_nonzero.any() else None
    quality[-1]["nonzero_p95_dist_m"] = float(np.percentile(distances[is_nonzero],95)) if is_nonzero.any() else None
measured = records[args.warmup:]
summary = {key:dict(mean=float(np.mean([r[key] for r in measured])),
    median=float(np.median([r[key] for r in measured])),p95=float(np.percentile([r[key] for r in measured],95)))
    for key in ["input_ms","exec_ms","output_ms","total_ms","nodes","edges","added_output_nodes","removed_output_nodes"]}
if has_statistics:
    summary["statistics"] = {name:float(np.mean([r["statistics"][name] for r in measured])) for name,_ in sampling_statistics._fields_}
quality_summary = {key:float(np.mean([entry[key] for entry in quality])) for key in
    ["mean_dist_m","p95_dist_m","rms_dist_m","coverage_0_2m","coverage_0_4m","coverage_1_0m","num_components","max_component_ratio"]}
for key in ["num_zero_points","nonzero_coverage_0_2m","nonzero_coverage_0_4m","nonzero_coverage_1_0m","nonzero_p95_dist_m"]:
    values = [entry[key] for entry in quality if entry[key] is not None]
    quality_summary[key] = float(np.mean(values)) if values else None
report = dict(library=args.library,config=args.config,frames=args.frames,warmup=args.warmup,bag_frames=len(clouds),
    library_sha256=hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    init_ms=init_ms,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    accepted=accepted,ignored=ignored,summary=summary,quality_summary=quality_summary,quality=quality,records=records)
Path(args.output).write_text(json.dumps(report,indent=2)+"\n")
np.savez_compressed(Path(args.output).with_suffix(".graph.npz"),
    input_points=snapshots[-1][1],nodes=snapshots[-1][2],edges=snapshots[-1][3])
print(json.dumps(dict(library=args.library,exec_ms=summary["exec_ms"],quality=quality_summary,nodes=result.num,edges=result.edges_num//2)))
