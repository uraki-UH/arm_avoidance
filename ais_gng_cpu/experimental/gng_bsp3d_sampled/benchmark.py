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
import struct

parser = argparse.ArgumentParser()
parser.add_argument("--library", required=True)
parser.add_argument("--config", required=True)
parser.add_argument("--bag", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--frames", type=int, default=50)
parser.add_argument("--warmup", type=int, default=10)
args = parser.parse_args()
assert args.frames > args.warmup >= 0
lib = ct.CDLL(args.library)
lib.gng_setParameter.argtypes = [ct.c_char_p, ct.c_uint32, ct.c_float]
lib.gng_setPointCloud.argtypes = [ct.c_void_p, ct.c_uint32, ct.POINTER(lidar)]
lib.gng_getTopologicalMap.restype = tmap
with open(args.config) as source:
    params = yaml.safe_load(source)["ais_gng_node"]["ros__parameters"]
accepted, ignored = {}, {}
for name, value in params.items():
    for idx, scalar in enumerate(value if isinstance(value, list) else [value]):
        if isinstance(scalar, (int, float, bool)):
            target = accepted if lib.gng_setParameter(name.encode(), idx, float(scalar)) else ignored
            target.setdefault(name, []).append(scalar)
assert lib.gng_init() == 0
# 同じ30フレームを事前読込。入出力のディスク時間は計測外。
with sqlite3.connect("file:" + args.bag + "?mode=ro", uri=True) as db:
    topic_id = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
    rows = db.execute("select data from messages where topic_id=? order by timestamp limit 30", (topic_id,)).fetchall()
clouds = []
for row in rows:
    msg = deserialize_message(row[0], PointCloud2)
    data = (ct.c_uint8 * len(msg.data)).from_buffer_copy(msg.data)
    clouds.append((data, msg.width * msg.height, lidar(vec(0,0,0), quat(0,0,0,1), msg.point_step)))
records = []
for frame_idx in range(args.frames):
    data, num, config = clouds[frame_idx % len(clouds)]
    start = time.perf_counter_ns()
    lib.gng_setPointCloud(data, num, ct.byref(config))
    after_input = time.perf_counter_ns()
    lib.gng_exec()
    after_exec = time.perf_counter_ns()
    result = lib.gng_getTopologicalMap()
    end = time.perf_counter_ns()
    digest = hashlib.sha256()
    for idx in range(result.num):
        point = result.nodes[idx]
        digest.update(struct.pack("<H7fBI", point.id, point.pos.x, point.pos.y, point.pos.z,
                                  point.normal.x, point.normal.y, point.normal.z, point.rho,
                                  point.label, point.frame))
    digest.update(ct.string_at(result.edges, result.edges_num * 2))
    records.append(dict(frame=frame_idx, input_ms=(after_input-start)/1e6,
                        exec_ms=(after_exec-after_input)/1e6, output_ms=(end-after_exec)/1e6,
                        total_ms=(end-start)/1e6, nodes=result.num, edges=result.edges_num//2,
                        graph_sha256=digest.hexdigest()))
measured = records[args.warmup:]
summary = {key: dict(mean=float(np.mean([r[key] for r in measured])),
                     median=float(np.median([r[key] for r in measured])),
                     p95=float(np.percentile([r[key] for r in measured],95)))
           for key in ["input_ms", "exec_ms", "output_ms", "total_ms"]}
report = dict(library=args.library, frames=args.frames, warmup=args.warmup,
              accepted=accepted, ignored=ignored, summary=summary, records=records)
with open(args.output, "w") as target:
    json.dump(report, target, indent=2)
print(json.dumps(dict(library=args.library, summary=summary, nodes=result.num, edges=result.edges_num//2)))
