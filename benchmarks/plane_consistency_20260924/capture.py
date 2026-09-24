"""交差点bagの読み取り専用GNG処理と、平面比較用の共通入力保存。"""

import argparse
import ctypes as ct
import sqlite3
import struct
import sys
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "gng_production_efficiency_20260924"))
from benchmark import lidar, quat, tmap, vec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--frames", type=int, default=150)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("frames must be positive")
    lib = ct.CDLL("/ros2_ws/install/gng_cpu/lib/libgng_cpu.so")
    lib.gng_setParameter.argtypes = [ct.c_char_p, ct.c_uint32, ct.c_float]
    lib.gng_setPointCloud.argtypes = [ct.c_void_p, ct.c_uint32, ct.POINTER(lidar)]
    lib.gng_getTopologicalMap.restype = tmap
    params = yaml.safe_load(Path(args.config).read_text())["ais_gng_node"]["ros__parameters"]
    for name, values in params.items():
        for idx, value in enumerate(values if isinstance(values, list) else [values]):
            if isinstance(value, (bool, int, float)):
                lib.gng_setParameter(name.encode(), idx, float(value))
    assert lib.gng_init() == 0
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as db, open(args.output, "wb") as output:
        topic_id = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        rows = db.execute("select data from messages where topic_id=? order by timestamp limit ?",
                          (topic_id, args.frames))
        num_frames = 0
        for frame_idx, (raw,) in enumerate(rows):
            cloud = deserialize_message(raw, PointCloud2)
            assert [(v.name, v.offset) for v in cloud.fields[:3]] == [("x", 0), ("y", 4), ("z", 8)]
            data = (ct.c_uint8 * len(cloud.data)).from_buffer_copy(cloud.data)
            sensor = lidar(vec(0, 0, 0), quat(0, 0, 0, 1), cloud.point_step)
            lib.gng_setPointCloud(data, cloud.width * cloud.height, ct.byref(sensor))
            lib.gng_exec()
            graph = lib.gng_getTopologicalMap()
            output.write(struct.pack("<II", graph.node_num, graph.edge_num))
            for idx in range(graph.node_num):
                value = graph.nodes[idx]
                output.write(struct.pack("<HB7f", value.id, value.label,
                                        value.pos.x, value.pos.y, value.pos.z,
                                        value.normal.x, value.normal.y, value.normal.z, value.rho))
            output.write(ct.string_at(graph.edges, graph.edge_num * 2))
            num_frames += 1
            if frame_idx % 25 == 0:
                print(f"captured={frame_idx + 1} nodes={graph.node_num}", flush=True)
        assert num_frames == args.frames


if __name__ == "__main__":
    main()
