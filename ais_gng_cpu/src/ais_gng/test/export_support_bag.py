"""読み取り専用bagからの決定的なXYZ入力列の抽出。"""
import argparse
import json
import sqlite3
import struct
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--max-frames", type=int, default=240)
    parser.add_argument("--max-points", type=int, default=20000)
    args = parser.parse_args()
    connection = sqlite3.connect(f"file:{Path(args.bag).resolve()}?mode=ro", uri=True)
    topic = connection.execute("SELECT id,name FROM topics WHERE type='sensor_msgs/msg/PointCloud2'").fetchone()
    if topic is None:
        raise RuntimeError("PointCloud2トピックがありません")
    records = connection.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT ?", (topic[0], args.max_frames))
    metadata = {"bag": args.bag, "topic": topic[1], "max_points": args.max_points, "frames": []}
    first_stamp = None
    with open(args.output, "xb") as output:
        for stamp, payload in records:
            message = deserialize_message(payload, PointCloud2)
            fields = {field.name: field for field in message.fields}
            if any(fields[name].datatype != 7 for name in ("x", "y", "z")):
                raise RuntimeError("XYZのfloat32形式が必要です")
            dtype = np.dtype({"names": ["x", "y", "z"], "formats": [(">" if message.is_bigendian else "<") + "f4"] * 3,
                              "offsets": [fields[name].offset for name in ("x", "y", "z")], "itemsize": message.point_step})
            cloud = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                               strides=(message.row_step, message.point_step))
            points = np.column_stack([cloud[name].ravel() for name in ("x", "y", "z")])
            points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 0.05)]
            if len(points) > args.max_points:
                points = points[np.linspace(0, len(points) - 1, args.max_points, dtype=int)]
            if first_stamp is None:
                first_stamp = stamp
            time_sec = (stamp - first_stamp) * 1e-9
            output.write(struct.pack("<dI", time_sec, len(points)))
            output.write(points.astype("<f4").tobytes())
            metadata["frames"].append({"stamp_sec": time_sec, "point_num": len(points), "frame_id": message.header.frame_id})
    Path(args.output + ".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(f"frames={len(metadata['frames'])} topic={topic[1]}")


if __name__ == "__main__":
    main()
