import sqlite3
import struct
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

root = Path(__file__).resolve().parents[2]
bag = "/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3"
with sqlite3.connect("file:" + bag + "?mode=ro", uri=True) as database:
    topic_id = database.execute("select id from topics where name='/lidar_points'").fetchone()[0]
    data = database.execute("select data from messages where topic_id=? order by timestamp limit 1", (topic_id,)).fetchone()[0]
cloud = deserialize_message(data,PointCloud2)
points = np.ndarray((cloud.width*cloud.height,3), dtype=np.float32, buffer=cloud.data, strides=(cloud.point_step,4)).copy()
output = root / "artifacts/gng_runtime_trials_20260924/voxel_input.bin"
output.write_bytes(struct.pack("<I", len(points)) + points.tobytes())
