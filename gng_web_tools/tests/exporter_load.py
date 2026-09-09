"""移動後のROSコマンド・出力先・点群エクスポートの有限テスト。"""

import json
from pathlib import Path
import subprocess
import tempfile

from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from gng_bundle_exporter import cli


def main():
    expected_root = Path(__file__).resolve().parents[1] / "exporter"
    assert cli._package_root() == expected_root
    assert cli._default_results_dir() == expected_root / "results"

    # 実トピックへの送信を伴わない、1メッセージの一時bag。
    with tempfile.TemporaryDirectory(prefix="gng-web-export-") as temp_dir:
        root = Path(temp_dir)
        bag_path = root / "bag"
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
                    ConverterOptions("cdr", "cdr"))
        writer.create_topic(TopicMetadata(
            name="/test_points", type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
        header = Header(frame_id="object_template")
        message = create_cloud_xyz32(header, [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0)])
        writer.write("/test_points", serialize_message(message), 1000000000)
        del writer
        config_path = root / "topics.yaml"
        config_path.write_text(
            "topics:\n  - alias: points\n    topic: /test_points\n"
            "    kind: pointcloud2\n    role: pointcloud\n    required: true\n",
            encoding="utf-8")
        output_path = root / "output.json"
        subprocess.run([
            "ros2", "run", "gng_bundle_exporter", "gng-bundle-export", "export",
            "--bag", str(bag_path), "--config", str(config_path),
            "--output", str(output_path),
        ], check=True, timeout=20)
        bundle = json.loads(output_path.read_text(encoding="utf-8"))
        assert bundle["version"] == "gng_html_bundle_v2"
        assert bundle["topics"]["points"]["row_count"] == 1
        assert len(bundle["topics"]["points"]["rows"]) == 1
    print("Exporter load OK: ROS command, source path, PointCloud2 export")


if __name__ == "__main__":
    main()
