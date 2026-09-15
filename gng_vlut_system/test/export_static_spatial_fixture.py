"""既存Tmapの有限購読による空間索引比較用座標の取得。配信・指令なし。"""

import argparse
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from ais_gng_msgs.msg import TopologicalMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--topic", default="/ToPoDualArm/Tmap_static")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("static_spatial_fixture_probe")
    received = []
    node.create_subscription(
        TopologicalMap, args.topic, received.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        end = time.monotonic() + 20
        while not received and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not received:
            raise RuntimeError("Tmap取得の時間切れ")
        points = [(entry.pos.x, entry.pos.y, entry.pos.z) for entry in received[-1].nodes]
        points = [point for point in points if all(math.isfinite(value) for value in point)]
        if not points:
            raise RuntimeError("有限座標のノードなし")
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text("".join(" ".join(format(value, ".17g") for value in point) + "\n"
                                  for point in points), encoding="ascii")
        print(f"topic={args.topic} frame={received[-1].header.frame_id} nodes={len(points)} output={output}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
