"""実ROSメッセージ生成・シリアライズの比較。HTTP・DDSの時間は対象外。"""
import argparse
import json
from pathlib import Path
from time import perf_counter
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message, deserialize_message
import depth_output
import native_points
from test_fixtures import depth_output_reference
from test_native_points import sample_packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('python', 'cpp'), required=True)
    parser.add_argument('--width', type=int, default=848)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    meta, data = sample_packet(args.width, args.height, seed=args.seed)
    num_pixels = args.width * args.height
    stamp = Time(sec=123, nanosec=456789)
    native_points.ensure_native()
    build = depth_output.create_point_messages if args.backend == 'cpp' else depth_output_reference.create_point_messages
    validate = native_points.validate_payload if args.backend == 'cpp' else depth_output_reference.validate_payload
    # 入力生成・ビルド・ROS型サポートの初回読込を定常計測から分離。
    cloud, images = build(meta, data, stamp)
    for message in (cloud, *images):
        serialize_message(message)
    start = perf_counter()
    validate(data, meta['count'], num_pixels, True)
    validated = perf_counter()
    cloud, images = build(meta, data, stamp)
    built = perf_counter()
    messages = [serialize_message(message) for message in (cloud, *images)]
    end = perf_counter()
    reference_cloud, reference_images = depth_output_reference.create_point_messages(meta, data, stamp)
    for serialized, expected in zip(messages, (reference_cloud, *reference_images)):
        assert deserialize_message(serialized, type(expected)) == expected
    metrics = dict(num_pixels=num_pixels, num_points=meta['count'], input_bytes=len(data),
                   ros_bytes=sum(map(len, messages)), validate_ms=(validated - start) * 1000,
                   build_ms=(built - validated) * 1000, serialize_ms=(end - built) * 1000,
                   total_ms=(end - start) * 1000, has_equal_ros_messages=1)
    args.output.write_text(json.dumps(metrics, indent=2) + '\n')
    print(json.dumps(metrics))


if __name__ == '__main__':
    main()
