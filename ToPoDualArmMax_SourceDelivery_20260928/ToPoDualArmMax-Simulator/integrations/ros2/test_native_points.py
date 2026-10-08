"""C++点群処理の境界・並行実行・ROSメッセージの互換性検証。"""
from array import array
from concurrent.futures import ThreadPoolExecutor
import random
import struct
import unittest

from native_points import build_depth_points, colorize_points, ensure_native, validate_payload


def sample_packet(width, height, seed=1, enable_color=True):
    random_source = random.Random(seed)
    depth = array('f', (random_source.uniform(.195, 3) if random_source.random() < .305 else 0
                        for _ in range(width * height)))
    count = sum(value > 0 for value in depth)
    xyz = array('f', (random_source.uniform(-3, 3) for _ in range(count * 3)))
    colors = bytes(random_source.randrange(2 if idx % 4 == 3 else 256) for idx in range(count * 4))
    calibration = dict(width=width, height=height, fx=448.713, fy=432.173, ppx=(width - 1) / 2 + .17,
                       ppy=(height - 1) / 2 - .23, model='none', coeffs=[0] * 5)
    meta = dict(source='rgbd', frame_id='base_footprint', count=count, depth_image=calibration, robot_pose={})
    if enable_color:
        meta['color_format'] = 'rgb8_valid8'
    return meta, xyz.tobytes() + depth.tobytes() + (colors if enable_color else b'')


class NativePointsTest(unittest.TestCase):
    def test_depth_and_color_layout(self):
        calibration = dict(width=3, height=2, fx=1., fy=2., ppx=.5, ppy=.75)
        depth = struct.pack('<6f', 0, 2, -0., .25, 3, 0)
        colors = bytes([255, 12, 34, 1, 1, 2, 3, 0, 45, 67, 89, 1])
        actual = build_depth_points(calibration, memoryview(b'x' + depth)[1:], colors)
        color_idx = 0
        expected = bytearray(120)
        for idx, (z,) in enumerate(struct.iter_unpack('<f', depth)):
            point = ((idx % 3 - .5) * z, (idx // 3 - .75) * z / 2, z) if z > 0 else (float('nan'),) * 3
            struct.pack_into('<fff', expected, idx * 20, *point)
            if z > 0:
                red, green, blue, valid = colors[color_idx:color_idx + 4]
                struct.pack_into('<IB', expected, idx * 20 + 12, (red << 16) | (green << 8) | blue, valid)
                color_idx += 4
        self.assertEqual(actual.tobytes(), expected)
        plain = build_depth_points(calibration, depth)
        self.assertEqual(colorize_points(plain, colors, depth), actual)
        empty = build_depth_points(calibration, bytes(24), b'')
        self.assertEqual(empty.tobytes(), (struct.pack('<III', 0x7fc00000, 0x7fc00000, 0x7fc00000) + bytes(8)) * 6)

    def test_invalid_buffers_and_values(self):
        calibration = dict(width=1, height=1, fx=1., fy=1., ppx=0., ppy=0.)
        for value in (-1., float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                build_depth_points(calibration, struct.pack('<f', value))
        for data, colors in [(b'', None), (struct.pack('<f', 1), b''),
                             (struct.pack('<f', 0), bytes(4)), (struct.pack('<f', 1), bytes([1, 2, 3, 2]))]:
            with self.subTest(data=data, colors=colors), self.assertRaises(ValueError):
                build_depth_points(calibration, data, colors)
        for key, value in [('width', 0), ('height', 1081), ('fx', 0.), ('fy', float('nan'))]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                build_depth_points(dict(calibration, **{key: value}), bytes(4))
        backend = ensure_native()
        with self.assertRaises(ValueError):
            backend.build_depth(bytes(4), None, 1, 1, 1., 1., 0., 0., bytearray(11))
        with self.assertRaises((BufferError, TypeError)):
            backend.build_depth(bytes(4), None, 1, 1, 1., 1., 0., 0., bytes(12))
        with self.assertRaises((BufferError, ValueError)):
            build_depth_points(calibration, memoryview(bytes(8))[::2])
        self.assertEqual(colorize_points(b'', b''), array('B'))
        with self.assertRaises(ValueError):
            colorize_points(bytes(11), bytes(4))
        with self.assertRaises(ValueError):
            colorize_points(bytes(12), b'')

    def test_payload_validation(self):
        valid = struct.pack('<3ff', 1, 2, 3, 1) + bytes([5, 6, 7, 1])
        validate_payload(valid, 1, 1, True)
        validate_payload(b'', 0, 0, False)
        for bad in [valid[:-1], struct.pack('<3ff', float('nan'), 2, 3, 1) + valid[-4:],
                    valid[:12] + struct.pack('<f', -1) + valid[-4:], valid[:-1] + b'\x02',
                    valid[:12] + bytes(4) + valid[-4:]]:
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_payload(bad, 1, 1, True)
        with self.assertRaises(ValueError):
            validate_payload(b'', -1, 0, False)

    def test_float32_overflow_boundary(self):
        max_float = struct.unpack('<f', struct.pack('<I', 0x7f7fffff))[0]
        depth = struct.pack('<f', max_float)
        calibration = dict(width=1, height=1, fx=.99999999999, fy=1., ppx=-1., ppy=0.)
        expected = struct.pack('<fff', max_float / calibration['fx'], 0., max_float)
        self.assertEqual(build_depth_points(calibration, depth).tobytes(), expected)
        with self.assertRaises(OverflowError):
            build_depth_points(dict(calibration, fx=.5), depth)

    def test_parallel_frames(self):
        meta, payload = sample_packet(97, 65)
        count = meta['count']
        depth, colors = payload[count * 12:count * 12 + 97 * 65 * 4], payload[count * 12 + 97 * 65 * 4:]
        expected = build_depth_points(meta['depth_image'], depth, colors)
        with ThreadPoolExecutor(max_workers=4) as executor:
            frames = list(executor.map(lambda _: build_depth_points(meta['depth_image'], depth, colors), range(12)))
        for frame in frames:
            self.assertEqual(frame, expected)

    def test_ros_serialized_output(self):
        from builtin_interfaces.msg import Time
        from rclpy.serialization import serialize_message, deserialize_message
        from depth_output import create_point_messages
        from test_fixtures.depth_output_reference import create_point_messages as reference
        for width, height in [(16, 16), (97, 65), (848, 480), (1280, 720)]:
            for enable_color in (False, True):
                with self.subTest(width=width, enable_color=enable_color):
                    meta, data = sample_packet(width, height, enable_color=enable_color)
                    validate_payload(data, meta['count'], width * height, enable_color)
                    stamp = Time(sec=123, nanosec=456789)
                    actual_cloud, actual_depth = create_point_messages(meta, data, stamp)
                    expected_cloud, expected_depth = reference(meta, data, stamp)
                    for actual, expected in zip([actual_cloud, *actual_depth], [expected_cloud, *expected_depth]):
                        serialized = serialize_message(actual)
                        # CDRの未使用アラインメント領域を除く、全フィールド・配列バイト列の一致。
                        self.assertEqual(actual, expected)
                        self.assertEqual(len(serialized), len(serialize_message(expected)))
                        self.assertEqual(deserialize_message(serialized, type(actual)), expected)
                    # 深度なしの通常点群・空点群との互換性。
                    meta.pop('depth_image')
                    compact_data = data[:meta['count'] * 12] + (data[-meta['count'] * 4:] if enable_color else b'')
                    actual, images = create_point_messages(meta, compact_data, stamp)
                    expected, _ = reference(meta, compact_data, stamp)
                    self.assertEqual(deserialize_message(serialize_message(actual), type(actual)), expected)
                    self.assertEqual(images, [])
        meta = dict(source='rgbd', count=0, frame_id='base_footprint', color_format='rgb8_valid8')
        self.assertEqual(create_point_messages(meta, b'', Time())[0], reference(meta, b'', Time())[0])


if __name__ == '__main__':
    unittest.main()
