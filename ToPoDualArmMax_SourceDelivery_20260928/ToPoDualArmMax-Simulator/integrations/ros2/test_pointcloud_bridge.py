"""独立点群ブリッジの入力境界検証。ROS環境への依存なし。"""
import json
import struct
import unittest
from pointcloud_bridge import parse_packet


def packet(meta, values=()):
    header = json.dumps(meta).encode()
    return b'TPC1' + struct.pack('<I', len(header)) + header + b'\0' * (-len(header) % 4) + struct.pack('<' + 'f' * len(values), *values)


class PointcloudBridgeTest(unittest.TestCase):
    def setUp(self):
        self.meta = dict(source='rgbd', frame_id='base_footprint', count=1, robot_pose={'joint': .2})

    def test_xyz_and_empty(self):
        meta, data = parse_packet(packet(self.meta, (1, 2, 3)))
        self.assertEqual(meta, self.meta)
        self.assertEqual(struct.unpack('<fff', data), (1, 2, 3))
        self.meta['count'] = 0
        self.assertEqual(parse_packet(packet(self.meta))[1], b'')

    def test_broken_packet(self):
        raw = packet(self.meta, (1, 2, 3))
        for invalid in (b'', b'BAD!' + raw[4:], raw[:-1], raw + b'\0'):
            with self.subTest(raw=invalid), self.assertRaises(ValueError):
                parse_packet(invalid)

    def test_nonfinite(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_packet(packet(self.meta, (value, 0, 0)))

    def test_metadata(self):
        for key, value in [('source', 'invalid'), ('count', True), ('count', -1), ('frame_id', 'map'), ('robot_pose', {'joint': 'bad'})]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                parse_packet(packet(dict(self.meta, **{key: value}), (1, 2, 3)))

    def test_object_pose(self):
        meta = dict(self.meta, source='object_full')
        with self.assertRaises(ValueError):
            parse_packet(packet(meta, (1, 2, 3)))
        meta.update(object_id=2, object_to_world=[1 if i % 5 == 0 else 0 for i in range(16)])
        self.assertEqual(parse_packet(packet(meta, (1, 2, 3)))[0]['object_id'], 2)


if __name__ == '__main__':
    unittest.main()
