"""深度パケット・画素対応・ROSメッセージの回帰検証。"""
import math
import struct
import unittest
from pointcloud_bridge import parse_packet
from test_pointcloud_bridge import packet
from depth_output import create_depth_messages
try:
    from builtin_interfaces.msg import Time
    has_ros = True
except ImportError:
    has_ros = False


class DepthPacketTest(unittest.TestCase):
    def setUp(self):
        self.calibration = dict(width=16, height=16, fx=10., fy=12., ppx=7.5, ppy=7.5,
                                model='none', coeffs=[0] * 5)
        self.meta = dict(source='rgbd', frame_id='base_footprint', count=0, depth_image=self.calibration)
        self.depth = [0.] * 256
        self.depth[31] = 2.

    def test_payload_and_legacy(self):
        raw = packet(self.meta, self.depth)
        self.assertEqual(len(parse_packet(raw)[1]), 1024)
        self.assertEqual(parse_packet(packet(dict(source='rgbd', frame_id='base_footprint', count=0)))[1], b'')
        with self.assertRaises(ValueError):
            parse_packet(raw[:-4])

    def test_invalid_depth(self):
        for value in [-1., float('nan'), float('inf')]:
            self.depth[0] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_packet(packet(self.meta, self.depth))

    def test_invalid_calibration(self):
        for key, value in [('width', 0), ('height', 1081), ('fx', 0), ('fy', float('nan')), ('ppx', 'bad'), ('model', 'distorted')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                parse_packet(packet(dict(self.meta, depth_image=dict(self.calibration, **{key: value})), self.depth))

    @unittest.skipUnless(has_ros, 'ROSメッセージ環境が必要')
    def test_pixel_correspondence(self):
        stamp = Time(sec=123, nanosec=456)
        image, info, cloud = create_depth_messages(self.calibration, struct.pack('<256f', *self.depth), stamp)
        from rclpy.serialization import serialize_message, deserialize_message
        for message in (image, info, cloud):
            self.assertEqual(deserialize_message(serialize_message(message), type(message)), message)
        self.assertEqual(image.header, info.header)
        self.assertEqual(image.header, cloud.header)
        self.assertEqual(cloud.header.stamp, stamp)
        self.assertEqual(cloud.header.frame_id, 'sim_camera_depth_optical_frame')
        self.assertEqual(image.encoding, '32FC1')
        self.assertEqual((cloud.width, cloud.height, cloud.row_step), (16, 16, 192))
        self.assertFalse(cloud.is_dense)
        for idx, (x, y, z) in enumerate(struct.iter_unpack('<fff', bytes(cloud.data))):
            if idx == 31:
                self.assertAlmostEqual(x, (15 - info.k[2]) * 2 / info.k[0])
                self.assertAlmostEqual(y, (1 - info.k[5]) * 2 / info.k[4])
                self.assertEqual(z, self.depth[idx])
            else:
                self.assertTrue(all(math.isnan(v) for v in (x, y, z)))


if __name__ == '__main__':
    unittest.main()
