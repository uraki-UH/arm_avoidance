#!/usr/bin/env python3
"""depthからの元画素番号保持・単位・欠損・行余白の検査。ROSノード起動なし。"""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

spec = importlib.util.spec_from_file_location('depth_pixel_points', Path(__file__).resolve().parents[1] / 'scripts/depth_pixel_points.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class depth_pixel_points_test(unittest.TestCase):
    def setUp(self):
        self.state = SimpleNamespace(rotation=np.eye(3), origin=np.zeros(3), cache_key=None,
                                     rays=None, depth_unit=0.001, target_frame='')
        self.info = CameraInfo()
        self.info.width, self.info.height = 3, 2
        self.info.header.frame_id = 'depth'
        self.info.header.stamp.sec = 1
        self.info.k = [2., 0., 1., 0., 2., 0.5, 0., 0., 1.]
        self.info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        self.image = Image()
        self.image.header = self.info.header
        self.image.width, self.image.height = 3, 2
        self.image.encoding = '16UC1'
        self.image.step = 8
        self.image.data = np.array([[1000, 0, 2000, 999], [0, 3000, 4000, 999]], dtype='<u2').tobytes()

    def convert(self):
        result = module.depth_pixel_points.convert(self.state, self.image, self.info)
        return np.frombuffer(result.data, dtype=[('xyz', '<f4', (3,)), ('pixel', '<u4')])

    def test_pixels_units_and_padding(self):
        records = self.convert()
        np.testing.assert_array_equal(records['pixel'], [0, 2, 4, 5])
        np.testing.assert_allclose(records['xyz'], [[-0.5, -0.25, 1], [1, -0.5, 2], [0, 0.75, 3], [2, 1, 4]])
        rays = self.state.rays
        self.convert()
        self.assertIs(rays, self.state.rays)
        self.image.is_bigendian = True
        self.image.data = np.array([[1000, 0, 2000, 999], [0, 3000, 4000, 999]], dtype='>u2').tobytes()
        np.testing.assert_array_equal(records, self.convert())

    def test_float_depth_and_transform(self):
        self.state.rotation = module.rotation_matrix([0, 0, 1, 0])
        self.state.origin = np.array([0.4, 0.1, 0.2])
        self.state.target_frame = 'base'
        self.image.encoding, self.image.step = '32FC1', 12
        self.image.data = np.array([[1, np.nan, 2], [0, -3, np.inf]], dtype='<f4').tobytes()
        records = self.convert()
        np.testing.assert_array_equal(records['pixel'], [0, 2])
        np.testing.assert_allclose(records['xyz'], [[0.9, 0.35, 1.2], [-0.6, 0.6, 2.2]], rtol=1e-6)

    def test_invalid_calibration(self):
        self.info.d = [0.1]
        with self.assertRaises(ValueError):
            self.convert()


if __name__ == '__main__':
    unittest.main()
