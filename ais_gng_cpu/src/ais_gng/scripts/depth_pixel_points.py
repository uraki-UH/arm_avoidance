#!/usr/bin/env python3
"""depth画像からのXYZ・元画素番号の同時生成。既存点群の逆投影なし。"""

import copy
from array import array
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def rotation_matrix(quaternion):
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        raise ValueError('sensor_rotation: 有限な4成分の指定')
    x, y, z, w = quaternion
    if abs(x*x + y*y + z*z + w*w - 1) > 1e-6:
        raise ValueError('sensor_rotation: 単位クォータニオンの指定')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float64)


class depth_pixel_points(Node):
    def __init__(self):
        super().__init__('depth_pixel_points')
        def option(name, value):
            return self.declare_parameter(name, value, ParameterDescriptor(read_only=True)).value
        self.depth_topic = option('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.camera_info_topic = option('camera_info_topic', '/camera/camera/depth/camera_info')
        self.output_topic = option('output_topic', '/observation_depth/points')
        self.output_camera_info_topic = option('output_camera_info_topic', '/observation_depth/camera_info')
        self.target_frame = option('target_frame', '')
        self.sensor_origin = option('sensor_origin', [0.0, 0.0, 0.0])
        self.sensor_rotation = option('sensor_rotation', [0.0, 0.0, 0.0, 1.0])
        self.depth_unit = option('depth_unit', 0.001)
        self.max_frame_num = option('max_frame_num', 0)
        if len(self.sensor_origin) != 3 or not all(math.isfinite(value) for value in self.sensor_origin):
            raise ValueError('sensor_origin: 有限な3成分の指定')
        if not math.isfinite(self.depth_unit) or self.depth_unit <= 0 or self.max_frame_num < 0:
            raise ValueError('depth_unit または max_frame_num の不正値')
        self.rotation = rotation_matrix(self.sensor_rotation)
        if not self.target_frame and (any(self.sensor_origin) or self.sensor_rotation != [0.0, 0.0, 0.0, 1.0]):
            raise ValueError('座標変換時のtarget_frameの指定')
        self.origin = np.array(self.sensor_origin, dtype=np.float64)
        self.images = {}
        self.infos = {}
        self.frame_num = 0
        self.last_warning_sec = 0.0
        self.cache_key = None
        self.rays = None
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.info_publisher = self.create_publisher(CameraInfo, self.output_camera_info_topic, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, self.depth_topic, self.on_image, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, qos_profile_sensor_data)

    def receive(self, message, own, other):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        own[key] = message
        while len(own) > 8:
            del own[next(iter(own))]
        if key not in other:
            return
        image, info = self.images.pop(key), self.infos.pop(key)
        if self.max_frame_num and self.frame_num >= self.max_frame_num:
            return
        try:
            start = time.perf_counter()
            cloud = self.convert(image, info)
            self.info_publisher.publish(info)
            self.publisher.publish(cloud)
            self.frame_num += 1
            if self.frame_num == 1 or self.frame_num % 30 == 0:
                self.get_logger().info(f'frames={self.frame_num} points={cloud.width} conversion_ms={(time.perf_counter()-start)*1000:.3f}')
        except ValueError as error:
            now = time.monotonic()
            if now - self.last_warning_sec > 5:
                self.get_logger().warning(str(error))
                self.last_warning_sec = now

    def on_image(self, message):
        self.receive(message, self.images, self.infos)

    def on_info(self, message):
        self.receive(message, self.infos, self.images)

    def convert(self, image, info):
        if image.header.frame_id != info.header.frame_id or image.width != info.width or image.height != info.height:
            raise ValueError('depth/CameraInfoの座標系・画像寸法の不一致')
        if image.header.stamp != info.header.stamp:
            raise ValueError('depth/CameraInfoの時刻不一致')
        if image.encoding not in ('16UC1', '32FC1') or not image.width or not image.height or image.width * image.height > 4194304:
            raise ValueError('未対応depth形式または空画像')
        dtype = np.dtype(('>' if image.is_bigendian else '<') + ('u2' if image.encoding == '16UC1' else 'f4'))
        if image.step < image.width * dtype.itemsize or len(image.data) < image.step * image.height:
            raise ValueError('depth画像の行長・データ長の不一致')
        depth = np.ndarray((image.height, image.width), dtype=dtype, buffer=image.data,
                           strides=(image.step, dtype.itemsize)).astype(np.float64)
        if image.encoding == '16UC1':
            depth *= self.depth_unit
        matrix = np.array(info.k, dtype=np.float64).reshape(3, 3)
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or any(info.d) or \
                matrix[0, 1] != 0 or matrix[1, 0] != 0 or not np.array_equal(matrix[2], [0, 0, 1]) or \
                not np.array_equal(np.array(info.r).reshape(3, 3), np.eye(3)) or \
                info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset or \
                info.roi.width or info.roi.height or info.roi.do_rectify:
            raise ValueError('歪み・ROI等を含む未対応のカメラ校正')
        projection = np.array(info.p, dtype=np.float64).reshape(3, 4)
        if projection[0, 0] != 0 and (not np.array_equal(projection[:, :3], matrix) or np.any(projection[:, 3])):
            raise ValueError('CameraInfoのK/P不一致')
        key = (info.width, info.height, *info.k)
        if self.cache_key != key:
            column, row = np.meshgrid(np.arange(info.width), np.arange(info.height))
            x = (column - matrix[0, 2]) / matrix[0, 0]
            y = (row - matrix[1, 2]) / matrix[1, 1]
            self.rays = np.stack([self.rotation[axis, 0]*x + self.rotation[axis, 1]*y + self.rotation[axis, 2]
                                  for axis in range(3)], axis=-1).reshape(-1, 3)
            self.cache_key = key
        # 有効深度の選択と同時の元画素番号保持。欠損画素の詰め直し後も対応は維持。
        flat_depth = depth.reshape(-1)
        pixel_ids = np.flatnonzero(np.isfinite(flat_depth) & (flat_depth > 0)).astype(np.uint32)
        records = np.empty(pixel_ids.size, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('pixel_idx', '<u4')])
        for axis, name in enumerate(('x', 'y', 'z')):
            records[name] = self.rays[pixel_ids, axis] * flat_depth[pixel_ids] + self.origin[axis]
        records['pixel_idx'] = pixel_ids
        # float32表現の範囲外点の除外。元画素番号との同時選択。
        records = records[np.isfinite(records['x']) & np.isfinite(records['y']) & np.isfinite(records['z'])]
        cloud = PointCloud2()
        cloud.header = copy.deepcopy(image.header)
        cloud.header.frame_id = self.target_frame or image.header.frame_id
        cloud.height, cloud.width = 1, len(records)
        cloud.is_dense = True
        cloud.point_step, cloud.row_step = 16, len(records) * 16
        cloud.fields = [PointField(name=name, offset=idx*4, datatype=PointField.FLOAT32, count=1)
                        for idx, name in enumerate(('x', 'y', 'z'))]
        cloud.fields.append(PointField(name='pixel_idx', offset=12, datatype=PointField.UINT32, count=1))
        cloud.data = array('B', records.tobytes())
        return cloud


def main():
    rclpy.init()
    node = None
    try:
        node = depth_pixel_points()
        while rclpy.ok() and (not node.max_frame_num or node.frame_num < node.max_frame_num):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
