"""同一画素配列・時刻での深度画像と点群の生成。深度の単位はメートル。"""
import struct
from array import array

depth_topics = ['/sim/camera/depth/image_rect_raw', '/sim/camera/depth/camera_info',
                '/sim/camera/depth/points']


def create_depth_messages(calibration, data, stamp, colors=None):
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
    width, height = calibration['width'], calibration['height']
    image = Image()
    image.header.stamp = stamp
    image.header.frame_id = 'sim_camera_depth_optical_frame'
    image.width, image.height = width, height
    image.encoding = '32FC1'
    image.is_bigendian = False
    image.step = width * 4
    image.data = array('B', data)
    info = CameraInfo()
    info.header = image.header
    info.width, info.height = width, height
    fx, fy, cx, cy = (float(calibration[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
    info.distortion_model = 'plumb_bob'
    info.d = [0.] * 5
    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    points = bytearray(width * height * 12)
    # 画素位置を維持した逆投影。無効深度0に対応するXYZは全成分NaN
    for idx, (z,) in enumerate(struct.iter_unpack('<f', data)):
        x, y = ((idx % width - cx) * z / fx, (idx // width - cy) * z / fy) if z > 0 else (float('nan'), float('nan'))
        struct.pack_into('<fff', points, idx * 12, x, y, z if z > 0 else float('nan'))
    cloud = PointCloud2()
    cloud.header = image.header
    cloud.width, cloud.height = width, height
    cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                    for idx, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian = False
    cloud.is_dense = False
    cloud.point_step = 12
    cloud.row_step = width * 12
    cloud.data = array('B', points)
    if colors is not None:
        colorize_cloud(cloud, colors, data)
    return image, info, cloud


def colorize_cloud(cloud, colors, depth=None):
    """RGBのpacked float表現と色有効フラグ。深度配列指定時は画素順を維持。"""
    from sensor_msgs.msg import PointField
    num_points = cloud.width * cloud.height
    packed = bytearray(num_points * 20)
    xyz = memoryview(cloud.data)
    color_idx = 0
    for idx in range(num_points):
        packed[idx * 20:idx * 20 + 12] = xyz[idx * 12:idx * 12 + 12]
        if depth is not None and struct.unpack_from('<f', depth, idx * 4)[0] <= 0:
            continue
        red, green, blue, valid = colors[color_idx:color_idx + 4]
        struct.pack_into('<IB', packed, idx * 20 + 12, (red << 16) | (green << 8) | blue, valid)
        color_idx += 4
    if color_idx != len(colors):
        raise ValueError('点群と色の点数が一致しません')
    cloud.fields.extend([PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                         PointField(name='color_valid', offset=16, datatype=PointField.UINT8, count=1)])
    cloud.point_step = 20
    cloud.row_step = cloud.width * 20
    cloud.data = array('B', packed)


def create_point_messages(meta, data, stamp):
    """比較専用の従来メッセージ構築。実行経路への接続なし。"""
    from sensor_msgs.msg import PointCloud2, PointField
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = meta['frame_id']
    cloud.width, cloud.height = meta['count'], 1
    cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                    for idx, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.point_step, cloud.row_step = 12, cloud.width * 12
    xyz_size = meta['count'] * 12
    depth_size = meta['depth_image']['width'] * meta['depth_image']['height'] * 4 if meta.get('depth_image') else 0
    colors = data[xyz_size + depth_size:] if meta.get('color_format') else None
    cloud.data = array('B', data[:xyz_size])
    if colors is not None:
        colorize_cloud(cloud, colors)
    depth_messages = create_depth_messages(meta['depth_image'], data[xyz_size:xyz_size + depth_size], stamp, colors) if meta.get('depth_image') else []
    return cloud, depth_messages


def validate_payload(data, num_points, num_pixels, has_color):
    """比較専用の従来画素ループ検査。"""
    import math
    color_offset = num_points * 12 + num_pixels * 4
    depth = data[num_points * 12:color_offset]
    if num_pixels and any(not math.isfinite(value[0]) or value[0] < 0 for value in struct.iter_unpack('<f', depth)):
        raise ValueError('深度値が不正です')
    if has_color:
        if any(value not in (0, 1) for value in data[color_offset + 3::4]):
            raise ValueError('色の有効フラグが不正です')
        if num_pixels and sum(value[0] > 0 for value in struct.iter_unpack('<f', depth)) != num_points:
            raise ValueError('深度と色付き点群の有効点数が一致しません')
    if any(not math.isfinite(value[0]) for value in struct.iter_unpack('<f', data[:num_points * 12])):
        raise ValueError('座標に非有限値があります')
