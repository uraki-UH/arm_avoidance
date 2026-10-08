"""同一画素配列・時刻での深度画像と点群の生成。深度の単位はメートル。"""
from native_points import build_depth_points, byte_array, colorize_points

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
    image.data = byte_array(data)
    info = CameraInfo()
    info.header = image.header
    info.width, info.height = width, height
    fx, fy, cx, cy = (float(calibration[k]) for k in ('fx', 'fy', 'ppx', 'ppy'))
    info.distortion_model = 'plumb_bob'
    info.d = [0.] * 5
    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
    # 逆投影・色付け・無効画素・パディングをC++の1回の走査で構築。
    points = build_depth_points(calibration, data, colors)
    cloud = PointCloud2()
    cloud.header = image.header
    cloud.width, cloud.height = width, height
    cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                    for idx, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian = False
    cloud.is_dense = False
    cloud.point_step = 20 if colors is not None else 12
    cloud.row_step = width * cloud.point_step
    cloud.data = points
    if colors is not None:
        add_color_fields(cloud)
    return image, info, cloud


def colorize_cloud(cloud, colors, depth=None):
    """RGBのpacked float表現と色有効フラグ。深度配列指定時は画素順を維持。"""
    if cloud.point_step != 12 or len(cloud.data) != cloud.width * cloud.height * 12 or cloud.is_bigendian:
        raise ValueError('色付け元の点群形式が不正です')
    cloud.data = colorize_points(cloud.data, colors, depth)
    add_color_fields(cloud)
    cloud.point_step = 20
    cloud.row_step = cloud.width * 20


def add_color_fields(cloud):
    from sensor_msgs.msg import PointField
    cloud.fields.extend([PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                         PointField(name='color_valid', offset=16, datatype=PointField.UINT8, count=1)])


def create_point_messages(meta, data, stamp):
    """通常点群と画素対応点群の生成。撮影データ・時刻・既存トピック形式の保持。"""
    from sensor_msgs.msg import PointCloud2, PointField
    payload = memoryview(data)
    xyz_size = meta['count'] * 12
    depth_size = meta['depth_image']['width'] * meta['depth_image']['height'] * 4 if meta.get('depth_image') else 0
    colors = payload[xyz_size + depth_size:] if meta.get('color_format') else None
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = meta['frame_id']
    cloud.height = 1
    cloud.width = meta['count']
    cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                    for idx, name in enumerate(('x', 'y', 'z'))]
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.point_step = 20 if colors is not None else 12
    cloud.row_step = cloud.width * cloud.point_step
    cloud.data = colorize_points(payload[:xyz_size], colors) if colors is not None else byte_array(payload[:xyz_size])
    if colors is not None:
        add_color_fields(cloud)
    depth_messages = create_depth_messages(meta['depth_image'], payload[xyz_size:xyz_size + depth_size], stamp, colors) if meta.get('depth_image') else []
    return cloud, depth_messages
