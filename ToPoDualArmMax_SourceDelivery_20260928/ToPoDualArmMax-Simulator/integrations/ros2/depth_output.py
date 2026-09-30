"""同一画素配列・時刻での深度画像と点群の生成。深度の単位はメートル。"""
import struct
from array import array

depth_topics = ['/sim/camera/depth/image_rect_raw', '/sim/camera/depth/camera_info',
                '/sim/camera/depth/points']


def create_depth_messages(calibration, data, stamp):
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
    return image, info, cloud
