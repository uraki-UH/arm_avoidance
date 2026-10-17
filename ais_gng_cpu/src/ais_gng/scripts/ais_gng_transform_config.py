"""Viewer補正とROS標準取付姿勢からの位置・クォータニオン変換。"""

import math


def read_vector(value, num, name):
    if isinstance(value, str):
        try:
            value = [float(item) for item in value.split()]
        except ValueError as error:
            raise ValueError(f'{name}: 数値列が必要です') from error
    if (not isinstance(value, list) or len(value) != num
            or any(type(item) not in (int, float) or not math.isfinite(item) for item in value)):
        raise ValueError(f'{name}: 有限値{num}個が必要です')
    return [float(item) for item in value]


def read_pose(config):
    """pos・rot_degはViewer XYZ、camera_mount_poseはROS標準RPY。"""
    if not isinstance(config, dict):
        raise ValueError('姿勢設定にはYAMLの辞書が必要です')
    has_viewer_pose = any(name in config for name in ('pos', 'rot_deg', 'scl'))
    has_mount_pose = 'camera_mount_pose' in config
    if has_viewer_pose == has_mount_pose:
        raise ValueError('pos・rot_degとcamera_mount_poseのどちらか一方が必要です')
    if has_viewer_pose:
        pos = read_vector(config.get('pos'), 3, 'pos')
        angles = [math.radians(value) for value in read_vector(config.get('rot_deg'), 3, 'rot_deg')]
        scale = read_vector(config.get('scl', [1, 1, 1]), 3, 'scl')
        if scale != [1, 1, 1]:
            raise ValueError('TFへの反映にはscl: 1 1 1が必要です')
    else:
        pose = read_vector(config['camera_mount_pose'], 6, 'camera_mount_pose')
        pos, angles = pose[:3], pose[3:]
    cx, cy, cz = [math.cos(value / 2) for value in angles]
    sx, sy, sz = [math.sin(value / 2) for value in angles]
    if has_viewer_pose:
        # Three.js Euler XYZと共通のRx*Ry*Rz
        rotation = [sx*cy*cz + cx*sy*sz, cx*sy*cz - sx*cy*sz,
                    cx*cy*sz + sx*sy*cz, cx*cy*cz - sx*sy*sz]
    else:
        # URDF・ROS標準RPYと共通のRz*Ry*Rx
        rotation = [sx*cy*cz - cx*sy*sz, cx*sy*cz + sx*cy*sz,
                    cx*cy*sz - sx*sy*cz, cx*cy*cz + sx*sy*sz]
    return pos, rotation
