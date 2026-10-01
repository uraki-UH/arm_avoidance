"""実点群と実機自己形状の座標整合・時刻検査・欠損関節の拒否の検証。"""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import JointState, PointCloud2, PointField

share = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(share / 'scripts'))
from external_pointcloud_bridge import (cloud_xyz, transform_xyz, has_fresh_input,
                                       external_pointcloud_bridge, pose_matrix, real_root_transform)


@pytest.mark.parametrize('is_bigendian', [True, False])
def test_organized_cloud_with_padding_and_invalid_point(is_bigendian):
    message = PointCloud2(height=2, width=1, point_step=16, row_step=20, is_bigendian=is_bigendian)
    message.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in [('x', 4), ('y', 8), ('z', 12)]]
    order = '>' if is_bigendian else '<'
    message.data = struct.pack(order+'ffffIffffI', 8, 1, 2, 3, 0, 9, float('nan'), 0, 0, 0)
    np.testing.assert_equal(cloud_xyz(message), [[1, 2, 3]])
    message.row_step = 12
    with pytest.raises(ValueError, match='欠損'):
        cloud_xyz(message)


def test_optical_forward_and_mount_rotation():
    # 光学Z前方→本体X前方→取付yaw 90度による仮想Y前方
    result = transform_xyz(np.array([[0., 0., 2.]]), [1., 2., 3., 0., 0., np.pi/2],
                           [0., 0., 0.], [-.5, .5, -.5, .5])
    np.testing.assert_allclose(result, [[1, 4, 3]], atol=1e-12)


@pytest.mark.parametrize('stamp,last,now,expected', [
    (1_000_000_000, 0, 1.1, True), (1_000_000_000, 0, 2.1, False),
    (1_000_000_000, 1_000_000_000, 1.1, False), (2_000_000_000, 0, 1., False)])
def test_stale_duplicate_future_input_rejected(stamp, last, now, expected):
    assert has_fresh_input(stamp, last, now, 1.) is expected


def test_missing_clock_never_relabels_old_data():
    target = SimpleNamespace(settings={'source_frame': 'optical', 'max_input_age_sec': 1.},
                             last_input_stamp=-1, sim_stamp=None, num_rejected=0)
    import time
    stamp = time.time_ns()
    message = PointCloud2()
    message.header.frame_id = 'optical'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    external_pointcloud_bridge.on_cloud(target, message)
    assert target.num_rejected == 1
    assert 'clock' in target.reason


def test_real_robot_mask_and_camera_points_share_transform():
    camera_pose = [.2, -.1, .6, .1, .4, -.3]
    mount = [.02, 0., .01, 0., .1, 0.]
    real_camera = pose_matrix([.1, .05, .4, -.2, .3, .5])
    camera_point = np.array([.2, .1, .4, 1.])
    real_point = real_camera @ pose_matrix(mount) @ camera_point
    placement = real_root_transform(camera_pose, mount, real_camera)
    np.testing.assert_allclose(placement @ real_point, pose_matrix(camera_pose) @ camera_point, atol=1e-12)


def test_incomplete_real_joints_never_default_to_zero():
    import time
    target = SimpleNamespace(geometry=SimpleNamespace(joint_names=['waist_joint', 'neck_joint'],
        limits=np.array([[-1., 1.], [-1., 1.]])), real_positions=None, last_real_stamp=-1,
        settings={'max_input_age_sec': 1.}, real_joint_time=0.)
    message = JointState(name=['neck_joint'], position=[.2])
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
    external_pointcloud_bridge.on_real_joints(target, message)
    assert target.real_positions is None
    assert 'waist_joint' in target.real_state_detail
    message.name, message.position = ['waist_joint', 'neck_joint'], [.1, .2]
    external_pointcloud_bridge.on_real_joints(target, message)
    np.testing.assert_equal(target.real_positions, [.1, .2])
    received = target.real_joint_time
    external_pointcloud_bridge.on_real_joints(target, message)
    assert target.real_joint_time == received
