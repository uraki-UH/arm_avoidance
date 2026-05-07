#!/usr/bin/env python3
"""
Pythonの変換とC++の変換の数値比較デバッグスクリプト
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

# ===== パラメータ (Python スクリプトと同じ) =====
yaw_deg, pitch_deg, roll_deg = -3.4, -28.9, -103.8
tx, ty, tz = 0.434, -0.693, 0.279

# ===== Python do_transform_cloud が行う変換の再現 =====
# transform_pointcloud.py:
#   rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
#   t.header.frame_id   = "base_link"   (parent)
#   t.child_frame_id    = camera_frame  (child)
#   do_transform_cloud(msg, t)
#
# do_transform_cloud は parent←child の変換を適用:
#   p_parent = R @ p_child + t

rot = R.from_euler('zyx', [yaw_deg, pitch_deg, roll_deg], degrees=True)
R_mat = rot.as_matrix()
t_vec = np.array([tx, ty, tz])

print("=" * 60)
print("Python rotation matrix R (from_euler 'zyx', [yaw,pitch,roll]):")
print(np.round(R_mat, 8))
print(f"Translation: {t_vec}")
print()

# Eigenが生成するはずの回転行列を再現 (Rz * Ry * Rx)
d2r = np.pi / 180.0
Rz = R.from_rotvec([0, 0, yaw_deg * d2r]).as_matrix()
Ry = R.from_rotvec([0, pitch_deg * d2r, 0]).as_matrix()
Rx = R.from_rotvec([roll_deg * d2r, 0, 0]).as_matrix()
R_cpp = Rx @ Ry @ Rz  # scipy from_euler('zyx',[yaw,pitch,roll]) = Rx@Ry@Rz

print("C++ Eigen (Rz*Ry*Rx) rotation matrix:")
print(np.round(R_cpp, 8))
print()

diff = R_mat - R_cpp
print("Difference (Python - C++):")
print(np.round(diff, 10))
print(f"Max absolute difference: {np.abs(diff).max():.2e}")
print()

# テスト点で変換
test_points = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.5, 0.3, 0.8],
])

print("=" * 60)
print("Point transformation comparison:")
print(f"{'Input':30s} {'Python out':40s} {'C++ out':40s} {'diff_max':10s}")
for p in test_points:
    py_out = R_mat @ p + t_vec
    cpp_out = R_cpp @ p + t_vec
    d = np.abs(py_out - cpp_out).max()
    print(f"{str(np.round(p,2)):30s} {str(np.round(py_out,6)):40s} {str(np.round(cpp_out,6)):40s} {d:.2e}")

print()
print("=" * 60)
# do_transform_cloud の内部実装を確認
print("Checking tf2_sensor_msgs.do_transform_cloud internals...")
try:
    import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm
    import inspect
    src = inspect.getsource(tf2_sm.do_transform_cloud)
    print(src)
except Exception as e:
    print(f"Could not inspect: {e}")
    # tf_transformations を試す
    try:
        import tf_transformations
        src2 = inspect.getsource(tf_transformations.quaternion_matrix)
        print(src2[:500])
    except Exception as e2:
        print(f"tf_transformations not found: {e2}")
