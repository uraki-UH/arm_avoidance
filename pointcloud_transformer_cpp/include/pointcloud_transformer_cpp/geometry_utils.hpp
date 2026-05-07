#ifndef POINTCLOUD_TRANSFORMER_CPP__GEOMETRY_UTILS_HPP_
#define POINTCLOUD_TRANSFORMER_CPP__GEOMETRY_UTILS_HPP_

#include <Eigen/Geometry>
#include <cmath>

namespace pointcloud_transformer_cpp {
namespace utils {

// Build the same 4x4 homogeneous transform as:
//   scipy: R.from_euler('zyx', [yaw_deg, pitch_deg, roll_deg], degrees=True)
//
// Empirically verified: scipy from_euler('zyx', [yaw, pitch, roll]) produces
// the matrix  Rx(roll) @ Ry(pitch) @ Rz(yaw)  (equivalent to extrinsic ZYX).
// In Eigen this is: (rot_x * rot_y * rot_z).toRotationMatrix()
inline Eigen::Matrix4d make_raw_matrix(
    double x, double y, double z,
    double roll_deg, double pitch_deg, double yaw_deg)
{
    const double d2r = M_PI / 180.0;

    Eigen::AngleAxisd rot_z(yaw_deg   * d2r, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rot_y(pitch_deg * d2r, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_x(roll_deg  * d2r, Eigen::Vector3d::UnitX());

    // scipy from_euler('zyx', [yaw, pitch, roll]) = Rx @ Ry @ Rz
    Eigen::Matrix3d R = (rot_x * rot_y * rot_z).toRotationMatrix();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;

    return T;
}

} // namespace utils
} // namespace pointcloud_transformer_cpp

#endif // POINTCLOUD_TRANSFORMER_CPP__GEOMETRY_UTILS_HPP_
