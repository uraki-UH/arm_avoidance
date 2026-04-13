#pragma once

#include <Eigen/Dense>

namespace robot_sim {
namespace digital_twin {

/**
 * @brief Manages coordinate transformations between real-world and simulation world frames.
 */
class CoordinateTransformer {
public:
    CoordinateTransformer() 
        : world_from_real_robot_(Eigen::Isometry3d::Identity()),
          world_from_real_env_(Eigen::Isometry3d::Identity()) {}

    /**
     * @brief Set the transform from Real Robot Base to Simulation World origin.
     */
    void setRobotBaseTransform(const Eigen::Vector3d& pos, const Eigen::Vector3d& euler_deg) {
        world_from_real_robot_ = makeTransform(pos, euler_deg);
    }

    /**
     * @brief Set the transform from Real Environment Sensor (Lidar) to Simulation World origin.
     */
    void setEnvSensorTransform(const Eigen::Vector3d& pos, const Eigen::Vector3d& euler_deg) {
        world_from_real_env_ = makeTransform(pos, euler_deg);
    }

    /**
     * @brief Transform a point from Real Robot Local frame to Simulation World frame.
     */
    Eigen::Vector3d transformRobotToWorld(const Eigen::Vector3d& p_local) const {
        return world_from_real_robot_ * p_local;
    }

    /**
     * @brief Transform a point from Real Environment (Sensor) frame to Simulation World frame.
     */
    Eigen::Vector3d transformEnvToWorld(const Eigen::Vector3d& p_env) const {
        return world_from_real_env_ * p_env;
    }

    /**
     * @brief Inverse transform from Simulation World back to Real Robot Local frame.
     */
    Eigen::Vector3d transformWorldToRobot(const Eigen::Vector3d& p_world) const {
        return world_from_real_robot_.inverse() * p_world;
    }

    const Eigen::Isometry3d& getRobotBaseTransform() const { return world_from_real_robot_; }
    const Eigen::Isometry3d& getEnvSensorTransform() const { return world_from_real_env_; }

private:
    Eigen::Isometry3d makeTransform(const Eigen::Vector3d& pos, const Eigen::Vector3d& euler_deg) {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.translate(pos);
        
        // Simple ZYX rotation
        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(euler_deg.z() * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(euler_deg.y() * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(euler_deg.x() * M_PI / 180.0, Eigen::Vector3d::UnitX());
        tf.rotate(rot);
        return tf;
    }

    Eigen::Isometry3d world_from_real_robot_;
    Eigen::Isometry3d world_from_real_env_;
};

} // namespace digital_twin
} // namespace robot_sim
