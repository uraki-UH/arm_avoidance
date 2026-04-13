#include "simulation/core/experiment_logger.hpp"
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define mkdir(path, mode) _mkdir(path)
#endif

namespace simulation {

ExperimentLogger::ExperimentLogger() {}

ExperimentLogger::~ExperimentLogger() { close(); }

void ExperimentLogger::initialize(int dof, const std::string &base_dir) {
  // Create base directory if it doesn't exist
  createDirectory(base_dir);

  // Generate timestamp for run directory
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  std::string timestamp = ss.str();

  current_run_dir_ = base_dir + "/run_" + timestamp;
  createDirectory(current_run_dir_);

  // Open CSV file
  std::string csv_path = current_run_dir_ + "/log_data.csv";
  log_file_.open(csv_path);

  if (!log_file_.is_open()) {
    std::cerr << "[ExperimentLogger] Failed to open log file: " << csv_path
              << " (Error: " << strerror(errno) << ")" << std::endl;
    return;
  }

  // Write Header
  log_file_ << "timestamp,scenario,mode,collision,";
  for (int i = 0; i < dof; ++i)
    log_file_ << "q_" << i << ",";
  for (int i = 0; i < dof; ++i)
    log_file_ << "dq_" << i << ",";
  for (int i = 0; i < dof; ++i)
    log_file_ << "tau_" << i << ",";
  log_file_ << "obs_x,obs_y,obs_z,min_dist,node_count,touched_node_count,comp_"
               "time_ms,env_update_ms,safety_update_ms"
            << std::endl;

  // Open Collision Log
  collision_log_file_.open(current_run_dir_ + "/collision_log.csv");
  if (collision_log_file_.is_open())
    collision_log_file_ << "Timestamp,Link,Other\n";

  // Open Obstacle Log
  obstacle_log_file_.open(current_run_dir_ + "/obstacle_log.csv");
  if (obstacle_log_file_.is_open())
    obstacle_log_file_ << "Timestamp,ID,X,Y,Z,VX,VY,VZ,Radius\n";

  // Open Robot Obstacle Log
  robot_obstacle_log_file_.open(current_run_dir_ + "/robot_obstacle_log.csv");
  if (robot_obstacle_log_file_.is_open())
    robot_obstacle_log_file_
        << "Timestamp,RobotName,JointCount,JointValues...,BaseX,BaseY,BaseZ\n";

  initialized_ = true;
}

void ExperimentLogger::logStep(
    double timestamp, const Eigen::VectorXf &joint_pos,
    const Eigen::VectorXf &joint_vel, const Eigen::VectorXf &joint_effort,
    const Eigen::Vector3d &obstacle_pos, double min_dist, int node_count,
    int touched_node_count, double computation_time_ms,
    double env_update_time_ms, double safety_update_time_ms,
    const std::string &scenario_name, const std::string &obstacle_mode,
    bool is_collision) {
  if (!initialized_)
    return;

  log_file_ << std::fixed << std::setprecision(6) << timestamp << ","
            << scenario_name << "," << obstacle_mode << ","
            << (is_collision ? 1 : 0) << ",";

  for (int i = 0; i < joint_pos.size(); ++i)
    log_file_ << joint_pos[i] << ",";
  for (int i = 0; i < joint_vel.size(); ++i)
    log_file_ << joint_vel[i] << ",";
  for (int i = 0; i < joint_effort.size(); ++i)
    log_file_ << joint_effort[i] << ",";

  log_file_ << obstacle_pos.x() << "," << obstacle_pos.y() << ","
            << obstacle_pos.z() << "," << min_dist << "," << node_count << ","
            << touched_node_count << "," << computation_time_ms << ","
            << env_update_time_ms << "," << safety_update_time_ms << std::endl;
}

void ExperimentLogger::logCollision(double timestamp,
                                    const std::string &link_name,
                                    const std::string &other_name) {
  if (!initialized_ || !collision_log_file_.is_open())
    return;
  collision_log_file_ << timestamp << "," << link_name << "," << other_name
                      << "\n";
}

void ExperimentLogger::logObstacle(double timestamp, int id,
                                   const Eigen::Vector3d &pos,
                                   const Eigen::Vector3d &vel, double radius) {
  if (!initialized_ || !obstacle_log_file_.is_open())
    return;

  obstacle_log_file_ << timestamp << "," << id << "," << pos.x() << ","
                     << pos.y() << "," << pos.z() << "," << vel.x() << ","
                     << vel.y() << "," << vel.z() << "," << radius << "\n";
}

void ExperimentLogger::logRobotObstacles(
    double timestamp, const std::vector<RobotObstacleState> &states) {
  if (!initialized_ || !robot_obstacle_log_file_.is_open())
    return;

  for (const auto &rs : states) {
    robot_obstacle_log_file_ << timestamp << "," << rs.name << ","
                             << rs.joint_positions.size();
    for (double q : rs.joint_positions) {
      robot_obstacle_log_file_ << "," << q;
    }
    robot_obstacle_log_file_ << "," << rs.base_pos.x() << "," << rs.base_pos.y()
                             << "," << rs.base_pos.z() << "\n";
  }
}

void ExperimentLogger::close() {
  if (log_file_.is_open()) {
    log_file_.close();
  }
  if (collision_log_file_.is_open())
    collision_log_file_.close();
  if (obstacle_log_file_.is_open())
    obstacle_log_file_.close();
  robot_obstacle_log_file_.close();

  initialized_ = false;
}

void ExperimentLogger::createDirectory(const std::string &path) {
#ifdef _WIN32
  _mkdir(path.c_str());
#else
  mkdir(path.c_str(), 0777);
#endif
}

} // namespace simulation
