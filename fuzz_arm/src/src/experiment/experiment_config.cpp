#include "experiment/experiment_config.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace robot_sim {
namespace experiment {

namespace {
// 簡易的なJSON解析ヘルパー関数
std::string trim(const std::string &str) {
  size_t first = str.find_first_not_of(" \t\n\r");
  if (first == std::string::npos)
    return "";
  size_t last = str.find_last_not_of(" \t\n\r");
  return str.substr(first, (last - first + 1));
}

double parseDouble(const std::string &value) { return std::stod(trim(value)); }

std::string parseString(const std::string &value) {
  std::string trimmed = trim(value);
  if (trimmed.front() == '"' && trimmed.back() == '"') {
    return trimmed.substr(1, trimmed.length() - 2);
  }
  return trimmed;
}

std::vector<double> parseArray(const std::string &value) {
  std::vector<double> result;
  std::string trimmed = trim(value);
  if (trimmed.front() == '[' && trimmed.back() == ']') {
    trimmed = trimmed.substr(1, trimmed.length() - 2);
    std::stringstream ss(trimmed);
    std::string item;
    while (std::getline(ss, item, ',')) {
      result.push_back(parseDouble(item));
    }
  }
  return result;
}
} // anonymous namespace

ExperimentConfig ExperimentConfig::loadFromFile(const std::string &filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "[ExperimentConfig] Warning: Could not open " << filepath
              << ". Using default configuration." << std::endl;
    return getDefault();
  }

  ExperimentConfig config = getDefault();
  std::string line;
  std::string current_section;

  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '{' || line[0] == '}')
      continue;

    // セクション検出
    if (line.find("\"obstacle\"") != std::string::npos) {
      current_section = "obstacle";
      continue;
    }

    // キー:値のペアを解析
    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos)
      continue;

    std::string key = line.substr(0, colon_pos);
    std::string value = line.substr(colon_pos + 1);

    // カンマを削除
    if (!value.empty() && value.back() == ',') {
      value = value.substr(0, value.length() - 1);
    }

    key = trim(key);
    value = trim(value);

    // キーから引用符を削除
    if (key.front() == '"' && key.back() == '"') {
      key = key.substr(1, key.length() - 2);
    }

    try {
      if (current_section == "obstacle") {
        if (key == "radius")
          config.obstacle.radius = parseDouble(value);
        else if (key == "orbit_radius")
          config.obstacle.orbit_radius = parseDouble(value);
        else if (key == "rotation_speed")
          config.obstacle.rotation_speed = parseDouble(value);
        else if (key == "z_min")
          config.obstacle.z_min = parseDouble(value);
        else if (key == "z_max")
          config.obstacle.z_max = parseDouble(value);
        else if (key == "z_oscillation_period")
          config.obstacle.z_oscillation_period = parseDouble(value);
      } else {
        if (key == "experiment_name")
          config.experiment_name = parseString(value);
        else if (key == "robot_urdf_path")
          config.robot_urdf_path = parseString(value);
        else if (key == "leaf_link_name")
          config.leaf_link_name = parseString(value);
        else if (key == "max_joint_velocity")
          config.max_joint_velocity = parseDouble(value);
        else if (key == "target_reach_threshold")
          config.target_reach_threshold = parseDouble(value);
        else if (key == "gng_map_file")
          config.gng_map_file = parseString(value);
        else if (key == "voxel_size")
          config.voxel_size = parseDouble(value);
        else if (key == "eef_collision_radius")
          config.eef_collision_radius = parseDouble(value);
        else if (key == "log_directory")
          config.log_directory = parseString(value);
        else if (key == "targets") {
          // ターゲット配列の解析（簡易版）
          config.targets.clear();
          std::string targets_str = value;
          if (targets_str.front() == '[' && targets_str.back() == ']') {
            targets_str = targets_str.substr(1, targets_str.length() - 2);
            size_t pos = 0;
            while (pos < targets_str.length()) {
              size_t start = targets_str.find('[', pos);
              size_t end = targets_str.find(']', start);
              if (start == std::string::npos || end == std::string::npos)
                break;

              std::string target_str =
                  targets_str.substr(start + 1, end - start - 1);
              auto coords = parseArray("[" + target_str + "]");
              if (coords.size() == 3) {
                config.targets.push_back(
                    Eigen::Vector3d(coords[0], coords[1], coords[2]));
              }
              pos = end + 1;
            }
          }
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "[ExperimentConfig] Warning: Failed to parse key '" << key
                << "': " << e.what() << std::endl;
    }
  }

  std::cout << "[ExperimentConfig] Loaded configuration from " << filepath
            << std::endl;
  std::cout << "  - Experiment: " << config.experiment_name << std::endl;
  std::cout << "  - Targets: " << config.targets.size() << std::endl;
  std::cout << "  - GNG Map: " << config.gng_map_file << std::endl;

  return config;
}

void ExperimentConfig::saveToFile(const std::string &filepath) const {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file for writing: " + filepath);
  }

  file << "{\n";
  file << "  \"experiment_name\": \"" << experiment_name << "\",\n";
  file << "  \"robot_urdf_path\": \"" << robot_urdf_path << "\",\n";
  file << "  \"leaf_link_name\": \"" << leaf_link_name << "\",\n";
  file << "  \"max_joint_velocity\": " << max_joint_velocity << ",\n";
  file << "  \"targets\": [\n";
  for (size_t i = 0; i < targets.size(); ++i) {
    file << "    [" << targets[i].x() << ", " << targets[i].y() << ", "
         << targets[i].z() << "]";
    if (i < targets.size() - 1)
      file << ",";
    file << "\n";
  }
  file << "  ],\n";
  file << "  \"target_reach_threshold\": " << target_reach_threshold << ",\n";
  file << "  \"gng_map_file\": \"" << gng_map_file << "\",\n";
  file << "  \"voxel_size\": " << voxel_size << ",\n";
  file << "  \"eef_collision_radius\": " << eef_collision_radius << ",\n";
  file << "  \"obstacle\": {\n";
  file << "    \"radius\": " << obstacle.radius << ",\n";
  file << "    \"orbit_radius\": " << obstacle.orbit_radius << ",\n";
  file << "    \"rotation_speed\": " << obstacle.rotation_speed << ",\n";
  file << "    \"z_min\": " << obstacle.z_min << ",\n";
  file << "    \"z_max\": " << obstacle.z_max << ",\n";
  file << "    \"z_oscillation_period\": " << obstacle.z_oscillation_period
       << "\n";
  file << "  },\n";
  file << "  \"log_directory\": \"" << log_directory << "\"\n";
  file << "}\n";

  std::cout << "[ExperimentConfig] Saved configuration to " << filepath
            << std::endl;
}

ExperimentConfig ExperimentConfig::getDefault() {
  ExperimentConfig config;

  // experiment_settings.mdに基づくデフォルト値
  config.targets = {
      Eigen::Vector3d(0.5, -0.1, 0.4), Eigen::Vector3d(-0.4, 0.5, 0.6),
      Eigen::Vector3d(0.1, -0.7, 0.3), Eigen::Vector3d(0.4, 0.4, 0.8),
      Eigen::Vector3d(-0.5, -0.2, 0.5)};

  return config;
}

} // namespace experiment
} // namespace robot_sim
