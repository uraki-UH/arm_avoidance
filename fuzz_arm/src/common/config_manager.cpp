#include "common/config_manager.hpp"
#include "common/resource_utils.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>

namespace common {

ConfigManager &ConfigManager::Instance() {
  static ConfigManager instance;
  return instance;
}

bool ConfigManager::Load(const std::string &filename) {
  // Use resource_utils to resolve the config file path
  std::string resolved_path = robot_sim::common::resolvePath(filename);

  std::ifstream ifs(resolved_path);
  if (!ifs.is_open()) {
    std::cerr << "[ConfigManager] Error: Could not open config file: "
              << resolved_path << " (Original: " << filename << ")"
              << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    // Trim whitespace
    if (line.empty())
      continue;

    // Remove comments
    size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    if (line.empty())
      continue;

    // Parse key=value
    size_t eq_pos = line.find('=');
    if (eq_pos != std::string::npos) {
      std::string key = line.substr(0, eq_pos);
      std::string val = line.substr(eq_pos + 1);

      // Trim key/val
      auto trim = [](std::string &s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
      };
      trim(key);
      trim(val);

      if (!key.empty()) {
        config_map_[key] = val;
      }
    }
  }

  std::cout << "[ConfigManager] Loaded " << config_map_.size()
            << " entries from " << resolved_path << std::endl;
  return true;
}

std::string ConfigManager::Get(const std::string &key,
                               const std::string &default_val) const {
  auto it = config_map_.find(key);
  if (it != config_map_.end()) {
    return it->second;
  }
  return default_val;
}

void ConfigManager::Set(const std::string &key, const std::string &val) {
  config_map_[key] = val;
}

double ConfigManager::GetDouble(const std::string &key,
                                double default_val) const {
  auto it = config_map_.find(key);
  if (it != config_map_.end()) {
    try {
      return std::stod(it->second);
    } catch (...) {
      std::cerr << "[ConfigManager] Warning: Failed to parse double for key "
                << key << std::endl;
    }
  }
  return default_val;
}

int ConfigManager::GetInt(const std::string &key, int default_val) const {
  auto it = config_map_.find(key);
  if (it != config_map_.end()) {
    try {
      return std::stoi(it->second);
    } catch (...) {
      std::cerr << "[ConfigManager] Warning: Failed to parse int for key "
                << key << std::endl;
    }
  }
  return default_val;
}

bool ConfigManager::GetBool(const std::string &key, bool default_val) const {
  auto it = config_map_.find(key);
  if (it != config_map_.end()) {
    std::string val = it->second;
    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    if (val == "true" || val == "1" || val == "on" || val == "yes")
      return true;
    if (val == "false" || val == "0" || val == "off" || val == "no")
      return false;
  }
  return default_val;
}

std::string
ConfigManager::GetFileName(const std::string &suffix_key,
                           const std::string &default_suffix) const {
  std::string exp_id = Get("experiment_id", "default_run");
  std::string data_dir = Get("data_directory", ".");

  // Resolve data_dir using resource_utils
  data_dir = robot_sim::common::resolvePath(data_dir);

  std::string suffix = Get(suffix_key, default_suffix);

  // Check if data_dir ends with slash
  if (!data_dir.empty() && data_dir.back() != '/') {
    data_dir += "/";
  }

  return data_dir + exp_id + suffix + ".bin";
}

std::string ConfigManager::ConstructPath(const std::string &suffix) const {
  std::string exp_id = Get("experiment_id", "default_run");
  std::string data_dir = Get("data_directory", ".");

  // Resolve data_dir using resource_utils
  data_dir = robot_sim::common::resolvePath(data_dir);

  if (!data_dir.empty() && data_dir.back() != '/') {
    data_dir += "/";
  }

  return data_dir + exp_id + suffix + ".bin";
}

} // namespace common
