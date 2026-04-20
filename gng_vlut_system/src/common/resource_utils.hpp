#pragma once

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#ifdef USE_ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace robot_sim {
namespace common {

/**
 * @brief Resolves a relative path to an absolute path based on project root or ROS 2 package share.
 */
inline std::string resolvePath(const std::string &relative_path) {
  if (relative_path.empty()) return "";
  
  std::filesystem::path input_path(relative_path);
  if (input_path.is_absolute()) {
    return relative_path;
  }

  // Handle "package://" URLs (ROS-style)
  std::string clean_rel = relative_path;
  if (relative_path.find("package://") == 0) {
    size_t second_slash = relative_path.find("/", 10);
    if (second_slash != std::string::npos) {
        std::string pkg_name = relative_path.substr(10, second_slash - 10);
        std::string sub_path = relative_path.substr(second_slash + 1);
        try {
#ifdef USE_ROS2
            std::string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
            return (std::filesystem::path(pkg_path) / sub_path).string();
#endif
        } catch (...) {}
    }
    clean_rel = relative_path.substr(10); 
  }
  std::filesystem::path clean_rel_path(clean_rel);

  const std::vector<std::string> search_prefixes = {
      "", "urdf/", "urdf/real_model/", "urdf/未作成/", "drawstuff/textures/", "experiment_settings/",
      "../", "../urdf/", "../urdf/real_model/"
  };

  auto find_in_base = [&](const std::filesystem::path &base) -> std::string {
    for (const auto &prefix : search_prefixes) {
      std::filesystem::path target = base / prefix / clean_rel_path;
      if (std::filesystem::exists(target)) return std::filesystem::absolute(target).string();
    }
    return "";
  };

  // 1. ROS 2 Package Share Directory (Priority)
#ifdef USE_ROS2
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("gng_safety");
    std::string found = find_in_base(std::filesystem::path(pkg_path));
    if (!found.empty()) return found;
  } catch (...) {}
#endif

  // 2. Environment variable
  const char *home_env = std::getenv("ML_GBGNG_HOME");
  if (home_env) {
    std::string found = find_in_base(std::filesystem::path(home_env));
    if (!found.empty()) return found;
  }

  // 3. CMake-defined project source dir
#ifdef PROJECT_SOURCE_DIR
  {
    std::string found = find_in_base(std::filesystem::path(PROJECT_SOURCE_DIR));
    if (!found.empty()) return found;
  }
#endif

  // 4. Fallback to current working directory
  {
    std::string found = find_in_base(std::filesystem::current_path());
    if (!found.empty()) return found;
  }

  return relative_path;
}

} // namespace common
} // namespace robot_sim
