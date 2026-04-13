#pragma once

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace robot_sim {
namespace common {

/**
 * @brief Resolves a relative path to an absolute path based on project root.
 * Priority:
 * 1. Environment variable ML_GBGNG_HOME
 * 2. CMake-defined PROJECT_SOURCE_DIR (if available)
 * 3. Current working directory
 */
inline std::string resolvePath(const std::string &relative_path) {
  std::filesystem::path rel(relative_path);
  if (rel.is_absolute()) {
    return relative_path;
  }

  // 1. Check environment variable
  const char *home_env = std::getenv("ML_GBGNG_HOME");
  if (home_env) {
    std::filesystem::path home(home_env);
    std::filesystem::path full = home / rel;
    if (std::filesystem::exists(full)) {
      return full.string();
    }
  }

  // 2. Check CMake-defined project source dir
#ifdef PROJECT_SOURCE_DIR
  {
    std::filesystem::path root(PROJECT_SOURCE_DIR);
    std::filesystem::path full = root / rel;
    if (std::filesystem::exists(full)) {
      return full.string();
    }
  }
#endif

  // 3. Fallback to current working directory or just return relative
  if (std::filesystem::exists(rel)) {
    return std::filesystem::absolute(rel).string();
  }

  // If not found, return the path as is (let the caller handle the failure)
  return relative_path;
}

} // namespace common
} // namespace robot_sim
