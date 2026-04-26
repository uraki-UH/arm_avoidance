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

inline std::string stripUriScheme(const std::string &path) {
  if (path.find("file://") == 0) {
    return path.substr(7);
  }
  return path;
}

inline std::filesystem::path deriveResourceRootFromMeshRoot(
    const std::string &mesh_root_dir) {
  std::filesystem::path mesh_root(stripUriScheme(mesh_root_dir));
  if (mesh_root.empty()) {
    return {};
  }

  // Accept either ".../meshes" or ".../meshes/<variant>" and return the
  // directory above meshes so package:// URIs can be rewritten generically.
  if (mesh_root.filename() == "meshes") {
    return mesh_root.parent_path();
  }
  if (mesh_root.has_parent_path() && mesh_root.parent_path().filename() == "meshes") {
    return mesh_root.parent_path().parent_path();
  }
  return mesh_root.parent_path();
}

inline std::string resolvePackageUris(const std::string &text) {
  if (text.empty()) {
    return text;
  }

  std::string rewritten = text;
  std::size_t pos = 0;
  while ((pos = rewritten.find("package://", pos)) != std::string::npos) {
    const std::size_t subpath_start = rewritten.find('/', pos + 10);
    if (subpath_start == std::string::npos) {
      break;
    }

    std::string pkg_name = rewritten.substr(pos + 10, subpath_start - (pos + 10));
    std::string pkg_path;
    try {
#ifdef USE_ROS2
      pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
#else
      // Fallback or error if not in ROS 2 environment
      pos = subpath_start;
      continue;
#endif
    } catch (...) {
      // Package not found, skip this one
      pos = subpath_start;
      continue;
    }

    const std::string replacement = "file://" + pkg_path + "/";
    rewritten.replace(pos, subpath_start - pos + 1, replacement);
    pos += replacement.size();
  }
  return rewritten;
}

/**
 * @brief Returns the root directory of a package. 
 * Prioritizes the source directory during development (if PROJECT_SOURCE_DIR is defined and exists),
 * otherwise falls back to the ROS 2 share directory.
 */
inline std::string getPackageRoot(const std::string &pkg_name = "gng_vlut_system") {
#ifdef PROJECT_SOURCE_DIR
  if (std::filesystem::exists(PROJECT_SOURCE_DIR)) {
    return PROJECT_SOURCE_DIR;
  }
#endif

#ifdef USE_ROS2
  try {
    return ament_index_cpp::get_package_share_directory(pkg_name);
  } catch (...) {}
#endif

  return ".";
}

/**
 * @brief Resolves a data path (like gng_results) relative to the package root.
 */
inline std::string resolveDataPath(const std::string &relative_path, const std::string &pkg_name = "gng_vlut_system") {
  if (relative_path.empty()) return "";
  
  std::filesystem::path rel(relative_path);
  if (rel.is_absolute()) return relative_path;

  std::filesystem::path root(getPackageRoot(pkg_name));
  return (root / rel).string();
}

/**
 * @brief Resolves a relative path to an absolute path based on project root or ROS 2 package share.
 */
inline std::string resolvePath(const std::string &relative_path) {
  if (relative_path.empty()) return "";

  // 1. Handle "package://" URLs (ROS-style)
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
  }

  std::string normalized_path = stripUriScheme(relative_path);
  std::filesystem::path input_path(normalized_path);
  if (input_path.is_absolute()) {
    return normalized_path;
  }

  std::string clean_rel = normalized_path;
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

  // 2. ROS 2 Package Share Directory (Priority)
#ifdef USE_ROS2
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    std::string found = find_in_base(std::filesystem::path(pkg_path));
    if (!found.empty()) return found;
  } catch (...) {}
#endif

  // 3. Environment variable
  const char *home_env = std::getenv("ML_GBGNG_HOME");
  if (home_env) {
    std::string found = find_in_base(std::filesystem::path(home_env));
    if (!found.empty()) return found;
  }

  // 4. CMake-defined project source dir
#ifdef PROJECT_SOURCE_DIR
  {
    std::string found = find_in_base(std::filesystem::path(PROJECT_SOURCE_DIR));
    if (!found.empty()) return found;
  }
#endif

  // 5. Fallback to current working directory
  {
    std::string found = find_in_base(std::filesystem::current_path());
    if (!found.empty()) return found;
  }

  return relative_path;
}

} // namespace common
} // namespace robot_sim
