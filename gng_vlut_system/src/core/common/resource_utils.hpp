#pragma once

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

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

inline std::filesystem::path findLocalPackageShareFallback(
    const std::string &pkg_name) {
  return {};
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
      pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
    } catch (...) {
      const auto fallback = findLocalPackageShareFallback(pkg_name);
      if (fallback.empty()) {
        // Package not found, skip this one
        pos = subpath_start;
        continue;
      }
      pkg_path = fallback.string();
    }

    const std::string replacement = "file://" + pkg_path + "/";
    rewritten.replace(pos, subpath_start - pos + 1, replacement);
    pos += replacement.size();
  }
  return rewritten;
}

inline std::filesystem::path resolveMeshRootForRelativeUris(
    const std::string &resource_root_dir,
    const std::string &mesh_root_dir) {
  if (!mesh_root_dir.empty()) {
    return std::filesystem::path(stripUriScheme(mesh_root_dir));
  }

  if (!resource_root_dir.empty()) {
    return std::filesystem::path(stripUriScheme(resource_root_dir)) / "meshes";
  }

  return {};
}

inline std::string rewriteRelativeMeshUris(const std::string &text,
                                           const std::string &resource_root_dir,
                                           const std::string &mesh_root_dir) {
  const auto mesh_root = resolveMeshRootForRelativeUris(resource_root_dir, mesh_root_dir);
  if (text.empty() || mesh_root.empty()) {
    return text;
  }

  std::string rewritten = text;
  const std::string prefix = "file://" + mesh_root.string() + "/";

  const std::string double_quote_key = "filename=\"meshes/";
  std::size_t pos = 0;
  while ((pos = rewritten.find(double_quote_key, pos)) != std::string::npos) {
    rewritten.replace(pos, double_quote_key.size(), "filename=\"" + prefix);
    pos += prefix.size();
  }

  const std::string single_quote_key = "filename='meshes/";
  pos = 0;
  while ((pos = rewritten.find(single_quote_key, pos)) != std::string::npos) {
    rewritten.replace(pos, single_quote_key.size(), "filename='" + prefix);
    pos += prefix.size();
  }

  return rewritten;
}

inline void replaceAll(std::string &text,
                       const std::string &from,
                       const std::string &to) {
  if (from.empty()) {
    return;
  }

  std::size_t pos = 0;
  while ((pos = text.find(from, pos)) != std::string::npos) {
    text.replace(pos, from.size(), to);
    pos += to.size();
  }
}

inline std::string extractQuotedAttribute(const std::string &tag,
                                          const std::string &attribute) {
  const std::string key = attribute + "=";
  const std::size_t key_pos = tag.find(key);
  if (key_pos == std::string::npos) {
    return "";
  }

  const std::size_t quote_pos = key_pos + key.size();
  if (quote_pos >= tag.size()) {
    return "";
  }
  const char quote = tag[quote_pos];
  if (quote != '"' && quote != '\'') {
    return "";
  }

  const std::size_t value_start = quote_pos + 1;
  const std::size_t value_end = tag.find(quote, value_start);
  if (value_end == std::string::npos) {
    return "";
  }
  return tag.substr(value_start, value_end - value_start);
}

inline std::string inlineReferencedMaterialColors(const std::string &text) {
  if (text.empty()) {
    return text;
  }

  std::unordered_map<std::string, std::string> material_colors;
  std::size_t pos = 0;
  while ((pos = text.find("<material", pos)) != std::string::npos) {
    const std::size_t tag_end = text.find('>', pos);
    if (tag_end == std::string::npos) {
      break;
    }

    const std::string material_tag = text.substr(pos, tag_end - pos + 1);
    const std::string name = extractQuotedAttribute(material_tag, "name");
    const std::size_t material_end = text.find("</material>", tag_end);
    if (!name.empty() && material_end != std::string::npos) {
      const std::string body = text.substr(tag_end + 1, material_end - tag_end - 1);
      const std::size_t color_pos = body.find("<color");
      if (color_pos != std::string::npos) {
        const std::size_t color_end = body.find('>', color_pos);
        if (color_end != std::string::npos) {
          const std::string color_tag = body.substr(color_pos, color_end - color_pos + 1);
          const std::string rgba = extractQuotedAttribute(color_tag, "rgba");
          if (!rgba.empty()) {
            material_colors[name] = rgba;
          }
        }
      }
      pos = material_end + 11;
    } else {
      pos = tag_end + 1;
    }
  }

  std::string rewritten = text;
  for (const auto &[name, rgba] : material_colors) {
    const std::string expanded =
        "<material name=\"" + name + "\"><color rgba=\"" + rgba + "\"/></material>";
    replaceAll(rewritten, "<material name=\"" + name + "\"/>", expanded);
    replaceAll(rewritten, "<material name=\"" + name + "\" />", expanded);
    replaceAll(rewritten, "<material name='" + name + "'/>", expanded);
    replaceAll(rewritten, "<material name='" + name + "' />", expanded);
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

  try {
    return ament_index_cpp::get_package_share_directory(pkg_name);
  } catch (...) {}

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
              std::string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
              return (std::filesystem::path(pkg_path) / sub_path).string();
          } catch (...) {}
          const auto fallback = findLocalPackageShareFallback(pkg_name);
          if (!fallback.empty()) {
              return (fallback / sub_path).string();
          }
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
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("gng_vlut_system");
    std::string found = find_in_base(std::filesystem::path(pkg_path));
    if (!found.empty()) return found;
  } catch (...) {}

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
