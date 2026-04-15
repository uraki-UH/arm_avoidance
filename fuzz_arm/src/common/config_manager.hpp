#pragma once
#include <map>
#include <string>

namespace common {

/**
 * @brief Singleton class to manage configuration parameters from a file.
 */
class ConfigManager {
public:
  static ConfigManager &Instance();

  /**
   * @brief Load configuration from a file (key = value format).
   * @param filename Path to the config file.
   * @return true if successful.
   */
  bool Load(const std::string &filename);

  /**
   * @brief Get a string value for a key.
   */
  std::string Get(const std::string &key,
                  const std::string &default_val = "") const;

  /**
   * @brief Manually set or override a configuration value.
   */
  void Set(const std::string &key, const std::string &val);

  /**
   * @brief Get a double value for a key.
   */
  double GetDouble(const std::string &key, double default_val = 0.0) const;

  /**
   * @brief Get an integer value for a key.
   */
  int GetInt(const std::string &key, int default_val = 0) const;
  bool GetBool(const std::string &key, bool default_val = false) const;

  /**
   * @brief Construct a standardized file path based on experiment_id and
   * directory.
   * @param suffix_key The key in config to get the suffix from.
   * @param default_suffix Default suffix if key is not found.
   */
  std::string GetFileName(const std::string &suffix_key,
                          const std::string &default_suffix) const;

  /**
   * @brief Directly construct a path using a given suffix.
   */
  std::string ConstructPath(const std::string &suffix) const;

private:
  ConfigManager() = default;
  std::map<std::string, std::string> config_map_;
};

} // namespace common
