#include "experiment/scenario_definition.hpp"
#include "common/resource_utils.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace robot_sim {
namespace experiment {

namespace {
// 簡易JSON解析ヘルパー（experiment_config.cppと同様のスタイル）
std::string trim(const std::string &str) {
  size_t first = str.find_first_not_of(" \t\n\r");
  if (first == std::string::npos)
    return "";
  size_t last = str.find_last_not_of(" \t\n\r");
  return str.substr(first, (last - first + 1));
}

std::string removeQuotes(std::string s) {
  s = trim(s);
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

simulation::ObstacleBehavior stringToBehavior(const std::string &s) {
  std::string upper = s;
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "PASSIVE")
    return simulation::ObstacleBehavior::PASSIVE;
  if (upper == "AGGRESSIVE")
    return simulation::ObstacleBehavior::AGGRESSIVE;
  if (upper == "EVASIVE")
    return simulation::ObstacleBehavior::EVASIVE;
  if (upper == "HYBRID")
    return simulation::ObstacleBehavior::HYBRID;
  if (upper == "RANDOM")
    return simulation::ObstacleBehavior::RANDOM;
  if (upper == "TRAJECTORY")
    return simulation::ObstacleBehavior::TRAJECTORY;
  return simulation::ObstacleBehavior::PASSIVE;
}

std::vector<double> parseDoubleArray(const std::string &s) {
  std::vector<double> res;
  std::string trimmed = trim(s);
  if (trimmed.front() == '[' && trimmed.back() == ']') {
    std::string content = trimmed.substr(1, trimmed.size() - 2);
    std::stringstream ss(content);
    std::string item;
    while (std::getline(ss, item, ',')) {
      try {
        res.push_back(std::stod(trim(item)));
      } catch (...) {
      }
    }
  }
  return res;
}
} // namespace

ScenarioDefinition
ScenarioDefinition::loadFromJson(const std::string &filepath) {
  ScenarioDefinition def;
  // Resolve the path to support running from build directory
  std::string resolved_path = robot_sim::common::resolvePath(filepath);
  std::ifstream ifs(resolved_path);
  if (!ifs.is_open()) {
    std::cerr << "[ScenarioDefinition] Warning: Could not open "
              << resolved_path << " (Original: " << filepath
              << "). Using default." << std::endl;
    return getDefault();
  }

  // 簡易パース: 行ベースで "key": "value" を探す (配列内は少し工夫が必要)
  std::string line;
  bool in_stages = false;
  ScenarioStage current_stage;
  bool has_active_stage = false;

  while (std::getline(ifs, line)) {
    std::string trimmed = trim(line);
    if (trimmed.empty())
      continue;

    if (trimmed.find("\"stages\"") != std::string::npos) {
      in_stages = true;
      continue;
    }

    if (in_stages) {
      if (trimmed == "}," || trimmed == "}") {
        if (has_active_stage) {
          def.stages.push_back(current_stage);
          current_stage = ScenarioStage();
          has_active_stage = false;
        }
        continue;
      }
      if (trimmed.find("{") != std::string::npos) {
        has_active_stage = true;
        continue;
      }
      if (trimmed == "]") {
        in_stages = false;
        continue;
      }
    }

    size_t colon = trimmed.find(':');
    if (colon == std::string::npos)
      continue;

    std::string key = removeQuotes(trimmed.substr(0, colon));
    std::string val_full = trim(trimmed.substr(colon + 1));
    if (val_full.back() == ',')
      val_full.pop_back();
    std::string val = removeQuotes(val_full);

    if (!in_stages) {
      if (key == "scenario_name")
        def.scenario_name = val;
    } else {
      if (key == "target") {
        std::string arr_str = val_full;
        // もし行が [ で終わっているか、]
        // が含まれていない場合は次以降の行も読み込む
        while (arr_str.find(']') == std::string::npos &&
               std::getline(ifs, line)) {
          arr_str += trim(line);
        }
        if (arr_str.back() == ',')
          arr_str.pop_back();

        auto arr = parseDoubleArray(arr_str);
        if (arr.size() == 3) {
          current_stage.target_pos = Eigen::Vector3d(arr[0], arr[1], arr[2]);
        }
      } else if (key == "behavior") {
        current_stage.behavior = stringToBehavior(val);
      } else if (key == "speed") {
        current_stage.obstacle_speed = std::stod(val);
      } else if (key == "radius") {
        current_stage.obstacle_radius = std::stod(val);
      } else if (key == "timeout") {
        current_stage.timeout = std::stod(val);
      } else if (key == "tolerance") {
        current_stage.target_tolerance = std::stod(val);
      } else if (key == "name") {
        current_stage.name = val;
      }
    }
  }

  std::cout << "[ScenarioDefinition] Loaded '" << def.scenario_name << "' with "
            << def.stages.size() << " stages." << std::endl;
  return def;
}

ScenarioDefinition ScenarioDefinition::getDefault() {
  ScenarioDefinition def;
  def.scenario_name = "Default Progress";

  ScenarioStage s1;
  s1.target_pos = Eigen::Vector3d(0.5, -0.1, 0.4);
  s1.behavior = simulation::ObstacleBehavior::PASSIVE;
  s1.obstacle_speed = 0.5;
  s1.name = "Beginning";
  def.stages.push_back(s1);

  ScenarioStage s2;
  s2.target_pos = Eigen::Vector3d(-0.4, 0.5, 0.6);
  s2.behavior = simulation::ObstacleBehavior::AGGRESSIVE;
  s2.obstacle_speed = 1.0;
  s2.obstacle_radius = 0.08;
  s2.name = "The Hunter";
  def.stages.push_back(s2);

  return def;
}

} // namespace experiment
} // namespace robot_sim
