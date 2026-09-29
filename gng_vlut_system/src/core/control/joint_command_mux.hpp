#pragma once

#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <utility>

namespace robot_sim::control {

// 関節単位の指令仲裁。時間は呼び出し側の単調時計、単位は秒
class joint_command_mux {
public:
  struct claim {
    std::set<std::string> joints;
    int priority = 0;
    bool is_exclusive = true;
    bool enable_source = true;
    double command_timeout_sec = 0.0;
  };
  struct alias {
    std::string parent;
    double multiplier = 1.0;
    double offset = 0.0;
  };
  bool set_alias(const std::string &name, const alias &value) {
    if (!is_allowed(name) || !is_allowed(value.parent) || name == value.parent ||
        !std::isfinite(value.multiplier) || value.multiplier == 0.0 ||
        !std::isfinite(value.offset)) return false;
    aliases_[name] = value;
    return true;
  }
  struct selection {
    std::map<std::string, double> active;
    std::map<std::string, double> held;
  };

  explicit joint_command_mux(std::set<std::string> joint_names = {})
      : allowed_joints_(std::move(joint_names)) {}

  bool set_claim(const std::string &topic, const claim &value) {
    if (topic.empty() || !std::isfinite(value.command_timeout_sec) ||
        value.command_timeout_sec < 0.0) return false;
    auto canonical = value;
    canonical.joints.clear();
    for (const auto &name : value.joints) {
      if (!is_allowed(name)) return false;
      const auto alias_iter = aliases_.find(name);
      canonical.joints.insert(alias_iter == aliases_.end() ? name : alias_iter->second.parent);
    }
    auto &source = sources_[topic];
    if (!value.enable_source || !source.settings.enable_source) source.values.clear();
    source.settings = canonical;
    for (auto iter = source.values.begin(); iter != source.values.end();) {
      if (!canonical.joints.empty() && !canonical.joints.count(iter->first)) {
        iter = source.values.erase(iter);
      } else {
        ++iter;
      }
    }
    return true;
  }

  // メッセージ全体の妥当性確認後の部分更新。省略関節の値・受信時刻を保持
  bool set_command(const std::string &topic, const std::vector<std::string> &names,
                   const std::vector<double> &positions, double received_sec) {
    auto source = sources_.find(topic);
    if (source == sources_.end() || !source->second.settings.enable_source ||
        names.empty() || names.size() != positions.size() || !std::isfinite(received_sec)) {
      return false;
    }
    std::set<std::string> seen;
    const auto &scope = source->second.settings.joints;
    std::map<std::string, double> canonical;
    for (std::size_t idx = 0; idx < names.size(); ++idx) {
      if (!is_allowed(names[idx]) || !seen.insert(names[idx]).second ||
          !std::isfinite(positions[idx])) return false;
      const auto alias_iter = aliases_.find(names[idx]);
      const auto parent = alias_iter == aliases_.end() ? names[idx] : alias_iter->second.parent;
      const double value = alias_iter == aliases_.end() ? positions[idx] :
          (positions[idx] - alias_iter->second.offset) / alias_iter->second.multiplier;
      if ((!scope.empty() && !scope.count(parent)) || !std::isfinite(value)) return false;
      const auto previous = canonical.find(parent);
      if (previous != canonical.end() && std::abs(previous->second - value) > 1e-6) return false;
      canonical[parent] = value;
    }
    for (const auto &[name, value] : canonical) {
      source->second.values[name] = {value, received_sec};
    }
    return true;
  }

  bool clear_command(const std::string &topic) {
    const auto source = sources_.find(topic);
    if (source == sources_.end()) return false;
    source->second.values.clear();
    return true;
  }

  selection resolve(double now_sec, bool enable_hold_last_output = true) {
    selection result;
    std::map<std::string, const source_state *> owners;
    // 同順位・同モードの競合はトピック名の辞書順。受信順に非依存
    for (const auto &[topic, source] : sources_) {
      (void)topic;
      if (!source.settings.enable_source) continue;
      for (const auto &[name, value] : source.values) {
        const auto timeout = source.settings.command_timeout_sec;
        if (timeout > 0.0 && now_sec - value.received_sec > timeout) continue;
        const auto owner = owners.find(name);
        bool can_take = owner == owners.end();
        if (!can_take) {
          const auto &current = owner->second->settings;
          can_take = source.settings.is_exclusive != current.is_exclusive
                         ? source.settings.is_exclusive
                         : source.settings.priority > current.priority;
        }
        if (can_take) {
          owners[name] = &source;
          result.active[name] = value.position;
        }
      }
    }
    if (!enable_hold_last_output) held_.clear();
    for (const auto &[name, value] : result.active) held_[name] = value;
    result.held = held_;
    return result;
  }

private:
  struct joint_value {
    double position;
    double received_sec;
  };
  struct source_state {
    claim settings;
    std::map<std::string, joint_value> values;
  };
  bool is_allowed(const std::string &name) const {
    return !name.empty() && (allowed_joints_.empty() || allowed_joints_.count(name));
  }
  std::map<std::string, alias> aliases_;
  std::set<std::string> allowed_joints_;
  std::map<std::string, source_state> sources_;
  std::map<std::string, double> held_;
};

}  // 名前空間robot_sim::control
