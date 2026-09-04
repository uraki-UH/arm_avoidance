#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <nlohmann/json.hpp>

namespace robot_sim::bridge
{
namespace
{

using json = nlohmann::json;

bool isFinite(double value)
{
  return std::isfinite(value);
}

}

class ObjectTemplateMatchValidatorNode : public rclcpp::Node
{
public:
  explicit ObjectTemplateMatchValidatorNode(const rclcpp::NodeOptions &options)
  : Node("object_template_match_validator_node", options)
  {
    template_id_ = declare_parameter<std::string>("template_id", "");
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "");
    state_topic_ = declare_parameter<std::string>("state_topic", "");
    min_visible_ratio_ = declare_parameter<double>("min_visible_ratio", 0.20);
    recognition_threshold_ = declare_parameter<double>("recognition_threshold", 0.55);
    min_template_score_margin_ = declare_parameter<double>("min_template_score_margin", 0.10);
    confirmation_time_sec_ = declare_parameter<double>("confirmation_time_sec", 0.5);
    activate_score_th_ = declare_parameter<double>("activate_score_th", 0.65);
    deactivate_score_th_ = declare_parameter<double>("deactivate_score_th", 0.40);
    min_matched_node_ratio_ = declare_parameter<double>("min_matched_node_ratio", 0.20);
    min_matched_edge_ratio_ = declare_parameter<double>("min_matched_edge_ratio", 0.15);
    enable_plane_cluster_evidence_ = declare_parameter<bool>("enable_plane_cluster_evidence", true);
    min_plane_support_score_ = declare_parameter<double>("min_plane_support_score", 0.60);
    max_missing_node_ratio_ = declare_parameter<double>("max_missing_node_ratio", 0.65);
    max_contradiction_point_ratio_ = declare_parameter<double>("max_contradiction_point_ratio", 0.20);
    enable_nonplane_component_evaluation_ = declare_parameter<bool>(
      "enable_nonplane_component_evaluation", true);
    min_nonplane_evidence_frame_num_ = declare_parameter<int>(
      "min_nonplane_evidence_frame_num", 5);
    min_nonplane_evidence_score_th_ = declare_parameter<double>(
      "min_nonplane_evidence_score_th", 0.55);
    min_confirmed_frame_num_ = declare_parameter<int>("min_confirmed_frame_num", 5);
    min_confirm_duration_sec_ = declare_parameter<double>("min_confirm_duration_sec", 0.5);
    max_lost_duration_sec_ = declare_parameter<double>("max_lost_duration_sec", 1.0);
    state_publish_hz_ = declare_parameter<double>("state_publish_hz", 2.0);
    validateParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ObjectTemplateMatchValidatorNode::onSetParameters, this, std::placeholders::_1));
    if (candidate_topic_.empty()) {
      candidate_topic_ = "/" + template_id_ + "/object_template_match_candidates";
    }
    if (state_topic_.empty()) {
      state_topic_ = "/" + template_id_ + "/object_template_match_state";
    }
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    candidate_subscription_ = create_subscription<std_msgs::msg::String>(
      candidate_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ObjectTemplateMatchValidatorNode::onCandidate, this, std::placeholders::_1));
    const auto period = std::chrono::duration<double>(1.0 / state_publish_hz_);
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObjectTemplateMatchValidatorNode::onStateTimer, this));
    publishState();
    RCLCPP_INFO(
      get_logger(), "物体テンプレート照合検証開始: template=%s candidate=%s state=%s",
      template_id_.c_str(), candidate_topic_.c_str(), state_topic_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    std::vector<std::function<void()>> rollbacks;
    const auto stage = [&rollbacks](auto &field, const auto value) {
        using Field = std::decay_t<decltype(field)>;
        Field *target = &field;
        const Field previous = field;
        const Field next = static_cast<Field>(value);
        if (previous != next) {
          rollbacks.emplace_back([target, previous]() {*target = previous;});
          field = next;
        }
      };

    try {
      for (const auto &parameter : parameters) {
        const std::string &name = parameter.get_name();
        if (name == "min_visible_ratio") stage(min_visible_ratio_, parameter.as_double());
        else if (name == "recognition_threshold") stage(recognition_threshold_, parameter.as_double());
        else if (name == "min_template_score_margin") {
          stage(min_template_score_margin_, parameter.as_double());
        }
        else if (name == "confirmation_time_sec") stage(confirmation_time_sec_, parameter.as_double());
        else if (name == "activate_score_th") stage(activate_score_th_, parameter.as_double());
        else if (name == "deactivate_score_th") stage(deactivate_score_th_, parameter.as_double());
        else if (name == "min_matched_node_ratio") stage(min_matched_node_ratio_, parameter.as_double());
        else if (name == "min_matched_edge_ratio") stage(min_matched_edge_ratio_, parameter.as_double());
        else if (name == "enable_plane_cluster_evidence") {
          stage(enable_plane_cluster_evidence_, parameter.as_bool());
        } else if (name == "min_plane_support_score") {
          stage(min_plane_support_score_, parameter.as_double());
        } else if (name == "max_missing_node_ratio") {
          stage(max_missing_node_ratio_, parameter.as_double());
        } else if (name == "max_contradiction_point_ratio") {
          stage(max_contradiction_point_ratio_, parameter.as_double());
        } else if (name == "enable_nonplane_component_evaluation") {
          stage(enable_nonplane_component_evaluation_, parameter.as_bool());
        } else if (name == "min_nonplane_evidence_frame_num") {
          stage(min_nonplane_evidence_frame_num_, parameter.as_int());
        } else if (name == "min_nonplane_evidence_score_th") {
          stage(min_nonplane_evidence_score_th_, parameter.as_double());
        } else if (name == "min_confirmed_frame_num") {
          stage(min_confirmed_frame_num_, parameter.as_int());
        } else if (name == "min_confirm_duration_sec") {
          stage(min_confirm_duration_sec_, parameter.as_double());
        } else if (name == "max_lost_duration_sec") {
          stage(max_lost_duration_sec_, parameter.as_double());
        } else if (name == "state_publish_hz") {
          throw std::runtime_error("state_publish_hzの変更にはノード再起動が必要です。");
        }
      }
      validateParameters();
      result.successful = true;
      result.reason = "success";
      return result;
    } catch (const std::exception &error) {
      for (auto iterator = rollbacks.rbegin(); iterator != rollbacks.rend(); ++iterator) {
        (*iterator)();
      }
      result.reason = error.what();
      return result;
    }
  }

  void validateParameters() const
  {
    if (template_id_.empty() || min_confirmed_frame_num_ <= 0 || state_publish_hz_ <= 0.0 ||
      !isFinite(min_visible_ratio_) || !isFinite(recognition_threshold_) ||
      !isFinite(min_template_score_margin_) ||
      !isFinite(confirmation_time_sec_) || min_visible_ratio_ < 0.0 || min_visible_ratio_ > 1.0 ||
      recognition_threshold_ < 0.0 || recognition_threshold_ > 1.0 || confirmation_time_sec_ < 0.0 ||
      min_template_score_margin_ < 0.0 || min_template_score_margin_ > 1.0 ||
      min_confirm_duration_sec_ < 0.0 || max_lost_duration_sec_ < 0.0 ||
      !isFinite(activate_score_th_) || !isFinite(deactivate_score_th_) ||
      !isFinite(min_matched_node_ratio_) || !isFinite(min_matched_edge_ratio_) ||
      !isFinite(min_plane_support_score_) ||
      !isFinite(max_missing_node_ratio_) || !isFinite(max_contradiction_point_ratio_) ||
      !isFinite(min_nonplane_evidence_score_th_) ||
      activate_score_th_ < 0.0 || activate_score_th_ > 1.0 ||
      deactivate_score_th_ < 0.0 || deactivate_score_th_ > activate_score_th_ ||
      min_matched_node_ratio_ < 0.0 || min_matched_node_ratio_ > 1.0 ||
      min_matched_edge_ratio_ < 0.0 || min_matched_edge_ratio_ > 1.0 ||
      min_plane_support_score_ < 0.0 || min_plane_support_score_ > 1.0 ||
      max_missing_node_ratio_ < 0.0 || max_missing_node_ratio_ > 1.0 ||
      max_contradiction_point_ratio_ < 0.0 || max_contradiction_point_ratio_ > 1.0 ||
      min_nonplane_evidence_frame_num_ <= 0 || min_nonplane_evidence_score_th_ < 0.0 ||
      min_nonplane_evidence_score_th_ > 1.0)
    {
      throw std::runtime_error("物体テンプレート照合検証パラメータが不正です。");
    }
  }

  bool hasCurrentNonplaneEvidence(const json &candidate) const
  {
    return candidate.value("is_nonplane_component_observed", false) &&
      candidate.value("nonplane_evidence_score", 0.0) >= min_nonplane_evidence_score_th_;
  }

  bool isBaseActivationCandidate(const json &candidate) const
  {
    const bool has_nonplane_evidence = enable_nonplane_component_evaluation_ &&
      hasCurrentNonplaneEvidence(candidate);
    const bool has_edge_evidence = candidate.value("matched_edge_ratio", 0.0) >=
      min_matched_edge_ratio_ || has_nonplane_evidence;
    const bool has_plane_evidence = !enable_plane_cluster_evidence_ ||
      !candidate.value("is_plane_cluster_observed", false) ||
      candidate.value("plane_support_score", 0.0) >= min_plane_support_score_ ||
      has_nonplane_evidence;
    const bool has_template_margin = !candidate.contains("template_score_margin") ||
      candidate.value("template_score_margin", 0.0) >= min_template_score_margin_;
    return !candidate.value("is_falsified", false) &&
      candidate.value("is_template_winner", true) &&
      has_template_margin &&
      candidate.value("score", 0.0) >= std::max(recognition_threshold_, activate_score_th_) &&
      candidate.value("visible_ratio", candidate.value("matched_node_ratio", 0.0)) >= min_visible_ratio_ &&
      has_edge_evidence &&
      candidate.value("missing_node_ratio", 1.0) <= max_missing_node_ratio_ &&
      candidate.value("contradiction_point_ratio", 1.0) <= max_contradiction_point_ratio_ &&
      has_plane_evidence;
  }

  bool requiresNonplaneEvidence(const json &candidate) const
  {
    return enable_nonplane_component_evaluation_ &&
      candidate.value("is_nonplane_component_observed", false);
  }

  bool isActivationCandidate(const json &candidate) const
  {
    return isBaseActivationCandidate(candidate) &&
      (!requiresNonplaneEvidence(candidate) ||
      nonplane_evidence_frame_num_ >= min_nonplane_evidence_frame_num_);
  }

  bool isContinuationCandidate(const json &candidate) const
  {
    const bool has_nonplane_evidence = enable_nonplane_component_evaluation_ &&
      hasCurrentNonplaneEvidence(candidate);
    const bool has_template_margin = !candidate.contains("template_score_margin") ||
      candidate.value("template_score_margin", 0.0) >= min_template_score_margin_;
    return !candidate.value("is_falsified", false) &&
      candidate.value("is_template_winner", true) &&
      has_template_margin &&
      candidate.value("score", 0.0) >= deactivate_score_th_ &&
      candidate.value("visible_ratio", candidate.value("matched_node_ratio", 0.0)) >= min_visible_ratio_ &&
      (candidate.value("matched_edge_ratio", 0.0) >= min_matched_edge_ratio_ ||
      has_nonplane_evidence) &&
      candidate.value("missing_node_ratio", 1.0) <= max_missing_node_ratio_ &&
      candidate.value("contradiction_point_ratio", 1.0) <= max_contradiction_point_ratio_;
  }

  void onCandidate(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const json candidate = json::parse(message->data);
      if (candidate.value("template_id", "") != template_id_) {
        return;
      }
      last_candidate_ = candidate;
      const auto now_time = std::chrono::steady_clock::now();
      if (is_confirmed_) {
        if (candidate.value("is_falsified", false)) {
          is_confirmed_ = false;
          consecutive_frame_num_ = 0;
          nonplane_evidence_frame_num_ = 0;
          first_confirmable_time_.reset();
          last_supported_time_.reset();
          RCLCPP_INFO(
            get_logger(), "物体テンプレート仮説破棄: template=%s contradiction_point_ratio=%.3f",
            template_id_.c_str(), candidate.value("contradiction_point_ratio", 1.0));
          publishState();
          return;
        }
        if (isContinuationCandidate(candidate)) {
          last_supported_time_ = now_time;
          publishState();
        }
        return;
      }
      if (!isBaseActivationCandidate(candidate)) {
        consecutive_frame_num_ = 0;
        nonplane_evidence_frame_num_ = 0;
        first_confirmable_time_.reset();
        publishState();
        return;
      }
      if (hasCurrentNonplaneEvidence(candidate)) {
        ++nonplane_evidence_frame_num_;
      }
      if (!isActivationCandidate(candidate)) {
        publishState();
        return;
      }
      if (consecutive_frame_num_ == 0) {
        first_confirmable_time_ = now_time;
      }
      ++consecutive_frame_num_;
      const double duration_sec = first_confirmable_time_ ?
        std::chrono::duration<double>(now_time - *first_confirmable_time_).count() : 0.0;
      if (consecutive_frame_num_ >= min_confirmed_frame_num_ &&
        duration_sec >= std::max(confirmation_time_sec_, min_confirm_duration_sec_))
      {
        is_confirmed_ = true;
        last_supported_time_ = now_time;
        RCLCPP_INFO(
          get_logger(), "物体テンプレート照合確定: template=%s score=%.3f",
          template_id_.c_str(), candidate.value("score", 0.0));
      }
      publishState();
    } catch (const json::exception &error) {
      RCLCPP_WARN(get_logger(), "物体テンプレート照合候補JSONの解析失敗: %s", error.what());
    }
  }

  void onStateTimer()
  {
    if (is_confirmed_ && last_supported_time_) {
      const double lost_duration_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *last_supported_time_).count();
      if (lost_duration_sec > max_lost_duration_sec_) {
        is_confirmed_ = false;
        consecutive_frame_num_ = 0;
        nonplane_evidence_frame_num_ = 0;
        first_confirmable_time_.reset();
        RCLCPP_INFO(get_logger(), "物体テンプレート照合解除: template=%s", template_id_.c_str());
      }
    }
    publishState();
  }

  void publishState()
  {
    json state = {
      {"template_id", template_id_},
      {"state", is_confirmed_ ? "confirmed" : "pending"},
      {"confirmed", is_confirmed_},
      {"consecutive_frame_num", consecutive_frame_num_},
      {"nonplane_evidence_frame_num", nonplane_evidence_frame_num_},
      {"enable_nonplane_component_evaluation", enable_nonplane_component_evaluation_},
      {"min_nonplane_evidence_frame_num", min_nonplane_evidence_frame_num_},
      {"min_nonplane_evidence_score_th", min_nonplane_evidence_score_th_},
      {"min_visible_ratio", min_visible_ratio_},
      {"recognition_threshold", recognition_threshold_},
      {"confirmation_time_sec", confirmation_time_sec_}};
    if (!last_candidate_.is_null()) {
      state["candidate"] = last_candidate_;
    }
    std_msgs::msg::String message;
    message.data = state.dump();
    state_publisher_->publish(message);
  }

  std::string template_id_;
  std::string candidate_topic_;
  std::string state_topic_;
  double min_visible_ratio_ = 0.20;
  double recognition_threshold_ = 0.55;
  double min_template_score_margin_ = 0.10;
  double confirmation_time_sec_ = 0.5;
  double activate_score_th_ = 0.0;
  double deactivate_score_th_ = 0.0;
  double min_matched_node_ratio_ = 0.0;
  double min_matched_edge_ratio_ = 0.0;
  bool enable_plane_cluster_evidence_ = true;
  double min_plane_support_score_ = 1.0;
  double max_missing_node_ratio_ = 1.0;
  double max_contradiction_point_ratio_ = 1.0;
  bool enable_nonplane_component_evaluation_ = true;
  int min_nonplane_evidence_frame_num_ = 1;
  double min_nonplane_evidence_score_th_ = 1.0;
  int min_confirmed_frame_num_ = 1;
  double min_confirm_duration_sec_ = 0.0;
  double max_lost_duration_sec_ = 0.0;
  double state_publish_hz_ = 1.0;
  bool is_confirmed_ = false;
  int consecutive_frame_num_ = 0;
  int nonplane_evidence_frame_num_ = 0;
  json last_candidate_;
  std::optional<std::chrono::steady_clock::time_point> first_confirmable_time_;
  std::optional<std::chrono::steady_clock::time_point> last_supported_time_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr candidate_subscription_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_sim::bridge::ObjectTemplateMatchValidatorNode>(rclcpp::NodeOptions()));
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_template_match_validator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
