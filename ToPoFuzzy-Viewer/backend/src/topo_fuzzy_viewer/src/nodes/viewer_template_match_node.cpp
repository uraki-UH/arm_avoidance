#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

enum class ParameterKind
{
  Boolean,
  Integer,
  Double,
};

struct ParameterSpec
{
  const char *name;
  ParameterKind kind;
};

const std::vector<ParameterSpec> kMatcherParameters{
  {"enable_yaw_search", ParameterKind::Boolean},
  {"yaw_min_deg", ParameterKind::Double},
  {"yaw_max_deg", ParameterKind::Double},
  {"yaw_step_deg", ParameterKind::Double},
  {"enable_roll_pitch_search", ParameterKind::Boolean},
  {"max_orientation_hypothesis_num", ParameterKind::Integer},
  {"max_normal_angle_full_deg", ParameterKind::Double},
  {"max_normal_angle_partial_deg", ParameterKind::Double},
  {"enable_rho_evaluation", ParameterKind::Boolean},
  {"max_rho_dev_full_ratio", ParameterKind::Double},
  {"max_rho_dev_partial_ratio", ParameterKind::Double},
  {"max_degree_dev_full", ParameterKind::Double},
  {"max_degree_dev_partial", ParameterKind::Double},
  {"normal_weight", ParameterKind::Double},
  {"rho_weight", ParameterKind::Double},
  {"degree_weight", ParameterKind::Double},
  {"min_node_score", ParameterKind::Double},
  {"enable_plane_cluster_evaluation", ParameterKind::Boolean},
  {"max_plane_normal_angle_deg", ParameterKind::Double},
  {"min_plane_extent_allow_ratio", ParameterKind::Double},
  {"min_plane_extent_full_match_ratio", ParameterKind::Double},
  {"max_plane_extent_full_match_ratio", ParameterKind::Double},
  {"max_plane_extent_overflow_ratio", ParameterKind::Double},
  {"plane_weight", ParameterKind::Double},
  {"plane_support_score_scale", ParameterKind::Double},
  {"min_plane_support_score", ParameterKind::Double},
  {"enable_oversized_plane_filter", ParameterKind::Boolean},
  {"edge_weight", ParameterKind::Double},
  {"enable_scale_evaluation", ParameterKind::Boolean},
  {"min_scale_allow_ratio", ParameterKind::Double},
  {"min_scale_full_match_ratio", ParameterKind::Double},
  {"max_scale_full_match_ratio", ParameterKind::Double},
  {"max_scale_allow_ratio", ParameterKind::Double},
  {"scale_weight", ParameterKind::Double},
  {"min_scale_edge_num", ParameterKind::Integer},
  {"enable_contradiction_evaluation", ParameterKind::Boolean},
  {"contradiction_weight", ParameterKind::Double},
  {"max_contradiction_point_ratio", ParameterKind::Double},
};

const std::vector<ParameterSpec> kValidatorParameters{
  {"activate_score_th", ParameterKind::Double},
  {"deactivate_score_th", ParameterKind::Double},
  {"min_matched_node_ratio", ParameterKind::Double},
  {"min_matched_edge_ratio", ParameterKind::Double},
  {"enable_plane_cluster_evidence", ParameterKind::Boolean},
  {"min_plane_support_score", ParameterKind::Double},
  {"max_missing_node_ratio", ParameterKind::Double},
  {"max_contradiction_point_ratio", ParameterKind::Double},
  {"min_confirmed_frame_num", ParameterKind::Integer},
  {"min_confirm_duration_sec", ParameterKind::Double},
  {"max_lost_duration_sec", ParameterKind::Double},
};

bool isValidNodeName(const std::string &name)
{
  return !name.empty() && name.front() == '/' &&
         std::all_of(name.begin(), name.end(), [](const char value) {
           return value == '/' || value == '_' || std::isalnum(static_cast<unsigned char>(value));
         });
}

std::unordered_map<std::string, ParameterKind> makeSpecMap(
  const std::vector<ParameterSpec> &specs)
{
  std::unordered_map<std::string, ParameterKind> result;
  result.reserve(specs.size());
  for (const auto &spec : specs) {
    result.emplace(spec.name, spec.kind);
  }
  return result;
}

class ViewerTemplateMatchNode : public rclcpp::Node
{
public:
  ViewerTemplateMatchNode()
  : Node("viewer_template_match_node")
  {
    rpc_request_sub_ = create_subscription<std_msgs::msg::String>(
      viewer_internal::topics::kRpcRequest, 20,
      std::bind(&ViewerTemplateMatchNode::handleRequest, this, std::placeholders::_1));
    rpc_response_pub_ = create_publisher<std_msgs::msg::String>(
      viewer_internal::topics::kRpcResponse, 20);
    RCLCPP_INFO(get_logger(), "viewer_template_match_node initialized");
  }

private:
  using Json = viewer_internal::json;

  std::shared_ptr<rclcpp::SyncParametersClient> makeClient(const std::string &target_node)
  {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    auto client_node = std::make_shared<rclcpp::Node>(
      "template_match_param_client_" + std::to_string(++client_serial_), options);
    client_nodes_.push_back(client_node);
    if (client_nodes_.size() > 4U) {
      client_nodes_.erase(client_nodes_.begin());
    }
    return std::make_shared<rclcpp::SyncParametersClient>(client_node, target_node);
  }

  Json readValues(
    const std::string &target_node,
    const std::vector<ParameterSpec> &specs)
  {
    auto client = makeClient(target_node);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      throw std::runtime_error("Parameter service unavailable: " + target_node);
    }
    std::vector<std::string> names;
    names.reserve(specs.size());
    for (const auto &spec : specs) {
      names.emplace_back(spec.name);
    }
    const auto parameters = client->get_parameters(names);
    Json result = Json::object();
    for (std::size_t index = 0; index < parameters.size() && index < specs.size(); ++index) {
      const auto &parameter = parameters[index];
      switch (specs[index].kind) {
        case ParameterKind::Boolean:
          result[specs[index].name] = parameter.as_bool();
          break;
        case ParameterKind::Integer:
          result[specs[index].name] = parameter.as_int();
          break;
        case ParameterKind::Double:
          result[specs[index].name] = parameter.as_double();
          break;
      }
    }
    return result;
  }

  std::vector<rclcpp::Parameter> parseUpdates(
    const Json &values,
    const std::vector<ParameterSpec> &specs) const
  {
    if (!values.is_object()) {
      throw std::runtime_error("Parameter values must be an object");
    }
    const auto spec_map = makeSpecMap(specs);
    std::vector<rclcpp::Parameter> updates;
    updates.reserve(values.size());
    for (auto iterator = values.begin(); iterator != values.end(); ++iterator) {
      const auto spec = spec_map.find(iterator.key());
      if (spec == spec_map.end()) {
        throw std::runtime_error("Unsupported parameter: " + iterator.key());
      }
      if (spec->second == ParameterKind::Boolean) {
        if (!iterator.value().is_boolean()) {
          throw std::runtime_error("Boolean parameter required: " + iterator.key());
        }
        updates.emplace_back(iterator.key(), iterator.value().get<bool>());
      } else if (spec->second == ParameterKind::Integer) {
        if (!iterator.value().is_number_integer()) {
          throw std::runtime_error("Integer parameter required: " + iterator.key());
        }
        updates.emplace_back(iterator.key(), iterator.value().get<std::int64_t>());
      } else {
        if (!iterator.value().is_number()) {
          throw std::runtime_error("Numeric parameter required: " + iterator.key());
        }
        const double value = iterator.value().get<double>();
        if (!std::isfinite(value)) {
          throw std::runtime_error("Finite parameter required: " + iterator.key());
        }
        updates.emplace_back(iterator.key(), value);
      }
    }
    return updates;
  }

  rcl_interfaces::msg::SetParametersResult applyValues(
    const std::string &target_node,
    const Json &values,
    const std::vector<ParameterSpec> &specs)
  {
    auto client = makeClient(target_node);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Parameter service unavailable: " + target_node;
      return result;
    }
    return client->set_parameters_atomically(parseUpdates(values, specs));
  }

  static std::string getNodeName(
    const Json &params, const char *key, const char *default_name)
  {
    const std::string value = params.value(key, default_name);
    if (!isValidNodeName(value)) {
      throw std::runtime_error(std::string("Invalid node name: ") + value);
    }
    return value;
  }

  void handleRequest(const std_msgs::msg::String::SharedPtr message)
  {
    Json request;
    if (!viewer_internal::parseJson(message->data, request)) {
      return;
    }
    const std::string method = viewer_internal::getMethod(request);
    if (method != "templateMatch.getConfig" && method != "templateMatch.applyConfig") {
      return;
    }
    const std::string id = viewer_internal::getId(request);
    try {
      const Json &params = viewer_internal::getParams(request);
      const std::string matcher_node = getNodeName(
        params, "matcherNode", "/object_template_matcher_node");
      const std::string validator_node = getNodeName(
        params, "validatorNode", "/object_template_match_validator_node");

      if (method == "templateMatch.applyConfig") {
        const Json previous_matcher = readValues(matcher_node, kMatcherParameters);
        const auto matcher_result = applyValues(
          matcher_node, params.at("matcher"), kMatcherParameters);
        if (!matcher_result.successful) {
          publishError(id, "MATCHER_REJECTED", matcher_result.reason);
          return;
        }
        const auto validator_result = applyValues(
          validator_node, params.at("validator"), kValidatorParameters);
        if (!validator_result.successful) {
          const auto rollback_result = applyValues(
            matcher_node, previous_matcher, kMatcherParameters);
          std::string reason = validator_result.reason;
          if (!rollback_result.successful) {
            reason += "; matcher rollback failed: " + rollback_result.reason;
          }
          publishError(id, "VALIDATOR_REJECTED", reason);
          return;
        }
      }

      Json result{
        {"success", true},
        {"matcherNode", matcher_node},
        {"validatorNode", validator_node},
        {"matcher", readValues(matcher_node, kMatcherParameters)},
        {"validator", readValues(validator_node, kValidatorParameters)},
      };
      rpc_response_pub_->publish(viewer_internal::toStringMsg(
        viewer_internal::makeOkResponse(id, result)));
    } catch (const std::exception &error) {
      publishError(id, "TEMPLATE_MATCH_CONFIG_ERROR", error.what());
    }
  }

  void publishError(
    const std::string &id,
    const std::string &code,
    const std::string &message)
  {
    rpc_response_pub_->publish(viewer_internal::toStringMsg(
      viewer_internal::makeErrorResponse(id, code, message)));
  }

  std::uint64_t client_serial_ = 0U;
  std::vector<rclcpp::Node::SharedPtr> client_nodes_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpc_request_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpc_response_pub_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViewerTemplateMatchNode>());
  rclcpp::shutdown();
  return 0;
}
