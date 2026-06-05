#include "topo_fuzzy_viewer/services/ros2_parameter_manager.h"
#include <rclcpp/rclcpp.hpp>

namespace {

constexpr const char* kUiInputTopicParam = "input.topic_name";
constexpr const char* kRosInputTopicParam = "input.topic_names";
constexpr const char* kDefaultInputTopic = "/topo_points";

std::string resolveQueryParamName(const std::string& param_name) {
    if (param_name == kUiInputTopicParam) {
        return kRosInputTopicParam;
    }
    return param_name;
}

std::string resolveDisplayInputTopic(const rclcpp::Parameter& param) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
        const auto topics = param.as_string_array();
        if (!topics.empty()) {
            return topics.front();
        }
        return kDefaultInputTopic;
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return param.as_string();
    }

    return kDefaultInputTopic;
}

} // namespace

namespace ros2_bridge {

ROS2ParameterManager::ROS2ParameterManager(const std::string& target_node)
    : target_node_(target_node) {
    
    // 専用の軽量ノードを作成（executor競合を回避）
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    param_node_ = std::make_shared<rclcpp::Node>("param_client_node", options);
    
    // 対象ノードのパラメータクライアントを作成
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(param_node_, target_node_);
    
    // 管理するパラメータ定義（ハードコード - 要件に基づく）
    // 注意: これらのパラメータはint型
    managed_params_ = {
        {"ds.all.num_max", "All Clusters", core::ParamType::INT, 10.0, 10000.0, 100.0, 1000.0},
        {"ds.unknown.num_max", "Unknown Objects", core::ParamType::INT, 10.0, 10000.0, 100.0, 500.0},
        {"ds.human.num_max", "Human Detection", core::ParamType::INT, 10.0, 1000.0, 10.0, 100.0},
        {kUiInputTopicParam, "Input Topic", core::ParamType::STRING, 0.0, 0.0, 0.0, kDefaultInputTopic}
    };
    
    RCLCPP_INFO(param_node_->get_logger(), "ROS2ParameterManager initialized for node: %s", target_node_.c_str());
}

std::optional<core::NodeParameters> ROS2ParameterManager::getNodeParameters() {
    if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(param_node_->get_logger(), "Parameter service not available for %s", target_node_.c_str());
        // Return static definitions with default values
        core::NodeParameters result;
        result.node_name = target_node_;
        result.parameters = managed_params_;
        return result;
    }

    // Fetch current values from the ROS node
    core::NodeParameters result;
    result.node_name = target_node_;
    
    std::vector<std::string> param_names;
    for (const auto& p : managed_params_) {
        param_names.push_back(resolveQueryParamName(p.name));
    }
    
    try {
        auto params = param_client_->get_parameters(param_names);
        
        for (size_t i = 0; i < managed_params_.size() && i < params.size(); ++i) {
            core::ParameterInfo info = managed_params_[i];
            const auto& param = params[i];
            
            if (managed_params_[i].name == kUiInputTopicParam) {
                info.value = resolveDisplayInputTopic(param);
            } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                info.value = param.as_double();
            } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                info.value = static_cast<double>(param.as_int());
            } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                info.value = param.as_string();
            } else {
                if (managed_params_[i].type == core::ParamType::STRING) {
                    info.value = std::get<std::string>(managed_params_[i].value);
                } else {
                    info.value = std::get<double>(managed_params_[i].value);
                }
            }
            result.parameters.push_back(info);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(param_node_->get_logger(), "Failed to get parameters: %s", e.what());
        result.parameters = managed_params_;
    }
    
    return result;
}

bool ROS2ParameterManager::setParameter(const std::string& param_name, double value) {
    if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(param_node_->get_logger(), "Parameter service not available for %s", target_node_.c_str());
        return false;
    }

    try {
        // Determine if this should be int or double based on the managed params
        bool isInt = false;
        for (const auto& p : managed_params_) {
            if (p.name == param_name && p.type == core::ParamType::INT) {
                isInt = true;
                break;
            }
        }

        std::vector<rclcpp::Parameter> params;
        if (isInt) {
            params.push_back(rclcpp::Parameter(param_name, static_cast<int64_t>(value)));
        } else {
            params.push_back(rclcpp::Parameter(param_name, value));
        }
        
        auto results = param_client_->set_parameters(params);
        
        if (!results.empty() && results[0].successful) {
            RCLCPP_INFO(param_node_->get_logger(), "Set %s = %f", param_name.c_str(), value);
            return true;
        } else {
            std::string reason = results.empty() ? "unknown" : results[0].reason;
            RCLCPP_WARN(param_node_->get_logger(), "Failed to set %s: %s", param_name.c_str(), reason.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(param_node_->get_logger(), "Exception setting parameter: %s", e.what());
        return false;
    }
}

bool ROS2ParameterManager::setParameter(const std::string& param_name, const std::string& value) {
    if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(param_node_->get_logger(), "Parameter service not available for %s", target_node_.c_str());
        return false;
    }

    try {
        const std::string actual_param_name = resolveQueryParamName(param_name);

        if (actual_param_name == kRosInputTopicParam) {
            std::vector<rclcpp::Parameter> params;
            params.emplace_back(actual_param_name, std::vector<std::string>{value});

            auto results = param_client_->set_parameters(params);

            if (!results.empty() && results[0].successful) {
                RCLCPP_INFO(param_node_->get_logger(), "Set %s = %s", param_name.c_str(), value.c_str());
                return true;
            }

            std::string reason = results.empty() ? "unknown" : results[0].reason;
            RCLCPP_WARN(param_node_->get_logger(), "Failed to set %s: %s", param_name.c_str(), reason.c_str());
            return false;
        }

        std::vector<rclcpp::Parameter> params;
        params.push_back(rclcpp::Parameter(actual_param_name, value));
        
        auto results = param_client_->set_parameters(params);
        
        if (!results.empty() && results[0].successful) {
            RCLCPP_INFO(param_node_->get_logger(), "Set %s = %s", param_name.c_str(), value.c_str());
            return true;
        } else {
            std::string reason = results.empty() ? "unknown" : results[0].reason;
            RCLCPP_WARN(param_node_->get_logger(), "Failed to set %s: %s", param_name.c_str(), reason.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(param_node_->get_logger(), "Exception setting parameter: %s", e.what());
        return false;
    }
}

} // namespace ros2_bridge
