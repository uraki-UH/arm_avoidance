#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/services/ros2_parameter_manager.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace {

class ViewerParamNode : public rclcpp::Node {
public:
    ViewerParamNode()
        : rclcpp::Node("viewer_param_node") {

        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcRequest,
            20,
            std::bind(&ViewerParamNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 20);
        parameterManager_ = std::make_unique<ros2_bridge::ROS2ParameterManager>(
            declare_parameter<std::string>("parameter_target_node", "/ais_gng"));
        calibration_request_pub_ = create_publisher<std_msgs::msg::String>("/sensor_calibration/request", 20);
        calibration_response_sub_ = create_subscription<std_msgs::msg::String>(
            "/sensor_calibration/response", 20,
            [this](std_msgs::msg::String::ConstSharedPtr message) {
                viewer_internal::json response;
                if (viewer_internal::parseJson(message->data, response) &&
                    calibration_pending_.erase(viewer_internal::getId(response))) {
                    rpcResponsePub_->publish(*message);
                }
            });
        calibration_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            const auto now = std::chrono::steady_clock::now();
            for (auto it = calibration_pending_.begin(); it != calibration_pending_.end();) {
                if (now - it->second > std::chrono::seconds(10)) {
                    publishError(it->first, "CALIBRATION_TIMEOUT", "校正管理ノードの応答待ちが期限を超えました");
                    it = calibration_pending_.erase(it);
                } else {
                    ++it;
                }
            }
        });

        RCLCPP_INFO(get_logger(), "viewer_param_node initialized");
    }

private:
    void handleRpcRequest(const std_msgs::msg::String::SharedPtr msg) {
        viewer_internal::json request;
        if (!viewer_internal::parseJson(msg->data, request)) {
            return;
        }

        const std::string id = viewer_internal::getId(request);
        const std::string method = viewer_internal::getMethod(request);
        const auto& params = viewer_internal::getParams(request);

        if (id.empty() || method.empty()) {
            publishError(id, "INVALID_REQUEST", "Missing id or method");
            return;
        }

        if (method.rfind("calibration.", 0) == 0) {
            if (calibration_request_pub_->get_subscription_count() == 0) {
                publishError(id, "CALIBRATION_UNAVAILABLE", "ROS側の校正管理ノードが未起動です");
                return;
            }
            calibration_pending_[id] = std::chrono::steady_clock::now();
            calibration_request_pub_->publish(*msg);
            return;
        }
        if (method.rfind("params.", 0) != 0) return;
        if (method == "params.get") {
            handleGet(id);
            return;
        }
        if (method == "params.set") {
            handleSet(id, params);
            return;
        }

        publishError(id, "METHOD_NOT_FOUND", "Unsupported params method");
    }

    void handleGet(const std::string& id) {
        auto paramsOpt = parameterManager_->getNodeParameters();
        if (!paramsOpt.has_value()) {
            viewer_internal::json result;
            result["nodeName"] = "unknown";
            result["parameters"] = viewer_internal::json::array();
            publishOk(id, result);
            return;
        }

        const auto& nodeParams = paramsOpt.value();
        viewer_internal::json outParams = viewer_internal::json::array();

        for (const auto& p : nodeParams.parameters) {
            viewer_internal::json item;
            item["name"] = p.name;
            item["description"] = p.description;
            item["min"] = p.min;
            item["max"] = p.max;
            item["step"] = p.step;

            switch (p.type) {
                case core::ParamType::BOOL:
                    item["type"] = "bool";
                    break;
                case core::ParamType::INT:
                    item["type"] = "int";
                    break;
                case core::ParamType::FLOAT:
                    item["type"] = "float";
                    break;
                case core::ParamType::STRING:
                    item["type"] = "string";
                    break;
            }

            if (std::holds_alternative<bool>(p.value)) {
                item["value"] = std::get<bool>(p.value);
            } else if (std::holds_alternative<int64_t>(p.value)) {
                item["value"] = std::get<int64_t>(p.value);
            } else if (std::holds_alternative<double>(p.value)) {
                item["value"] = std::get<double>(p.value);
            } else if (std::holds_alternative<std::string>(p.value)) {
                item["value"] = std::get<std::string>(p.value);
            }

            outParams.push_back(item);
        }

        viewer_internal::json result;
        result["nodeName"] = nodeParams.node_name;
        result["parameters"] = outParams;
        publishOk(id, result);
    }

    void handleSet(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("paramName") || !params["paramName"].is_string()) {
            publishError(id, "INVALID_PARAMS", "paramName is required");
            return;
        }

        if (!params.contains("value")) {
            publishError(id, "INVALID_PARAMS", "value is required");
            return;
        }

        const std::string paramName = params["paramName"].get<std::string>();
        const auto& value = params["value"];

        bool success = false;
        if (value.is_string()) {
            success = parameterManager_->setParameter(paramName, value.get<std::string>());
        } else if (value.is_boolean()) {
            success = parameterManager_->setParameter(paramName, value.get<bool>() ? 1.0 : 0.0);
        } else if (value.is_number()) {
            success = parameterManager_->setParameter(paramName, value.get<double>());
        } else {
            publishError(id, "INVALID_PARAMS", "value must be string/boolean/number");
            return;
        }

        viewer_internal::json result;
        result["success"] = success;
        result["paramName"] = paramName;
        result["value"] = value;
        publishOk(id, result);
    }

    void publishOk(const std::string& id, const viewer_internal::json& result = viewer_internal::json::object()) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void publishError(const std::string& id, const std::string& code, const std::string& message) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeErrorResponse(id, code, message)));
    }

private:
    std::unique_ptr<ros2_bridge::ROS2ParameterManager> parameterManager_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calibration_request_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calibration_response_sub_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> calibration_pending_;
    rclcpp::TimerBase::SharedPtr calibration_timer_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerParamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
