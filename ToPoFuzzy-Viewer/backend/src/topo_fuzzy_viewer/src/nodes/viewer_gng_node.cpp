#include "topo_fuzzy_viewer/common/path_utils.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/services/gng_process_manager.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace {

class ViewerGngNode : public rclcpp::Node {
public:
    ViewerGngNode()
        : rclcpp::Node("viewer_gng_node") {
        gngProcessManager_ = std::make_unique<ros2_bridge::GngProcessManager>();

        const auto projectRoot = viewer_internal::resolveProjectRootFromExe();
        const std::filesystem::path configDir = projectRoot / "config" / "gng";
        gngProcessManager_->setProjectRoot(projectRoot.string());
        if (std::filesystem::exists(configDir)) {
            gngProcessManager_->setConfigDir(configDir.string());
        }

        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcGngRequest,
            20,
            std::bind(&ViewerGngNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 20);

        RCLCPP_INFO(get_logger(), "viewer_gng_node initialized");
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

        if (method == "gng.listConfigs") {
            handleListConfigs(id);
            return;
        }
        if (method == "gng.start") {
            handleStart(id, params);
            return;
        }
        if (method == "gng.stop") {
            handleStop(id);
            return;
        }
        if (method == "gng.status") {
            handleStatus(id);
            return;
        }

        publishError(id, "METHOD_NOT_FOUND", "Unsupported gng method");
    }

    void handleListConfigs(const std::string& id) {
        viewer_internal::json configs = viewer_internal::json::array();
        const auto list = gngProcessManager_->listConfigFiles();
        for (const auto& cfg : list) {
            configs.push_back({
                {"name", cfg.name},
                {"path", cfg.path}
            });
        }

        viewer_internal::json result;
        result["configs"] = configs;
        publishOk(id, result);
    }

    void handleStart(const std::string& id, const viewer_internal::json& params) {
        ros2_bridge::GngProcessManager::GngParams gngParams;

        if (params.contains("inputTopic") && params["inputTopic"].is_string()) {
            gngParams.inputTopic = params["inputTopic"].get<std::string>();
        }
        if (params.contains("configFile") && params["configFile"].is_string()) {
            gngParams.configFile = params["configFile"].get<std::string>();
        }
        if (params.contains("maxNodes") && params["maxNodes"].is_number_integer()) {
            gngParams.maxNodes = params["maxNodes"].get<int>();
        }
        if (params.contains("learningNum") && params["learningNum"].is_number_integer()) {
            gngParams.learningNum = params["learningNum"].get<int>();
        }
        if (params.contains("voxelGridUnit") &&
            (params["voxelGridUnit"].is_number_float() || params["voxelGridUnit"].is_number_integer())) {
            gngParams.voxelGridUnit = params["voxelGridUnit"].get<double>();
        }

        const bool success = gngProcessManager_->startGng(gngParams);
        const auto status = gngProcessManager_->getStatus();

        viewer_internal::json result;
        result["success"] = success;
        result["pid"] = status.pid;
        result["inputTopic"] = status.inputTopic;
        publishOk(id, result);
    }

    void handleStop(const std::string& id) {
        const bool success = gngProcessManager_->stopGng();
        viewer_internal::json result;
        result["success"] = success;
        publishOk(id, result);
    }

    void handleStatus(const std::string& id) {
        const auto status = gngProcessManager_->getStatus();
        viewer_internal::json result;
        result["isRunning"] = status.isRunning;
        result["pid"] = status.pid;
        result["inputTopic"] = status.inputTopic;
        publishOk(id, result);
    }

    void publishOk(const std::string& id, const viewer_internal::json& result = viewer_internal::json::object()) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void publishError(const std::string& id, const std::string& code, const std::string& message) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeErrorResponse(id, code, message)));
    }

private:
    std::unique_ptr<ros2_bridge::GngProcessManager> gngProcessManager_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerGngNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
