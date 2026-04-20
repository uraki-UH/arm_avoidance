#include "topo_fuzzy_viewer/common/path_utils.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/services/rosbag_manager.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace {

class ViewerRosbagNode : public rclcpp::Node {
public:
    ViewerRosbagNode()
        : rclcpp::Node("viewer_rosbag_node") {
        const auto projectRoot = viewer_internal::resolveProjectRootFromExe();
        const std::filesystem::path rosbagRoot = projectRoot / "data" / "rosbag";

        rosbagManager_ = std::make_unique<ros2_bridge::RosbagManager>(rosbagRoot.string());

        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcRosbagRequest,
            20,
            std::bind(&ViewerRosbagNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 20);

        RCLCPP_INFO(get_logger(), "viewer_rosbag_node initialized. root=%s", rosbagRoot.string().c_str());
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

        if (method == "rosbag.list") {
            handleRosbagList(id);
            return;
        }
        if (method == "rosbag.play") {
            handleRosbagPlay(id, params);
            return;
        }
        if (method == "rosbag.stop") {
            handleRosbagStop(id);
            return;
        }
        if (method == "rosbag.status") {
            handleRosbagStatus(id);
            return;
        }

        publishError(id, "METHOD_NOT_FOUND", "Unsupported rosbag method");
    }

    void handleRosbagList(const std::string& id) {
        viewer_internal::json list = viewer_internal::json::array();
        const auto bags = rosbagManager_->scanBags();
        for (const auto& bag : bags) {
            list.push_back({
                {"path", bag.path},
                {"name", bag.name},
                {"relativePath", bag.relativePath}
            });
        }

        viewer_internal::json result;
        result["bags"] = list;
        publishOk(id, result);
    }

    void handleRosbagPlay(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("path") || !params["path"].is_string()) {
            publishError(id, "INVALID_PARAMS", "path is required");
            return;
        }

        const std::string path = params["path"].get<std::string>();
        bool loop = params.contains("loop") && params["loop"].is_boolean() ? params["loop"].get<bool>() : false;

        std::vector<std::string> remaps;
        if (params.contains("remaps") && params["remaps"].is_array()) {
            for (const auto& item : params["remaps"]) {
                if (item.is_string()) {
                    remaps.push_back(item.get<std::string>());
                }
            }
        }

        const bool success = rosbagManager_->playBag(path, remaps, loop);
        viewer_internal::json result;
        result["success"] = success;
        publishOk(id, result);
    }

    void handleRosbagStop(const std::string& id) {
        const bool success = rosbagManager_->stopBag();
        viewer_internal::json result;
        result["success"] = success;
        publishOk(id, result);
    }

    void handleRosbagStatus(const std::string& id) {
        const auto status = rosbagManager_->getStatus();
        viewer_internal::json result;
        result["isPlaying"] = status.isPlaying;
        result["currentBag"] = status.currentBag;
        result["pid"] = status.pid;
        publishOk(id, result);
    }

    void publishOk(const std::string& id, const viewer_internal::json& result = viewer_internal::json::object()) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void publishError(const std::string& id, const std::string& code, const std::string& message) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeErrorResponse(id, code, message)));
    }

private:
    std::unique_ptr<ros2_bridge::RosbagManager> rosbagManager_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerRosbagNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
