#include "topo_fuzzy_viewer/services/pointcloud_file_manager.h"
#include "topo_fuzzy_viewer/common/path_utils.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <string>

namespace {

class ViewerFileNode : public rclcpp::Node {
public:
    ViewerFileNode()
        : rclcpp::Node("viewer_file_node") {
        const auto projectRoot = viewer_internal::resolveProjectRootFromExe();
        const std::filesystem::path pointCloudRoot = projectRoot / "data" / "pointcloud";

        fileManager_ = std::make_unique<ros2_bridge::PointCloudFileManager>(pointCloudRoot.string());

        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcFileRequest,
            20,
            std::bind(&ViewerFileNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 20);
        loadedCloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            viewer_internal::topics::kFileLoadedCloud,
            rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

        RCLCPP_INFO(get_logger(), "viewer_file_node initialized. root=%s", pointCloudRoot.string().c_str());
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
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_REQUEST", "Missing id or method")));
            return;
        }

        if (method == "files.list") {
            handleFilesList(id);
            return;
        }

        if (method == "files.load") {
            handleFilesLoad(id, params);
            return;
        }

        rpcResponsePub_->publish(viewer_internal::toStringMsg(
            viewer_internal::makeErrorResponse(id, "METHOD_NOT_FOUND", "Unsupported file method")));
    }

    void handleFilesList(const std::string& id) {
        viewer_internal::json files = viewer_internal::json::array();
        const auto scanned = fileManager_->scanFiles();

        for (const auto& file : scanned) {
            files.push_back({
                {"path", file.path},
                {"name", file.name},
                {"relativePath", file.relativePath},
                {"format", file.format},
                {"fileSize", file.fileSize}
            });
        }

        viewer_internal::json result;
        result["files"] = files;
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void handleFilesLoad(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("path") || !params["path"].is_string()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "path is required")));
            return;
        }

        const std::string path = params["path"].get<std::string>();
        if (path.empty()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "path is empty")));
            return;
        }

        auto loaded = fileManager_->loadFile(path);
        if (!loaded.success) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "FILE_LOAD_FAILED", loaded.errorMessage)));
            return;
        }

        utils::PointCloudData cloudData;
        cloudData.positions = loaded.positions;
        cloudData.colors = loaded.colors;
        cloudData.intensities = loaded.intensities;
        cloudData.pointCount = loaded.pointCount;
        cloudData.dataMask = 0;
        if (!cloudData.colors.empty()) {
            cloudData.dataMask |= pcd_protocol::MASK_RGB;
        }
        if (!cloudData.intensities.empty()) {
            cloudData.dataMask |= pcd_protocol::MASK_INTENSITY;
        }

        auto rosCloud = utils::convertToRosMsg(cloudData.positions, cloudData.colors, "map", now());
        loadedCloudPub_->publish(rosCloud);

        viewer_internal::json result;
        result["success"] = true;
        result["pointCount"] = loaded.pointCount;
        result["frameId"] = "map";
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

private:
    std::unique_ptr<ros2_bridge::PointCloudFileManager> fileManager_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loadedCloudPub_;
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerFileNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
