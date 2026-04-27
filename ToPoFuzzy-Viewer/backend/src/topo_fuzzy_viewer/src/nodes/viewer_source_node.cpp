#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

class ViewerSourceNode : public rclcpp::Node {
public:
    ViewerSourceNode()
        : rclcpp::Node("viewer_source_node") {
        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcSourceRequest,
            50,
            std::bind(&ViewerSourceNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 50);

        streamPointCloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            viewer_internal::topics::kStreamPointCloud,
            rclcpp::QoS(rclcpp::KeepLast(10)));

        streamPointCloudMetaPub_ = create_publisher<std_msgs::msg::String>(
            viewer_internal::topics::kStreamPointCloudMeta,
            rclcpp::QoS(rclcpp::KeepLast(50)));

        streamGraphPub_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
            viewer_internal::topics::kStreamGraph,
            rclcpp::QoS(rclcpp::KeepLast(10)));

        loadedCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            viewer_internal::topics::kFileLoadedCloud,
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&ViewerSourceNode::handleLoadedCloud, this, std::placeholders::_1));

        gngSourceSub_ = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
            "/topological_map",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            std::bind(&ViewerSourceNode::handleGraph, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "viewer_source_node initialized");
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

        if (method == "sources.list") {
            handleSourcesList(id);
            return;
        }

        if (method == "sources.setActive") {
            handleSourcesSetActive(id, params);
            return;
        }

        if (method == "publish.startContinuous") {
            handleStartContinuous(id, params);
            return;
        }

        if (method == "publish.stopContinuous") {
            handleStopContinuous(id);
            return;
        }

        if (method == "publish.status") {
            handleContinuousStatus(id);
            return;
        }

        rpcResponsePub_->publish(viewer_internal::toStringMsg(
            viewer_internal::makeErrorResponse(id, "METHOD_NOT_FOUND", "Unsupported source method")));
    }

    void handleSourcesList(const std::string& id) {
        viewer_internal::json sources = viewer_internal::json::array();

        const auto topicNamesAndTypes = get_topic_names_and_types();

        std::lock_guard<std::mutex> lock(subscriptionMutex_);

        for (const auto& entry : topicNamesAndTypes) {
            const std::string& topic = entry.first;
            if (viewer_internal::startsWith(topic, "/viewer/internal/")) {
                continue;
            }

            const auto& types = entry.second;
            const bool isPointCloud = std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end();
            if (!isPointCloud) {
                continue;
            }

            const bool active = activePointCloudSubs_.find(topic) != activePointCloudSubs_.end();
            sources.push_back({
                {"id", topic},
                {"name", topic},
                {"type", "pointcloud"},
                {"active", active}
            });
        }

        viewer_internal::json result;
        result["sources"] = sources;
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void handleSourcesSetActive(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sourceId") || !params["sourceId"].is_string() ||
            !params.contains("active") || !params["active"].is_boolean()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "sourceId(string) and active(bool) are required")));
            return;
        }

        std::string sourceId = params["sourceId"].get<std::string>();
        bool active = params["active"].get<bool>();
        if (sourceId.empty()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "sourceId is empty")));
            return;
        }

        if (sourceId.front() != '/') {
            sourceId = "/" + sourceId;
        }

        bool success = true;
        {
            std::lock_guard<std::mutex> lock(subscriptionMutex_);

            if (active) {
                if (activePointCloudSubs_.find(sourceId) == activePointCloudSubs_.end()) {
                    // Use transient_local QoS for edited topics to receive latched messages
                    const bool isEdited = sourceId.size() >= 7 &&
                        sourceId.compare(sourceId.size() - 7, 7, "/edited") == 0;
                    rclcpp::QoS qos(rclcpp::KeepLast(10));
                    if (isEdited) {
                        qos.transient_local();
                    }

                    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
                        sourceId,
                        qos,
                        [this, sourceId](const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
                            this->forwardPointCloud(sourceId, cloudMsg);
                        });
                    activePointCloudSubs_[sourceId] = sub;
                }
            } else {
                activePointCloudSubs_.erase(sourceId);
            }
        }

        viewer_internal::json result;
        result["success"] = success;
        result["sourceId"] = sourceId;
        result["active"] = active;
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void handleStartContinuous(const std::string& id, const viewer_internal::json& params) {
        std::string topic = "/offline_pointcloud";
        double rateHz = 10.0;

        if (params.contains("topic") && params["topic"].is_string()) {
            topic = params["topic"].get<std::string>();
        }
        if (params.contains("rateHz") && (params["rateHz"].is_number_float() || params["rateHz"].is_number_integer())) {
            rateHz = params["rateHz"].get<double>();
        }

        if (topic.empty()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "topic is empty")));
            return;
        }
        if (rateHz <= 0.0 || rateHz > 100.0) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "rateHz must be in (0,100]")));
            return;
        }

        std::lock_guard<std::mutex> dataLock(storedCloudMutex_);
        if (storedPositions_.empty()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "NO_CLOUD", "No loaded cloud available. Run files.load first.")));
            return;
        }

        if (topic.front() != '/') {
            topic = "/" + topic;
        }

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        continuousPublisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
        continuousTopic_ = topic;
        continuousRateHz_ = rateHz;

        if (continuousTimer_) {
            continuousTimer_->cancel();
            continuousTimer_.reset();
        }

        auto period = std::chrono::duration<double>(1.0 / continuousRateHz_);
        continuousTimer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ViewerSourceNode::publishContinuousTick, this));

        viewer_internal::json result;
        result["success"] = true;
        result["topic"] = continuousTopic_;
        result["rateHz"] = continuousRateHz_;
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void handleStopContinuous(const std::string& id) {
        if (continuousTimer_) {
            continuousTimer_->cancel();
            continuousTimer_.reset();
        }
        continuousPublisher_.reset();
        continuousTopic_.clear();
        continuousRateHz_ = 0.0;

        viewer_internal::json result;
        result["success"] = true;
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void handleContinuousStatus(const std::string& id) {
        viewer_internal::json result;
        result["isPublishing"] = static_cast<bool>(continuousTimer_);
        result["topic"] = continuousTopic_;
        result["rateHz"] = continuousRateHz_;
        {
            std::lock_guard<std::mutex> lock(storedCloudMutex_);
            result["pointCount"] = storedPositions_.size() / 3;
        }
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void forwardPointCloud(const std::string& topic, const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
        viewer_internal::json meta;
        meta["type"] = "stream.pointcloud.meta";
        meta["topic"] = topic;
        streamPointCloudMetaPub_->publish(viewer_internal::toStringMsg(meta));
        streamPointCloudPub_->publish(*cloudMsg);
    }

    void handleGraph(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
        streamGraphPub_->publish(*msg);
    }

    void handleLoadedCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto cloud = utils::convertFromRosMsg(msg);
        std::lock_guard<std::mutex> lock(storedCloudMutex_);
        storedPositions_ = std::move(cloud.positions);
        storedColors_ = std::move(cloud.colors);
    }

    void publishContinuousTick() {
        if (!continuousPublisher_) {
            return;
        }

        std::vector<float> positions;
        std::vector<uint8_t> colors;
        {
            std::lock_guard<std::mutex> lock(storedCloudMutex_);
            positions = storedPositions_;
            colors = storedColors_;
        }

        if (positions.empty()) {
            return;
        }

        auto msg = utils::convertToRosMsg(positions, colors, "map", now());
        continuousPublisher_->publish(msg);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr streamPointCloudPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr streamPointCloudMetaPub_;
    rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr streamGraphPub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr loadedCloudSub_;
    rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr gngSourceSub_;

    std::mutex subscriptionMutex_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> activePointCloudSubs_;

    std::mutex storedCloudMutex_;
    std::vector<float> storedPositions_;
    std::vector<uint8_t> storedColors_;

    rclcpp::TimerBase::SharedPtr continuousTimer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr continuousPublisher_;
    std::string continuousTopic_;
    double continuousRateHz_ = 0.0;
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerSourceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
