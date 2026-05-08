#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr const char* kMarkerType = "marker";

bool isPointCloudTopic(const std::vector<std::string>& types) {
    return std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end();
}

bool isMarkerTopic(const std::vector<std::string>& types) {
    return std::find(types.begin(), types.end(), "visualization_msgs/msg/Marker") != types.end() ||
           std::find(types.begin(), types.end(), "visualization_msgs/msg/MarkerArray") != types.end();
}

std::string markerTypeToString(uint8_t type) {
    switch (type) {
    case visualization_msgs::msg::Marker::ARROW: return "arrow";
    case visualization_msgs::msg::Marker::CUBE: return "cube";
    case visualization_msgs::msg::Marker::SPHERE: return "sphere";
    case visualization_msgs::msg::Marker::CYLINDER: return "cylinder";
    case visualization_msgs::msg::Marker::LINE_STRIP: return "line_strip";
    case visualization_msgs::msg::Marker::LINE_LIST: return "line_list";
    case visualization_msgs::msg::Marker::CUBE_LIST: return "cube_list";
    case visualization_msgs::msg::Marker::SPHERE_LIST: return "sphere_list";
    case visualization_msgs::msg::Marker::POINTS: return "points";
    case visualization_msgs::msg::Marker::TEXT_VIEW_FACING: return "text";
    case visualization_msgs::msg::Marker::MESH_RESOURCE: return "mesh_resource";
    case visualization_msgs::msg::Marker::TRIANGLE_LIST: return "triangle_list";
    default: return "unknown";
    }
}

std::string markerActionToString(uint8_t action) {
    if (action == visualization_msgs::msg::Marker::ADD ||
        action == visualization_msgs::msg::Marker::MODIFY) {
        return "add";
    }
    switch (action) {
    case visualization_msgs::msg::Marker::DELETE: return "delete";
    case visualization_msgs::msg::Marker::DELETEALL: return "deleteall";
    default: return "unknown";
    }
}

viewer_internal::json pointToJson(const geometry_msgs::msg::Point& point) {
    return {point.x, point.y, point.z};
}

viewer_internal::json markerToJson(const visualization_msgs::msg::Marker& marker) {
    viewer_internal::json pose;
    pose["position"] = {
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z
    };
    pose["orientation"] = {
        marker.pose.orientation.x,
        marker.pose.orientation.y,
        marker.pose.orientation.z,
        marker.pose.orientation.w
    };

    viewer_internal::json color;
    color["r"] = marker.color.r;
    color["g"] = marker.color.g;
    color["b"] = marker.color.b;
    color["a"] = marker.color.a;

    viewer_internal::json points = viewer_internal::json::array();
    for (const auto& point : marker.points) {
        points.push_back(pointToJson(point));
    }

    viewer_internal::json colors = viewer_internal::json::array();
    for (const auto& col : marker.colors) {
        colors.push_back({
            {"r", col.r},
            {"g", col.g},
            {"b", col.b},
            {"a", col.a}
        });
    }

    viewer_internal::json out;
    out["ns"] = marker.ns;
    out["id"] = marker.id;
    out["type"] = markerTypeToString(marker.type);
    out["action"] = markerActionToString(marker.action);
    out["frameId"] = marker.header.frame_id;
    out["pose"] = pose;
    out["scale"] = {marker.scale.x, marker.scale.y, marker.scale.z};
    out["color"] = color;
    out["points"] = points;
    if (!colors.empty()) {
        out["colors"] = colors;
    }
    out["text"] = marker.text;
    out["meshResource"] = marker.mesh_resource;
    out["meshUseEmbeddedMaterials"] = marker.mesh_use_embedded_materials;
    out["frameLocked"] = marker.frame_locked;
    return out;
}

viewer_internal::json markerArrayToJson(const visualization_msgs::msg::MarkerArray::SharedPtr& msg,
                                        const std::string& tag) {
    viewer_internal::json markers = viewer_internal::json::array();
    std::string frameId;
    for (const auto& marker : msg->markers) {
        if (frameId.empty()) {
            frameId = marker.header.frame_id;
        }
        markers.push_back(markerToJson(marker));
    }

    viewer_internal::json out;
    out["type"] = "stream.marker_array";
    out["tag"] = tag;
    out["frameId"] = frameId;
    out["markers"] = markers;
    return out;
}

viewer_internal::json markerToPayload(const visualization_msgs::msg::Marker::SharedPtr& msg,
                                     const std::string& tag) {
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(*msg);
    return markerArrayToJson(std::make_shared<visualization_msgs::msg::MarkerArray>(array), tag);
}

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
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

        streamMarkerArrayPub_ = create_publisher<std_msgs::msg::String>(
            viewer_internal::topics::kStreamMarkerArray,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

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
            const bool pointCloud = isPointCloudTopic(types);
            const bool marker = isMarkerTopic(types);
            if (!pointCloud && !marker) {
                continue;
            }

            const bool active = pointCloud
                ? activePointCloudSubs_.find(topic) != activePointCloudSubs_.end()
                : (activeMarkerArraySubs_.find(topic) != activeMarkerArraySubs_.end() ||
                   activeMarkerSubs_.find(topic) != activeMarkerSubs_.end());

            sources.push_back({
                {"id", topic},
                {"name", topic},
                {"type", pointCloud ? "pointcloud" : kMarkerType},
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

        const auto topicNamesAndTypes = get_topic_names_and_types();
        auto topicIt = std::find_if(topicNamesAndTypes.begin(), topicNamesAndTypes.end(),
            [&sourceId](const auto& entry) {
                return entry.first == sourceId;
            });
        if (topicIt == topicNamesAndTypes.end()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(
                viewer_internal::makeErrorResponse(id, "INVALID_PARAMS", "sourceId not found")));
            return;
        }

        const auto& types = topicIt->second;
        const bool pointCloud = isPointCloudTopic(types);
        const bool marker = isMarkerTopic(types);

        bool success = true;
        {
            std::lock_guard<std::mutex> lock(subscriptionMutex_);

            if (active) {
                if (pointCloud && activePointCloudSubs_.find(sourceId) == activePointCloudSubs_.end()) {
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
                } else if (marker) {
                    rclcpp::QoS qos(rclcpp::KeepLast(10));
                    qos.reliable().transient_local();

                    if (std::find(types.begin(), types.end(), "visualization_msgs/msg/MarkerArray") != types.end()) {
                        if (activeMarkerArraySubs_.find(sourceId) == activeMarkerArraySubs_.end()) {
                            auto sub = create_subscription<visualization_msgs::msg::MarkerArray>(
                                sourceId,
                                qos,
                                [this, sourceId](const visualization_msgs::msg::MarkerArray::SharedPtr markerMsg) {
                                    this->forwardMarkerArray(sourceId, markerMsg);
                                });
                            activeMarkerArraySubs_[sourceId] = sub;
                        }
                    } else if (std::find(types.begin(), types.end(), "visualization_msgs/msg/Marker") != types.end()) {
                        if (activeMarkerSubs_.find(sourceId) == activeMarkerSubs_.end()) {
                            auto sub = create_subscription<visualization_msgs::msg::Marker>(
                                sourceId,
                                qos,
                                [this, sourceId](const visualization_msgs::msg::Marker::SharedPtr markerMsg) {
                                    this->forwardMarker(sourceId, markerMsg);
                                });
                            activeMarkerSubs_[sourceId] = sub;
                        }
                    }
                }
            } else {
                activePointCloudSubs_.erase(sourceId);
                activeMarkerArraySubs_.erase(sourceId);
                activeMarkerSubs_.erase(sourceId);
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

    void forwardMarkerArray(const std::string& topic, const visualization_msgs::msg::MarkerArray::SharedPtr& markerMsg) {
        streamMarkerArrayPub_->publish(viewer_internal::toStringMsg(markerArrayToJson(markerMsg, topic)));
    }

    void forwardMarker(const std::string& topic, const visualization_msgs::msg::Marker::SharedPtr& markerMsg) {
        streamMarkerArrayPub_->publish(viewer_internal::toStringMsg(markerToPayload(markerMsg, topic)));
    }

    void handleGraph(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Forwarding topological map to %s (%zu nodes, %zu edges, %zu clusters)",
            viewer_internal::topics::kStreamGraph,
            msg->nodes.size(),
            msg->edges.size() / 2,
            msg->clusters.size());
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr streamMarkerArrayPub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr loadedCloudSub_;
    rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr gngSourceSub_;

    std::mutex subscriptionMutex_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> activePointCloudSubs_;
    std::unordered_map<std::string, rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr> activeMarkerArraySubs_;
    std::unordered_map<std::string, rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr> activeMarkerSubs_;

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
