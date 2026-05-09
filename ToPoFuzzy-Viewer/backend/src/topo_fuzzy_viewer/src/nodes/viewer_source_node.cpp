#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

bool containsType(const std::vector<std::string>& types, const std::string& target) {
    for (const auto& t : types) {
        if (t.find(target) != std::string::npos) return true;
    }
    return false;
}

bool isPointCloudTopic(const std::vector<std::string>& types) {
    return containsType(types, "PointCloud2");
}

bool isMarkerTopic(const std::vector<std::string>& types) {
    return containsType(types, "Marker");
}

bool isGraphTopic(const std::vector<std::string>& types) {
    return containsType(types, "TopologicalMap");
}

class ViewerSourceNode : public rclcpp::Node {
public:
    ViewerSourceNode()
        : rclcpp::Node("viewer_source_node") {
        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcSourceRequest,
            10,
            std::bind(&ViewerSourceNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 50);

        RCLCPP_INFO(get_logger(), "viewer_source_node (Metadata Only) initialized");
    }

private:
    void handleRpcRequest(const std_msgs::msg::String::SharedPtr msg) {
        viewer_internal::json in;
        if (!viewer_internal::parseJson(msg->data, in)) return;
        std::string method = in.value("method", "");
        std::string id = in.value("id", "");

        if (method == "sources.list") {
            handleSourcesList(id);
        } else if (method == "sources.setActive") {
            handleSourcesSetActive(id, in.value("params", viewer_internal::json::object()));
        }
    }

    void handleSourcesList(const std::string& id) {
        const auto topicNamesAndTypes = get_topic_names_and_types();
        viewer_internal::json sources = viewer_internal::json::array();

        for (const auto& [topic, types] : topicNamesAndTypes) {
            bool pointCloud = isPointCloudTopic(types);
            bool marker = isMarkerTopic(types);
            bool graph = isGraphTopic(types);

            if (!pointCloud && !marker && !graph) continue;

            std::string sourceType;
            if (pointCloud) sourceType = "pointcloud";
            else if (marker) sourceType = "marker";
            else if (graph) sourceType = "topological_map";

            sources.push_back({
                {"id", topic},
                {"name", topic},
                {"label", topic},
                {"type", sourceType},
                {"active", activeSourceIds_.count(topic) > 0}
            });
        }

        viewer_internal::json result;
        result["sources"] = sources;
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void handleSourcesSetActive(const std::string& id, const viewer_internal::json& params) {
        std::string sourceId = params.value("sourceId", "");
        bool active = params.value("active", false);
        if (sourceId.empty()) return;
        if (sourceId.front() != '/') sourceId = "/" + sourceId;

        if (active) activeSourceIds_.insert(sourceId);
        else activeSourceIds_.erase(sourceId);

        viewer_internal::json result;
        result["success"] = true;
        result["sourceId"] = sourceId;
        result["active"] = active;
        if (!id.empty()) {
            rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;
    std::set<std::string> activeSourceIds_;
};

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerSourceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
