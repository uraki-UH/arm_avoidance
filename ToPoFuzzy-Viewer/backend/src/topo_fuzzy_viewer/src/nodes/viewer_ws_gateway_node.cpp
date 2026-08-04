#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/protocol.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_feature_msgs/msg/topological_node_feature.hpp>
#include <ais_gng_feature_msgs/msg/topological_cluster_feature.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <voxel_msgs/msg/voxel.hpp>
#include <App.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {
using json = nlohmann::json;
struct PerSocketData {};
using WebSocket = uWS::WebSocket<false, true, PerSocketData>;

namespace topic_utils {
    bool isInternal(const std::string& t) { 
        if (t.find("/viewer/") == 0) return true; // Hide ALL internal viewer topics
        if (t == "/parameter_events" || t == "/rosout") return true; // Hide system topics
        return false; 
    }
    bool isBrowsableSourceType(const std::string& type) {
        return type == "pointcloud" ||
               type == "topological_map" ||
               type == "marker" ||
               type == "voxel";
    }
    std::string detectType(const std::vector<std::string>& types) {
        for (const auto& t : types) {
            if (t.find("PointCloud2") != std::string::npos) return "pointcloud";
            if (t.find("TopologicalMap") != std::string::npos) return "topological_map";
            if (t.find("TopologicalNodeFeature") != std::string::npos) return "topological_node_feature";
            if (t.find("TopologicalClusterFeature") != std::string::npos) return "topological_cluster_feature";
            if (t.find("Marker") != std::string::npos) return "marker";
            if (t.find("Voxel") != std::string::npos) return "voxel";
        }
        return "";
    }
}

namespace converter {
    std::string getMarkerTypeStr(int type) {
        switch(type) {
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

    json marker_to_json(const visualization_msgs::msg::Marker& m) {
        json j = {{"id", m.id}, {"ns", m.ns}, {"type", getMarkerTypeStr(m.type)}, {"action", m.action},
                  {"pos", {m.pose.position.x, m.pose.position.y, m.pose.position.z}},
                  {"quat", {m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w}},
                  {"scale", {m.scale.x, m.scale.y, m.scale.z}},
                  {"color", {m.color.r, m.color.g, m.color.b, m.color.a}},
                  {"frameId", m.header.frame_id}};
        if (!m.points.empty()) { json pts = json::array(); for (auto& p : m.points) pts.push_back({p.x, p.y, p.z}); j["points"] = pts; }
        if (!m.colors.empty()) { json cols = json::array(); for (auto& c : m.colors) cols.push_back({c.r, c.g, c.b, c.a}); j["colors"] = cols; }
        return j;
    }
    json to_json(const visualization_msgs::msg::MarkerArray::SharedPtr msg, const std::string& tag) {
        json markers = json::array(); for (auto& m : msg->markers) markers.push_back(marker_to_json(m));
        return {{"type", "stream.marker_array"}, {"tag", tag}, {"markers", markers}};
    }
    json to_json(const ais_gng_feature_msgs::msg::TopologicalNodeFeature& feature, const std::string& tag) {
        return {
            {"type", "stream.topological_node_feature"},
            {"tag", tag},
            {"feature", {
                {"id", feature.node_id},
                {"node_id", feature.node_id},
                {"is_goal", feature.is_goal},
                {"manip_valid", feature.manip_valid},
                {"manip_value", feature.manip_value},
                {"manip_condition_number", feature.manip_condition_number},
                {"manip_scale", {feature.manip_scale.x, feature.manip_scale.y, feature.manip_scale.z}},
                {"manip_orientation", {feature.manip_orientation.x, feature.manip_orientation.y, feature.manip_orientation.z, feature.manip_orientation.w}},
                {"rotational_manip_valid", feature.rotational_manip_valid},
                {"rotational_manip_value", feature.rotational_manip_value},
                {"rotational_manip_condition_number", feature.rotational_manip_condition_number},
                {"rotational_manip_scale", {feature.rotational_manip_scale.x, feature.rotational_manip_scale.y, feature.rotational_manip_scale.z}},
                {"rotational_manip_orientation", {feature.rotational_manip_orientation.x, feature.rotational_manip_orientation.y, feature.rotational_manip_orientation.z, feature.rotational_manip_orientation.w}}
            }}
        };
    }
    json to_json(const ais_gng_feature_msgs::msg::TopologicalClusterFeature& feature, const std::string& tag) {
        return {
            {"type", "stream.topological_cluster_feature"},
            {"tag", tag},
            {"feature", {
                {"id", feature.cluster_id},
                {"cluster_id", feature.cluster_id},
                {"has_velocity_observation", feature.has_velocity_observation},
                {"vel_cov_xx", feature.vel_cov_xx},
                {"vel_cov_xy", feature.vel_cov_xy},
                {"vel_cov_yy", feature.vel_cov_yy}
            }}
        };
    }
    json to_json(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg, const std::string& tag,
                 const std::vector<json>& node_features = {}, const std::vector<json>& cluster_features = {}) {
        json nodes = json::array(), clusters = json::array();
        for (auto& n : msg->nodes) {
            const auto age = msg->frame_number >= n.frame ? msg->frame_number - n.frame : 0U;
            nodes.push_back({
                {"id", n.id},
                {"x", n.pos.x},
                {"y", n.pos.y},
                {"z", n.pos.z},
                {"nx", n.normal.x},
                {"ny", n.normal.y},
                {"nz", n.normal.z},
                {"label", n.label},
                {"semanticLabel", n.semantic_label},
                {"semanticReliability", n.semantic_reliability},
                {"isGoal", n.is_goal},
                {"age", age},
                {"winnerPointCount", n.winner_point_count},
                {"winnerPointCovariance", {
                    n.winner_point_covariance[0], n.winner_point_covariance[1], n.winner_point_covariance[2],
                    n.winner_point_covariance[3], n.winner_point_covariance[4], n.winner_point_covariance[5],
                    n.winner_point_covariance[6], n.winner_point_covariance[7], n.winner_point_covariance[8]
                }}
            });
        }
        for (auto& c : msg->clusters) {
            const auto age = msg->frame_number >= c.frame ? msg->frame_number - c.frame : 0U;
            clusters.push_back({{"id",c.id},{"label",c.label},{"semanticLabel",c.semantic_label},{"semanticReliability",c.semantic_reliability},{"pos",{c.pos.x,c.pos.y,c.pos.z}},{"scale",{c.scale.x,c.scale.y,c.scale.z}},{"quat",{c.quat.x,c.quat.y,c.quat.z,c.quat.w}},{"match",c.match},{"reliability",c.label_reliability},{"velocity",{c.velocity.x,c.velocity.y,c.velocity.z}},{"nodeIds",c.nodes},{"age",age}});
        }
        return {{"type", "stream.graph"}, {"tag", tag}, {"graph", {
            {"timestamp", msg->header.stamp.sec}, {"tag", tag}, {"mode", (tag.find("static") != std::string::npos ? "static" : "dynamic")},
            {"frameId", msg->header.frame_id}, {"nodes", nodes}, {"edges", msg->edges}, {"clusters", clusters},
            {"node_features", node_features}, {"cluster_features", cluster_features}
        }}};
    }
    json to_json(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        json tfs = json::array(); for (auto& ts : msg->transforms) tfs.push_back({{"frameId",ts.header.frame_id},{"childFrameId",ts.child_frame_id},{"pos",{ts.transform.translation.x,ts.transform.translation.y,ts.transform.translation.z}},{"quat",{ts.transform.rotation.x,ts.transform.rotation.y,ts.transform.rotation.z,ts.transform.rotation.w}}});
        return {{"type", "stream.tf"}, {"transforms", tfs}};
    }

    //idからボクセルを復元して可視化する
    json to_json(const voxel_msgs::msg::Voxel::SharedPtr msg, const std::string& tag) {
        json ids = json::array(); for (auto id : msg->data) ids.push_back(std::to_string(id));
        return {{"type", "stream.voxel"}, {"tag", tag}, {"data", ids}, {"frameId", msg->header.frame_id},
                {"layout", {{"voxelSize", std::round(msg->voxel_size * 10000.0) / 10000.0}, 
                            {"originX", msg->origin_x},
                            {"originY", msg->origin_y},
                            {"originZ", msg->origin_z},
                            {"xShift", msg->x_shift}, {"yShift", msg->y_shift}, {"zShift", msg->z_shift}, {"offset", msg->offset}}}};
    }
}

std::filesystem::path findLocalPackageShare(const std::string& pkg_name) {
    std::vector<std::filesystem::path> bases;
    auto add_base = [&](std::filesystem::path base) {
        if (base.empty()) return;
        if (std::find(bases.begin(), bases.end(), base) == bases.end()) {
            bases.push_back(std::move(base));
        }
    };

#ifdef PROJECT_SOURCE_DIR
    {
        std::filesystem::path cur(PROJECT_SOURCE_DIR);
        for (int i = 0; i < 5 && !cur.empty(); ++i) {
            add_base(cur);
            const auto parent = cur.parent_path();
            if (parent == cur) break;
            cur = parent;
        }
    }
#endif
    {
        std::filesystem::path cur = std::filesystem::current_path();
        for (int i = 0; i < 5 && !cur.empty(); ++i) {
            add_base(cur);
            const auto parent = cur.parent_path();
            if (parent == cur) break;
            cur = parent;
        }
    }

    for (const auto& base : bases) {
        std::error_code ec;
        if (!std::filesystem::exists(base, ec) || ec) continue;
        std::filesystem::directory_options opts = std::filesystem::directory_options::skip_permission_denied;
        for (std::filesystem::recursive_directory_iterator it(base, opts, ec), end; it != end && !ec; it.increment(ec)) {
            if (!it->is_directory()) continue;
            if (it->path().filename() != pkg_name) continue;
            const auto candidate = it->path();
            if (std::filesystem::exists(candidate / "meshes", ec) && !ec) {
                return candidate;
            }
        }
    }
    return {};
}

class ViewerWsGatewayNode : public rclcpp::Node {
public:
    ViewerWsGatewayNode() : Node("viewer_ws_gateway_node") {
        const int port = declare_parameter<int>("port", 9001);
        rpcRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcRequest, 100);
        rpcResponseSub_ = create_subscription<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 100, [this](const std_msgs::msg::String::SharedPtr msg) { rpc_.handleResponse(msg->data); });
        tfSub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTfTime_).count() > 33) { broadcastText(converter::to_json(msg).dump()); lastTfTime_ = now; }
        });
        livenessTimer_ = create_wall_timer(std::chrono::seconds(1), [this]() { checkLiveness(); });
        serverThread_ = std::thread([this, port]() { runServerLoop(port); });
        RCLCPP_INFO(get_logger(), "Gateway (Unified) initialized on port %d", port);
    }
    ~ViewerWsGatewayNode() override { serverRunning_ = false; if (loop_) loop_->defer([this]() { if (listenSocket_) { us_listen_socket_close(0, listenSocket_); listenSocket_ = nullptr; } std::lock_guard<std::mutex> l(connectionMutex_); for (auto* ws : connections_) ws->close(); connections_.clear(); }); if (serverThread_.joinable()) serverThread_.join(); }

private:
    void onWsMessage(WebSocket* ws, std::string_view msg) {
        json in = json::parse(msg, nullptr, false); if (in.is_discarded()) return;
        std::string type = in.value("type", ""), method = in.value("method", ""), id = in.value("id", ""), tag = in.value("tag", "");
        if (type == "request.state") { sendCurrentState(ws); return; }
        if (type.find(".delete") != std::string::npos || type.find(".remove") != std::string::npos) { 
            std::string target = tag.empty() ? in.value("topic", "") : tag;
            if (!target.empty()) sendStreamDelete(target); return; 
        }
        if (method == "sources.list") { handleSourcesList(ws, id); }
        else if (method == "sources.setActive") { handleSourcesSetActive(ws, id, in.value("params", json::object())); }
        else { forwardRpcRequest(ws, id, method, std::string(msg)); }
    }

    void handleSourcesList(WebSocket* ws, const std::string& id) {
        auto topics = get_topic_names_and_types(); json sources = json::array();
        for (auto const& [topic, types] : topics) {
            if (topic_utils::isInternal(topic)) continue;
            std::string st = topic_utils::detectType(types); if (st.empty()) continue;
            if (!topic_utils::isBrowsableSourceType(st)) continue;
            if (count_publishers(topic) == 0) continue;
            sources.push_back({{"id",topic},{"name",topic},{"label",topic},{"type",st},{"active",activeDynamicSubs_.count(topic)>0}});
        }
        ws->send(viewer_internal::makeOkResponse(id, {{"sources", sources}}), uWS::OpCode::TEXT);
    }

    void broadcastSourcesList() {
        auto topics = get_topic_names_and_types(); json sources = json::array();
        for (auto const& [topic, types] : topics) {
            if (topic_utils::isInternal(topic)) continue;
            std::string st = topic_utils::detectType(types); if (st.empty()) continue;
            if (!topic_utils::isBrowsableSourceType(st)) continue;
            if (count_publishers(topic) == 0) continue;
            sources.push_back({{"id",topic},{"name",topic},{"label",topic},{"type",st},{"active",activeDynamicSubs_.count(topic)>0}});
        }
        broadcastText(viewer_internal::makeOkResponse("sync_sources", {{"sources", sources}}));
    }

    void handleSourcesSetActive(WebSocket* ws, const std::string& id, const json& params) {
        std::string sid = params.value("sourceId", "");
        bool active = params.value("active", false);
        bool remove_layer = params.value("removeLayer", false);
        if (sid.empty()) return;
        if (sid.front() != '/') sid = "/" + sid;
        std::lock_guard<std::mutex> lock(connectionMutex_);
        if (active) {
            if (activeDynamicSubs_.count(sid)) return;
            auto topics = get_topic_names_and_types();
            if (topics.count(sid)) {
                std::string st = topic_utils::detectType(topics[sid]);
                if (!topic_utils::isBrowsableSourceType(st)) {
                    broadcastText(viewer_internal::makeErrorResponse(
                        id, "UNSUPPORTED_SOURCE_TYPE",
                        "This source type is metadata-only and is not exposed as a scene layer"));
                    return;
                }
                if (count_publishers(sid) == 0) {
                    broadcastText(viewer_internal::makeErrorResponse(
                        id, "SOURCE_HAS_NO_PUBLISHER",
                        "This topic currently has no publisher"));
                    return;
                }
                if (st == "pointcloud") {
                    activeSubTypes_[sid] = "pointcloud";
                    activeDynamicSubs_[sid] = create_subscription<sensor_msgs::msg::PointCloud2>(sid, 10, [this, sid](const sensor_msgs::msg::PointCloud2::SharedPtr m) { broadcastPointCloud(sid, utils::convertToProtocolMessage(utils::convertFromRosMsg(m)).serialize()); });
                } else if (st == "topological_map") {
                    activeSubTypes_[sid] = "topological_map";
                    activeDynamicSubs_[sid] = create_subscription<ais_gng_msgs::msg::TopologicalMap>(sid, rclcpp::QoS(10).reliable().transient_local(), [this, sid](const ais_gng_msgs::msg::TopologicalMap::SharedPtr m) {
                        std::vector<json> node_features;
                        std::vector<json> cluster_features;
                        {
                            std::lock_guard<std::mutex> lock(nodeFeatureMutex_);
                            node_features.reserve(lastNodeFeaturePayloads_.size());
                            for (const auto& [_, payload] : lastNodeFeaturePayloads_) {
                                node_features.push_back(payload);
                            }
                        }
                        {
                            std::lock_guard<std::mutex> lock(clusterFeatureMutex_);
                            cluster_features.reserve(lastClusterFeaturePayloads_.size());
                            for (const auto& [_, payload] : lastClusterFeaturePayloads_) {
                                cluster_features.push_back(payload);
                            }
                        }
                        broadcastText(converter::to_json(m, sid, node_features, cluster_features).dump());
                    });
                } else if (st == "topological_node_feature") {
                    activeSubTypes_[sid] = "topological_node_feature";
                    activeDynamicSubs_[sid] = create_subscription<ais_gng_feature_msgs::msg::TopologicalNodeFeature>(sid, rclcpp::QoS(10).reliable().transient_local(), [this, sid](const ais_gng_feature_msgs::msg::TopologicalNodeFeature::SharedPtr m) {
                        {
                            std::lock_guard<std::mutex> lock(nodeFeatureMutex_);
                            lastNodeFeaturePayloads_[sid] = converter::to_json(*m, sid);
                        }
                        broadcastText(converter::to_json(*m, sid).dump());
                    });
                } else if (st == "topological_cluster_feature") {
                    activeSubTypes_[sid] = "topological_cluster_feature";
                    activeDynamicSubs_[sid] = create_subscription<ais_gng_feature_msgs::msg::TopologicalClusterFeature>(sid, rclcpp::QoS(10).reliable().transient_local(), [this, sid](const ais_gng_feature_msgs::msg::TopologicalClusterFeature::SharedPtr m) {
                        {
                            std::lock_guard<std::mutex> lock(clusterFeatureMutex_);
                            lastClusterFeaturePayloads_[sid] = converter::to_json(*m, sid);
                        }
                        broadcastText(converter::to_json(*m, sid).dump());
                    });
                } else if (st == "marker") {
                    activeSubTypes_[sid] = "marker";
                    activeDynamicSubs_[sid] = create_subscription<visualization_msgs::msg::MarkerArray>(sid, rclcpp::QoS(10).reliable().transient_local(), [this, sid](const visualization_msgs::msg::MarkerArray::SharedPtr m) { broadcastText(converter::to_json(m, sid).dump()); });
                } else if (st == "voxel") {
                    activeSubTypes_[sid] = "voxel";
                    activeDynamicSubs_[sid] = create_subscription<voxel_msgs::msg::Voxel>(sid, rclcpp::QoS(1).reliable().transient_local(), [this, sid](const voxel_msgs::msg::Voxel::SharedPtr m) { broadcastText(converter::to_json(m, sid).dump()); });
                }
            }
        } else {
            if (activeDynamicSubs_.count(sid)) {
                if (remove_layer) sendStreamDelete(sid);
                activeDynamicSubs_.erase(sid);
                activeSubTypes_.erase(sid);
            }
        }
        broadcastText(viewer_internal::makeOkResponse(id, {{"success",true},{"sourceId",sid},{"active",active},{"remove",remove_layer}}));
        broadcastSourcesList();
    }

    void forwardRpcRequest(WebSocket* ws, const std::string& id, const std::string& method, const std::string& raw) {
        std::thread([this, ws, id, raw]() {
            std::string resp = rpc_.call(id, raw, rpcRequestPub_, 30000);
            loop_->defer([this, ws, resp]() { std::lock_guard<std::mutex> lock(connectionMutex_); if (std::find(connections_.begin(), connections_.end(), ws) != connections_.end()) ws->send(resp, uWS::OpCode::TEXT); });
        }).detach();
    }

    void broadcastPointCloud(const std::string& topic, const std::vector<uint8_t>& data) {
        // Prepend topic name for reliable identification (ROS-style multiplexing)
        uint8_t topicLen = static_cast<uint8_t>(std::min<size_t>(topic.length(), 255));
        std::vector<uint8_t> packet;
        packet.reserve(1 + topicLen + data.size());
        packet.push_back(topicLen);
        packet.insert(packet.end(), topic.begin(), topic.begin() + topicLen);
        packet.insert(packet.end(), data.begin(), data.end());

        auto s = std::make_shared<std::vector<uint8_t>>(std::move(packet));
        // We still send the meta for legacy/sync reasons, but the binary now contains its own ID
        auto meta = std::make_shared<std::string>(json({{"type", "stream.pointcloud.meta"}, {"topic", topic}, {"tag", topic}}).dump());
        
        loop_->defer([this, meta, s]() { 
            std::lock_guard<std::mutex> l(connectionMutex_); 
            for (auto* ws : connections_) { 
                ws->send(*meta, uWS::OpCode::TEXT); 
                ws->send(std::string_view(reinterpret_cast<const char*>(s->data()), s->size()), uWS::OpCode::BINARY); 
            } 
        });
    }

    void broadcastText(const std::string& payload) { auto s = std::make_shared<std::string>(payload); loop_->defer([this, s]() { std::lock_guard<std::mutex> l(connectionMutex_); for (auto* ws : connections_) ws->send(*s, uWS::OpCode::TEXT); }); }

    void sendStreamDelete(const std::string& sid) {
        std::string id = (sid.front() == '/') ? sid : "/" + sid;
        broadcastText(json({{"type", "stream.delete"}, {"id", id}, {"tag", id}, {"topic", id}}).dump());
        broadcastText(json({{"type", "stream.remove_layer"}, {"id", id}, {"tag", id}, {"topic", id}}).dump());
        broadcastText(json({{"type", "stream.pointcloud.meta"}, {"tag", id}, {"topic", id}, {"active", false}, {"action", "remove"}}).dump());
        broadcastText(json({{"type", "stream.pointcloud.delete"}, {"tag", id}, {"topic", id}}).dump());
        broadcastText(json({{"type", "stream.graph.delete"}, {"tag", id}, {"topic", id}}).dump());
        broadcastText(json({{"type", "stream.topological_node_feature.delete"}, {"tag", id}, {"topic", id}}).dump());
        broadcastText(json({{"type", "stream.topological_cluster_feature.delete"}, {"tag", id}, {"topic", id}}).dump());
        broadcastText(json({{"type", "stream.marker_array.delete"}, {"tag", id}}).dump());
        // Clear any cached graph data for this topic
        {
            std::lock_guard<std::mutex> l(graphMutex_);
            lastGraphPayloads_.erase(id);
        }
        {
            std::lock_guard<std::mutex> l(nodeFeatureMutex_);
            lastNodeFeaturePayloads_.erase(id);
        }
        {
            std::lock_guard<std::mutex> l(clusterFeatureMutex_);
            lastClusterFeaturePayloads_.erase(id);
        }
    }

    void sendCurrentState(WebSocket* ws) {
        std::lock_guard<std::mutex> l1(graphMutex_);
        std::lock_guard<std::mutex> l2(nodeFeatureMutex_);
        std::lock_guard<std::mutex> l3(clusterFeatureMutex_);
        std::lock_guard<std::mutex> l4(robotMutex_);
        std::lock_guard<std::mutex> l5(markerMutex_);
        for (auto& p : lastGraphPayloads_) ws->send(p.second, uWS::OpCode::TEXT);
        for (auto& p : lastNodeFeaturePayloads_) ws->send(p.second.dump(), uWS::OpCode::TEXT);
        for (auto& p : lastClusterFeaturePayloads_) ws->send(p.second.dump(), uWS::OpCode::TEXT);
        for (auto& p : lastRobotDescriptions_) ws->send(p.second, uWS::OpCode::TEXT);
        for (auto& p : lastMarkerPayloads_) ws->send(p.second, uWS::OpCode::TEXT);
    }

    void runServerLoop(int port) {
        loop_ = uWS::Loop::get(); serverRunning_ = true;
        uWS::App::WebSocketBehavior<PerSocketData> behavior; behavior.maxPayloadLength = 64 * 1024 * 1024;
        behavior.open = [this](auto* ws) { std::lock_guard<std::mutex> l(connectionMutex_); if (connections_.empty()) subscribeStreamingTopics(); connections_.push_back(ws); sendCurrentState(ws); };
        behavior.message = [this](auto* ws, std::string_view msg, uWS::OpCode op) { if (op == uWS::OpCode::TEXT) onWsMessage(ws, msg); };
        behavior.close = [this](auto* ws, int, std::string_view) { std::lock_guard<std::mutex> lock(connectionMutex_); connections_.erase(std::remove(connections_.begin(), connections_.end(), ws), connections_.end()); if (connections_.empty()) unsubscribeStreamingTopics(); };
        uWS::App().get("/*", [this](auto* res, auto* req) { meshServer_.handle(res, req); }).ws<PerSocketData>("/*", std::move(behavior)).listen(port, [this, port](auto* s) { if (s) { listenSocket_ = s; RCLCPP_INFO(get_logger(), "WS Server on %d", port); } }).run();
    }

    void subscribeStreamingTopics() {
        std::string base = viewer_internal::topics::kStreamRobot;
        robotDescSub_ = create_subscription<std_msgs::msg::String>(base + "/description", rclcpp::QoS(1).reliable().transient_local(), [this](const std_msgs::msg::String::SharedPtr m) { handleRobotData(m, true); });
        robotPoseSub_ = create_subscription<std_msgs::msg::String>(base + "/pose", rclcpp::QoS(1).best_effort(), [this](const std_msgs::msg::String::SharedPtr m) { handleRobotData(m, false); });
        jobEventSub_ = create_subscription<std_msgs::msg::String>(viewer_internal::topics::kEditJobEvents, 100, [this](const std_msgs::msg::String::SharedPtr m) { broadcastText(m->data); });
    }
    void unsubscribeStreamingTopics() {
        robotDescSub_.reset();
        robotPoseSub_.reset();
        jobEventSub_.reset();
        activeDynamicSubs_.clear();
        activeSubTypes_.clear();
        lastGraphPayloads_.clear();
        lastNodeFeaturePayloads_.clear();
        lastClusterFeaturePayloads_.clear();
    }
    void handleRobotData(const std_msgs::msg::String::SharedPtr msg, bool is_desc) {
        if (msg->data.empty()) return;
        if (is_desc) { json j = json::parse(msg->data, nullptr, false); if (!j.is_discarded()) { std::lock_guard<std::mutex> lock(robotMutex_); lastRobotDescriptions_[j.value("tag", "default")] = msg->data; } }
        broadcastText(msg->data);
    }
    void checkLiveness() {
        if (this->get_publishers_info_by_topic(std::string(viewer_internal::topics::kStreamRobot) + "/description").empty()) {
            std::lock_guard<std::mutex> lock(robotMutex_);
            for (auto const& [tag, _] : lastRobotDescriptions_) {
                broadcastText(json({{"type", "stream.robot.delete"}, {"tag", tag}}).dump());
            }
            lastRobotDescriptions_.clear();
        }
    }

    class UnifiedRpc {
    public:
        struct Pending { std::mutex mutex; std::condition_variable cv; bool done = false; std::string payload; };
        std::string call(const std::string& id, const std::string& req, const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub, int timeoutMs) { auto p = std::make_shared<Pending>(); { std::lock_guard<std::mutex> l(mutex_); pending_[id] = p; } std_msgs::msg::String m; m.data = req; pub->publish(m); std::unique_lock<std::mutex> lock(p->mutex); bool ready = p->cv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&]() { return p->done; }); { std::lock_guard<std::mutex> l(mutex_); pending_.erase(id); } return ready ? p->payload : viewer_internal::makeErrorResponse(id, "TIMEOUT", "Backend timeout"); }
        void handleResponse(const std::string& data) { json resp = json::parse(data, nullptr, false); std::string id = resp.value("id", ""); std::shared_ptr<Pending> p; { std::lock_guard<std::mutex> l(mutex_); if (pending_.count(id)) p = pending_[id]; } if (p) { std::lock_guard<std::mutex> l(p->mutex); p->payload = data; p->done = true; p->cv.notify_all(); } }
    private:
        std::mutex mutex_; std::unordered_map<std::string, std::shared_ptr<Pending>> pending_;
    } rpc_;

    class MeshServer {
    public:
        void handle(uWS::HttpResponse<false>* res, uWS::HttpRequest* req) {
            std::string url(req->getUrl()); res->writeHeader("Access-Control-Allow-Origin", "*");
            if (url.rfind("/meshes/", 0) == 0) {
                std::string sub = url.substr(8); size_t slash = sub.find('/');
                if (slash != std::string::npos) {
                    const std::string pkg = sub.substr(0, slash);
                    const std::string rel = sub.substr(slash + 1);
                    auto tryServe = [&](const std::filesystem::path& root, const std::string& relative) -> bool {
                        if (root.empty()) {
                            return false;
                        }
                        const std::filesystem::path path = root / relative;
                        const auto data = readFile(path);
                        if (!data) {
                            return false;
                        }
                        res->writeHeader("Content-Type", "application/octet-stream");
                        res->writeHeader("Cache-Control", "no-store");
                        res->end(*data);
                        return true;
                    };

                    const auto& share_root = packageShareRoot(pkg);
                    if (tryServe(share_root, rel) || tryServe(share_root, std::string("meshes/") + rel)) {
                        return;
                    }

                    const auto& fallback_root = localPackageRoot(pkg);
                    if (tryServe(fallback_root, rel) || tryServe(fallback_root, std::string("meshes/") + rel)) {
                        return;
                    }
                }
            }
            res->writeStatus("404 Not Found")->end("Not found");
        }

    private:
        struct CachedFile {
            std::filesystem::file_time_type modified;
            std::uintmax_t size = 0;
            std::shared_ptr<const std::string> data;
        };

        const std::filesystem::path& packageShareRoot(const std::string& pkg) {
            const auto cached = packageShareRoots_.find(pkg);
            if (cached != packageShareRoots_.end()) {
                return cached->second;
            }

            std::filesystem::path root;
            try {
                root = ament_index_cpp::get_package_share_directory(pkg);
            } catch (...) {}
            return packageShareRoots_.emplace(pkg, std::move(root)).first->second;
        }

        const std::filesystem::path& localPackageRoot(const std::string& pkg) {
            const auto cached = localPackageRoots_.find(pkg);
            if (cached != localPackageRoots_.end()) {
                return cached->second;
            }
            return localPackageRoots_.emplace(pkg, findLocalPackageShare(pkg)).first->second;
        }

        std::shared_ptr<const std::string> readFile(const std::filesystem::path& path) {
            std::error_code ec;
            if (!std::filesystem::exists(path, ec) || ec || std::filesystem::is_directory(path, ec) || ec) {
                return {};
            }

            const auto modified = std::filesystem::last_write_time(path, ec);
            if (ec) return {};
            const auto size = std::filesystem::file_size(path, ec);
            if (ec) return {};

            const std::string key = path.string();
            const auto cached = files_.find(key);
            if (cached != files_.end() && cached->second.modified == modified && cached->second.size == size) {
                return cached->second.data;
            }

            std::ifstream file(path, std::ios::binary);
            if (!file) return {};
            auto data = std::make_shared<const std::string>(
                std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
            files_[key] = CachedFile{modified, size, data};
            return data;
        }

        std::unordered_map<std::string, std::filesystem::path> packageShareRoots_;
        std::unordered_map<std::string, std::filesystem::path> localPackageRoots_;
        std::unordered_map<std::string, CachedFile> files_;
    } meshServer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcRequestPub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcResponseSub_, jobEventSub_, robotDescSub_, robotPoseSub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> activeDynamicSubs_;
    std::unordered_map<std::string, std::string> activeSubTypes_, lastGraphPayloads_, lastRobotDescriptions_, lastMarkerPayloads_;
    std::unordered_map<std::string, json> lastNodeFeaturePayloads_, lastClusterFeaturePayloads_;
    std::chrono::steady_clock::time_point lastTfTime_;
    std::mutex connectionMutex_, graphMutex_, nodeFeatureMutex_, clusterFeatureMutex_, robotMutex_, markerMutex_;
    std::vector<WebSocket*> connections_;
    std::thread serverThread_; us_listen_socket_t* listenSocket_ = nullptr;
    std::atomic<bool> serverRunning_{false}; uWS::Loop* loop_ = nullptr;
    rclcpp::TimerBase::SharedPtr livenessTimer_;
};
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); auto node = std::make_shared<ViewerWsGatewayNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node); executor.spin(); executor.remove_node(node);
    rclcpp::shutdown(); return 0;
}
