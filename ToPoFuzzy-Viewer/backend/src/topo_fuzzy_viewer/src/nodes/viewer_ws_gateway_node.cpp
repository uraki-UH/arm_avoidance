#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/protocol.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <App.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
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

// --- 1. Protocol Converter ---
namespace converter {
    json marker_to_json(const visualization_msgs::msg::Marker& m) {
        json j = {{"id", m.id}, {"ns", m.ns}, {"type", m.type}, {"action", m.action},
                  {"pos", {m.pose.position.x, m.pose.position.y, m.pose.position.z}},
                  {"quat", {m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w}},
                  {"scale", {m.scale.x, m.scale.y, m.scale.z}},
                  {"color", {m.color.r, m.color.g, m.color.b, m.color.a}},
                  {"frameId", m.header.frame_id}};
        if (!m.points.empty()) {
            json pts = json::array();
            for (auto& p : m.points) pts.push_back({p.x, p.y, p.z});
            j["points"] = pts;
        }
        if (!m.colors.empty()) {
            json cols = json::array();
            for (auto& c : m.colors) cols.push_back({c.r, c.g, c.b, c.a});
            j["colors"] = cols;
        }
        return j;
    }

    json marker_array_to_json(const visualization_msgs::msg::MarkerArray::SharedPtr msg, const std::string& tag) {
        json markers = json::array();
        for (auto& m : msg->markers) markers.push_back(marker_to_json(m));
        return {{"type", "stream.marker_array"}, {"tag", tag}, {"markers", markers}};
    }

    json to_json(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg, const std::string& tag) {
        json nodes = json::array();
        for (auto& n : msg->nodes) nodes.push_back({{"x",n.pos.x},{"y",n.pos.y},{"z",n.pos.z},{"nx",n.normal.x},{"ny",n.normal.y},{"nz",n.normal.z},{"label",n.label},{"age",n.age}});
        json clusters = json::array();
        for (auto& c : msg->clusters) {
            clusters.push_back({{"id",c.id},{"label",c.label},{"pos",{c.pos.x,c.pos.y,c.pos.z}},{"scale",{c.scale.x,c.scale.y,c.scale.z}},{"quat",{c.quat.x,c.quat.y,c.quat.z,c.quat.w}},{"match",c.match},{"reliability",c.reliability},{"velocity",{c.velocity.x,c.velocity.y,c.velocity.z}},{"nodeIds",c.nodes}});
        }
        return {{"type", "stream.graph"}, {"tag", tag}, {"graph", {
            {"timestamp", msg->header.stamp.sec}, {"tag", tag}, {"mode", (tag.find("static") != std::string::npos ? "static" : "dynamic")},
            {"frameId", msg->header.frame_id}, {"nodes", nodes}, {"edges", msg->edges}, {"clusters", clusters}
        }}};
    }

    json to_json(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        json tfs = json::array();
        for (auto& ts : msg->transforms) tfs.push_back({{"frameId",ts.header.frame_id},{"childFrameId",ts.child_frame_id},{"pos",{ts.transform.translation.x,ts.transform.translation.y,ts.transform.translation.z}},{"quat",{ts.transform.rotation.x,ts.transform.rotation.y,ts.transform.rotation.z,ts.transform.rotation.w}}});
        return {{"type", "stream.tf"}, {"transforms", tfs}};
    }
}

// --- 2. RPC Manager ---
class RpcManager {
public:
    struct Pending {
        std::mutex mutex; std::condition_variable cv; bool done = false; std::string payload;
    };
    std::string call(const std::string& id, const std::string& req, const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub, int timeoutMs) {
        auto p = std::make_shared<Pending>();
        { std::lock_guard<std::mutex> l(mutex_); pending_[id] = p; }
        std_msgs::msg::String msg; msg.data = req; pub->publish(msg);
        std::unique_lock<std::mutex> lock(p->mutex);
        bool ready = p->cv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&]() { return p->done; });
        { std::lock_guard<std::mutex> l(mutex_); pending_.erase(id); }
        return ready ? p->payload : viewer_internal::makeErrorResponse(id, "TIMEOUT", "Backend timeout");
    }
    void handleResponse(const std::string& data) {
        json resp = json::parse(data, nullptr, false); std::string id = resp.value("id", "");
        std::shared_ptr<Pending> p; { std::lock_guard<std::mutex> l(mutex_); if (pending_.count(id)) p = pending_[id]; }
        if (p) { std::lock_guard<std::mutex> l(p->mutex); p->payload = data; p->done = true; p->cv.notify_all(); }
    }
private:
    std::mutex mutex_; std::unordered_map<std::string, std::shared_ptr<Pending>> pending_;
};

// --- 3. Mesh Server ---
class MeshServer {
public:
    void handle(uWS::HttpResponse<false>* res, uWS::HttpRequest* req) {
        std::string url(req->getUrl()); res->writeHeader("Access-Control-Allow-Origin", "*");
        if (url.rfind("/meshes/", 0) == 0) {
            std::string sub = url.substr(8); size_t slash = sub.find('/');
            if (slash != std::string::npos) {
                try {
                    auto path = std::filesystem::path(ament_index_cpp::get_package_share_directory(sub.substr(0, slash))) / sub.substr(slash + 1);
                    if (std::filesystem::exists(path) && !std::filesystem::is_directory(path)) {
                        std::ifstream f(path, std::ios::binary); res->end(std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>())); return;
                    }
                } catch (...) {}
            }
        }
        res->writeStatus("404 Not Found")->end("Not found");
    }
};

// --- 4. Main Gateway Node ---
class ViewerWsGatewayNode : public rclcpp::Node {
public:
    ViewerWsGatewayNode() : Node("viewer_ws_gateway_node") {
        const int port = declare_parameter<int>("port", 9001);
        auto cpub = [&](const std::string& t) { return create_publisher<std_msgs::msg::String>(t, 50); };
        rpcPubs_["sources"] = rpcPubs_["publish"] = cpub(viewer_internal::topics::kRpcSourceRequest);
        rpcPubs_["files"] = cpub(viewer_internal::topics::kRpcFileRequest);
        rpcPubs_["rosbag"] = cpub(viewer_internal::topics::kRpcRosbagRequest);
        rpcPubs_["gng"] = cpub(viewer_internal::topics::kRpcGngRequest);
        rpcPubs_["params"] = cpub(viewer_internal::topics::kRpcParamRequest);
        rpcPubs_["edit"] = cpub(viewer_internal::topics::kRpcEditRequest);

        rpcResponseSub_ = create_subscription<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 100, [this](const std_msgs::msg::String::SharedPtr msg) { rpc_.handleResponse(msg->data); });

        tfSub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100, [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTfTime_).count() > 33) { // ~30Hz limit
                broadcastText(converter::to_json(msg).dump());
                lastTfTime_ = now;
            }
        });
        livenessTimer_ = create_wall_timer(std::chrono::seconds(1), [this]() { checkLiveness(); });
        serverThread_ = std::thread([this, port]() { runServerLoop(port); });
        RCLCPP_INFO(get_logger(), "Gateway initialized on port %d", port);
    }

    ~ViewerWsGatewayNode() override {
        serverRunning_ = false;
        if (loop_) loop_->defer([this]() {
            if (listenSocket_) { us_listen_socket_close(0, listenSocket_); listenSocket_ = nullptr; }
            std::lock_guard<std::mutex> l(connectionMutex_); for (auto* ws : connections_) ws->close(); connections_.clear();
        });
        if (serverThread_.joinable()) serverThread_.join();
    }

private:
    void runServerLoop(int port) {
        loop_ = uWS::Loop::get(); serverRunning_ = true;
        uWS::App::WebSocketBehavior<PerSocketData> behavior;
        behavior.maxPayloadLength = 64 * 1024 * 1024;
        behavior.open = [this](auto* ws) {
            std::vector<std::string> cached;
            { std::lock_guard<std::mutex> l(graphMutex_); for (auto& p : lastGraphPayloads_) cached.push_back(p.second); }
            { std::lock_guard<std::mutex> l(robotMutex_); for (auto& p : lastRobotDescriptions_) cached.push_back(p.second); }
            { std::lock_guard<std::mutex> l(markerMutex_); for (auto& p : lastMarkerPayloads_) cached.push_back(p.second); }
            { std::lock_guard<std::mutex> l(connectionMutex_); if (connections_.empty()) subscribeStreamingTopics(); connections_.push_back(ws); }
            for (auto& p : cached) ws->send(p, uWS::OpCode::TEXT);
        };
        behavior.message = [this](auto* ws, std::string_view msg, uWS::OpCode op) {
            if (op != uWS::OpCode::TEXT) return;
            json in = json::parse(msg, nullptr, false); if (in.is_discarded()) return;
            std::string type = in.value("type", ""), tag = in.value("tag", "");
            if (type == "stream.graph.delete" && !tag.empty()) {
                { std::lock_guard<std::mutex> l(graphMutex_); lastGraphPayloads_.erase(tag); }
                broadcastText(json({{"type", "stream.graph.delete"}, {"tag", tag}}).dump()); return;
            }
            if (type == "request.state") {
                std::lock_guard<std::mutex> l1(graphMutex_); std::lock_guard<std::mutex> l2(robotMutex_); std::lock_guard<std::mutex> l3(markerMutex_);
                for (auto& p : lastGraphPayloads_) ws->send(p.second, uWS::OpCode::TEXT);
                for (auto& p : lastRobotDescriptions_) ws->send(p.second, uWS::OpCode::TEXT);
                for (auto& p : lastMarkerPayloads_) ws->send(p.second, uWS::OpCode::TEXT);
                return;
            }
            handleWsRequest(ws, std::string(msg), in);
        };
        behavior.close = [this](auto* ws, int, std::string_view) {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            connections_.erase(std::remove(connections_.begin(), connections_.end(), ws), connections_.end());
            if (connections_.empty()) unsubscribeStreamingTopics();
        };
        uWS::App().get("/*", [this](auto* res, auto* req) { meshServer_.handle(res, req); })
            .ws<PerSocketData>("/*", std::move(behavior))
            .listen(port, [this, port](auto* s) { if (s) { listenSocket_ = s; RCLCPP_INFO(get_logger(), "WS listening on %d", port); } })
            .run();
    }

    void handleWsRequest(WebSocket* ws, const std::string& text, const json& req) {
        std::string id = req.value("id", ""), method = req.value("method", "");
        if (method == "sources.setActive") {
            handleSourcesSetActive(id, req.value("params", json::object()));
            return;
        }
        auto it = rpcPubs_.find(method.substr(0, method.find('.')));
        if (it == rpcPubs_.end()) { ws->send(viewer_internal::makeErrorResponse(id, "METHOD_NOT_FOUND", "No route"), uWS::OpCode::TEXT); return; }
        std::thread([this, ws, id, text, pub = it->second]() {
            std::string resp = rpc_.call(id, text, pub, 30000);
            loop_->defer([this, ws, resp]() {
                std::lock_guard<std::mutex> lock(connectionMutex_);
                if (std::find(connections_.begin(), connections_.end(), ws) != connections_.end()) ws->send(resp, uWS::OpCode::TEXT);
            });
        }).detach();
    }

    void handleSourcesSetActive(const std::string& id, const json& params) {
        std::string sourceId = params.value("sourceId", "");
        bool active = params.value("active", false);
        if (sourceId.empty()) return;
        if (sourceId.front() != '/') sourceId = "/" + sourceId;

        std::lock_guard<std::mutex> lock(connectionMutex_);
        if (active) {
            if (activeDynamicSubs_.count(sourceId)) return;
            // Topic type discovery would normally happen via source_node rpc,
            // here we just subscribe based on name/prefix for simplicity or implement a quick lookup
            auto topicNamesAndTypes = this->get_topic_names_and_types();
            if (topicNamesAndTypes.count(sourceId)) {
                const auto& types = topicNamesAndTypes[sourceId];
                if (std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end()) {
                    activeDynamicSubs_[sourceId] = create_subscription<sensor_msgs::msg::PointCloud2>(sourceId, 10, [this, sourceId](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                        this->broadcastPointCloud(sourceId, utils::convertToProtocolMessage(utils::convertFromRosMsg(msg)).serialize());
                    });
                } else if (std::find(types.begin(), types.end(), "ais_gng_msgs/msg/TopologicalMap") != types.end()) {
                    activeDynamicSubs_[sourceId] = create_subscription<ais_gng_msgs::msg::TopologicalMap>(sourceId, rclcpp::QoS(10).reliable().transient_local(), [this, sourceId](const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
                        broadcastText(converter::to_json(msg, sourceId).dump());
                    });
                } else if (std::find(types.begin(), types.end(), "visualization_msgs/msg/MarkerArray") != types.end()) {
                    activeDynamicSubs_[sourceId] = create_subscription<visualization_msgs::msg::MarkerArray>(sourceId, rclcpp::QoS(10).reliable().transient_local(), [this, sourceId](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                        broadcastText(converter::marker_array_to_json(msg, sourceId).dump());
                    });
                }
            }
        } else {
            if (activeDynamicSubs_.count(sourceId)) {
                // Determine type before erasing to send correct delete message
                auto topicNamesAndTypes = this->get_topic_names_and_types();
                if (topicNamesAndTypes.count(sourceId)) {
                    const auto& types = topicNamesAndTypes[sourceId];
                    if (std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end()) {
                        broadcastText(json({{"type", "stream.pointcloud.delete"}, {"topic", sourceId}}).dump());
                    } else if (std::find(types.begin(), types.end(), "ais_gng_msgs/msg/TopologicalMap") != types.end()) {
                        broadcastText(json({{"type", "stream.graph.delete"}, {"tag", sourceId}}).dump());
                    } else if (std::find(types.begin(), types.end(), "visualization_msgs/msg/MarkerArray") != types.end()) {
                        broadcastText(json({{"type", "stream.marker_array.delete"}, {"tag", sourceId}}).dump());
                    }
                }
                activeDynamicSubs_.erase(sourceId);
            }
        }
        // Notify source node of the change so its state stays in sync
        json syncReq = {{"method", "sources.setActive"}, {"params", params}};
        std_msgs::msg::String msg; msg.data = syncReq.dump();
        rpcPubs_["sources"]->publish(msg);

        // Send response back to GUI to confirm the state change
        json result = {{"success", true}, {"sourceId", sourceId}, {"active", active}};
        broadcastText(viewer_internal::makeOkResponse(id, result));
    }

    void subscribeStreamingTopics() {
        std::string base = viewer_internal::topics::kStreamRobot;
        robotDescSub_ = create_subscription<std_msgs::msg::String>(base + "/description", rclcpp::QoS(1).reliable().transient_local(), [this](const std_msgs::msg::String::SharedPtr msg) { handleRobotArm(msg); });
        robotPoseSub_ = create_subscription<std_msgs::msg::String>(base + "/pose", rclcpp::QoS(1).best_effort(), [this](const std_msgs::msg::String::SharedPtr msg) { handleRobotArm(msg); });
        jobEventSub_ = create_subscription<std_msgs::msg::String>(viewer_internal::topics::kEditJobEvents, 100, [this](const std_msgs::msg::String::SharedPtr msg) { broadcastText(msg->data); });
    }

    void unsubscribeStreamingTopics() { robotDescSub_.reset(); robotPoseSub_.reset(); jobEventSub_.reset(); }

    void handleRobotArm(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.empty()) return;
        json j = json::parse(msg->data, nullptr, false);
        if (!j.is_discarded() && j.value("type", "") == "stream.robot.description") {
            std::lock_guard<std::mutex> lock(robotMutex_); lastRobotDescriptions_[j.value("tag", "default")] = msg->data;
        }
        broadcastText(msg->data);
    }

    void handleMarkerArray(const std::string& payload) {
        if (payload.empty()) {
            return;
        }

        json j = json::parse(payload, nullptr, false);
        if (!j.is_discarded()) {
            std::string type = j.value("type", "");
            std::string tag = j.value("tag", "default");
            if (type == "stream.marker_array") {
                std::lock_guard<std::mutex> lock(markerMutex_);
                lastMarkerPayloads_[tag] = payload;
            } else if (type == "stream.graph") {
                std::lock_guard<std::mutex> lock(graphMutex_);
                lastGraphPayloads_[tag] = payload;
            }
        }
        broadcastText(payload);
    }

    void checkLiveness() {
        const std::string descTopic = std::string(viewer_internal::topics::kStreamRobot) + "/description";
        if (this->get_publishers_info_by_topic(descTopic).empty()) {
            std::lock_guard<std::mutex> lock(robotMutex_);
            for (auto const& [tag, _] : lastRobotDescriptions_) broadcastText(json({{"type", "stream.robot.delete"}, {"tag", tag}}).dump());
            lastRobotDescriptions_.clear();
        }
    }

    void broadcastPointCloud(const std::string& topic, const std::vector<uint8_t>& data) {
        auto s = std::make_shared<std::vector<uint8_t>>(data);
        auto meta = std::make_shared<std::string>(json({{"type", "stream.pointcloud.meta"}, {"topic", topic}}).dump());
        loop_->defer([this, meta, s]() {
            std::lock_guard<std::mutex> l(connectionMutex_);
            for (auto* ws : connections_) {
                ws->send(*meta, uWS::OpCode::TEXT);
                ws->send(std::string_view(reinterpret_cast<const char*>(s->data()), s->size()), uWS::OpCode::BINARY);
            }
        });
    }

    void broadcastBinary(const std::vector<uint8_t>& payload) {
        auto s = std::make_shared<std::vector<uint8_t>>(payload);
        loop_->defer([this, s]() {
            std::lock_guard<std::mutex> l(connectionMutex_);
            for (auto* ws : connections_) ws->send(std::string_view(reinterpret_cast<const char*>(s->data()), s->size()), uWS::OpCode::BINARY);
        });
    }

    void broadcastText(const std::string& payload) {
        auto s = std::make_shared<std::string>(payload);
        loop_->defer([this, s]() { std::lock_guard<std::mutex> l(connectionMutex_); for (auto* ws : connections_) ws->send(*s, uWS::OpCode::TEXT); });
    }

    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> rpcPubs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcResponseSub_, jobEventSub_, robotDescSub_, robotPoseSub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> activeDynamicSubs_;
    std::chrono::steady_clock::time_point lastTfTime_;
    std::mutex connectionMutex_, graphMutex_, robotMutex_, markerMutex_;
    std::vector<WebSocket*> connections_;
    std::unordered_map<std::string, std::string> lastGraphPayloads_, lastRobotDescriptions_, lastMarkerPayloads_;
    RpcManager rpc_; MeshServer meshServer_;
    std::thread serverThread_; us_listen_socket_t* listenSocket_ = nullptr;
    std::atomic<bool> serverRunning_{false}; uWS::Loop* loop_ = nullptr;
    rclcpp::TimerBase::SharedPtr livenessTimer_;
};
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerWsGatewayNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node); executor.spin(); executor.remove_node(node);
    rclcpp::shutdown(); return 0;
}
