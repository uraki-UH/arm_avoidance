#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/protocol.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
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
struct PendingResponse {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    std::string payload;
};

class ViewerWsGatewayNode : public rclcpp::Node {
public:
    ViewerWsGatewayNode() : Node("viewer_ws_gateway_node") {
        const int port = declare_parameter<int>("port", 9001);

        const std::vector<std::pair<std::string, std::string>> rpc_configs = {
            {"sources", viewer_internal::topics::kRpcSourceRequest},
            {"publish", viewer_internal::topics::kRpcSourceRequest},
            {"files",   viewer_internal::topics::kRpcFileRequest},
            {"rosbag",  viewer_internal::topics::kRpcRosbagRequest},
            {"gng",     viewer_internal::topics::kRpcGngRequest},
            {"params",  viewer_internal::topics::kRpcParamRequest},
            {"edit",    viewer_internal::topics::kRpcEditRequest}
        };
        for (const auto& [prefix, topic] : rpc_configs) {
            rpcPubs_[prefix] = create_publisher<std_msgs::msg::String>(topic, 50);
        }

        rpcResponseSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcResponse, 100, std::bind(&ViewerWsGatewayNode::handleRpcResponse, this, std::placeholders::_1));

        pointCloudMetaSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kStreamPointCloudMeta, 100, 
            [this](const std_msgs::msg::String::SharedPtr msg) { broadcastText(msg->data); });

        pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            viewer_internal::topics::kStreamPointCloud, rclcpp::QoS(10), 
            std::bind(&ViewerWsGatewayNode::handlePointCloud, this, std::placeholders::_1));

        auto gngTopics = declare_parameter<std::vector<std::string>>("gng_topics", {viewer_internal::topics::kStreamGraph});
        for (const auto& topic : gngTopics) {
            auto qos = rclcpp::QoS(1);
            if (topic.find("topological_map") != std::string::npos) qos.reliable().transient_local();
            graphSubs_.push_back(create_subscription<ais_gng_msgs::msg::TopologicalMap>(
                topic, qos, [this, topic](const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) { handleGraph(msg, topic); }));
        }

        tfSub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100, std::bind(&ViewerWsGatewayNode::handleTf, this, std::placeholders::_1));
        livenessTimer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ViewerWsGatewayNode::checkLiveness, this));

        startServer(port);
        RCLCPP_INFO(get_logger(), "Gateway initialized on port %d", port);
    }

    ~ViewerWsGatewayNode() override {
        stopServer();
        if (serverThread_.joinable()) serverThread_.join();
    }

private:
    void startServer(int port) {
        serverRunning_ = true;
        serverThread_ = std::thread([this, port]() { runServerLoop(port); });
    }

    void stopServer() {
        serverRunning_ = false;
        if (loop_) {
            loop_->defer([this]() {
                if (listenSocket_) { us_listen_socket_close(0, listenSocket_); listenSocket_ = nullptr; }
                std::lock_guard<std::mutex> lock(connectionMutex_);
                for (auto* ws : connections_) ws->close();
                connections_.clear();
            });
        }
    }

    void runServerLoop(int port) {
        loop_ = uWS::Loop::get();
        uWS::App::WebSocketBehavior<PerSocketData> behavior;
        behavior.maxPayloadLength = 64 * 1024 * 1024;
        behavior.open = [this](auto* ws) {
            std::vector<std::string> cached;
            { std::lock_guard<std::mutex> l(graphMutex_); for (auto& p : lastGraphPayloads_) cached.push_back(p.second); }
            { std::lock_guard<std::mutex> l(robotMutex_); for (auto& p : lastRobotDescriptions_) cached.push_back(p.second); }
            {
                std::lock_guard<std::mutex> l(connectionMutex_);
                if (connections_.empty()) subscribeStreamingTopics();
                connections_.push_back(ws);
            }
            for (auto& p : cached) ws->send(p, uWS::OpCode::TEXT);
        };
        behavior.message = [this](auto* ws, std::string_view msg, uWS::OpCode op) {
            if (op != uWS::OpCode::TEXT) return;
            json incoming = json::parse(msg, nullptr, false);
            if (incoming.is_discarded()) return;
            std::string type = incoming.value("type", ""), tag = incoming.value("tag", "");
            if (type == "stream.graph.delete" && !tag.empty()) { removeGraphLayer(tag); return; }
            if (type == "request.state") {
                std::vector<std::string> payloads;
                { std::lock_guard<std::mutex> l(graphMutex_); for (auto& p : lastGraphPayloads_) payloads.push_back(p.second); }
                { std::lock_guard<std::mutex> l(robotMutex_); for (auto& p : lastRobotDescriptions_) payloads.push_back(p.second); }
                for (auto& p : payloads) ws->send(p, uWS::OpCode::TEXT);
                return;
            }
            handleWsRequest(ws, std::string(msg));
        };
        behavior.close = [this](auto* ws, int, std::string_view) {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            connections_.erase(std::remove(connections_.begin(), connections_.end(), ws), connections_.end());
            if (connections_.empty()) unsubscribeStreamingTopics();
        };

        uWS::App().get("/*", [this](auto* res, auto* req) { handleHttpGet(res, req); })
            .ws<PerSocketData>("/*", std::move(behavior))
            .listen(port, [this, port](auto* s) { if (s) { listenSocket_ = s; RCLCPP_INFO(get_logger(), "WS listening on %d", port); } })
            .run();
    }

    void handleWsRequest(uWS::WebSocket<false, true, PerSocketData>* ws, const std::string& text) {
        json req = json::parse(text, nullptr, false);
        if (req.is_discarded()) return;
        std::string id = req.value("id", ""), method = req.value("method", "");
        auto it = rpcPubs_.find(method.substr(0, method.find('.')));
        auto pub = (it != rpcPubs_.end()) ? it->second : nullptr;
        if (!pub) { ws->send(viewer_internal::makeErrorResponse(id, "METHOD_NOT_FOUND", "Unsupported"), uWS::OpCode::TEXT); return; }

        std::thread([this, ws, id, text, pub]() {
            std::string resp = callInternalRpc(id, text, pub, 30000);
            loop_->defer([this, ws, resp]() {
                std::lock_guard<std::mutex> lock(connectionMutex_);
                if (std::find(connections_.begin(), connections_.end(), ws) != connections_.end()) ws->send(resp, uWS::OpCode::TEXT);
            });
        }).detach();
    }

    void subscribeStreamingTopics() {
        std::string base = viewer_internal::topics::kStreamRobot;
        robotDescSub_ = create_subscription<std_msgs::msg::String>(base + "/description", rclcpp::QoS(1).reliable().transient_local(), std::bind(&ViewerWsGatewayNode::handleRobotArm, this, std::placeholders::_1));
        robotPoseSub_ = create_subscription<std_msgs::msg::String>(base + "/pose", rclcpp::QoS(1).best_effort(), std::bind(&ViewerWsGatewayNode::handleRobotArm, this, std::placeholders::_1));
        jobEventSub_ = create_subscription<std_msgs::msg::String>(viewer_internal::topics::kEditJobEvents, 100, [this](const std_msgs::msg::String::SharedPtr msg) { broadcastText(msg->data); });
        RCLCPP_INFO(get_logger(), "Subscribed to streaming topics");
    }

    void unsubscribeStreamingTopics() {
        robotDescSub_.reset(); robotPoseSub_.reset(); jobEventSub_.reset();
        RCLCPP_INFO(get_logger(), "Unsubscribed (No active clients)");
    }

    std::string callInternalRpc(const std::string& id, const std::string& request, const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub, int timeoutMs) {
        auto pending = std::make_shared<PendingResponse>();
        { std::lock_guard<std::mutex> l(pendingMutex_); pendingResponses_[id] = pending; }
        pub->publish(viewer_internal::toStringMsg(request));
        std::unique_lock<std::mutex> lock(pending->mutex);
        bool ready = pending->cv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&]() { return pending->done; });
        { std::lock_guard<std::mutex> l(pendingMutex_); pendingResponses_.erase(id); }
        return ready ? pending->payload : viewer_internal::makeErrorResponse(id, "TIMEOUT", "Backend node timeout");
    }

    void handleRpcResponse(const std_msgs::msg::String::SharedPtr msg) {
        json resp = json::parse(msg->data, nullptr, false);
        std::string id = resp.value("id", "");
        if (id.empty()) return;
        std::shared_ptr<PendingResponse> pending;
        { std::lock_guard<std::mutex> l(pendingMutex_); auto it = pendingResponses_.find(id); if (it != pendingResponses_.end()) pending = it->second; }
        if (pending) { std::lock_guard<std::mutex> l(pending->mutex); pending->payload = msg->data; pending->done = true; pending->cv.notify_all(); }
    }

    void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        broadcastBinary(utils::convertToProtocolMessage(utils::convertFromRosMsg(msg)).serialize());
    }

    void handleGraph(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg, const std::string& tag) {
        json g;
        g["timestamp"] = msg->header.stamp.sec;
        g["tag"] = tag;
        g["mode"] = (tag.find("static") != std::string::npos) ? "static" : "dynamic";
        g["frameId"] = msg->header.frame_id;
        
        auto& nodes = g["nodes"] = json::array();
        for (auto& n : msg->nodes) nodes.push_back({{"x",n.pos.x},{"y",n.pos.y},{"z",n.pos.z},{"nx",n.normal.x},{"ny",n.normal.y},{"nz",n.normal.z},{"label",n.label},{"age",n.age}});
        g["edges"] = msg->edges;
        
        auto& clusters = g["clusters"] = json::array();
        for (auto& c : msg->clusters) {
            clusters.push_back({{"id",c.id},{"label",c.label},{"pos",{c.pos.x,c.pos.y,c.pos.z}},{"scale",{c.scale.x,c.scale.y,c.scale.z}},{"quat",{c.quat.x,c.quat.y,c.quat.z,c.quat.w}},{"match",c.match},{"reliability",c.reliability},{"velocity",{c.velocity.x,c.velocity.y,c.velocity.z}},{"nodeIds",c.nodes}});
        }

        json event = {{"type", "stream.graph"}, {"tag", tag}, {"graph", g}};
        std::string payload = event.dump();
        { std::lock_guard<std::mutex> l(graphMutex_); lastGraphPayloads_[tag] = payload; }
        broadcastText(payload);
    }

    void handleHttpGet(uWS::HttpResponse<false>* res, uWS::HttpRequest* req) {
        std::string url(req->getUrl());
        res->writeHeader("Access-Control-Allow-Origin", "*");
        if (url.rfind("/meshes/", 0) == 0) {
            std::string sub = url.substr(8); size_t slash = sub.find('/');
            if (slash != std::string::npos) {
                try {
                    auto path = std::filesystem::path(ament_index_cpp::get_package_share_directory(sub.substr(0, slash))) / sub.substr(slash + 1);
                    if (std::filesystem::exists(path) && !std::filesystem::is_directory(path)) {
                        std::ifstream f(path, std::ios::binary);
                        res->end(std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>()));
                        return;
                    }
                } catch (...) {}
            }
        }
        res->writeStatus("404 Not Found")->end("Not found");
    }

    void handleTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        json tfs = json::array();
        for (auto& ts : msg->transforms) tfs.push_back({{"frameId",ts.header.frame_id},{"childFrameId",ts.child_frame_id},{"pos",{ts.transform.translation.x,ts.transform.translation.y,ts.transform.translation.z}},{"quat",{ts.transform.rotation.x,ts.transform.rotation.y,ts.transform.rotation.z,ts.transform.rotation.w}}});
        broadcastText(json({{"type", "stream.tf"}, {"transforms", tfs}}).dump());
    }

    void handleRobotArm(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.empty()) return;
        json j = json::parse(msg->data, nullptr, false);
        if (!j.is_discarded() && j.value("type", "") == "stream.robot.description") {
            std::lock_guard<std::mutex> lock(robotMutex_);
            lastRobotDescriptions_[j.value("tag", "default")] = msg->data;
        }
        broadcastText(msg->data);
    }

    void checkLiveness() {
        const std::string descTopic = std::string(viewer_internal::topics::kStreamRobot) + "/description";
        if (this->get_publishers_info_by_topic(descTopic).empty()) {
            std::lock_guard<std::mutex> lock(robotMutex_);
            for (auto const& [tag, _] : lastRobotDescriptions_) {
                json ev = {{"type", "stream.robot.delete"}, {"tag", tag}};
                broadcastText(ev.dump());
            }
            lastRobotDescriptions_.clear();
        }
    }

    void broadcastBinary(const std::vector<uint8_t>& payload) {
        auto shared = std::make_shared<std::vector<uint8_t>>(payload);
        loop_->defer([this, shared]() {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            std::string_view view(reinterpret_cast<const char*>(shared->data()), shared->size());
            for (auto* ws : connections_) ws->send(view, uWS::OpCode::BINARY);
        });
    }

    void broadcastText(const std::string& payload) {
        auto shared = std::make_shared<std::string>(payload);
        loop_->defer([this, shared]() {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            for (auto* ws : connections_) ws->send(*shared, uWS::OpCode::TEXT);
        });
    }

    void removeGraphLayer(const std::string& tag) {
        { std::lock_guard<std::mutex> l(graphMutex_); lastGraphPayloads_.erase(tag); }
        broadcastText(json({{"type", "stream.graph.delete"}, {"tag", tag}}).dump());
    }

    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> rpcPubs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcResponseSub_, pointCloudMetaSub_, jobEventSub_, robotDescSub_, robotPoseSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_;
    std::vector<rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr> graphSubs_;
    std::mutex pendingMutex_, connectionMutex_, graphMutex_, robotMutex_;
    std::unordered_map<std::string, std::shared_ptr<PendingResponse>> pendingResponses_;
    std::vector<uWS::WebSocket<false, true, PerSocketData>*> connections_;
    std::unordered_map<std::string, std::string> lastGraphPayloads_, lastRobotDescriptions_;
    std::thread serverThread_;
    us_listen_socket_t* listenSocket_ = nullptr;
    std::atomic<bool> serverRunning_{false};
    uWS::Loop* loop_ = nullptr;
    rclcpp::TimerBase::SharedPtr livenessTimer_;
};
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerWsGatewayNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}
