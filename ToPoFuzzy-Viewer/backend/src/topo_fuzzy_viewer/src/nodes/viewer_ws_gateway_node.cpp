#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/protocol/protocol.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>

#include <App.h>

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
#include <type_traits>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {

struct PerSocketData {};

struct PendingResponse {
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;
    std::string payload;
};

template <typename T, typename = void>
struct has_tag_member : std::false_type {};

template <typename T>
struct has_tag_member<T, std::void_t<decltype(std::declval<const T&>().tag)>> : std::true_type {};

template <typename T, typename = void>
struct has_mode_member : std::false_type {};

template <typename T>
struct has_mode_member<T, std::void_t<decltype(std::declval<const T&>().mode)>> : std::true_type {};

template <typename MsgT>
std::string extractGraphTag(const MsgT& msg, const std::string& fallbackTag) {
    if constexpr (has_tag_member<MsgT>::value) {
        return msg.tag.empty() ? fallbackTag : msg.tag;
    } else {
        return fallbackTag;
    }
}

template <typename MsgT>
std::string extractGraphMode(const MsgT& msg) {
    if constexpr (has_mode_member<MsgT>::value) {
        return (msg.mode == ais_gng_msgs::msg::TopologicalMap::STATIC) ? "static" : "dynamic";
    } else {
        return "dynamic";
    }
}

class ViewerWsGatewayNode : public rclcpp::Node {
public:
    ViewerWsGatewayNode()
        : rclcpp::Node("viewer_ws_gateway_node") {
        this->declare_parameter<int>("port", 9001);
        const int port = this->get_parameter("port").as_int();

        sourceRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcSourceRequest, 50);
        fileRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcFileRequest, 50);
        rosbagRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcRosbagRequest, 50);
        gngRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcGngRequest, 50);
        paramRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcParamRequest, 50);
        editRequestPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcEditRequest, 50);

        rpcResponseSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcResponse,
            100,
            std::bind(&ViewerWsGatewayNode::handleRpcResponse, this, std::placeholders::_1));

        pointCloudMetaSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kStreamPointCloudMeta,
            100,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->broadcastText(msg->data);
            });

        pointCloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            viewer_internal::topics::kStreamPointCloud,
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&ViewerWsGatewayNode::handlePointCloud, this, std::placeholders::_1));

        // Declare and get GNG topics
        this->declare_parameter<std::vector<std::string>>("gng_topics", {viewer_internal::topics::kStreamGraph});
        auto gngTopics = this->get_parameter("gng_topics").as_string_array();

        for (const auto& topic : gngTopics) {
            std::string subscriptionTag = topic;
            
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            if (topic == "/topological_map" || topic == "/topological_map_transformed") {
                qos.reliable().transient_local();
            }

            auto sub = create_subscription<ais_gng_msgs::msg::TopologicalMap>(
                topic,
                qos,
                [this, subscriptionTag](const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
                    this->handleGraph(msg, subscriptionTag);
                });
            graphSubs_.push_back(sub);
            RCLCPP_INFO(this->get_logger(), "Subscribed to GNG topic: %s (default tag: %s)", topic.c_str(), subscriptionTag.c_str());
        }

        robotArmSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kStreamRobotArm,
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&ViewerWsGatewayNode::handleRobotArm, this, std::placeholders::_1));

        jobEventSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kEditJobEvents,
            100,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->broadcastText(msg->data);
            });

        startServer(port);

        RCLCPP_INFO(get_logger(), "viewer_ws_gateway_node initialized on ws://0.0.0.0:%d", port);
    }

    ~ViewerWsGatewayNode() override {
        stopServer();
    }

private:
    void startServer(int port) {
        if (serverRunning_) {
            return;
        }
        serverRunning_ = true;
        serverThread_ = std::thread([this, port]() {
            this->runServerLoop(port);
        });
        serverThread_.detach();
    }

    void stopServer() {
        if (!serverRunning_) {
            return;
        }
        serverRunning_ = false;
        if (listenSocket_) {
            us_listen_socket_close(0, listenSocket_);
            listenSocket_ = nullptr;
        }
    }

    void runServerLoop(int port) {
        loop_ = uWS::Loop::get();
        uWS::App::WebSocketBehavior<PerSocketData> behavior;
        behavior.maxPayloadLength = 64 * 1024 * 1024;
        behavior.maxBackpressure = 16 * 1024 * 1024;
        behavior.open = [this](auto* ws) {
            std::vector<std::string> cachedGraphs;
            {
                std::lock_guard<std::mutex> graphLock(graphMutex_);
                cachedGraphs.reserve(lastGraphPayloads_.size());
                for (const auto& [tag, payload] : lastGraphPayloads_) {
                    (void)tag;
                    cachedGraphs.push_back(payload);
                }
            }
            std::lock_guard<std::mutex> lock(connectionMutex_);
            connections_.push_back(ws);
            for (const auto& payload : cachedGraphs) {
                ws->send(payload, uWS::OpCode::TEXT);
            }
        };
        behavior.message = [this](auto* ws, std::string_view message, uWS::OpCode opCode) {
            if (opCode != uWS::OpCode::TEXT) {
                return;
            }
            this->handleWsRequest(ws, std::string(message));
        };
        behavior.close = [this](auto* ws, int, std::string_view) {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            connections_.erase(
                std::remove(connections_.begin(), connections_.end(), ws),
                connections_.end());
        };

        uWS::App()
            .get("/*", [this](auto* res, auto* req) {
                this->handleHttpGet(res, req);
            })
            .ws<PerSocketData>("/*", std::move(behavior))
            .listen(port, [this, port](auto* socket) {
                if (socket) {
                    listenSocket_ = socket;
                    RCLCPP_INFO(this->get_logger(), "WebSocket listening on port %d", port);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to bind port %d", port);
                }
            })
            .run();
    }

    void handleWsRequest(uWS::WebSocket<false, true, PerSocketData>* ws, const std::string& text) {
        viewer_internal::json request;
        if (!viewer_internal::parseJson(text, request)) {
            ws->send(
                viewer_internal::makeErrorResponse("", "INVALID_JSON", "Failed to parse JSON request"),
                uWS::OpCode::TEXT);
            return;
        }

        const std::string id = viewer_internal::getId(request);
        const std::string method = viewer_internal::getMethod(request);
        if (id.empty() || method.empty()) {
            ws->send(
                viewer_internal::makeErrorResponse(id, "INVALID_REQUEST", "Missing id or method"),
                uWS::OpCode::TEXT);
            return;
        }

        auto publisher = routePublisher(method);
        if (!publisher) {
            ws->send(
                viewer_internal::makeErrorResponse(id, "METHOD_NOT_FOUND", "Method is not supported"),
                uWS::OpCode::TEXT);
            return;
        }

        const int timeoutMs = 30000;
        std::thread([this, ws, id, text, publisher, timeoutMs]() {
            const std::string response = callInternalRpc(id, text, publisher, timeoutMs);
            loop_->defer([this, ws, response]() {
                std::lock_guard<std::mutex> lock(connectionMutex_);
                auto it = std::find(connections_.begin(), connections_.end(), ws);
                if (it != connections_.end()) {
                    ws->send(response, uWS::OpCode::TEXT);
                }
            });
        }).detach();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr routePublisher(const std::string& method) {
        if (viewer_internal::startsWith(method, "sources.")) {
            return sourceRequestPub_;
        }
        if (viewer_internal::startsWith(method, "publish.")) {
            return sourceRequestPub_;
        }
        if (viewer_internal::startsWith(method, "files.")) {
            return fileRequestPub_;
        }
        if (viewer_internal::startsWith(method, "rosbag.")) {
            return rosbagRequestPub_;
        }
        if (viewer_internal::startsWith(method, "gng.")) {
            return gngRequestPub_;
        }
        if (viewer_internal::startsWith(method, "params.")) {
            return paramRequestPub_;
        }
        if (viewer_internal::startsWith(method, "edit.")) {
            return editRequestPub_;
        }
        return nullptr;
    }

    std::string callInternalRpc(
        const std::string& id,
        const std::string& request,
        const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher,
        int timeoutMs) {
        auto pending = std::make_shared<PendingResponse>();

        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pendingResponses_[id] = pending;
        }

        publisher->publish(viewer_internal::toStringMsg(request));

        std::unique_lock<std::mutex> lock(pending->mutex);
        const bool ready = pending->cv.wait_for(
            lock,
            std::chrono::milliseconds(timeoutMs),
            [&pending]() { return pending->done; });

        {
            std::lock_guard<std::mutex> pendingLock(pendingMutex_);
            pendingResponses_.erase(id);
        }

        if (!ready) {
            return viewer_internal::makeErrorResponse(id, "TIMEOUT", "Timed out waiting backend node response");
        }
        return pending->payload;
    }

    void handleRpcResponse(const std_msgs::msg::String::SharedPtr msg) {
        viewer_internal::json response;
        if (!viewer_internal::parseJson(msg->data, response)) {
            return;
        }

        const std::string id = viewer_internal::getId(response);
        if (id.empty()) {
            return;
        }

        std::shared_ptr<PendingResponse> pending;
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            auto it = pendingResponses_.find(id);
            if (it == pendingResponses_.end()) {
                return;
            }
            pending = it->second;
        }

        {
            std::lock_guard<std::mutex> lock(pending->mutex);
            pending->payload = msg->data;
            pending->done = true;
        }
        pending->cv.notify_all();
    }

    void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto converted = utils::convertFromRosMsg(msg);
        auto protocolMessage = utils::convertToProtocolMessage(converted);
        auto serialized = protocolMessage.serialize();
        broadcastBinary(serialized);
    }

    void handleGraph(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg, const std::string& subscriptionTag) {
        const std::string tag = extractGraphTag(*msg, subscriptionTag);
        const std::string mode = extractGraphMode(*msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Received graph topic tag=%s mode=%s frame_id=%s nodes=%zu edges=%zu clusters=%zu",
            tag.c_str(),
            mode.c_str(),
            msg->header.frame_id.c_str(),
            msg->nodes.size(),
            msg->edges.size() / 2,
            msg->clusters.size());

        viewer_internal::json graph;
        graph["timestamp"] = msg->header.stamp.sec;
        graph["tag"] = tag;
        graph["mode"] = mode;

        viewer_internal::json nodes = viewer_internal::json::array();
        for (const auto& node : msg->nodes) {
            nodes.push_back({
                {"x", node.pos.x},
                {"y", node.pos.y},
                {"z", node.pos.z},
                {"nx", node.normal.x},
                {"ny", node.normal.y},
                {"nz", node.normal.z},
                {"label", node.label},
                {"age", node.age}
            });
        }

        viewer_internal::json edges = viewer_internal::json::array();
        for (const auto edge : msg->edges) {
            edges.push_back(edge);
        }

        viewer_internal::json clusters = viewer_internal::json::array();
        for (const auto& cluster : msg->clusters) {
            viewer_internal::json nodeIds = viewer_internal::json::array();
            for (const auto nodeId : cluster.nodes) {
                nodeIds.push_back(nodeId);
            }
            clusters.push_back({
                {"id", cluster.id},
                {"label", cluster.label},
                {"pos", {cluster.pos.x, cluster.pos.y, cluster.pos.z}},
                {"scale", {cluster.scale.x, cluster.scale.y, cluster.scale.z}},
                {"quat", {cluster.quat.x, cluster.quat.y, cluster.quat.z, cluster.quat.w}},
                {"match", cluster.match},
                {"reliability", cluster.reliability},
                {"velocity", {cluster.velocity.x, cluster.velocity.y, cluster.velocity.z}},
                {"nodeIds", nodeIds}
            });
        }

        graph["nodes"] = nodes;
        graph["edges"] = edges;
        graph["clusters"] = clusters;
        graph["frameId"] = msg->header.frame_id;

        viewer_internal::json event;
        event["type"] = "stream.graph";
        event["tag"] = tag;
        event["graph"] = graph;
        const std::string payload = event.dump();
        {
            std::lock_guard<std::mutex> lock(graphMutex_);
            lastGraphPayloads_[tag] = payload;
        }
        broadcastText(payload);

        RCLCPP_INFO(
            this->get_logger(),
            "Broadcasted stream.graph tag=%s payload_bytes=%zu",
            tag.c_str(),
            payload.size());
    }

    void handleHttpGet(uWS::HttpResponse<false>* res, uWS::HttpRequest* req) {
        std::string url = std::string(req->getUrl());
        
        if (url.rfind("/meshes/", 0) == 0) {
            std::string subpath = url.substr(8); // remove "/meshes/"
            size_t slash_pos = subpath.find('/');
            if (slash_pos != std::string::npos) {
                std::string pkg_name = subpath.substr(0, slash_pos);
                std::string rel_path = subpath.substr(slash_pos + 1);
                
                try {
                    std::string pkg_share = ament_index_cpp::get_package_share_directory(pkg_name);
                    std::filesystem::path full_path = std::filesystem::path(pkg_share) / rel_path;
                    
                    if (std::filesystem::exists(full_path) && !std::filesystem::is_directory(full_path)) {
                        std::ifstream file(full_path, std::ios::binary);
                        if (file) {
                            std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                            res->end(content);
                            return;
                        }
                    }
                } catch (...) {}
            }
        }
        
        res->writeStatus("404 Not Found")->end("File not found");
    }

    void handleRobotArm(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.empty()) {
            return;
        }
        broadcastText(msg->data);
    }

    void broadcastBinary(const std::vector<uint8_t>& payload) {
        // Copy payload for deferred execution on uWS event loop thread
        auto shared = std::make_shared<std::vector<uint8_t>>(payload);
        loop_->defer([this, shared]() {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            std::string_view view(reinterpret_cast<const char*>(shared->data()), shared->size());
            for (auto* ws : connections_) {
                ws->send(view, uWS::OpCode::BINARY);
            }
        });
    }

    void broadcastText(const std::string& payload) {
        // Copy payload for deferred execution on uWS event loop thread
        auto shared = std::make_shared<std::string>(payload);
        loop_->defer([this, shared]() {
            std::lock_guard<std::mutex> lock(connectionMutex_);
            for (auto* ws : connections_) {
                ws->send(*shared, uWS::OpCode::TEXT);
            }
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sourceRequestPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fileRequestPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rosbagRequestPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gngRequestPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr paramRequestPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr editRequestPub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcResponseSub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pointCloudMetaSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub_;
    std::vector<rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr> graphSubs_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotArmSub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jobEventSub_;

    std::mutex pendingMutex_;
    std::unordered_map<std::string, std::shared_ptr<PendingResponse>> pendingResponses_;

    std::mutex connectionMutex_;
    std::vector<uWS::WebSocket<false, true, PerSocketData>*> connections_;

    std::mutex graphMutex_;
    std::unordered_map<std::string, std::string> lastGraphPayloads_;

    std::thread serverThread_;
    us_listen_socket_t* listenSocket_ = nullptr;
    std::atomic<bool> serverRunning_{false};
    uWS::Loop* loop_ = nullptr;
};

} // namespace

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
