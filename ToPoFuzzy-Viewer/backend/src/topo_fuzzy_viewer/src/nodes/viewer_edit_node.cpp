#include "topo_fuzzy_viewer/protocol/rpc.h"
#include "topo_fuzzy_viewer/common/topic_names.h"
#include "topo_fuzzy_viewer/common/pcl_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct EditRegion {
    std::string regionId;
    std::array<float, 3> min;
    std::array<float, 3> max;
    std::string frameId = "map";
};

struct EditSession {
    std::string sessionId;
    std::string sourceTopic;
    std::string targetFrame = "map";
    utils::PointCloudData snapshot;
    std::string snapshotFrameId = "map";
    std::vector<EditRegion> regions;
    Clock::time_point createdAt;
    Clock::time_point updatedAt;
};

struct TopicSnapshot {
    std::mutex mutex;
    std::condition_variable cv;
    bool ready = false;
    utils::PointCloudData cloud;
    std::string frameId = "map";
};

struct UserTransform {
    std::array<float, 3> position{0.0f, 0.0f, 0.0f};
    std::array<float, 3> rotation{0.0f, 0.0f, 0.0f};
    std::array<float, 3> scale{1.0f, 1.0f, 1.0f};
};

std::string ensureLeadingSlash(std::string topic) {
    if (!topic.empty() && topic.front() != '/') {
        topic.insert(topic.begin(), '/');
    }
    return topic;
}

std::string defaultEditedTopic(std::string sourceTopic) {
    sourceTopic = ensureLeadingSlash(std::move(sourceTopic));
    constexpr const char* suffix = "/edited";
    constexpr size_t suffixLen = 7;
    while (sourceTopic.size() > suffixLen &&
           sourceTopic.compare(sourceTopic.size() - suffixLen, suffixLen, suffix) == 0) {
        sourceTopic.erase(sourceTopic.size() - suffixLen);
    }
    return sourceTopic + suffix;
}

bool parseVec3(const viewer_internal::json& value, std::array<float, 3>& out) {
    if (!value.is_array() || value.size() != 3) {
        return false;
    }

    for (size_t i = 0; i < 3; ++i) {
        if (!(value[i].is_number_float() || value[i].is_number_integer())) {
            return false;
        }
        out[i] = value[i].get<float>();
    }
    return true;
}

void normalizeBounds(std::array<float, 3>& minVal, std::array<float, 3>& maxVal) {
    for (size_t i = 0; i < 3; ++i) {
        if (minVal[i] > maxVal[i]) {
            std::swap(minVal[i], maxVal[i]);
        }
    }
}

bool pointInRegion(const float x, const float y, const float z, const EditRegion& region) {
    return x >= region.min[0] && x <= region.max[0] &&
           y >= region.min[1] && y <= region.max[1] &&
           z >= region.min[2] && z <= region.max[2];
}

} // namespace

class ViewerEditNode : public rclcpp::Node {
public:
    ViewerEditNode()
        : rclcpp::Node("viewer_edit_node")
        , tfBuffer_(get_clock())
        , tfListener_(tfBuffer_) {
        rpcRequestSub_ = create_subscription<std_msgs::msg::String>(
            viewer_internal::topics::kRpcEditRequest,
            50,
            std::bind(&ViewerEditNode::handleRpcRequest, this, std::placeholders::_1));

        rpcResponsePub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kRpcResponse, 50);
        jobEventPub_ = create_publisher<std_msgs::msg::String>(viewer_internal::topics::kEditJobEvents, 100);

        cleanupTimer_ = create_wall_timer(
            std::chrono::minutes(1),
            std::bind(&ViewerEditNode::cleanupStaleSessions, this));

        RCLCPP_INFO(get_logger(), "viewer_edit_node initialized");
    }

private:
    std::string makeSessionId() {
        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        return "edit-" + std::to_string(nowMs) + "-" + std::to_string(++sessionCounter_);
    }

    std::string makeRegionId() {
        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        return "region-" + std::to_string(nowMs) + "-" + std::to_string(++regionCounter_);
    }

    std::string makeJobId() {
        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        return "job-" + std::to_string(nowMs) + "-" + std::to_string(++jobCounter_);
    }

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

        if (method == "edit.openSession") {
            handleOpenSession(id, params);
            return;
        }
        if (method == "edit.addRegion") {
            handleAddRegion(id, params);
            return;
        }
        if (method == "edit.removeRegion") {
            handleRemoveRegion(id, params);
            return;
        }
        if (method == "edit.clearRegions") {
            handleClearRegions(id, params);
            return;
        }
        if (method == "edit.getSession") {
            handleGetSession(id, params);
            return;
        }
        if (method == "edit.commit") {
            handleCommit(id, params);
            return;
        }
        if (method == "edit.cancelSession") {
            handleCancelSession(id, params);
            return;
        }

        publishError(id, "METHOD_NOT_FOUND", "Unsupported edit method");
    }

    std::shared_ptr<TopicSnapshot> ensureSnapshotSubscription(const std::string& sourceTopic) {
        std::lock_guard<std::mutex> lock(topicMutex_);

        auto snapshotIt = topicSnapshots_.find(sourceTopic);
        if (snapshotIt != topicSnapshots_.end()) {
            return snapshotIt->second;
        }

        auto snapshot = std::make_shared<TopicSnapshot>();

        auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            sourceTopic,
            rclcpp::QoS(rclcpp::KeepLast(10)),
            [snapshot](const sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg) {
                auto converted = utils::convertFromRosMsg(cloudMsg);
                std::lock_guard<std::mutex> dataLock(snapshot->mutex);
                snapshot->cloud = std::move(converted);
                snapshot->frameId = cloudMsg->header.frame_id.empty() ? "map" : cloudMsg->header.frame_id;
                snapshot->ready = true;
                snapshot->cv.notify_all();
            });

        sourceSubscriptions_[sourceTopic] = sub;
        topicSnapshots_[sourceTopic] = snapshot;
        return snapshot;
    }

    void handleOpenSession(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sourceTopic") || !params["sourceTopic"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sourceTopic is required");
            return;
        }

        std::string sourceTopic = ensureLeadingSlash(params["sourceTopic"].get<std::string>());
        std::string targetFrame = "map";
        if (params.contains("targetFrame") && params["targetFrame"].is_string()) {
            targetFrame = params["targetFrame"].get<std::string>();
        }

        if (targetFrame != "map") {
            publishError(id, "INVALID_PARAMS", "targetFrame must be map");
            return;
        }

        auto snapshot = ensureSnapshotSubscription(sourceTopic);

        utils::PointCloudData snapshotData;
        std::string snapshotFrame = "map";
        {
            std::unique_lock<std::mutex> lock(snapshot->mutex);
            if (!snapshot->ready) {
                snapshot->cv.wait_for(lock, std::chrono::seconds(5), [&snapshot]() { return snapshot->ready; });
            }
            if (!snapshot->ready) {
                publishError(id, "SNAPSHOT_NOT_READY", "No source snapshot available yet. Retry after stream update.");
                return;
            }
            snapshotData = snapshot->cloud;
            snapshotFrame = snapshot->frameId;
        }

        EditSession session;
        session.sessionId = makeSessionId();
        session.sourceTopic = sourceTopic;
        session.targetFrame = targetFrame;
        session.snapshot = std::move(snapshotData);
        session.snapshotFrameId = snapshotFrame.empty() ? "map" : snapshotFrame;
        session.createdAt = Clock::now();
        session.updatedAt = session.createdAt;

        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            sessions_[session.sessionId] = session;
        }

        viewer_internal::json result;
        result["sessionId"] = session.sessionId;
        result["sourceTopic"] = session.sourceTopic;
        result["targetFrame"] = session.targetFrame;
        result["sourceFrameId"] = session.snapshotFrameId;
        result["pointCount"] = session.snapshot.pointCount;
        result["regions"] = viewer_internal::json::array();
        publishOk(id, result);
    }

    void handleAddRegion(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sessionId") || !params["sessionId"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sessionId is required");
            return;
        }
        if (!params.contains("min") || !params.contains("max")) {
            publishError(id, "INVALID_PARAMS", "min and max are required");
            return;
        }

        std::array<float, 3> minVal;
        std::array<float, 3> maxVal;
        if (!parseVec3(params["min"], minVal) || !parseVec3(params["max"], maxVal)) {
            publishError(id, "INVALID_PARAMS", "min/max must be [x,y,z]");
            return;
        }

        std::string frameId = "map";
        if (params.contains("frameId") && params["frameId"].is_string()) {
            frameId = params["frameId"].get<std::string>();
        }
        if (frameId != "map") {
            publishError(id, "INVALID_FRAME", "Only map frame is supported for edit regions");
            return;
        }

        normalizeBounds(minVal, maxVal);

        const std::string sessionId = params["sessionId"].get<std::string>();
        EditRegion region;
        region.regionId = makeRegionId();
        region.min = minVal;
        region.max = maxVal;
        region.frameId = frameId;

        size_t regionCount = 0;
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            auto it = sessions_.find(sessionId);
            if (it == sessions_.end()) {
                publishError(id, "SESSION_NOT_FOUND", "edit session not found");
                return;
            }
            it->second.regions.push_back(region);
            it->second.updatedAt = Clock::now();
            regionCount = it->second.regions.size();
        }

        viewer_internal::json regionJson;
        regionJson["regionId"] = region.regionId;
        regionJson["frameId"] = region.frameId;
        regionJson["min"] = {region.min[0], region.min[1], region.min[2]};
        regionJson["max"] = {region.max[0], region.max[1], region.max[2]};

        viewer_internal::json result;
        result["sessionId"] = sessionId;
        result["region"] = regionJson;
        result["regionCount"] = regionCount;
        publishOk(id, result);
    }

    void handleRemoveRegion(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sessionId") || !params["sessionId"].is_string() ||
            !params.contains("regionId") || !params["regionId"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sessionId and regionId are required");
            return;
        }

        const std::string sessionId = params["sessionId"].get<std::string>();
        const std::string regionId = params["regionId"].get<std::string>();

        bool removed = false;
        size_t regionCount = 0;
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            auto it = sessions_.find(sessionId);
            if (it == sessions_.end()) {
                publishError(id, "SESSION_NOT_FOUND", "edit session not found");
                return;
            }

            auto& regions = it->second.regions;
            auto before = regions.size();
            regions.erase(
                std::remove_if(regions.begin(), regions.end(), [&](const EditRegion& region) {
                    return region.regionId == regionId;
                }),
                regions.end());
            removed = regions.size() != before;
            regionCount = regions.size();
            it->second.updatedAt = Clock::now();
        }

        viewer_internal::json result;
        result["sessionId"] = sessionId;
        result["regionId"] = regionId;
        result["removed"] = removed;
        result["regionCount"] = regionCount;
        publishOk(id, result);
    }

    void handleClearRegions(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sessionId") || !params["sessionId"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sessionId is required");
            return;
        }

        const std::string sessionId = params["sessionId"].get<std::string>();
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            auto it = sessions_.find(sessionId);
            if (it == sessions_.end()) {
                publishError(id, "SESSION_NOT_FOUND", "edit session not found");
                return;
            }
            it->second.regions.clear();
            it->second.updatedAt = Clock::now();
        }

        viewer_internal::json result;
        result["sessionId"] = sessionId;
        result["regionCount"] = 0;
        publishOk(id, result);
    }

    void handleGetSession(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sessionId") || !params["sessionId"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sessionId is required");
            return;
        }

        const std::string sessionId = params["sessionId"].get<std::string>();
        EditSession session;
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            auto it = sessions_.find(sessionId);
            if (it == sessions_.end()) {
                publishError(id, "SESSION_NOT_FOUND", "edit session not found");
                return;
            }
            session = it->second;
        }

        viewer_internal::json regions = viewer_internal::json::array();
        for (const auto& region : session.regions) {
            regions.push_back({
                {"regionId", region.regionId},
                {"frameId", region.frameId},
                {"min", {region.min[0], region.min[1], region.min[2]}},
                {"max", {region.max[0], region.max[1], region.max[2]}}
            });
        }

        viewer_internal::json result;
        result["sessionId"] = session.sessionId;
        result["sourceTopic"] = session.sourceTopic;
        result["targetFrame"] = session.targetFrame;
        result["sourceFrameId"] = session.snapshotFrameId;
        result["pointCount"] = session.snapshot.pointCount;
        result["regions"] = regions;
        publishOk(id, result);
    }

    void handleCancelSession(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sessionId") || !params["sessionId"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sessionId is required");
            return;
        }

        const std::string sessionId = params["sessionId"].get<std::string>();

        bool removed = false;
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            removed = sessions_.erase(sessionId) > 0;
        }

        viewer_internal::json result;
        result["success"] = removed;
        result["sessionId"] = sessionId;
        publishOk(id, result);
    }

    void handleCommit(const std::string& id, const viewer_internal::json& params) {
        if (!params.contains("sessionId") || !params["sessionId"].is_string()) {
            publishError(id, "INVALID_PARAMS", "sessionId is required");
            return;
        }

        const std::string sessionId = params["sessionId"].get<std::string>();

        EditSession session;
        {
            std::lock_guard<std::mutex> lock(sessionMutex_);
            auto it = sessions_.find(sessionId);
            if (it == sessions_.end()) {
                publishError(id, "SESSION_NOT_FOUND", "edit session not found");
                return;
            }
            it->second.updatedAt = Clock::now();
            session = it->second;
        }

        UserTransform userTransform;
        if (params.contains("transform") && params["transform"].is_object()) {
            const auto& transform = params["transform"];
            if (transform.contains("position")) {
                parseVec3(transform["position"], userTransform.position);
            }
            if (transform.contains("rotation")) {
                parseVec3(transform["rotation"], userTransform.rotation);
            }
            if (transform.contains("scale")) {
                parseVec3(transform["scale"], userTransform.scale);
            }
        }

        std::string outputTopic;
        if (params.contains("outputTopic") && params["outputTopic"].is_string()) {
            outputTopic = params["outputTopic"].get<std::string>();
        }
        if (outputTopic.empty()) {
            outputTopic = defaultEditedTopic(session.sourceTopic);
        }
        outputTopic = ensureLeadingSlash(outputTopic);

        const std::string jobId = makeJobId();

        std::thread worker(
            [this, session, userTransform, outputTopic, jobId]() {
                this->runCommitJob(session, userTransform, outputTopic, jobId);
            });
        worker.detach();

        viewer_internal::json result;
        result["accepted"] = true;
        result["sessionId"] = sessionId;
        result["jobId"] = jobId;
        result["outputTopic"] = outputTopic;
        publishOk(id, result);
    }

    void runCommitJob(EditSession session,
                      UserTransform userTransform,
                      std::string outputTopic,
                      std::string jobId) {
        const auto startedAt = Clock::now();
        publishJobProgress(jobId, session.sessionId, 5, "starting");

        bool hasSourceToTarget = false;
        tf2::Transform sourceToTarget;

        if (!session.snapshotFrameId.empty() && session.snapshotFrameId != session.targetFrame) {
            publishJobProgress(jobId, session.sessionId, 10, "resolving_tf");
            try {
                auto tf = tfBuffer_.lookupTransform(
                    session.targetFrame,
                    session.snapshotFrameId,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.5));

                tf2::Quaternion q(
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w);
                tf2::Vector3 t(
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
                sourceToTarget = tf2::Transform(q, t);
                hasSourceToTarget = true;
            } catch (const std::exception& ex) {
                publishJobFailed(jobId, session.sessionId, "TF_UNAVAILABLE", ex.what());
                return;
            }
        }

        publishJobProgress(jobId, session.sessionId, 20, "filtering");

        const auto& positions = session.snapshot.positions;
        const auto& colors = session.snapshot.colors;

        const uint32_t pointCount = static_cast<uint32_t>(session.snapshot.pointCount);
        const bool hasColors = colors.size() == static_cast<size_t>(pointCount) * 3;

        std::vector<float> outPositions;
        std::vector<uint8_t> outColors;

        outPositions.reserve(static_cast<size_t>(pointCount) * 3);
        if (hasColors) {
            outColors.reserve(static_cast<size_t>(pointCount) * 3);
        }

        const float cx = std::cos(userTransform.rotation[0]);
        const float sx = std::sin(userTransform.rotation[0]);
        const float cy = std::cos(userTransform.rotation[1]);
        const float sy = std::sin(userTransform.rotation[1]);
        const float cz = std::cos(userTransform.rotation[2]);
        const float sz = std::sin(userTransform.rotation[2]);

        const float r00 = cy * cz * userTransform.scale[0];
        const float r01 = (sx * sy * cz - cx * sz) * userTransform.scale[1];
        const float r02 = (cx * sy * cz + sx * sz) * userTransform.scale[2];
        const float r10 = cy * sz * userTransform.scale[0];
        const float r11 = (sx * sy * sz + cx * cz) * userTransform.scale[1];
        const float r12 = (cx * sy * sz - sx * cz) * userTransform.scale[2];
        const float r20 = -sy * userTransform.scale[0];
        const float r21 = sx * cy * userTransform.scale[1];
        const float r22 = cx * cy * userTransform.scale[2];

        constexpr uint32_t kProgressStep = 25000;
        for (uint32_t i = 0; i < pointCount; ++i) {
            float x = positions[i * 3];
            float y = positions[i * 3 + 1];
            float z = positions[i * 3 + 2];

            if (hasSourceToTarget) {
                tf2::Vector3 p(x, y, z);
                tf2::Vector3 transformed = sourceToTarget * p;
                x = static_cast<float>(transformed.x());
                y = static_cast<float>(transformed.y());
                z = static_cast<float>(transformed.z());
            }

            bool inDeleteRegion = false;
            for (const auto& region : session.regions) {
                if (pointInRegion(x, y, z, region)) {
                    inDeleteRegion = true;
                    break;
                }
            }
            if (inDeleteRegion) {
                continue;
            }

            const float tx = r00 * x + r01 * y + r02 * z + userTransform.position[0];
            const float ty = r10 * x + r11 * y + r12 * z + userTransform.position[1];
            const float tz = r20 * x + r21 * y + r22 * z + userTransform.position[2];

            outPositions.push_back(tx);
            outPositions.push_back(ty);
            outPositions.push_back(tz);

            if (hasColors) {
                outColors.push_back(colors[i * 3]);
                outColors.push_back(colors[i * 3 + 1]);
                outColors.push_back(colors[i * 3 + 2]);
            }

            if ((i % kProgressStep) == 0) {
                const int progress = 20 + static_cast<int>((static_cast<double>(i) / std::max<uint32_t>(1, pointCount)) * 70.0);
                publishJobProgress(jobId, session.sessionId, progress, "processing");
            }
        }

        publishJobProgress(jobId, session.sessionId, 92, "publishing");

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
        {
            std::lock_guard<std::mutex> lock(publisherMutex_);
            auto it = editedPublishers_.find(outputTopic);
            if (it == editedPublishers_.end()) {
                rclcpp::QoS qos(rclcpp::KeepLast(1));
                qos.transient_local();
                publisher = create_publisher<sensor_msgs::msg::PointCloud2>(outputTopic, qos);
                editedPublishers_[outputTopic] = publisher;
            } else {
                publisher = it->second;
            }
        }

        auto rosMsg = utils::convertToRosMsg(outPositions, outColors, session.targetFrame, now());
        publisher->publish(rosMsg);

        publishJobProgress(jobId, session.sessionId, 100, "completed");

        const auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - startedAt).count();

        viewer_internal::json event;
        event["type"] = "job.completed";
        event["jobId"] = jobId;
        event["sessionId"] = session.sessionId;
        event["publishedTopic"] = outputTopic;
        event["pointCount"] = outPositions.size() / 3;
        event["durationMs"] = durationMs;
        event["message"] = "Edit commit finished";
        jobEventPub_->publish(viewer_internal::toStringMsg(event));
    }

    void publishJobProgress(const std::string& jobId,
                            const std::string& sessionId,
                            int progress,
                            const std::string& stage) {
        viewer_internal::json event;
        event["type"] = "job.progress";
        event["jobId"] = jobId;
        event["sessionId"] = sessionId;
        event["progress"] = progress;
        event["stage"] = stage;
        jobEventPub_->publish(viewer_internal::toStringMsg(event));
    }

    void publishJobFailed(const std::string& jobId,
                          const std::string& sessionId,
                          const std::string& code,
                          const std::string& message) {
        viewer_internal::json event;
        event["type"] = "job.failed";
        event["jobId"] = jobId;
        event["sessionId"] = sessionId;
        event["error"] = {
            {"code", code},
            {"message", message}
        };
        jobEventPub_->publish(viewer_internal::toStringMsg(event));
    }

    void cleanupStaleSessions() {
        const auto now = Clock::now();
        const auto ttl = std::chrono::hours(2);

        std::lock_guard<std::mutex> lock(sessionMutex_);
        for (auto it = sessions_.begin(); it != sessions_.end();) {
            if (now - it->second.updatedAt > ttl) {
                it = sessions_.erase(it);
            } else {
                ++it;
            }
        }
    }

    void publishOk(const std::string& id, const viewer_internal::json& result = viewer_internal::json::object()) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeOkResponse(id, result)));
    }

    void publishError(const std::string& id, const std::string& code, const std::string& message) {
        rpcResponsePub_->publish(viewer_internal::toStringMsg(viewer_internal::makeErrorResponse(id, code, message)));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rpcRequestSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpcResponsePub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jobEventPub_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    std::mutex topicMutex_;
    std::unordered_map<std::string, std::shared_ptr<TopicSnapshot>> topicSnapshots_;
    std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sourceSubscriptions_;

    std::mutex sessionMutex_;
    std::unordered_map<std::string, EditSession> sessions_;

    std::mutex publisherMutex_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> editedPublishers_;

    rclcpp::TimerBase::SharedPtr cleanupTimer_;

    std::atomic<uint64_t> sessionCounter_{0};
    std::atomic<uint64_t> regionCounter_{0};
    std::atomic<uint64_t> jobCounter_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ViewerEditNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}
