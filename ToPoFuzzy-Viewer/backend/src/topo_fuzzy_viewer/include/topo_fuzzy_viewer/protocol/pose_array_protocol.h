#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <string>
#include <utility>

namespace pose_array_protocol {

// PoseArrayから描画用のローカルZ軸矢印への変換。ROS Markerトピックの生成なし
inline nlohmann::json serialize(
    const geometry_msgs::msg::PoseArray& poses, const std::string& tag)
{
    using json = nlohmann::json;
    json markers = json::array();
    for (std::size_t idx = 0; idx < poses.poses.size(); ++idx) {
        const auto& pose = poses.poses[idx];
        const auto& p = pose.position;
        const auto& q = pose.orientation;
        const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(norm) || norm < 1.0e-12) continue;
        const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
        constexpr double length = 0.08;
        markers.push_back({
            {"id", idx}, {"ns", "pose_array"}, {"type", "arrow"}, {"action", 0},
            {"frameId", poses.header.frame_id},
            {"header_stamp", {poses.header.stamp.sec, poses.header.stamp.nanosec}},
            {"pos", {0.0, 0.0, 0.0}}, {"quat", {0.0, 0.0, 0.0, 1.0}},
            {"scale", {0.008, 0.016, 0.02}}, {"color", {0.15, 0.8, 1.0, 1.0}},
            {"points", {{p.x, p.y, p.z},
                {p.x + length * 2.0 * (x * z + w * y),
                 p.y + length * 2.0 * (y * z - w * x),
                 p.z + length * (1.0 - 2.0 * (x * x + y * y))}}}
        });
    }
    // 毎回の全置換。空配列も送信して旧候補を消去
    return {{"type", "stream.marker_array"}, {"tag", tag},
            {"source_type", "pose_array"}, {"markers", std::move(markers)}};
}

}  // namespace pose_array_protocol
