#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <gng_control_msgs/msg/grasp_candidate_array.hpp>
#include <nlohmann/json.hpp>
#include <arrow_visualization/state_colors.hpp>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <map>
#include <string>

namespace arrow_protocol {
using json = nlohmann::json;

// 姿勢の意味情報を保持した共通入力。色・寸法・矢先位置の生成なし
inline json pose_entry(const geometry_msgs::msg::Pose &pose, const std::string &frame,
                       std::uint32_t id) {
    const auto &p = pose.position;
    const auto &q = pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < 1e-12) return nullptr;
    return {{"id", id}, {"ns", "pose_array"}, {"type", "arrow"}, {"action", 0},
            {"frameId", frame}, {"pos", {p.x, p.y, p.z}},
            {"orientation", {q.x / norm, q.y / norm, q.z / norm, q.w / norm}}};
}
inline json pose_stream(const geometry_msgs::msg::PoseArray &msg, const std::string &tag) {
    json markers = json::array();
    for (std::size_t idx = 0; idx < msg.poses.size(); ++idx) {
        auto entry = pose_entry(msg.poses[idx], msg.header.frame_id, idx);
        if (!entry.is_null()) markers.push_back(std::move(entry));
    }
    return {{"type", "stream.marker_array"}, {"tag", tag}, {"source_type", "pose_array"},
            {"markers", std::move(markers)}};
}
inline json pose_stream(const gng_control_msgs::msg::GraspCandidateArray &msg, const std::string &tag) {
    static const json state_palette = [] {
        json colors = json::object();
        for (std::size_t idx = 0; idx < arrow_visualization::state_colors_srgb.size(); ++idx) {
            std::ostringstream color;
            color << '#' << std::hex << std::setw(6) << std::setfill('0')
                  << arrow_visualization::state_colors_srgb[idx];
            colors[std::to_string(idx)] = color.str();
        }
        return json{{"candidate_state", {{"state_colors", colors}}}};
    }();
    json markers = json::array();
    for (const auto &candidate : msg.candidates) {
        auto entry = pose_entry(candidate.pose, msg.header.frame_id, candidate.id);
        if (!entry.is_null()) {
            entry["state"] = candidate.state;
            entry["arrow_style_id"] = "candidate_state";
            markers.push_back(std::move(entry));
        }
    }
    return {{"type", "stream.marker_array"}, {"tag", tag}, {"source_type", "pose_array"},
            {"update_id", msg.update_id}, {"arrow_styles", state_palette}, {"markers", std::move(markers)}};
}

// 標準ROS Markerの矢印スタイルを配列内で共有。辞書は変更時と再接続時だけ送信
inline json with_shared_styles(json payload) {
    json styles = payload.value("arrow_styles", json::object());
    std::map<std::string, std::string> ids;
    for (auto &marker : payload["markers"]) {
        if (marker.value("type", "") != "arrow" || marker.value("action", 0) >= 2) continue;
        if (!marker.contains("scale") && !marker.contains("color")) continue;
        json style = json::object();
        for (const auto *key : {"scale", "color"}) {
            if (marker.contains(key)) { style[key] = marker[key]; marker.erase(key); }
        }
        const auto signature = style.dump();
        auto it = ids.find(signature);
        if (it == ids.end()) {
            const auto id = std::to_string(ids.size());
            it = ids.emplace(signature, id).first;
            styles[id] = std::move(style);
        }
        marker["arrow_style_id"] = it->second;
    }
    payload["arrow_styles"] = std::move(styles);
    return payload;
}
}  // arrow_protocol 名前空間
