#pragma once

#include <ais_gng_msgs/msg/topological_map.hpp>

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace topological_map_protocol {

constexpr std::uint32_t kMagic = 0x31474d54U;
constexpr std::uint16_t kVersion = 1U;
constexpr std::size_t kMaxPacketBytes = 64U * 1024U * 1024U;

#pragma pack(push, 1)
struct Header {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t reserved;
    std::uint32_t tag_size;
    std::uint32_t frame_id_size;
    std::uint32_t timestamp_sec;
    std::uint32_t node_num;
    std::uint32_t edge_num;
    std::uint32_t cluster_num;
    std::uint32_t payload_size;
};

struct NodeRecord {
    std::uint16_t id;
    std::uint8_t label;
    std::uint8_t semantic_label;
    std::uint8_t is_goal;
    // 既存予約領域の1バイトによるGNG側の境界候補属性
    std::uint8_t is_boundary_candidate;
    std::uint8_t boundary_evidence;
    std::uint8_t reserved;
    std::uint32_t age;
    std::uint32_t nonplane_component_id;
    std::uint32_t winner_point_count;
    float semantic_reliability;
    float pos[3];
    float normal[3];
    float winner_point_covariance[9];
};

struct ClusterRecord {
    std::uint32_t id;
    std::uint8_t label;
    std::uint8_t semantic_label;
    std::uint16_t reserved;
    float semantic_reliability;
    float pos[3];
    float scale[3];
    float quat[4];
    float match;
    float reliability;
    float velocity[3];
    std::uint32_t age;
    std::uint32_t node_num;
};
#pragma pack(pop)

static_assert(sizeof(Header) == 36U);
static_assert(sizeof(NodeRecord) == 84U);
static_assert(sizeof(ClusterRecord) == 80U);

inline void append_bytes(
    std::vector<std::uint8_t>& buffer, const void* data, const std::size_t size)
{
    if (size == 0U) return;
    const auto* begin = static_cast<const std::uint8_t*>(data);
    buffer.insert(buffer.end(), begin, begin + size);
}

template <typename Value>
inline void append_value(std::vector<std::uint8_t>& buffer, const Value& value)
{
    append_bytes(buffer, &value, sizeof(Value));
}

inline std::vector<std::uint8_t> serialize(
    const ais_gng_msgs::msg::TopologicalMap& map, const std::string& tag)
{
    const auto tag_size = tag.size();
    const auto frame_id_size = map.header.frame_id.size();
    if (tag_size > std::numeric_limits<std::uint32_t>::max() ||
        frame_id_size > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::length_error("topological map string size exceeds protocol capacity");
    }

    std::size_t payload_size = tag_size + frame_id_size +
        map.nodes.size() * sizeof(NodeRecord) +
        map.edges.size() * sizeof(std::uint16_t);
    for (const auto& cluster : map.clusters) {
        payload_size += sizeof(ClusterRecord) + cluster.nodes.size() * sizeof(std::uint16_t);
    }
    if (payload_size > kMaxPacketBytes ||
        payload_size > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::length_error("topological map payload exceeds protocol capacity");
    }

    Header header{};
    header.magic = kMagic;
    header.version = kVersion;
    header.tag_size = static_cast<std::uint32_t>(tag_size);
    header.frame_id_size = static_cast<std::uint32_t>(frame_id_size);
    header.timestamp_sec = map.header.stamp.sec;
    header.node_num = static_cast<std::uint32_t>(map.nodes.size());
    header.edge_num = static_cast<std::uint32_t>(map.edges.size());
    header.cluster_num = static_cast<std::uint32_t>(map.clusters.size());
    header.payload_size = static_cast<std::uint32_t>(payload_size);

    std::vector<std::uint8_t> buffer;
    buffer.reserve(sizeof(Header) + payload_size);
    append_value(buffer, header);
    append_bytes(buffer, tag.data(), tag_size);
    append_bytes(buffer, map.header.frame_id.data(), frame_id_size);

    for (const auto& node : map.nodes) {
        NodeRecord record{};
        record.id = node.id;
        record.label = node.label;
        record.semantic_label = node.semantic_label;
        record.is_goal = node.is_goal ? 1U : 0U;
        record.is_boundary_candidate = node.is_boundary_candidate ? 1U : 0U;
        record.boundary_evidence = node.boundary_evidence;
        record.age = map.frame_number >= node.frame ? map.frame_number - node.frame : 0U;
        record.nonplane_component_id = node.nonplane_component_id;
        record.winner_point_count = node.winner_point_count;
        record.semantic_reliability = node.semantic_reliability;
        record.pos[0] = node.pos.x; record.pos[1] = node.pos.y; record.pos[2] = node.pos.z;
        record.normal[0] = node.normal.x; record.normal[1] = node.normal.y; record.normal[2] = node.normal.z;
        std::memcpy(record.winner_point_covariance, node.winner_point_covariance.data(), sizeof(record.winner_point_covariance));
        append_value(buffer, record);
    }
    append_bytes(buffer, map.edges.data(), map.edges.size() * sizeof(std::uint16_t));

    for (const auto& cluster : map.clusters) {
        ClusterRecord record{};
        record.id = cluster.id;
        record.label = cluster.label;
        record.semantic_label = cluster.semantic_label;
        record.semantic_reliability = cluster.semantic_reliability;
        record.pos[0] = cluster.pos.x; record.pos[1] = cluster.pos.y; record.pos[2] = cluster.pos.z;
        record.scale[0] = cluster.scale.x; record.scale[1] = cluster.scale.y; record.scale[2] = cluster.scale.z;
        record.quat[0] = cluster.quat.x; record.quat[1] = cluster.quat.y; record.quat[2] = cluster.quat.z; record.quat[3] = cluster.quat.w;
        record.match = cluster.match;
        record.reliability = cluster.label_reliability;
        record.velocity[0] = cluster.velocity.x; record.velocity[1] = cluster.velocity.y; record.velocity[2] = cluster.velocity.z;
        record.age = map.frame_number >= cluster.frame ? map.frame_number - cluster.frame : 0U;
        record.node_num = static_cast<std::uint32_t>(cluster.nodes.size());
        append_value(buffer, record);
        append_bytes(buffer, cluster.nodes.data(), cluster.nodes.size() * sizeof(std::uint16_t));
    }
    return buffer;
}

}
