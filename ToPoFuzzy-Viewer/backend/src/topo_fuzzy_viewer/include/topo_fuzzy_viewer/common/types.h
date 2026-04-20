#pragma once
#include <vector>
#include <string>

// Core types independent of ROS2 headers

namespace core {

struct Vector3 {
    double x, y, z;
};

struct Quaternion {
    double x, y, z, w;
};

struct GngNode {
    Vector3 pos;
    Vector3 normal;
    int32_t label;
    float age;
};

struct GngCluster {
    int32_t id;
    int32_t label;
    Vector3 pos;
    Vector3 scale;
    Quaternion quat;
    int32_t match;
    float reliability;
    Vector3 velocity;
    std::vector<int32_t> nodeIds;  // IDs of nodes belonging to this cluster
};

struct TopologicalMap {
    uint32_t timestamp;
    std::vector<GngNode> nodes;
    std::vector<int32_t> edges;
    std::vector<GngCluster> clusters;
};

} // namespace core
