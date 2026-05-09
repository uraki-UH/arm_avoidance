#pragma once

namespace viewer_internal::topics {

constexpr const char* kRpcRequest = "/viewer/internal/rpc/request";

constexpr const char* kRpcResponse = "/viewer/internal/rpc/response";

constexpr const char* kStreamGraph = "/viewer/internal/stream/graph";
constexpr const char* kStreamRobot = "/viewer/internal/stream/robot";
constexpr const char* kStreamMarkerArray = "/viewer/internal/stream/marker_array";

constexpr const char* kFileLoadedCloud = "/viewer/internal/file/loaded_cloud";

constexpr const char* kEditJobEvents = "/viewer/internal/events/job";

} // namespace viewer_internal::topics
