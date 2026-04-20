#pragma once

namespace viewer_internal::topics {

constexpr const char* kRpcSourceRequest = "/viewer/internal/rpc/source/request";
constexpr const char* kRpcFileRequest = "/viewer/internal/rpc/file/request";
constexpr const char* kRpcRosbagRequest = "/viewer/internal/rpc/rosbag/request";
constexpr const char* kRpcGngRequest = "/viewer/internal/rpc/gng/request";
constexpr const char* kRpcParamRequest = "/viewer/internal/rpc/param/request";
constexpr const char* kRpcEditRequest = "/viewer/internal/rpc/edit/request";

constexpr const char* kRpcResponse = "/viewer/internal/rpc/response";

constexpr const char* kStreamPointCloud = "/viewer/internal/stream/pointcloud";
constexpr const char* kStreamPointCloudMeta = "/viewer/internal/stream/pointcloud_meta";
constexpr const char* kStreamGraph = "/viewer/internal/stream/graph";

constexpr const char* kFileLoadedCloud = "/viewer/internal/file/loaded_cloud";

constexpr const char* kEditJobEvents = "/viewer/internal/events/job";

} // namespace viewer_internal::topics
