#pragma once
#include <point_cloud_store.hpp>

namespace robot_sim::bridge
{
// 既存利用側の型名とAPIの維持。実体はROS・fuzzy非依存の共通索引。
using voxel_idx::world_bucket_key;
using voxel_idx::world_bucket_key_hash;
using voxel_idx::world_bucket_query_stats;
using voxel_idx::world_point_bucket_index;
}
