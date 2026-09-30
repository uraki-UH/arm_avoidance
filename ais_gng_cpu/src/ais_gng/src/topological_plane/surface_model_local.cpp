#include "ais_gng/topological_plane/surface_model_local.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <stdexcept>

namespace fuzzrobo::surface_model
{
namespace
{
Eigen::Vector3d position(const geometry_msgs::msg::Point32 &point)
{
  return {point.x,point.y,point.z};
}
}

// 平面核を持つ到達成分だけのモデル探索。対象外ノードは出力対象外。
result extract_plane_local(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config,
  const std::vector<region> &retained, patch_history *history)
{
  const auto begin=std::chrono::steady_clock::now();
  const auto num=map.nodes.size();
  const auto invalid=std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> parent(num),sizes(num,1),plane_owner(num,invalid);
  std::vector<bool> is_finite(num,false);
  for (std::size_t idx=0;idx<num;++idx) {
    parent[idx]=idx;
    is_finite[idx]=position(map.nodes[idx].pos).allFinite();
  }
  const auto root=[&](std::size_t idx) {
    while (parent[idx]!=idx) {
      parent[idx]=parent[parent[idx]];
      idx=parent[idx];
    }
    return idx;
  };
  const auto join=[&](std::size_t a,std::size_t b) {
    a=root(a); b=root(b);
    if (a==b) return;
    if (sizes[a]<sizes[b]) std::swap(a,b);
    parent[b]=a; sizes[a]+=sizes[b];
  };
  std::vector<std::size_t> plane_seed(planes.clusters.size(),invalid);
  // 既存extractと同じ先着優先の平面所属。同一平面の全有効点を一体の候補として扱う構造。
  for (std::size_t plane_idx=0;plane_idx<planes.clusters.size();++plane_idx) {
    for (auto idx:planes.clusters[plane_idx].node_indices) {
      if (idx>=num || !is_finite[idx] || plane_owner[idx]!=invalid) continue;
      plane_owner[idx]=plane_idx;
      if (plane_seed[plane_idx]==invalid) plane_seed[plane_idx]=idx;
      else join(plane_seed[plane_idx],idx);
    }
  }
  const double max_link_length_squared=config.max_link_length*config.max_link_length;
  // 部分取り込み後の接平面変化も含めた到達可能性。法線による候補除外なし。
  for (std::size_t idx=0;idx+1<map.edges.size();idx+=2) {
    const auto a=map.edges[idx],b=map.edges[idx+1];
    if (a>=num || b>=num || !is_finite[a] || !is_finite[b]) continue;
    const double dist_squared=(position(map.nodes[a].pos)-position(map.nodes[b].pos)).squaredNorm();
    if (dist_squared<=max_link_length_squared) join(a,b);
  }
  std::vector<std::size_t> plane_num(num,0);
  std::vector<bool> is_active(num,false);
  for (auto idx:plane_seed) if (idx!=invalid) ++plane_num[root(idx)];
  for (std::size_t idx=0;idx<num;++idx)
    if (plane_num[idx]>=config.min_candidate_plane_patches) is_active[idx]=true;
  // 既存曲面の保持資格は現在の平面枚数から独立。保持候補と交差する全成分の保護。
  for (const auto &surface:retained) for (auto idx:surface.node_indices)
    if (idx<num && is_finite[idx]) is_active[root(idx)]=true;
  std::vector<std::size_t> selected(num,invalid);
  std::vector<std::uint32_t> original_nodes;
  original_nodes.reserve(num);
  for (std::size_t idx=0;idx<num;++idx) {
    if (!is_finite[idx] || !is_active[root(idx)]) continue;
    selected[idx]=original_nodes.size();
    original_nodes.push_back(static_cast<std::uint32_t>(idx));
  }
  options local_config=config;
  local_config.enable_plane_local_search=false;
  result out;
  const auto finish=[&](result &value) {
    value.has_candidate_filter=true;
    value.num_input_nodes=num;
    value.num_candidate_nodes=original_nodes.size();
    value.update_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  };
  if (original_nodes.empty()) {
    if (history) history->clear();
    finish(out);
    return out;
  }
  if (original_nodes.size()==num) {
    out=extract(map,planes,local_config,retained,history);
    finish(out);
    return out;
  }
  ais_gng_msgs::msg::TopologicalMap local_map;
  local_map.header=map.header; local_map.frame_number=map.frame_number;
  local_map.nodes.reserve(original_nodes.size());
  for (auto idx:original_nodes) local_map.nodes.push_back(map.nodes[idx]);
  local_map.edges.reserve(map.edges.size());
  // 候補選定後も元の全内部エッジを保持。長いエッジの表示・既存判定の互換性。
  for (std::size_t idx=0;idx+1<map.edges.size();idx+=2) {
    const auto a=map.edges[idx],b=map.edges[idx+1];
    if (a>=num || b>=num || selected[a]==invalid || selected[b]==invalid) continue;
    local_map.edges.push_back(static_cast<std::uint16_t>(selected[a]));
    local_map.edges.push_back(static_cast<std::uint16_t>(selected[b]));
  }
  ais_gng_msgs::msg::PlaneClusterArray local_planes;
  local_planes.header=planes.header; local_planes.frame_number=planes.frame_number;
  std::vector<std::size_t> original_planes;
  for (std::size_t plane_idx=0;plane_idx<planes.clusters.size();++plane_idx) {
    auto plane=planes.clusters[plane_idx];
    plane.node_indices.clear();
    for (auto idx:planes.clusters[plane_idx].node_indices) {
      if (idx<num && selected[idx]!=invalid)
        plane.node_indices.push_back(static_cast<std::uint32_t>(selected[idx]));
    }
    if (plane.node_indices.empty()) continue;
    local_planes.clusters.push_back(std::move(plane));
    original_planes.push_back(plane_idx);
  }
  std::vector<region> local_retained;
  std::map<std::uint32_t,std::uint32_t> original_ids;
  std::map<std::uint32_t,std::uint32_t> retained_ids;
  // 配列添字由来IDと永続IDの衝突回避。内部だけの一時IDと出力時の完全復元。
  for (const auto &surface:retained) {
    auto candidate=surface;
    candidate.node_indices.clear(); candidate.patch_indices.clear();
    for (auto idx:surface.node_indices)
      if (idx<num && selected[idx]!=invalid)
        candidate.node_indices.push_back(static_cast<std::uint32_t>(selected[idx]));
    if (candidate.node_indices.empty()) continue;
    auto found=retained_ids.find(surface.id);
    if (found==retained_ids.end()) {
      const std::size_t next_id=local_map.nodes.size()+retained_ids.size();
      if (next_id>=std::numeric_limits<std::uint32_t>::max())
        throw std::overflow_error("surface local retained ID exhausted");
      found=retained_ids.emplace(surface.id,static_cast<std::uint32_t>(next_id)).first;
      original_ids.emplace(found->second,surface.id);
    }
    candidate.id=found->second;
    local_retained.push_back(std::move(candidate));
  }
  out=extract(local_map,local_planes,local_config,local_retained,history);
  for (auto &patch:out.patches) {
    if (patch.plane_cluster_idx>=0)
      patch.plane_cluster_idx=static_cast<int>(original_planes.at(patch.plane_cluster_idx));
    for (auto &idx:patch.node_indices) idx=original_nodes.at(idx);
  }
  const auto original_id=[&](std::uint32_t id) {
    const auto found=original_ids.find(id);
    return found!=original_ids.end() ? found->second:original_nodes.at(id);
  };
  for (auto &surface:out.regions) {
    for (auto &idx:surface.node_indices) idx=original_nodes.at(idx);
    surface.id=original_id(surface.id);
    if (surface.support_parent_id!=std::numeric_limits<std::uint32_t>::max())
      surface.support_parent_id=original_id(surface.support_parent_id);
  }
  for (auto &idx:out.connected_edges) idx=static_cast<std::uint16_t>(original_nodes.at(idx));
  finish(out);
  return out;
}

}  // 名前空間 fuzzrobo::surface_model
