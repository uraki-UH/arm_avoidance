#include "ais_gng/topological_plane/surface_model_tracking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace fuzzrobo::surface_model
{
void tracker::clear()
{
  tracks_.clear();
  frame_id_.clear();
  frame_number_=0;
  stamp_=0;
}

result tracker::update(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config,
  const retention_options &retention, std::size_t min_seed_plane_patches)
{
  const auto begin=std::chrono::steady_clock::now();
  if (!retention.enable_retention) { clear(); return extract(map,planes,config); }
  if (!std::isfinite(retention.max_point_residual) || retention.max_point_residual<=0 ||
    !std::isfinite(retention.max_rms) || retention.max_rms<=0 ||
    !(retention.max_normal_deg>0 && retention.max_normal_deg<90) ||
    !(retention.min_inlier_ratio>0 && retention.min_inlier_ratio<=1) ||
    !std::isfinite(retention.max_node_displacement) || retention.max_node_displacement<=0) {
    throw std::invalid_argument("invalid surface retention options");
  }
  const std::int64_t stamp=static_cast<std::int64_t>(map.header.stamp.sec)*1000000000+map.header.stamp.nanosec;
  if (map.nodes.empty() || map.header.frame_id!=frame_id_ || map.frame_number<frame_number_ || stamp<stamp_) clear();
  std::unordered_map<std::uint16_t,std::size_t> indices;
  bool has_duplicate_id=false;
  for (std::size_t i=0; i<map.nodes.size(); ++i) {
    if (!indices.emplace(map.nodes[i].id,i).second) has_duplicate_id=true;
  }
  // 重複IDでは対応付けを中止。現在フレームの新規抽出だけを出力。
  if (has_duplicate_id) { clear(); return extract(map,planes,config); }

  const double min_cos=std::cos(retention.max_normal_deg*3.14159265358979323846/180.0);
  std::vector<region> proposals;
  std::set<std::size_t> claimed;
  for (const auto &previous:tracks_) {
    region candidate=previous.surface;
    candidate.node_indices.clear();
    candidate.patch_indices.clear();
    candidate.is_retained=true;
    double error_sum=0;
    for (const auto &reference:previous.reference) {
      const auto found=indices.find(reference.id);
      if (found==indices.end() || claimed.count(found->second)) continue;
      const auto &node=map.nodes[found->second];
      const Eigen::Vector3d p(node.pos.x,node.pos.y,node.pos.z);
      if (!p.allFinite() || (p-reference.position).norm()>retention.max_node_displacement) continue;
      const auto error=model_dev(candidate.shape,node);
      if (error.dist>retention.max_point_residual || error.normal_cos<min_cos) continue;
      candidate.node_indices.push_back(found->second);
      error_sum+=error.dist*error.dist;
    }
    candidate.rejected_node_num=previous.reference.size()-candidate.node_indices.size();
    if (candidate.node_indices.size()<config.min_fit_nodes ||
      candidate.node_indices.size()<retention.min_inlier_ratio*previous.reference.size()) continue;
    const double rms=std::sqrt(error_sum/candidate.node_indices.size());
    if (rms>retention.max_rms) continue;
    candidate.shape.score+=rms*rms-candidate.shape.rms*candidate.shape.rms;
    candidate.shape.rms=rms;
    claimed.insert(candidate.node_indices.begin(),candidate.node_indices.end());
    proposals.push_back(std::move(candidate));
  }
  if (!proposals.empty()) {
    std::vector<bool> is_blocked(map.nodes.size(),false);
    for (const auto &previous:tracks_) for (const auto &reference:previous.reference) {
      const auto found=indices.find(reference.id);
      if (found!=indices.end()) is_blocked[found->second]=true;
    }
    for (const auto &plane:planes.clusters) for (auto idx:plane.node_indices) {
      if (idx<map.nodes.size()) is_blocked[idx]=true;
    }
    std::vector<std::vector<std::uint32_t>> neighbors(map.nodes.size());
    for (std::size_t i=0; i+1<map.edges.size(); i+=2) {
      const auto a=map.edges[i],b=map.edges[i+1];
      if (a>=map.nodes.size() || b>=map.nodes.size() || (is_blocked[a] && is_blocked[b])) continue;
      const auto &pa=map.nodes[a].pos;
      const auto &pb=map.nodes[b].pos;
      const Eigen::Vector3d delta(pa.x-pb.x,pa.y-pb.y,pa.z-pb.z);
      if (!delta.allFinite() || delta.squaredNorm()>config.max_link_length*config.max_link_length) continue;
      neighbors[a].push_back(b); neighbors[b].push_back(a);
    }
    // 新規所属・維持の両方を満たす距離と法線条件。追加後RMSの許容範囲も維持。
    const double min_growth_cos=std::max(min_cos,std::cos(config.max_normal_deg*3.14159265358979323846/180.0));
    const double max_growth_dist=std::min({config.max_patch_rms,config.max_point_residual,
      retention.max_point_residual,retention.max_rms});
    for (auto &candidate:proposals) {
      const double old_rms=candidate.shape.rms;
      double error_sum=old_rms*old_rms*candidate.node_indices.size();
      std::vector<bool> has_visited(map.nodes.size(),false);
      // 成立済みの追跡核からの非平面ノード成長。逸脱した参照IDの再取込みなし。
      for (std::size_t i=0; i<candidate.node_indices.size(); ++i) {
        for (auto idx:neighbors[candidate.node_indices[i]]) {
          if (is_blocked[idx] || has_visited[idx]) continue;
          has_visited[idx]=true;
          const auto dev=model_dev(candidate.shape,map.nodes[idx]);
          if (!std::isfinite(dev.dist) || dev.dist>max_growth_dist || dev.normal_cos<min_growth_cos) continue;
          candidate.node_indices.push_back(idx);
          is_blocked[idx]=true;
          error_sum+=dev.dist*dev.dist;
        }
      }
      // 追加点は当該フレームの所属のみ。追跡成立の参照集合・支持率の分母は維持。
      candidate.shape.rms=std::sqrt(error_sum/candidate.node_indices.size());
      candidate.shape.score+=candidate.shape.rms*candidate.shape.rms-old_rms*old_rms;
    }
  }
  const double retention_ms=std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-begin).count();
  auto out=extract(map,planes,config,proposals);
  std::vector<track> next;
  for (auto &surface:out.regions) {
    if (surface.shape.type=="unknown" || surface.shape.type=="plane") continue;
    const auto previous=std::find_if(tracks_.begin(),tracks_.end(),[&](const auto &entry) {
      return entry.surface.id==surface.id;
    });
    track entry;
    if (surface.is_retained && previous!=tracks_.end()) {
      entry=*previous;
      std::set<std::uint16_t> accepted;
      for (auto idx:surface.node_indices) accepted.insert(map.nodes[idx].id);
      // 外れたノードの再検証候補を保持。表示する座標・所属は当該フレームの適合点だけ。
      for (auto &reference:entry.reference) if (accepted.count(reference.id)) {
        const auto &p=map.nodes[indices.at(reference.id)].pos;
        reference.position=Eigen::Vector3d(p.x,p.y,p.z);
      }
    } else {
      const auto plane_num=plane_patch_num(out,surface);
      if (plane_num<min_seed_plane_patches) continue;
      if (next_id_>=static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        throw std::overflow_error("surface tracking ID exhausted");
      }
      surface.id=next_id_++;
      surface.seed_plane_patch_num=plane_num;
      for (auto idx:surface.node_indices) {
        const auto &node=map.nodes[idx];
        entry.reference.push_back({node.id,Eigen::Vector3d(node.pos.x,node.pos.y,node.pos.z)});
      }
    }
    entry.surface=surface;
    next.push_back(std::move(entry));
  }
  tracks_=std::move(next);
  frame_id_=map.header.frame_id;
  frame_number_=map.frame_number;
  stamp_=stamp;
  out.retention_ms=retention_ms;
  out.update_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  return out;
}
}  // namespace fuzzrobo::surface_model
