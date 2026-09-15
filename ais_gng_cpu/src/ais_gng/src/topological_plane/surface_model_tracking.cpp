#include "ais_gng/topological_plane/surface_model_tracking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace fuzzrobo::surface_model
{
void tracker::clear()
{
  tracks_.clear();
  smooth_nodes_.clear();
  smooth_links_.clear();
  frame_id_.clear();
  frame_number_=0;
  stamp_=0;
}

result tracker::update(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config,
  const retention_options &retention, std::size_t min_seed_plane_patches)
{
  if (config.method!="model" && config.method!="smooth_graph")
    throw std::invalid_argument("surface_model.method must be model or smooth_graph");
  if (config.method!=method_) { clear(); method_=config.method; }
  if (config.method=="smooth_graph") return update_smooth_graph(map,planes,config,retention);
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
    candidate.support_parent_id=std::numeric_limits<std::uint32_t>::max();
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
    const bool has_support_split=surface.support_parent_id!=std::numeric_limits<std::uint32_t>::max();
    const auto source_id=has_support_split ? surface.support_parent_id:surface.id;
    const auto previous=std::find_if(tracks_.begin(),tracks_.end(),[&](const auto &entry) {
      return entry.surface.id==source_id;
    });
    track entry;
    if (surface.is_retained && previous!=tracks_.end()) {
      entry=*previous;
      if (has_support_split) {
        // 分割後は各領域だけを参照集合に採用。離れた旧領域の再吸収の防止。
        entry.reference.clear();
        for (auto idx:surface.node_indices) {
          const auto &node=map.nodes[idx];
          entry.reference.push_back({node.id,Eigen::Vector3d(node.pos.x,node.pos.y,node.pos.z)});
        }
        surface.rejected_node_num=0;
        if (surface.id!=previous->surface.id) {
          if (next_id_>=static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
            throw std::overflow_error("surface tracking ID exhausted");
          surface.id=next_id_++;
        }
      }
      std::set<std::uint16_t> accepted;
      for (auto idx:surface.node_indices) accepted.insert(map.nodes[idx].id);
      // 外れたノードの再検証候補を保持。表示する座標・所属は当該フレームの適合点だけ。
      for (auto &reference:entry.reference) if (accepted.count(reference.id)) {
        const auto &p=map.nodes[indices.at(reference.id)].pos;
        reference.position=Eigen::Vector3d(p.x,p.y,p.z);
      }
    } else {
      const auto plane_num=plane_patch_num(out,surface);
      // 成立済み曲面の再統合では表示資格を継承し、係数・参照集合を新しい核へ更新。
      const bool has_retained_seed=surface.seed_plane_patch_num>0 &&
        surface.seed_plane_patch_num>=min_seed_plane_patches;
      if (plane_num<min_seed_plane_patches && !has_retained_seed) continue;
      if (next_id_>=static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        throw std::overflow_error("surface tracking ID exhausted");
      }
      surface.id=next_id_++;
      surface.seed_plane_patch_num=std::max(plane_num,surface.seed_plane_patch_num);
      surface.is_retained=has_retained_seed;
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
result tracker::update_smooth_graph(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config,
  const retention_options &retention)
{
  const auto begin=std::chrono::steady_clock::now();
  if (map.frame_number!=planes.frame_number || map.header.frame_id!=planes.header.frame_id)
    throw std::invalid_argument("surface models require matching map/plane frames");
  if (!std::isfinite(config.max_link_length) || config.max_link_length<=0 ||
    !(config.max_link_normal_deg>0 && config.max_link_normal_deg<90) ||
    !(config.max_link_tangent_deg>0 && config.max_link_tangent_deg<90) || config.min_fit_nodes==0 ||
    !std::isfinite(retention.max_node_displacement) || retention.max_node_displacement<=0)
    throw std::invalid_argument("invalid smooth_graph options");
  const std::int64_t stamp=static_cast<std::int64_t>(map.header.stamp.sec)*1000000000+map.header.stamp.nanosec;
  if (map.nodes.empty() || map.header.frame_id!=frame_id_ || map.frame_number<frame_number_ || stamp<stamp_) clear();
  result out;
  out.method="smooth_graph";
  if (!retention.enable_retention) clear();
  const auto num=map.nodes.size();
  const auto invalid=std::numeric_limits<std::uint32_t>::max();
  std::unordered_map<std::uint32_t,std::size_t> indices;
  indices.reserve(num);
  bool has_duplicate_id=false;
  for (std::size_t i=0; i<num; ++i)
    if (!indices.emplace(map.nodes[i].id,i).second) has_duplicate_id=true;
  // 重複IDではフレーム内だけの代替キー。履歴との対応付けの禁止。
  if (has_duplicate_id) { clear(); indices.clear(); }
  std::vector<std::uint32_t> keys(num), previous_regions(num,invalid);
  std::vector<smooth_node> nodes;
  nodes.reserve(num);
  std::vector<bool> has_geometry_change(num,true);
  std::set<std::uint32_t> dirty_regions;
  const std::array<double,3> limits={config.max_link_length,
    config.max_link_normal_deg,config.max_link_tangent_deg};
  for (std::size_t i=0; i<num; ++i) {
    keys[i]=has_duplicate_id ? 65536U+static_cast<std::uint32_t>(i):map.nodes[i].id;
    if (has_duplicate_id) indices.emplace(keys[i],i);
    const auto &input=map.nodes[i];
    smooth_node node{{input.pos.x,input.pos.y,input.pos.z},
      {input.normal.x,input.normal.y,input.normal.z},invalid};
    const double length=node.normal.norm();
    if (std::isfinite(length) && length>1e-8) node.normal/=length;
    else node.normal.setZero();
    const auto previous=smooth_nodes_.find(keys[i]);
    if (previous!=smooth_nodes_.end()) {
      const auto &old=previous->second;
      has_geometry_change[i]=limits!=smooth_limits_ || node.position!=old.position || node.normal!=old.normal;
      if (node.position.allFinite() && old.position.allFinite() &&
        (node.position-old.position).norm()<=retention.max_node_displacement)
        previous_regions[i]=old.region_id;
      else dirty_regions.insert(old.region_id);
    }
    nodes.push_back(std::move(node));
  }
  for (const auto &[key,node]:smooth_nodes_)
    if (!indices.count(key)) dirty_regions.insert(node.region_id);
  const auto dirty_key=[&](std::uint32_t key) {
    const auto found=smooth_nodes_.find(key);
    if (found!=smooth_nodes_.end()) dirty_regions.insert(found->second.region_id);
  };
  const auto dirty_link=[&](std::uint64_t key) {
    dirty_key(static_cast<std::uint32_t>(key>>32));
    dirty_key(static_cast<std::uint32_t>(key));
  };
  std::vector<std::pair<std::uint64_t,bool>> links;
  links.reserve(map.edges.size()/2);
  for (std::size_t i=0; i+1<map.edges.size(); i+=2) {
    const auto a=map.edges[i], b=map.edges[i+1];
    if (a>=num || b>=num || a==b) continue;
    links.emplace_back((static_cast<std::uint64_t>(std::min(keys[a],keys[b]))<<32)|
      std::max(keys[a],keys[b]),false);
  }
  std::sort(links.begin(),links.end());
  links.erase(std::unique(links.begin(),links.end()),links.end());
  constexpr double angle_scale=3.14159265358979323846/180;
  const double min_cos=std::cos(config.max_link_normal_deg*angle_scale);
  const double max_sin=std::sin(config.max_link_tangent_deg*angle_scale);
  std::size_t old_idx=0;
  for (auto &[key,can_connect]:links) {
    while (old_idx<smooth_links_.size() && smooth_links_[old_idx].first<key) {
      if (smooth_links_[old_idx].second) dirty_link(smooth_links_[old_idx].first);
      ++old_idx;
    }
    const bool has_previous=old_idx<smooth_links_.size() && smooth_links_[old_idx].first==key;
    const bool has_previous_connection=has_previous && smooth_links_[old_idx].second;
    const auto a=indices.at(static_cast<std::uint32_t>(key>>32));
    const auto b=indices.at(static_cast<std::uint32_t>(key));
    if (has_previous && !has_geometry_change[a] && !has_geometry_change[b]) can_connect=has_previous_connection;
    else {
      ++out.link_check_num;
      const auto &left=nodes[a], &right=nodes[b];
      const Eigen::Vector3d delta=right.position-left.position;
      const double dist2=delta.squaredNorm();
      const double left_dev=left.normal.dot(delta), right_dev=right.normal.dot(delta);
      can_connect=std::isfinite(dist2) && dist2<=config.max_link_length*config.max_link_length &&
        left.normal.squaredNorm()>0.5 && right.normal.squaredNorm()>0.5 &&
        std::abs(left.normal.dot(right.normal))>=min_cos &&
        std::max(left_dev*left_dev,right_dev*right_dev)<=max_sin*max_sin*dist2;
    }
    if (has_previous_connection!=can_connect) dirty_link(key);
    if (has_previous) ++old_idx;
    if (can_connect) {
      out.connected_edges.push_back(static_cast<std::uint16_t>(a));
      out.connected_edges.push_back(static_cast<std::uint16_t>(b));
    }
  }
  for (;old_idx<smooth_links_.size();++old_idx)
    if (smooth_links_[old_idx].second) dirty_link(smooth_links_[old_idx].first);

  // 接続可否不変の成分は所属を再利用。切断・追加・ID消失の影響成分だけ再探索。
  std::vector<bool> has_connectivity_change(num), has_visited(num,false);
  std::map<std::uint32_t,std::size_t> stable_regions;
  for (std::size_t i=0; i<num; ++i) {
    has_connectivity_change[i]=previous_regions[i]==invalid || dirty_regions.count(previous_regions[i]);
    if (has_connectivity_change[i]) continue;
    const auto [found,inserted]=stable_regions.emplace(previous_regions[i],out.regions.size());
    if (inserted) out.regions.emplace_back();
    out.regions[found->second].node_indices.push_back(static_cast<std::uint32_t>(i));
  }
  if (std::any_of(has_connectivity_change.begin(),has_connectivity_change.end(),[](bool value){return value;})) {
    std::vector<std::vector<std::uint32_t>> neighbors(num);
    for (std::size_t i=0;i<out.connected_edges.size();i+=2) {
      const auto a=out.connected_edges[i],b=out.connected_edges[i+1];
      neighbors[a].push_back(b); neighbors[b].push_back(a);
    }
    for (std::size_t i=0; i<num; ++i) {
      if (!has_connectivity_change[i] || has_visited[i]) continue;
      region surface;
      surface.node_indices.push_back(static_cast<std::uint32_t>(i));
      has_visited[i]=true;
      for (std::size_t j=0; j<surface.node_indices.size(); ++j) {
        ++out.connectivity_node_num;
        for (auto next:neighbors[surface.node_indices[j]]) {
          if (!has_connectivity_change[next] || has_visited[next]) continue;
          has_visited[next]=true; surface.node_indices.push_back(next);
        }
      }
      out.regions.push_back(std::move(surface));
    }
  }
  // 分割時は大きい子成分が旧IDを継承。統合時は最多の旧所属を優先。
  for (auto &surface:out.regions) {
    surface.id=invalid;
    std::sort(surface.node_indices.begin(),surface.node_indices.end());
    for (auto idx:surface.node_indices) surface.id=std::min(surface.id,keys[idx]);
  }
  std::sort(out.regions.begin(),out.regions.end(),[](const auto &a,const auto &b) {
    return a.node_indices.size()!=b.node_indices.size() ?
      a.node_indices.size()>b.node_indices.size():a.id<b.id;
  });
  std::set<std::uint32_t> claimed_ids;
  for (auto &surface:out.regions) {
    std::map<std::uint32_t,std::size_t> counts;
    for (auto idx:surface.node_indices) if (previous_regions[idx]!=invalid) ++counts[previous_regions[idx]];
    surface.id=invalid;
    std::size_t max_count=0;
    for (const auto &[id,count]:counts) if (!claimed_ids.count(id) && count>max_count) {
      surface.id=id; max_count=count;
    }
    surface.is_retained=surface.id!=invalid;
    if (!surface.is_retained) {
      if (next_id_>=static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
        throw std::overflow_error("surface tracking ID exhausted");
      surface.id=next_id_++;
    }
    claimed_ids.insert(surface.id);
    if (surface.node_indices.size()>=config.min_fit_nodes) surface.shape.type="smooth_surface";
    for (auto idx:surface.node_indices) nodes[idx].region_id=surface.id;
  }
  std::sort(out.regions.begin(),out.regions.end(),[](const auto &a,const auto &b){return a.id<b.id;});
  std::vector<int> plane_owner(num,-1);
  for (std::size_t i=0;i<planes.clusters.size();++i)
    for (auto idx:planes.clusters[i].node_indices)
      if (idx<num && plane_owner[idx]<0) plane_owner[idx]=static_cast<int>(i);
  std::vector<std::uint32_t> node_patch(num);
  for (auto &surface:out.regions) {
    std::map<int,std::uint32_t> plane_patches;
    for (auto idx:surface.node_indices) {
      const int owner=plane_owner[idx];
      const auto found=plane_patches.find(owner);
      std::uint32_t patch_idx;
      if (owner>=0 && found!=plane_patches.end()) patch_idx=found->second;
      else {
        patch_idx=static_cast<std::uint32_t>(out.patches.size());
        out.patches.emplace_back();
        out.patches.back().plane_cluster_idx=owner;
        surface.patch_indices.push_back(patch_idx);
        if (owner>=0) plane_patches.emplace(owner,patch_idx);
      }
      node_patch[idx]=patch_idx;
      auto &patch=out.patches[patch_idx];
      patch.node_indices.push_back(idx);
      if (nodes[idx].position.allFinite()) patch.center+=nodes[idx].position;
    }
  }
  for (auto &patch:out.patches) patch.center/=static_cast<double>(patch.node_indices.size());
  const auto patch_links=[&](const auto &edges) {
    std::set<std::array<std::uint32_t,2>> unique;
    for (std::size_t i=0;i+1<edges.size();i+=2) {
      const auto a=edges[i],b=edges[i+1];
      if (a>=num || b>=num || node_patch[a]==node_patch[b]) continue;
      unique.insert({std::min(node_patch[a],node_patch[b]),std::max(node_patch[a],node_patch[b])});
    }
    return std::vector<std::array<std::uint32_t,2>>(unique.begin(),unique.end());
  };
  out.patch_edges=patch_links(map.edges);
  out.smooth_edges=patch_links(out.connected_edges);
  smooth_nodes_.clear();
  smooth_nodes_.reserve(num);
  for (std::size_t i=0;i<num;++i) smooth_nodes_.emplace(keys[i],std::move(nodes[i]));
  smooth_links_=std::move(links);
  smooth_limits_=limits;
  frame_id_=map.header.frame_id; frame_number_=map.frame_number; stamp_=stamp;
  if (has_duplicate_id) clear();
  out.update_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  return out;
}
}  // 名前空間 fuzzrobo::surface_model
