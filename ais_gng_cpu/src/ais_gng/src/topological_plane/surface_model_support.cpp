#include "ais_gng/topological_plane/surface_model.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <set>

namespace fuzzrobo::surface_model
{
namespace
{
using vec = Eigen::Vector3d;
using pair_type = std::array<std::uint32_t,2>;
vec position(const geometry_msgs::msg::Point32 &point) { return {point.x,point.y,point.z}; }

struct components
{
  std::vector<std::size_t> parent;
  explicit components(std::size_t num):parent(num) { std::iota(parent.begin(),parent.end(),0); }
  std::size_t root(std::size_t idx)
  {
    while (parent[idx]!=idx) { parent[idx]=parent[parent[idx]]; idx=parent[idx]; }
    return idx;
  }
  bool join(std::size_t a,std::size_t b)
  {
    a=root(a); b=root(b);
    if (a==b) return false;
    parent[std::max(a,b)]=std::min(a,b); return true;
  }
};

vec model_normal(const model &shape,const vec &point)
{
  const vec p=(point-shape.origin)/shape.scale;
  const auto &q=shape.q;
  vec normal(2*q(0)*p.x()+q(3)*p.y()+q(4)*p.z()+q(6),
    2*q(1)*p.y()+q(3)*p.x()+q(5)*p.z()+q(7),
    2*q(2)*p.z()+q(4)*p.x()+q(5)*p.y()+q(8));
  if (!normal.allFinite() || normal.norm()<1e-8) return vec::Zero();
  return normal.normalized();
}
}

void split_support_regions(result &surfaces,
  const ais_gng_msgs::msg::TopologicalMap &map,const options &config)
{
  const auto begin=std::chrono::steady_clock::now();
  const auto num=map.nodes.size();
  const double max_gap=std::min(config.max_support_gap,config.max_link_length);
  components connected(num);
  std::vector<int> owner(num,-1);
  std::vector<double> spacing(num,std::numeric_limits<double>::infinity());
  for (std::size_t idx=0;idx<surfaces.regions.size();++idx) {
    const auto &region=surfaces.regions[idx];
    if (region.shape.type=="plane" || region.shape.type=="unknown") continue;
    for (auto node_idx:region.node_indices) owner[node_idx]=idx;
  }
  // 実GNGエッジによる領域内の連結。長過ぎるエッジは支持対象外。
  for (std::size_t idx=0;idx+1<map.edges.size();idx+=2) {
    const auto a=map.edges[idx],b=map.edges[idx+1];
    if (a>=num || b>=num || owner[a]<0 || owner[a]!=owner[b]) continue;
    const double dist=(position(map.nodes[a].pos)-position(map.nodes[b].pos)).norm();
    if (!std::isfinite(dist) || dist>config.max_link_length) continue;
    connected.join(a,b);
    if (dist>1e-8) { spacing[a]=std::min(spacing[a],dist); spacing[b]=std::min(spacing[b],dist); }
  }
  std::vector<region> regions;
  bool has_split=false;
  for (const auto &previous:surfaces.regions) {
    if (previous.shape.type=="plane" || previous.shape.type=="unknown") {
      regions.push_back(previous); continue;
    }
    std::set<std::size_t> roots;
    for (auto idx:previous.node_indices) roots.insert(connected.root(idx));
    if (roots.size()>1 && max_gap>0) {
      using cell_type=std::array<std::int64_t,3>;
      std::map<cell_type,std::vector<std::uint32_t>> cells;
      std::vector<std::pair<std::uint32_t,cell_type>> entries;
      for (auto idx:previous.node_indices) {
        const vec scaled=position(map.nodes[idx].pos)/max_gap;
        // セル添字変換時の有限性・整数範囲の保護。
        if (!scaled.allFinite() || scaled.cwiseAbs().maxCoeff()>1e15) continue;
        const cell_type cell={static_cast<std::int64_t>(std::floor(scaled.x())),
          static_cast<std::int64_t>(std::floor(scaled.y())),static_cast<std::int64_t>(std::floor(scaled.z()))};
        cells[cell].push_back(idx); entries.emplace_back(idx,cell);
      }
      // 近傍距離の収集と補完判定の二段階。走査順による局所間隔の差の防止。
      for (int pass=0;pass<2;++pass) for (const auto &[a,cell]:entries) {
        for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
          const auto found=cells.find({cell[0]+x,cell[1]+y,cell[2]+z});
          if (found==cells.end()) continue;
          for (auto b:found->second) {
            if (a>=b) continue;
            const vec pa=position(map.nodes[a].pos),pb=position(map.nodes[b].pos),delta=pb-pa;
            const double dist=delta.norm();
            if (dist>max_gap || !std::isfinite(dist)) continue;
            if (pass==0) {
              if (dist>1e-8) { spacing[a]=std::min(spacing[a],dist); spacing[b]=std::min(spacing[b],dist); }
              continue;
            }
            if (connected.root(a)==connected.root(b)) continue;
            if (dist<1e-8) {
              if (connected.join(a,b)) ++surfaces.support_gap_links;
              continue;
            }
            if (dist>config.max_support_spacing_ratio*std::min(spacing[a],spacing[b])) continue;
            const auto free_space=ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE;
            if ((map.nodes[a].boundary_evidence | map.nodes[b].boundary_evidence)&free_space) continue;
            const vec na=model_normal(previous.shape,pa),nb=model_normal(previous.shape,pb);
            if (na.squaredNorm()<0.5 || nb.squaredNorm()<0.5 || std::abs(na.dot(nb))<0.8660254037844386 ||
              std::abs(na.dot(delta))/dist>0.5 || std::abs(nb.dot(delta))/dist>0.5) continue;
            bool can_bridge=true;
            // 曲面を横切る近道の抑止。接線整合に加えた隙間内3点のモデル残差検査。
            for (double fraction:{0.25,0.5,0.75}) {
              ais_gng_msgs::msg::TopologicalNode node;
              const vec point=pa+fraction*delta;
              node.pos.x=point.x(); node.pos.y=point.y(); node.pos.z=point.z();
              if (model_dev(previous.shape,node).dist>config.max_patch_rms) can_bridge=false;
            }
            if (can_bridge && connected.join(a,b)) ++surfaces.support_gap_links;
          }
        }
      }
    }
    std::map<std::size_t,std::vector<std::uint32_t>> groups;
    for (auto idx:previous.node_indices) groups[connected.root(idx)].push_back(idx);
    if (groups.size()==1) { regions.push_back(previous); continue; }
    has_split=true;
    surfaces.support_split_num+=groups.size()-1;
    const auto largest=std::max_element(groups.begin(),groups.end(),[](const auto &a,const auto &b) {
      return a.second.size()<b.second.size();
    })->first;
    for (auto &[root,nodes]:groups) {
      region fragment=previous;
      fragment.node_indices=std::move(nodes);
      fragment.patch_indices.clear();
      fragment.support_parent_id=previous.id;
      if (!previous.is_retained || root!=largest)
        fragment.id=*std::min_element(fragment.node_indices.begin(),fragment.node_indices.end());
      if (fragment.node_indices.size()<config.min_fit_nodes) {
        fragment.shape=model{}; fragment.is_retained=false;
        fragment.id=*std::min_element(fragment.node_indices.begin(),fragment.node_indices.end());
      } else {
        double sum=0;
        for (auto idx:fragment.node_indices) { const double dist=model_dev(fragment.shape,map.nodes[idx]).dist; sum+=dist*dist; }
        const double rms=std::sqrt(sum/fragment.node_indices.size());
        fragment.shape.score+=rms*rms-fragment.shape.rms*fragment.shape.rms;
        fragment.shape.rms=rms;
      }
      regions.push_back(std::move(fragment));
    }
  }
  if (has_split) {
    // パッチ所属も分割後の領域に一致。旧パッチを複数領域で共有しない構造。
    std::vector<int> region_owner(num,-1),patch_owner(num,-1);
    for (std::size_t idx=0;idx<regions.size();++idx) {
      regions[idx].patch_indices.clear();
      for (auto node_idx:regions[idx].node_indices) region_owner[node_idx]=idx;
    }
    std::vector<local_patch> patches;
    std::vector<std::uint32_t> original;
    for (std::size_t idx=0;idx<surfaces.patches.size();++idx) {
      const auto &previous=surfaces.patches[idx];
      std::map<int,std::vector<std::uint32_t>> groups;
      for (auto node_idx:previous.node_indices) groups[region_owner[node_idx]].push_back(node_idx);
      for (auto &[region_idx,nodes]:groups) {
        local_patch patch=previous;
        patch.node_indices=std::move(nodes);
        if (groups.size()>1) {
          patch.center=vec::Zero();
          for (auto node_idx:patch.node_indices) patch.center+=position(map.nodes[node_idx].pos);
          patch.center/=patch.node_indices.size();
          patch.curvature=estimate_curvature(patch,map);
        }
        regions.at(region_idx).patch_indices.push_back(patches.size());
        for (auto node_idx:patch.node_indices) patch_owner[node_idx]=patches.size();
        patches.push_back(std::move(patch)); original.push_back(idx);
      }
    }
    const std::set<pair_type> old_smooth(surfaces.smooth_edges.begin(),surfaces.smooth_edges.end());
    const std::set<pair_type> old_sharp(surfaces.sharp_edges.begin(),surfaces.sharp_edges.end());
    const std::set<pair_type> old_uncertain(surfaces.uncertain_edges.begin(),surfaces.uncertain_edges.end());
    std::set<pair_type> edges,smooth,sharp,uncertain;
    for (std::size_t idx=0;idx+1<map.edges.size();idx+=2) {
      const auto a=map.edges[idx],b=map.edges[idx+1];
      if (a>=num || b>=num || patch_owner[a]<0 || patch_owner[b]<0 || patch_owner[a]==patch_owner[b]) continue;
      const pair_type pair={static_cast<std::uint32_t>(std::min(patch_owner[a],patch_owner[b])),
        static_cast<std::uint32_t>(std::max(patch_owner[a],patch_owner[b]))};
      edges.insert(pair);
      if ((position(map.nodes[a].pos)-position(map.nodes[b].pos)).norm()>config.max_link_length) continue;
      const pair_type old_pair={std::min(original[pair[0]],original[pair[1]]),std::max(original[pair[0]],original[pair[1]])};
      if (old_smooth.count(old_pair)) smooth.insert(pair);
      if (old_sharp.count(old_pair)) sharp.insert(pair);
      if (old_uncertain.count(old_pair)) uncertain.insert(pair);
    }
    surfaces.patches=std::move(patches); surfaces.regions=std::move(regions);
    surfaces.patch_edges.assign(edges.begin(),edges.end()); surfaces.smooth_edges.assign(smooth.begin(),smooth.end());
    surfaces.sharp_edges.assign(sharp.begin(),sharp.end()); surfaces.uncertain_edges.assign(uncertain.begin(),uncertain.end());
    for (auto &region:surfaces.regions) if (region.shape.type!="unknown" && region.shape.type!="plane") {
      region.shape.max_patch_rms=0;
      for (auto idx:region.patch_indices) {
        double sum=0;
        for (auto node_idx:surfaces.patches[idx].node_indices) {
          const double dist=model_dev(region.shape,map.nodes[node_idx]).dist; sum+=dist*dist;
        }
        region.shape.max_patch_rms=std::max(region.shape.max_patch_rms,std::sqrt(sum/surfaces.patches[idx].node_indices.size()));
      }
    }
  }
  surfaces.support_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
}
}  // namespace fuzzrobo::surface_model
