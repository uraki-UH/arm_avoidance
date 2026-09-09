// 保存済みmap/planesの有限オフライン再評価。ROS入出力・既存ノードへの変更なし。
#include "ais_gng/topological_plane/surface_model_tracking.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <set>

namespace surface = fuzzrobo::surface_model;
using json = nlohmann::json;

template<class vector_type>
void read_vector(const json &value, vector_type &out)
{
  out.x=value.at("x"); out.y=value.at("y"); out.z=value.at("z");
}

json summarize(const surface::result &result, const ais_gng_msgs::msg::PlaneClusterArray &planes)
{
  json out={{"update_ms",result.update_ms},{"curvature_ms",result.curvature_ms},
    {"fits",result.model_fits},{"patches",json::array()},
    {"sharp",json::array()},{"smooth",json::array()},{"shown",json::array()}};
  for (const auto &patch:result.patches) {
    if (patch.plane_cluster_idx<0) continue;
    const auto &c=patch.curvature;
    out["patches"].push_back({{"plane",planes.clusters[patch.plane_cluster_idx].id},
      {"nodes",patch.node_indices.size()},{"valid",c.valid},{"confidence",c.confidence},
      {"fit_error",c.fit_error},{"kappa",{c.kappa.x(),c.kappa.y()}}});
  }
  for (const auto &kind:{"sharp","smooth"}) {
    const auto &edges=std::string(kind)=="sharp" ? result.sharp_edges:result.smooth_edges;
    for (const auto &edge:edges) {
      const auto a=result.patches[edge[0]].plane_cluster_idx;
      const auto b=result.patches[edge[1]].plane_cluster_idx;
      if (a>=0 && b>=0) out[kind].push_back({planes.clusters[a].id,planes.clusters[b].id});
    }
  }
  for (const auto &region:result.regions) {
    if (region.shape.type=="plane" || region.shape.type=="unknown" ||
      (!region.is_retained && surface::plane_patch_num(result,region)<2)) continue;
    std::set<std::uint32_t> ids;
    for (auto idx:region.patch_indices) {
      const auto plane=result.patches[idx].plane_cluster_idx;
      if (plane>=0) ids.insert(planes.clusters[plane].id);
    }
    out["shown"].push_back({{"id",region.id},{"type",region.shape.type},
      {"nodes",region.node_indices.size()},{"planes",ids},
      {"rms",region.shape.rms},{"is_retained",region.is_retained}});
  }
  return out;
}

int main(int argc, char **argv)
{
  if (argc!=2) { std::cerr<<"usage: replay_surface_models observed.json\n"; return 2; }
  try {
    std::ifstream input(argv[1]);
    const auto records=json::parse(input);
    surface::tracker tracking;
    for (const auto &record:records.at("models")) {
      if (!record.contains("map") || !record.contains("planes")) continue;
      ais_gng_msgs::msg::TopologicalMap map;
      ais_gng_msgs::msg::PlaneClusterArray planes;
      const auto &raw_map=record.at("map");
      map.frame_number=raw_map.at("frame_number");
      planes.frame_number=record.at("planes").at("frame_number");
      map.header.frame_id=raw_map.at("header").at("frame_id");
      planes.header.frame_id=record.at("planes").at("header").at("frame_id");
      map.header.stamp.sec=raw_map.at("header").at("stamp").at("sec");
      map.header.stamp.nanosec=raw_map.at("header").at("stamp").at("nanosec");
      planes.header.stamp=map.header.stamp;
      for (const auto &raw:raw_map.at("nodes")) {
        ais_gng_msgs::msg::TopologicalNode node;
        node.id=raw.at("id");
        read_vector(raw.at("pos"),node.pos); read_vector(raw.at("normal"),node.normal);
        map.nodes.push_back(node);
      }
      map.edges=raw_map.at("edges").get<decltype(map.edges)>();
      for (const auto &raw:record.at("planes").at("clusters")) {
        ais_gng_msgs::msg::PlaneCluster plane;
        plane.id=raw.at("id"); read_vector(raw.at("normal"),plane.normal);
        plane.node_indices=raw.at("node_indices").get<decltype(plane.node_indices)>();
        planes.clusters.push_back(plane);
      }
      const auto independent=surface::extract(map,planes);
      const auto tracked=tracking.update(map,planes);
      std::cout<<json({{"frame",map.frame_number},
        {"independent",summarize(independent,planes)},
        {"tracked",summarize(tracked,planes)}}).dump()<<'\n';
    }
  } catch (const std::exception &error) {
    std::cerr<<error.what()<<'\n'; return 1;
  }
}
