#include "ais_gng/topological_plane/surface_model_visualization.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fuzzrobo::surface_model
{
namespace
{
using json = nlohmann::json;
using marker = visualization_msgs::msg::Marker;

std_msgs::msg::ColorRGBA color(std::uint32_t id, bool is_unknown)
{
  std_msgs::msg::ColorRGBA c;
  c.a = 1.0F;
  if (is_unknown) { c.r = c.g = c.b = 0.45F; return c; }
  // 同一SurfaceModelの全パッチ・ノード・edgeで共通の決定的な色。
  const double hue = std::fmod((id+1)*0.618033988749895,1.0)*6;
  const float low = 0.18F, high = 0.95F;
  const float up = low+(high-low)*(hue-std::floor(hue));
  const float down = high+low-up;
  switch (static_cast<int>(hue)) {
    case 0: c.r=high; c.g=up; c.b=low; break;
    case 1: c.r=down; c.g=high; c.b=low; break;
    case 2: c.r=low; c.g=high; c.b=up; break;
    case 3: c.r=low; c.g=down; c.b=high; break;
    case 4: c.r=up; c.g=low; c.b=high; break;
    default: c.r=high; c.g=low; c.b=down; break;
  }
  return c;
}
template<class p> geometry_msgs::msg::Point point(const p &v)
{
  geometry_msgs::msg::Point out;
  out.x=v.x; out.y=v.y; out.z=v.z;
  return out;
}
geometry_msgs::msg::Point point(const Eigen::Vector3d &v)
{
  geometry_msgs::msg::Point out;
  out.x=v.x(); out.y=v.y(); out.z=v.z();
  return out;
}
json vector_json(const Eigen::Vector3d &v) { return {v.x(),v.y(),v.z()}; }

bool is_display_candidate(const result &surfaces, const region &r, std::size_t min_plane_patches)
{
  return r.shape.type!="unknown" && r.shape.type!="plane" &&
    std::max(plane_patch_num(surfaces,r),r.is_retained ? r.seed_plane_patch_num:0U)>=min_plane_patches;
}
}

ais_gng_msgs::msg::TopologicalMap make_graph(
  const result &surfaces, const ais_gng_msgs::msg::TopologicalMap &map)
{
  ais_gng_msgs::msg::TopologicalMap out;
  out.header=map.header;
  out.frame_number=map.frame_number;
  out.nodes=map.nodes;
  // 入力点群の所属一覧のみ省略。ノードID・座標・法線・共分散・属性は元のまま。
  for (auto &node:out.nodes) node.inpcl_ids.clear();
  std::vector<int> node_region(map.nodes.size(),-1);
  for (std::size_t i=0; i<surfaces.regions.size(); ++i) {
    const auto &r=surfaces.regions[i];
    ais_gng_msgs::msg::TopologicalCluster cluster;
    cluster.id=r.id;
    cluster.frame=map.frame_number;
    cluster.quat.w=1;
    Eigen::Vector3d center=Eigen::Vector3d::Zero();
    Eigen::Vector3d lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi=-lo;
    for (const auto idx:r.node_indices) {
      if (idx>=map.nodes.size()) continue;
      node_region[idx]=static_cast<int>(i);
      const auto &node=map.nodes[idx];
      // clusters.nodesはノードID、edgesは配列添字という既存契約。
      cluster.nodes.push_back(node.id);
      const Eigen::Vector3d p(node.pos.x,node.pos.y,node.pos.z);
      center+=p; lo=lo.cwiseMin(p); hi=hi.cwiseMax(p);
    }
    // 未確定ノードはクラスタ未所属として表示。形状種別・誤差は/modelsに保持。
    if (r.shape.type=="unknown" || cluster.nodes.empty()) continue;
    center/=static_cast<double>(cluster.nodes.size());
    cluster.pos.x=center.x(); cluster.pos.y=center.y(); cluster.pos.z=center.z();
    const Eigen::Vector3d size=2*(center-lo).cwiseMax(hi-center);
    cluster.scale.x=size.x(); cluster.scale.y=size.y(); cluster.scale.z=size.z();
    out.clusters.push_back(std::move(cluster));
  }
  out.edges.reserve(map.edges.size());
  for (std::size_t i=0; i+1<map.edges.size(); i+=2) {
    const auto a=map.edges[i], b=map.edges[i+1];
    if (a>=node_region.size() || b>=node_region.size() || a==b) continue;
    if (node_region[a]<0 || node_region[a]!=node_region[b]) continue;
    out.edges.push_back(a); out.edges.push_back(b);
  }
  return out;
}

visualization_msgs::msg::MarkerArray make_markers(
  const result &surfaces, const ais_gng_msgs::msg::TopologicalMap &map,
  bool enable_labels, bool enable_patch_graph,
  std::set<std::pair<std::string,int>> &published, std::size_t min_display_plane_patches)
{
  visualization_msgs::msg::MarkerArray out;
  // 疎なregion IDを連番へ写像。ID 0と21等で色相が偶然近接する問題の抑止。
  std::map<int,std::uint32_t> color_slots;
  for (const auto &r:surfaces.regions) {
    if (is_display_candidate(surfaces,r,min_display_plane_patches)) color_slots.emplace(r.id,0);
  }
  std::uint32_t slot=0;
  for (auto &entry:color_slots) {
    // 追跡IDは生成順の色を保持。他モデルの消失による色スロットの詰め直しを回避。
    entry.second=entry.first>=(1<<30) ? entry.first-(1<<30):slot;
    ++slot;
  }
  std::vector<int> node_region(map.nodes.size(),-1), patch_region(surfaces.patches.size(),-1);
  std::vector<marker> nodes(surfaces.regions.size()), edges(surfaces.regions.size());
  const auto base = [&](const std::string &ns, int id, int type, double size, bool is_unknown) {
    marker m;
    m.header=map.header; m.ns=ns; m.id=id; m.type=type; m.action=marker::ADD;
    m.pose.orientation.w=1; m.scale.x=m.scale.y=m.scale.z=size;
    m.color=color(is_unknown ? 0 : color_slots.at(id),is_unknown);
    return m;
  };
  for (std::size_t i=0; i<surfaces.regions.size(); ++i) {
    const auto &r=surfaces.regions[i];
    if (!color_slots.count(r.id)) continue;
    const bool is_unknown=r.shape.type=="unknown";
    nodes[i]=base("surface_nodes",r.id,marker::SPHERE_LIST,0.008,is_unknown);
    edges[i]=base("surface_edges",r.id,marker::LINE_LIST,0.0025,is_unknown);
    for (auto idx:r.node_indices) {
      node_region[idx]=static_cast<int>(i);
      nodes[i].points.push_back(point(map.nodes[idx].pos));
    }
    for (auto idx:r.patch_indices) patch_region[idx]=static_cast<int>(i);
    if (enable_labels && !is_unknown) {
      auto label=base("surface_labels",r.id,marker::TEXT_VIEW_FACING,0.022,false);
      Eigen::Vector3d center=Eigen::Vector3d::Zero();
      for (auto idx:r.patch_indices) center+=surfaces.patches[idx].center;
      center/=static_cast<double>(r.patch_indices.size());
      label.pose.position=point(center);
      label.pose.position.z+=0.025;
      std::ostringstream text;
      text << r.shape.type << " #" << r.id << " " << std::fixed << std::setprecision(1)
           << r.shape.rms*1000 << "mm";
      label.text=text.str();
      out.markers.push_back(std::move(label));
    }
  }
  // 全GNG edgeの一回走査。同一モデル所属の実edgeのみ描画し、架空の接続を生成しない。
  for (std::size_t i=0; i+1<map.edges.size(); i+=2) {
    const auto a=map.edges[i], b=map.edges[i+1];
    if (a>=node_region.size() || b>=node_region.size() || a==b) continue;
    const int r=node_region[a];
    if (r<0 || r!=node_region[b]) continue;
    edges[r].points.push_back(point(map.nodes[a].pos));
    edges[r].points.push_back(point(map.nodes[b].pos));
  }
  // 未統合・平面・未確定の再表示を省略し、複数平面の曲面統合結果だけを表示。
  for (std::size_t i=0; i<surfaces.regions.size(); ++i) {
    if (!color_slots.count(surfaces.regions[i].id)) continue;
    out.markers.push_back(std::move(nodes[i]));
    if (!edges[i].points.empty()) out.markers.push_back(std::move(edges[i]));
  }
  if (enable_patch_graph) {
    // Viewerの単色Markerに合わせ、上位グラフもSurfaceModel単位で集約。
    std::map<int,marker> patches,links;
    auto boundaries=base("surface_patch_boundaries",0,marker::LINE_LIST,0.0015,true);
    for (std::size_t i=0; i<surfaces.patches.size(); ++i) {
      if (patch_region[i]<0) continue;
      const auto &r=surfaces.regions[patch_region[i]];
      const bool is_unknown=r.shape.type=="unknown";
      const int id=is_unknown ? -1 : static_cast<int>(r.id);
      if (!patches.count(id)) patches[id]=base("surface_patches",id,marker::SPHERE_LIST,0.014,is_unknown);
      patches[id].points.push_back(point(surfaces.patches[i].center));
    }
    for (const auto &e:surfaces.patch_edges) {
      if (patch_region[e[0]]<0 || patch_region[e[1]]<0) continue;
      const bool has_same_model=patch_region[e[0]]==patch_region[e[1]];
      const auto &r=surfaces.regions[patch_region[e[0]]];
      const bool is_unknown=r.shape.type=="unknown";
      const int id=is_unknown ? -1 : static_cast<int>(r.id);
      marker *line=&boundaries;
      if (has_same_model) {
        if (!links.count(id)) links[id]=base("surface_patch_edges",id,marker::LINE_LIST,0.0015,is_unknown);
        line=&links[id];
      }
      line->points.push_back(point(surfaces.patches[e[0]].center));
      line->points.push_back(point(surfaces.patches[e[1]].center));
    }
    for (auto &entry:patches) out.markers.push_back(std::move(entry.second));
    for (auto &entry:links) out.markers.push_back(std::move(entry.second));
    if (!boundaries.points.empty()) out.markers.push_back(std::move(boundaries));
  }
  std::set<std::pair<std::string,int>> current;
  for (const auto &m:out.markers) current.emplace(m.ns,m.id);
  for (const auto &key:published) if (!current.count(key)) {
    auto remove=base(key.first,key.second,marker::POINTS,1,true);
    remove.action=marker::DELETE;
    out.markers.push_back(std::move(remove));
  }
  published=std::move(current);
  return out;
}

std::string serialize(const result &surfaces,const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes, std::size_t min_display_plane_patches)
{
  json out={{"schema","surface_region_graph_v1"},{"frame_id",map.header.frame_id},
    {"frame_number",map.frame_number},{"stamp",{{"sec",map.header.stamp.sec},{"nanosec",map.header.stamp.nanosec}}},
    {"sample_source","gng_node_positions_and_normals"},{"update_ms",surfaces.update_ms},
    {"model_fits",surfaces.model_fits},{"curvature_ms",surfaces.curvature_ms},
    {"boundary_ms",surfaces.boundary_ms},{"boundary_fit_num",surfaces.boundary_fit_num},
    {"retention_ms",surfaces.retention_ms},
    {"patch_edges",surfaces.patch_edges},{"smooth_edges",surfaces.smooth_edges},
    {"sharp_edges",surfaces.sharp_edges},
    {"uncertain_edges",surfaces.uncertain_edges},
    {"min_display_plane_patches",min_display_plane_patches},
    {"patches",json::array()},{"models",json::array()}};
  for (std::size_t i=0; i<surfaces.patches.size(); ++i) {
    const auto &p=surfaces.patches[i];
    json patch={{"id",i},{"kind",p.plane_cluster_idx>=0 ? "plane_patch":"nonplane_node"},
      {"node_indices",p.node_indices},{"center",vector_json(p.center)}};
    if (p.plane_cluster_idx>=0) patch["plane_cluster_id"]=planes.clusters[p.plane_cluster_idx].id;
    const auto &c = p.curvature;
    patch["curvature"] = {{"valid",c.valid},{"sample_num",c.sample_num},
      {"method","position_quadratic"}};
    patch["curvature"].update({{"fit_iter",c.fit_iter},
      {"has_svd_fallback",c.has_svd_fallback}});
    if (c.valid) {
      patch["curvature"].update({{"normal",vector_json(c.normal)},
        {"axis_u",vector_json(c.axis_u)},{"axis_v",vector_json(c.axis_v)},
        {"tensor",{{c.tensor(0,0),c.tensor(0,1)},{c.tensor(1,0),c.tensor(1,1)}}},
        {"kappa_per_m",{c.kappa.x(),c.kappa.y()}},
        {"directions_uv",{{c.directions_uv(0,0),c.directions_uv(1,0)},
          {c.directions_uv(0,1),c.directions_uv(1,1)}}},
        {"support_cov_m2",{{c.support_cov(0,0),c.support_cov(0,1)},
          {c.support_cov(1,0),c.support_cov(1,1)}}},
        {"normal_scatter",{{c.normal_scatter(0,0),c.normal_scatter(0,1),c.normal_scatter(0,2)},
          {c.normal_scatter(1,0),c.normal_scatter(1,1),c.normal_scatter(1,2)},
          {c.normal_scatter(2,0),c.normal_scatter(2,1),c.normal_scatter(2,2)}}},
        {"plane_rms_m",c.plane_rms},{"position_rms_m",c.position_rms},
        {"fit_error",c.fit_error},{"confidence",c.confidence}});
    }
    out["patches"].push_back(std::move(patch));
  }
  for (const auto &r:surfaces.regions) {
    const auto &s=r.shape;
    json model={{"id",r.id},{"type",s.type},{"patch_indices",r.patch_indices},{"node_indices",r.node_indices}};
    model["plane_patch_num"]=plane_patch_num(surfaces,r);
    model["is_retained"]=r.is_retained;
    model["seed_plane_patch_num"]=r.seed_plane_patch_num;
    model["rejected_node_num"]=r.rejected_node_num;
    model["is_display_candidate"]=is_display_candidate(surfaces,r,min_display_plane_patches);
    if (s.type!="unknown") {
      model["fit"]={{"origin",vector_json(s.origin)},{"coordinate_scale",s.scale},
        {"q",std::vector<double>(s.q.data(),s.q.data()+10)},
        {"rms_m",s.rms},{"max_patch_rms_m",s.max_patch_rms},{"score_m2",s.score},
        {"center",vector_json(s.center)},{"axis",vector_json(s.axis)},
        {"major_direction",vector_json(s.major_direction)},{"radii_m",{s.radii.x(),s.radii.y()}}};
    }
    out["models"].push_back(std::move(model));
  }
  return out.dump();
}

publisher::publisher(rclcpp::Node &node):node_(node)
{
  enable_=node.declare_parameter("surface_model.enable",true);
  enable_labels_=node.declare_parameter("surface_model.enable_labels",false);
  enable_patch_graph_=node.declare_parameter("surface_model.enable_patch_graph",false);
  enable_markers_=node.declare_parameter("surface_model.enable_markers",true);
  min_display_plane_patches_=std::max<std::int64_t>(0,
    node.declare_parameter("surface_model.min_display_plane_patches",2));
  const bool enable_graph=node.declare_parameter("surface_model.enable_graph",false);
  const double hz=node.declare_parameter("surface_model.hz",2.0);
  if (!std::isfinite(hz) || hz<=0) throw std::invalid_argument("surface_model.hz must be finite and positive");
  period_=1.0/hz;
  config_.max_link_length=node.declare_parameter("surface_model.max_link_length",config_.max_link_length);
  config_.max_link_normal_deg=node.declare_parameter("surface_model.max_link_normal_deg",config_.max_link_normal_deg);
  config_.max_patch_rms=node.declare_parameter("surface_model.max_patch_rms",config_.max_patch_rms);
  config_.max_point_residual=node.declare_parameter("surface_model.max_point_residual",config_.max_point_residual);
  config_.max_normal_deg=node.declare_parameter("surface_model.max_normal_deg",config_.max_normal_deg);
  config_.max_curvature_normal_error=node.declare_parameter(
    "surface_model.max_curvature_normal_error",config_.max_curvature_normal_error);
  config_.protect_dominant_flat_patches=node.declare_parameter(
    "surface_model.protect_dominant_flat_patches",config_.protect_dominant_flat_patches);
  config_.complexity_penalty=node.declare_parameter("surface_model.complexity_penalty",config_.complexity_penalty);
  config_.max_radius=node.declare_parameter("surface_model.max_radius",config_.max_radius);
  config_.min_fit_nodes=std::max<std::int64_t>(10,node.declare_parameter("surface_model.min_fit_nodes",12));
  config_.max_fit_samples=std::max<std::int64_t>(10,node.declare_parameter("surface_model.max_fit_samples",256));
  config_.max_model_fits=std::max<std::int64_t>(1,node.declare_parameter("surface_model.max_model_fits",128));
  config_.max_boundary_fits=std::max<std::int64_t>(0,node.declare_parameter("surface_model.max_boundary_fits",32));
  retention_.enable_retention=node.declare_parameter("surface_model.retention.enable",true);
  retention_.max_point_residual=node.declare_parameter(
    "surface_model.retention.max_point_residual",retention_.max_point_residual);
  retention_.max_normal_deg=node.declare_parameter("surface_model.retention.max_normal_deg",retention_.max_normal_deg);
  retention_.max_rms=node.declare_parameter("surface_model.retention.max_rms",retention_.max_rms);
  retention_.min_inlier_ratio=node.declare_parameter("surface_model.retention.min_inlier_ratio",retention_.min_inlier_ratio);
  retention_.max_node_displacement=node.declare_parameter(
    "surface_model.retention.max_node_displacement",retention_.max_node_displacement);
  const auto topic=node.declare_parameter<std::string>("surface_model.output_topic","/curved_surface_clusters");
  if (!enable_) return;
  if (enable_graph) {
    graph_=node.create_publisher<ais_gng_msgs::msg::TopologicalMap>(topic,rclcpp::QoS(1).transient_local());
  }
  data_=node.create_publisher<std_msgs::msg::String>(topic+"/models",rclcpp::QoS(1).transient_local());
  if (enable_markers_) {
    markers_=node.create_publisher<visualization_msgs::msg::MarkerArray>(topic+"/markers",rclcpp::QoS(1).transient_local());
  }
  RCLCPP_INFO(node.get_logger(),"Curved surface clusters: %s/markers | %.1f Hz | markers=%s graph=%s",
    topic.c_str(),hz,enable_markers_ ? "on":"off",enable_graph ? "on":"off");
}

void publisher::update(const ais_gng_msgs::msg::TopologicalMap &map,
  const ais_gng_msgs::msg::PlaneClusterArray &planes)
{
  if (!enable_) return;
  const auto now=std::chrono::steady_clock::now();
  if (!map.nodes.empty() && std::chrono::duration<double>(now-last_).count()<period_) return;
  last_=now;
  try {
    const auto surfaces=tracker_.update(map,planes,config_,retention_,min_display_plane_patches_);
    if (graph_) graph_->publish(make_graph(surfaces,map));
    if (markers_) markers_->publish(make_markers(surfaces,map,enable_labels_,enable_patch_graph_,published_,min_display_plane_patches_));
    std_msgs::msg::String message;
    message.data=serialize(surfaces,map,planes,min_display_plane_patches_);
    data_->publish(message);
    std::size_t curved=0, planar=0, unknown=0, shown=0, shown_nodes=0, retained=0;
    for (const auto &r:surfaces.regions) {
      retained+=r.is_retained;
      if (r.shape.type=="unknown") ++unknown;
      else if (r.shape.type=="plane") ++planar;
      else ++curved;
      if (is_display_candidate(surfaces,r,min_display_plane_patches_)) {
        ++shown;
        shown_nodes+=r.node_indices.size();
      }
    }
    RCLCPP_INFO_THROTTLE(node_.get_logger(),*node_.get_clock(),2000,
      "Surface: %.2f ms (K: %.3f ms, keep: %.3f ms) | patches=%zu plane=%zu curved=%zu unknown=%zu shown=%zu retained=%zu nodes=%zu fits=%zu/%zu",
      surfaces.update_ms,surfaces.curvature_ms,surfaces.retention_ms,surfaces.patches.size(),planar,curved,unknown,shown,retained,shown_nodes,
      surfaces.model_fits,config_.max_model_fits);
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(),*node_.get_clock(),2000,"Surface skipped: %s",e.what());
  }
}
}  // namespace fuzzrobo::surface_model
