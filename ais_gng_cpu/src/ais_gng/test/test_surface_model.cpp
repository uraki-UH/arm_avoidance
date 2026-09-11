#include "ais_gng/topological_plane/surface_model.hpp"
#include "ais_gng/topological_plane/surface_model_visualization.hpp"
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <set>

namespace
{
using namespace fuzzrobo::surface_model;
using vec = Eigen::Vector3d;
using map_type = ais_gng_msgs::msg::TopologicalMap;
using planes_type = ais_gng_msgs::msg::PlaneClusterArray;
constexpr double pi = 3.14159265358979323846;

void add_node(map_type &map, const vec &p, const vec &n)
{
  ais_gng_msgs::msg::TopologicalNode node;
  node.id=static_cast<std::uint16_t>(map.nodes.size()+100);
  node.pos.x=p.x(); node.pos.y=p.y(); node.pos.z=p.z();
  node.normal.x=n.x(); node.normal.y=n.y(); node.normal.z=n.z();
  map.nodes.push_back(node);
}
void edge(map_type &map, std::size_t a, std::size_t b)
{
  map.edges.push_back(static_cast<std::uint16_t>(a));
  map.edges.push_back(static_cast<std::uint16_t>(b));
}
struct scene { map_type map; planes_type planes; };

scene cylinder(double a, double b, int around=48, bool has_plane_patches=true,
  const Eigen::Matrix3d &rotation=Eigen::Matrix3d::Identity(), const vec &offset=vec::Zero())
{
  scene s;
  s.map.header.frame_id=s.planes.header.frame_id="map";
  for (int t=0;t<around;++t) for (int z=0;z<6;++z) {
    const double angle=2*pi*t/around;
    add_node(s.map,rotation*vec(a*std::cos(angle),b*std::sin(angle),0.04*z-0.1)+offset,
      rotation*vec(std::cos(angle)/a,std::sin(angle)/b,0).normalized());
    if (z) edge(s.map,t*6+z,t*6+z-1);
    edge(s.map,t*6+z,((t+around-1)%around)*6+z);
  }
  if (has_plane_patches) for (int begin=0;begin<around;begin+=6) {
    ais_gng_msgs::msg::PlaneCluster plane;
    plane.id=static_cast<std::uint32_t>(begin+21);
    for (int t=begin;t<std::min(around,begin+4);++t) for (int z=0;z<6;++z) plane.node_indices.push_back(t*6+z);
    s.planes.clusters.push_back(plane);
  }
  return s;
}
void coverage(const result &r, std::size_t num)
{
  std::set<std::uint32_t> members;
  std::size_t total=0;
  for (const auto &region:r.regions) for (auto idx:region.node_indices) { members.insert(idx); ++total; }
  EXPECT_EQ(total,num);
  EXPECT_EQ(members.size(),num);
}
}

TEST(SurfaceModel, MixedPlanePatchesAndNonplaneNodesBecomeOneCylinder)
{
  const auto s=cylinder(0.1,0.1);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"cylinder");
  EXPECT_EQ(r.regions[0].node_indices.size(),s.map.nodes.size());
  EXPECT_NEAR(r.regions[0].shape.radii.x(),0.1,1e-5);
  EXPECT_EQ(r.regions[0].patch_indices.size(),r.patches.size());
  EXPECT_GT(r.patches.size(),s.planes.clusters.size());
  coverage(r,s.map.nodes.size());
}

TEST(SurfaceModel, EllipticCylinderSurvivesRotationTranslationAndDensityChange)
{
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(0.7,vec(1,2,3).normalized()).toRotationMatrix();
  for (int around:{24,48,72}) {
    const auto s=cylinder(0.12,0.055,around,true,rotation,vec(1.1,-0.8,2.0));
    const auto r=extract(s.map,s.planes);
    ASSERT_EQ(r.regions.size(),1U);
    EXPECT_EQ(r.regions[0].shape.type,"elliptic_cylinder");
    EXPECT_NEAR(r.regions[0].shape.radii.x(),0.12,1e-4);
    EXPECT_NEAR(r.regions[0].shape.radii.y(),0.055,1e-4);
    EXPECT_GT(std::abs(r.regions[0].shape.axis.dot(rotation*vec::UnitZ())),0.999);
  }
}

TEST(SurfaceModel, plane_core_check_can_be_disabled_for_nonplane_only_curve)
{
  const auto s=cylinder(0.1,0.1,48,false);
  for (double ratio:{0.0,0.5}) {
    options config; config.min_plane_usage_ratio=ratio;
    const auto r=extract(s.map,s.planes,config);
    ASSERT_EQ(r.regions.size(),1U);
    EXPECT_EQ(r.regions[0].shape.type,ratio==0 ? "cylinder":"unknown");
    coverage(r,s.map.nodes.size());
  }
}

TEST(SurfaceModel, connected_history_fragments_of_one_plane_reunite)
{
  auto s=cylinder(0.1,0.1);
  const auto first=extract(s.map,s.planes);
  ASSERT_EQ(first.regions.size(),1U);
  std::vector<region> retained(2,first.regions[0]);
  for (std::size_t i=0;i<retained.size();++i) {
    retained[i].id=1000+i;
    retained[i].node_indices.clear();
    for (std::size_t idx=0;idx<s.map.nodes.size();++idx)
      if ((idx%6<3)==(i==0)) retained[i].node_indices.push_back(idx);
  }
  options config; config.min_plane_usage_ratio=1.0;
  const auto merged=extract(s.map,s.planes,config,retained);
  ASSERT_EQ(merged.regions.size(),1U);
  EXPECT_EQ(merged.regions[0].shape.type,"cylinder");
  EXPECT_FALSE(merged.regions[0].is_retained);
  EXPECT_LE(merged.model_fits,2U);
  coverage(merged,s.map.nodes.size());
}

TEST(SurfaceTracking, reconnected_plane_fragments_do_not_split_again)
{
  auto s=cylinder(0.1,0.1);
  const auto original_edges=s.map.edges;
  s.map.edges.clear();
  for (std::size_t i=0;i+1<original_edges.size();i+=2) {
    const auto a=original_edges[i],b=original_edges[i+1];
    if ((a%6<3)==(b%6<3)) edge(s.map,a,b);
  }
  tracker tracking;
  const auto separated=tracking.update(s.map,s.planes);
  ASSERT_EQ(separated.regions.size(),2U);
  s.map.edges=original_edges;
  // 現在の元平面数が減っても、成立済み曲面の統合による表示消失なし。
  s.planes.clusters.resize(1);
  s.planes.clusters[0].node_indices.clear();
  for (std::size_t idx=0;idx<s.map.nodes.size();++idx) s.planes.clusters[0].node_indices.push_back(idx);
  std::uint32_t merged_id=0;
  for (int iter=0;iter<4;++iter) {
    ++s.map.frame_number; s.planes.frame_number=s.map.frame_number;
    const auto current=tracking.update(s.map,s.planes);
    ASSERT_EQ(current.regions.size(),1U);
    EXPECT_TRUE(current.regions[0].is_retained);
    EXPECT_GE(current.regions[0].seed_plane_patch_num,2U);
    if (iter==0) merged_id=current.regions[0].id;
    else { EXPECT_EQ(current.regions[0].id,merged_id); EXPECT_TRUE(current.regions[0].is_retained); }
    coverage(current,s.map.nodes.size());
  }
}

TEST(SurfaceModel, mixed_plane_root_does_not_block_two_distinct_curves)
{
  auto s=cylinder(0.1,0.1);
  const auto body_num=s.map.nodes.size();
  const auto handle=cylinder(0.04,0.04,48,true,
    Eigen::AngleAxisd(pi/2,vec::UnitX()).toRotationMatrix(),vec(0.16,0,0));
  for (const auto &node:handle.map.nodes)
    add_node(s.map,vec(node.pos.x,node.pos.y,node.pos.z),vec(node.normal.x,node.normal.y,node.normal.z));
  for (std::size_t i=0;i+1<handle.map.edges.size();i+=2)
    edge(s.map,body_num+handle.map.edges[i],body_num+handle.map.edges[i+1]);
  for (auto plane:handle.planes.clusters) {
    plane.id+=100;
    for (auto &idx:plane.node_indices) idx+=body_num;
    if (plane.id==145) {
      // 胴体の元平面へ混入した取っ手内側の根元。元の所属自体は変更対象外。
      s.planes.clusters[0].node_indices.insert(s.planes.clusters[0].node_indices.end(),
        plane.node_indices.begin(),plane.node_indices.end());
    } else s.planes.clusters.push_back(plane);
  }
  edge(s.map,3,body_num+24*6+3);
  options config;
  const auto r=extract(s.map,s.planes,config);
  std::string summary="fits="+std::to_string(r.model_fits);
  for (const auto &surface:r.regions) summary+=" "+surface.shape.type+":"+std::to_string(surface.node_indices.size());
  SCOPED_TRACE(summary);
  std::set<std::size_t> body_regions,handle_regions;
  for (std::size_t i=0;i<r.regions.size();++i) {
    const auto &surface=r.regions[i];
    if (surface.shape.type!="cylinder") continue;
    for (auto idx:surface.node_indices) {
      if (idx<body_num) body_regions.insert(i);
      else handle_regions.insert(i);
    }
  }
  ASSERT_EQ(body_regions.size(),1U);
  ASSERT_EQ(handle_regions.size(),1U);
  EXPECT_NE(*body_regions.begin(),*handle_regions.begin());
  EXPECT_EQ(r.regions[*body_regions.begin()].node_indices.size(),body_num);
  EXPECT_EQ(r.regions[*handle_regions.begin()].node_indices.size(),handle.map.nodes.size());
  for (auto idx:s.planes.clusters[0].node_indices) {
    const auto &surface=r.regions[idx<body_num ? *body_regions.begin():*handle_regions.begin()];
    EXPECT_NE(std::find(surface.node_indices.begin(),surface.node_indices.end(),idx),surface.node_indices.end());
  }
  coverage(r,s.map.nodes.size());
  const auto retained=extract(s.map,s.planes,config,r.regions);
  ASSERT_EQ(retained.regions.size(),2U);
  for (const auto &surface:retained.regions) EXPECT_TRUE(surface.is_retained);
  coverage(retained,s.map.nodes.size());
}

TEST(SurfaceModel, OrthogonalPlanesRemainSeparate)
{
  scene s;
  for (int side=0;side<2;++side) {
    ais_gng_msgs::msg::PlaneCluster plane;
    plane.id=side+1;
    const std::size_t base=s.map.nodes.size();
    for (int i=0;i<6;++i) for (int j=0;j<6;++j) {
      add_node(s.map,side ? vec(0,0.02*i,0.02*j):vec(0.02*j,0.02*i,0),
        side ? vec::UnitX():vec::UnitZ());
      plane.node_indices.push_back(base+i*6+j);
    }
    s.planes.clusters.push_back(plane);
  }
  edge(s.map,0,36);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),2U);
  EXPECT_EQ(r.patch_edges.size(),1U);
  EXPECT_TRUE(r.smooth_edges.empty());
  for (const auto &region:r.regions) EXPECT_EQ(region.shape.type,"plane");
}

TEST(SurfaceModel, TangentWallIsNotAbsorbedIntoCylinder)
{
  auto s=cylinder(0.1,0.1);
  const std::size_t wall_begin=s.map.nodes.size();
  ais_gng_msgs::msg::PlaneCluster wall;
  wall.id=999;
  for (int y=0;y<15;++y) for (int z=0;z<8;++z) {
    wall.node_indices.push_back(s.map.nodes.size());
    add_node(s.map,vec(0.1,0.02*y-0.14,0.04*z-0.14),vec::UnitX());
  }
  s.planes.clusters.push_back(wall);
  edge(s.map,3,wall_begin+7*8+4);
  options config;
  config.max_model_fits=256;
  const auto r=extract(s.map,s.planes,config);
  bool has_wall=false, has_curve=false;
  for (const auto &region:r.regions) {
    if (std::find(region.node_indices.begin(),region.node_indices.end(),wall_begin)!=region.node_indices.end()) {
      has_wall=true;
      EXPECT_TRUE(region.shape.type=="plane" || region.shape.type=="unknown")
        << region.shape.type << " rms=" << region.shape.rms;
    }
    if (region.shape.type=="cylinder") has_curve=true;
  }
  EXPECT_TRUE(has_wall);
  EXPECT_TRUE(has_curve);
  coverage(r,s.map.nodes.size());
}

TEST(SurfaceModel, SmoothedCornerNormalsDoNotJoinNearOrthogonalPlanePatches)
{
  scene s;
  for (int side=0; side<2; ++side) {
    ais_gng_msgs::msg::PlaneCluster plane;
    plane.id=side+1;
    const vec n=side ? vec::UnitX():vec::UnitZ();
    plane.normal.x=n.x(); plane.normal.y=n.y(); plane.normal.z=n.z();
    for (int x=0; x<10; ++x) for (int y=0; y<8; ++y) {
      const double along=0.01+0.006*x;
      const double noise=2e-6/along+1e-5*std::sin(3*x+y);
      plane.node_indices.push_back(s.map.nodes.size());
      add_node(s.map,side ? vec(noise,0.006*y,along):vec(along,0.006*y,noise),
        x==0 && y==0 ? vec(1,0,1).normalized():n);
    }
    s.planes.clusters.push_back(plane);
  }
  edge(s.map,0,80);
  const auto r=extract(s.map,s.planes);
  EXPECT_TRUE(r.smooth_edges.empty());
  ASSERT_EQ(r.sharp_edges.size(),1U);
  ASSERT_EQ(r.regions.size(),2U);
  for (const auto &region:r.regions) EXPECT_EQ(region.shape.type,"plane");
  // 境界から離れた代表法線による、非平面ノードを介した迂回経路。
  add_node(s.map,vec(0.006,0,0.005),vec(0.5,0,std::sqrt(0.75)));
  add_node(s.map,vec(0.005,0,0.006),vec(std::sqrt(0.75),0,0.5));
  edge(s.map,0,160); edge(s.map,160,161); edge(s.map,161,80);
  const auto detour=extract(s.map,s.planes);
  EXPECT_EQ(detour.smooth_edges.size(),3U);
  ASSERT_EQ(detour.sharp_edges.size(),1U);
  for (const auto &region:detour.regions) {
    EXPECT_FALSE(std::find(region.patch_indices.begin(),region.patch_indices.end(),0)!=region.patch_indices.end() &&
      std::find(region.patch_indices.begin(),region.patch_indices.end(),1)!=region.patch_indices.end());
  }
  coverage(detour,s.map.nodes.size());
  const auto data=nlohmann::json::parse(serialize(detour,s.map,s.planes));
  EXPECT_EQ(data["sharp_edges"].size(),1U);
  // 維持候補でも、現在の鋭い境界をまたぐ所属は不採用。
  region previous;
  previous.id=1U<<30;
  previous.shape.type="cylinder";
  for (std::size_t i=0; i<s.map.nodes.size(); ++i) previous.node_indices.push_back(i);
  const auto rejected=extract(s.map,s.planes,{}, {previous});
  for (const auto &surface:rejected.regions) EXPECT_FALSE(surface.is_retained);
  coverage(rejected,s.map.nodes.size());
}

TEST(SurfaceModel, ShallowCreaseIsNotAQuadricMadeOfTwoPlanes)
{
  scene s;
  for (int side=0; side<2; ++side) {
    ais_gng_msgs::msg::PlaneCluster plane;
    plane.id=side+1;
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(side*pi/6,vec::UnitY()).toRotationMatrix();
    for (int x=0; x<10; ++x) for (int y=0; y<10; ++y) {
      plane.node_indices.push_back(s.map.nodes.size());
      add_node(s.map,rotation*vec((side ? -1:1)*(0.01+0.04*x),0.04*y-0.18,0),
        rotation*vec::UnitZ());
    }
    s.planes.clusters.push_back(plane);
  }
  edge(s.map,0,100);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.smooth_edges.size(),1U);
  ASSERT_EQ(r.regions.size(),2U);
  for (const auto &region:r.regions) EXPECT_EQ(region.shape.type,"plane");
}

TEST(SurfaceModel, ExactlyTwoPlanePatchesCanMergeIntoVisibleCylinder)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int t=0; t<20; ++t) for (int z=0; z<6; ++z) {
    const double angle=(t-9.5)*pi/45;
    const vec normal(std::cos(angle),std::sin(angle),0);
    const auto idx=s.map.nodes.size();
    s.planes.clusters[t/10].node_indices.push_back(idx);
    add_node(s.map,0.1*normal+vec(0,0,0.02*z),normal);
    if (z) edge(s.map,idx,idx-1);
    if (t) edge(s.map,idx,idx-6);
  }
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"cylinder");
  ASSERT_EQ(r.patches.size(),2U);
  for (const auto &patch:r.patches) EXPECT_LT(patch.curvature.plane_rms,0.002);
  std::set<std::pair<std::string,int>> published;
  const auto markers=make_markers(r,s.map,false,false,published);
  ASSERT_FALSE(markers.markers.empty());
  EXPECT_EQ(markers.markers[0].points.size(),s.map.nodes.size());
  const auto data=nlohmann::json::parse(serialize(r,s.map,s.planes));
  EXPECT_EQ(data["models"][0]["plane_patch_num"],2);
  EXPECT_EQ(data["models"][0]["is_display_candidate"],true);
  EXPECT_EQ(data["patches"][0]["curvature"]["method"],"position_quadratic");
  EXPECT_TRUE(data["patches"][0]["curvature"].contains("position_rms_m"));
}

TEST(SurfaceModel, CurvedRegionCanContainOrthogonalPlanePatches)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int side=0; side<2; ++side) {
    const double angle=(side ? 1:-1)*pi/4;
    const double sign=side ? -1:1;
    auto &n=s.planes.clusters[side].normal;
    n.x=sign*std::cos(angle); n.y=sign*std::sin(angle);
  }
  for (int t=0; t<19; ++t) for (int z=0; z<6; ++z) {
    const double angle=(t-9)*pi/30;
    const vec n(std::cos(angle),std::sin(angle),0);
    const auto idx=s.map.nodes.size();
    if (t<4) s.planes.clusters[0].node_indices.push_back(idx);
    if (t>=15) s.planes.clusters[1].node_indices.push_back(idx);
    add_node(s.map,0.02*n+vec(0,0,0.004*z),n);
    if (z) edge(s.map,idx,idx-1);
    if (t) edge(s.map,idx,idx-6);
  }
  // 曲率を持つ2パッチ間の直接edgeと、曲面に沿う非平面ノードの経路。
  edge(s.map,3*6,15*6);
  const auto r=extract(s.map,s.planes);
  for (const auto &patch:r.patches) {
    if (patch.plane_cluster_idx<0) continue;
    EXPECT_LT(patch.curvature.plane_rms,0.002);
    EXPECT_GT(patch.curvature.confidence,0.5);
  }
  EXPECT_TRUE(r.sharp_edges.empty());
  const std::array<std::uint32_t,2> pair{0,1};
  EXPECT_NE(std::find(r.patch_edges.begin(),r.patch_edges.end(),pair),r.patch_edges.end());
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"cylinder");
}

TEST(SurfaceModel, SpherePreferredToMoreComplexQuadric)
{
  scene s;
  for (int latitude=1;latitude<12;++latitude) for (int t=0;t<24;++t) {
    const double phi=pi*latitude/12, theta=2*pi*t/24;
    const vec n(std::sin(phi)*std::cos(theta),std::sin(phi)*std::sin(theta),std::cos(phi));
    add_node(s.map,0.12*n,n);
    const auto idx=s.map.nodes.size()-1;
    if (t) edge(s.map,idx,idx-1);
    if (latitude>1) edge(s.map,idx,idx-24);
  }
  options config; config.min_plane_usage_ratio=0;
  const auto r=extract(s.map,s.planes,config);
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"sphere");
}

TEST(SurfaceModel, UnknownSingletonsAndBudgetExhaustionPreserveAllNodes)
{
  auto s=cylinder(0.1,0.055);
  add_node(s.map,vec(10,10,10),vec::Zero());
  add_node(s.map,vec(12,12,12),vec::Zero());
  options config;
  config.max_model_fits=1;
  const auto r=extract(s.map,s.planes,config);
  coverage(r,s.map.nodes.size());
  EXPECT_EQ(r.model_fits,1U);
  EXPECT_EQ(std::count_if(r.regions.begin(),r.regions.end(),[](const auto &region) {
    return region.shape.type=="unknown";
  }),2);
}

TEST(SurfaceModel, FrameMismatchRejected)
{
  auto s=cylinder(0.1,0.1);
  s.planes.frame_number=2;
  EXPECT_THROW(extract(s.map,s.planes),std::invalid_argument);
  s.planes.frame_number=0;
  s.planes.header.frame_id="other";
  EXPECT_THROW(extract(s.map,s.planes),std::invalid_argument);
}

TEST(SurfaceModel, MarkersUseUnifiedMembershipColorsAndDeleteVanishedEdges)
{
  auto s=cylinder(0.1,0.1);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),1U);
  std::set<std::pair<std::string,int>> published;
  const auto markers=make_markers(r,s.map,true,true,published);
  std_msgs::msg::ColorRGBA node_color, edge_color;
  std::size_t node_num=0, edge_num=0;
  for (const auto &m:markers.markers) {
    if (m.ns=="surface_nodes") {
      EXPECT_EQ(m.type,visualization_msgs::msg::Marker::SPHERE_LIST);
      node_color=m.color; node_num+=m.points.size();
    }
    if (m.ns=="surface_edges") { edge_color=m.color; edge_num+=m.points.size()/2; }
  }
  EXPECT_EQ(node_num,s.map.nodes.size());
  EXPECT_EQ(edge_num,s.map.edges.size()/2);
  EXPECT_EQ(node_color,edge_color);
  for (const auto &m:markers.markers) {
    if (m.ns=="surface_patches" || m.ns=="surface_patch_edges") { EXPECT_EQ(m.color,node_color); }
  }
  s.map.edges.clear();
  const auto next=make_markers(r,s.map,false,false,published);
  EXPECT_TRUE(std::any_of(next.markers.begin(),next.markers.end(),[](const auto &m) {
    return m.ns=="surface_edges" && m.action==visualization_msgs::msg::Marker::DELETE;
  }));
  const auto data=nlohmann::json::parse(serialize(r,s.map,s.planes));
  EXPECT_EQ(data["models"][0]["type"],"cylinder");
  EXPECT_EQ(data["models"][0]["node_indices"].size(),s.map.nodes.size());
}

TEST(SurfaceModel, SeparateRegionsHaveDifferentColorsAndUnknownIsHidden)
{
  auto s=cylinder(0.1,0.1);
  const std::size_t base=s.map.nodes.size();
  auto second=cylinder(0.08,0.08,48,true,Eigen::Matrix3d::Identity(),vec(0.4,0,0));
  s.map.nodes.insert(s.map.nodes.end(),second.map.nodes.begin(),second.map.nodes.end());
  for (auto idx:second.map.edges) s.map.edges.push_back(base+idx);
  for (auto patch:second.planes.clusters) {
    for (auto &idx:patch.node_indices) idx+=base;
    s.planes.clusters.push_back(patch);
  }
  add_node(s.map,vec(10,10,10),vec::Zero());
  auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),3U);
  // IDの差が黄金比色相の周期に近い場合にも、可視的な色差を確保。
  r.regions[1].id=21;
  std::set<std::pair<std::string,int>> published;
  const auto markers=make_markers(r,s.map,false,false,published);
  std::vector<std_msgs::msg::ColorRGBA> colors;
  bool has_gray=false;
  for (const auto &m:markers.markers) {
    if (m.ns=="surface_nodes") colors.push_back(m.color);
    if (m.ns=="surface_unknown_nodes") {
      EXPECT_FLOAT_EQ(m.color.r,m.color.g);
      EXPECT_FLOAT_EQ(m.color.g,m.color.b);
      has_gray=true;
    }
  }
  ASSERT_EQ(colors.size(),2U);
  EXPECT_NE(colors[0],colors[1]);
  EXPECT_GT(std::abs(colors[0].r-colors[1].r)+std::abs(colors[0].g-colors[1].g)+
    std::abs(colors[0].b-colors[1].b),0.8F);
  EXPECT_FALSE(has_gray);
}

TEST(SurfaceModel, SinglePlaneWithNonplaneNodesIsHiddenAndDeletesPreviousMarkers)
{
  const auto s=cylinder(0.1,0.1);
  auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),1U);
  std::set<std::pair<std::string,int>> published;
  make_markers(r,s.map,true,true,published);
  ASSERT_FALSE(published.empty());
  const auto previous=published;
  // 表示判定単体の試験用所属。先頭の平面以外を非平面パッチへ変更。
  for (std::size_t i=1; i<r.patches.size(); ++i) r.patches[i].plane_cluster_idx=-1;
  const auto markers=make_markers(r,s.map,true,true,published);
  EXPECT_TRUE(published.empty());
  EXPECT_EQ(markers.markers.size(),previous.size());
  for (const auto &m:markers.markers) {
    EXPECT_EQ(m.action,visualization_msgs::msg::Marker::DELETE);
    EXPECT_EQ(previous.count({m.ns,m.id}),1U);
  }
  EXPECT_TRUE(make_markers(r,s.map,true,true,published).markers.empty());
  const auto data=nlohmann::json::parse(serialize(r,s.map,s.planes));
  EXPECT_EQ(data["models"][0]["plane_patch_num"],1);
  EXPECT_EQ(data["models"][0]["is_display_candidate"],false);
  EXPECT_EQ(data["models"][0]["node_indices"].size(),s.map.nodes.size());
  EXPECT_FALSE(make_markers(r,s.map,true,true,published,1).markers.empty());
}

TEST(SurfaceModel, NonplaneOnlyCurveNeedsExplicitDisplayOverride)
{
  const auto s=cylinder(0.1,0.1,48,false);
  options config; config.min_plane_usage_ratio=0;
  const auto r=extract(s.map,s.planes,config);
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"cylinder");
  std::set<std::pair<std::string,int>> published;
  EXPECT_TRUE(make_markers(r,s.map,true,true,published).markers.empty());
  EXPECT_FALSE(make_markers(r,s.map,true,true,published,0).markers.empty());
}

TEST(SurfaceModel, PlaneModelsAreHiddenEvenWithMultiplePlanePatches)
{
  const auto s=cylinder(0.1,0.1);
  auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),1U);
  r.regions[0].shape.type="plane";
  std::set<std::pair<std::string,int>> published;
  EXPECT_TRUE(make_markers(r,s.map,true,true,published).markers.empty());
  EXPECT_TRUE(make_markers(r,s.map,true,true,published,0).markers.empty());
}

TEST(SurfaceModel, TopologicalGraphPreservesNodeIdsAndOnlyRealInternalEdges)
{
  auto s=cylinder(0.1,0.1);
  add_node(s.map,vec(10,10,10),vec::Zero());
  const auto unknown=s.map.nodes.size()-1;
  edge(s.map,0,unknown);
  edge(s.map,0,0);
  s.map.nodes[0].inpcl_ids={1,2,3};
  s.map.nodes[0].winner_point_count=42;
  s.map.nodes[0].label=map_type::WALL;
  s.map.nodes[0].winner_point_covariance[0]=0.001F;
  s.map.frame_number=7;
  s.planes.frame_number=7;
  const auto surfaces=extract(s.map,s.planes);
  const auto graph=make_graph(surfaces,s.map);
  EXPECT_EQ(graph.header,s.map.header);
  EXPECT_EQ(graph.frame_number,7U);
  ASSERT_EQ(graph.nodes.size(),s.map.nodes.size());
  for (std::size_t i=0; i<graph.nodes.size(); ++i) {
    auto expected=s.map.nodes[i];
    expected.inpcl_ids.clear();
    EXPECT_EQ(graph.nodes[i],expected);
  }
  ASSERT_EQ(graph.clusters.size(),1U);
  EXPECT_EQ(graph.clusters[0].nodes.front(),s.map.nodes[0].id);
  EXPECT_NE(graph.clusters[0].nodes.front(),0U);
  EXPECT_EQ(graph.clusters[0].nodes.size(),unknown);
  EXPECT_EQ(graph.clusters[0].frame,7U);
  EXPECT_EQ(graph.edges.size(),s.map.edges.size()-4);
  EXPECT_EQ(std::count(graph.edges.begin(),graph.edges.end(),unknown),0);
  EXPECT_EQ(make_graph(result{},map_type{}).nodes.size(),0U);
}

TEST(PatchCurvature, CylinderRetainsDirectionalBendingInsidePlanePatch)
{
  auto s=cylinder(0.1,0.1,72);
  auto r=extract(s.map,s.planes);
  for (const auto &patch:r.patches) {
    if (patch.plane_cluster_idx<0) { EXPECT_FALSE(patch.curvature.valid); continue; }
    const auto &c=patch.curvature;
    ASSERT_TRUE(c.valid);
    // 円弧の二次近似による有限幅誤差と、世界座標基準の法線符号。
    EXPECT_NEAR(std::abs(c.kappa.x()),10.0,0.1);
    EXPECT_NEAR(c.kappa.y(),0,1e-4);
    EXPECT_LT(c.plane_rms,0.001);
    EXPECT_LT(c.fit_error,1e-6);
    const vec flat_dir=c.axis_u*c.directions_uv(0,1)+c.axis_v*c.directions_uv(1,1);
    EXPECT_GT(std::abs(flat_dir.dot(vec::UnitZ())),0.999);
  }
  for (std::size_t i=1;i<s.map.nodes.size();i+=2) {
    auto &n=s.map.nodes[i].normal;
    n.x=-n.x; n.y=-n.y; n.z=-n.z;
  }
  const auto flipped=extract(s.map,s.planes);
  ASSERT_TRUE(flipped.patches[0].curvature.valid);
  EXPECT_NEAR(flipped.patches[0].curvature.kappa.x(),r.patches[0].curvature.kappa.x(),1e-4);
}

TEST(PatchCurvature, PlaneSphereAndSaddleAreDistinguished)
{
  for (int type=0;type<3;++type) {
    map_type map;
    local_patch patch;
    for (int i=-3;i<=3;++i) for (int j=-3;j<=3;++j) {
      const double x=0.003*i,y=0.003*j;
      vec p(x,y,0),n=vec::UnitZ();
      if (type==1) { p.z()=std::sqrt(0.04-x*x-y*y); n=p.normalized(); }
      if (type==2) { p.z()=2.5*(x*x-y*y); n=vec(-5*x,5*y,1).normalized(); }
      patch.node_indices.push_back(map.nodes.size());
      add_node(map,p,n);
    }
    const auto c=estimate_curvature(patch,map);
    ASSERT_TRUE(c.valid);
    if (type==0) { EXPECT_LT(c.kappa.norm(),1e-8); }
    if (type==1) {
      EXPECT_NEAR(c.kappa.x(),-5.0,0.02);
      EXPECT_NEAR(c.kappa.y(),-5.0,0.02);
    }
    if (type==2) { EXPECT_LT(c.kappa.x()*c.kappa.y(),-24.0); }
    EXPECT_GT(c.confidence,0.99);
  }
}

TEST(PatchCurvature, LineSupportIsInvalidNotFlat)
{
  map_type map;
  local_patch patch;
  for (int i=0;i<12;++i) {
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(0.01*i,0,0),vec::UnitZ());
  }
  EXPECT_FALSE(estimate_curvature(patch,map).valid);
  for (auto &node:map.nodes) node.normal=geometry_msgs::msg::Point32();
  EXPECT_FALSE(estimate_curvature(patch,map).valid);
}

TEST(PatchCurvature, PositionsAloneRecoverQuadraticWithArbitraryNormals)
{
  map_type map;
  local_patch patch;
  for (int i=-5;i<=5;++i) for (int j=-5;j<=5;++j) {
    const double x=0.004*i,y=0.004*j;
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(x,y,3*x*x-y*y),vec::Zero());
  }
  const auto original=estimate_curvature(patch,map);
  ASSERT_TRUE(original.valid);
  EXPECT_NEAR(original.kappa.x(),6,1e-4);
  EXPECT_NEAR(original.kappa.y(),-2,1e-4);
  EXPECT_GT(original.confidence,0.99);
  for (std::size_t i=0;i<map.nodes.size();++i) {
    auto &n=map.nodes[i].normal;
    n.x=std::sin(i); n.y=std::cos(i); n.z=std::numeric_limits<float>::quiet_NaN();
  }
  const auto changed=estimate_curvature(patch,map);
  ASSERT_TRUE(changed.valid);
  EXPECT_NEAR((changed.tensor-original.tensor).norm(),0,1e-12);
  const vec point(0.013,-0.007,0);
  const vec expected=vec(-6*point.x(),2*point.y(),1).normalized();
  EXPECT_GT(patch_normal_at(changed,point).dot(expected),0.999999);
}

TEST(PatchCurvature, RotationTranslationAndDensityPreserveGeometry)
{
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(0.8,vec(1,2,3).normalized()).toRotationMatrix();
  const vec offset(0.2,-0.3,0.4);
  for (int count:{7,11,19}) {
    map_type map;
    local_patch patch;
    for (int i=0;i<count;++i) for (int j=0;j<count;++j) {
      const double x=0.02*(2.0*i/(count-1)-1),y=0.02*(2.0*j/(count-1)-1);
      patch.node_indices.push_back(map.nodes.size());
      add_node(map,rotation*vec(x,y,3*x*x-y*y)+offset,vec::Zero());
    }
    const auto c=estimate_curvature(patch,map);
    ASSERT_TRUE(c.valid);
    EXPECT_NEAR(std::abs(c.kappa.x()),6,0.001);
    EXPECT_NEAR(std::abs(c.kappa.y()),2,0.001);
    EXPECT_LT(c.kappa.prod(),0);
    EXPECT_GT(std::abs(patch_normal_at(c,offset).dot(rotation*vec::UnitZ())),0.999999);
  }
}

TEST(PatchCurvature, RobustFitLimitsIsolatedHeightOutlier)
{
  map_type map;
  local_patch patch;
  for (int i=-6;i<=6;++i) for (int j=-6;j<=6;++j) {
    const double x=0.004*i,y=0.004*j;
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(x,y,3*x*x+(i==2 && j==1 ? 0.006:0)),vec::Zero());
  }
  const auto c=estimate_curvature(patch,map);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(std::abs(c.kappa.x()),6,0.1);
  EXPECT_LT(std::abs(c.kappa.y()),0.1);
}

TEST(PatchCurvature, DenseSubregionDoesNotDominatePositionFit)
{
  map_type map;
  local_patch patch;
  for (int i=-5;i<=5;++i) for (int j=-5;j<=5;++j) {
    const double x=0.004*i,y=0.004*j;
    const int repeats=(i>1 && j>1) ? 12:1;
    for (int copy=0;copy<repeats;++copy) {
      patch.node_indices.push_back(map.nodes.size());
      add_node(map,vec(x,y,3*x*x+0.00001*std::sin(i+3*j)),vec::Zero());
    }
  }
  const auto c=estimate_curvature(patch,map);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(std::abs(c.kappa.x()),6,0.15);
  EXPECT_LT(std::abs(c.kappa.y()),0.1);
  EXPECT_GT(std::abs(patch_normal_at(c,vec::Zero()).dot(vec::UnitZ())),0.9999);
}

TEST(PatchCurvature, UnstructuredHeightNoiseHasLowConfidence)
{
  map_type map;
  local_patch patch;
  for (int i=-5;i<=5;++i) for (int j=-5;j<=5;++j) {
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(0.004*i,0.004*j,0.002*std::sin(17*i+13*j)),vec::UnitZ());
  }
  const auto c=estimate_curvature(patch,map);
  ASSERT_TRUE(c.valid);
  EXPECT_LT(c.confidence,0.5);
}

TEST(PatchCurvature, InvalidPositionsDoNotContaminateFit)
{
  map_type map;
  local_patch patch;
  for (int i=-2;i<=2;++i) for (int j=-2;j<=2;++j) {
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(0.01*i,0.01*j,0),vec::Zero());
  }
  patch.node_indices.push_back(map.nodes.size());
  add_node(map,vec(std::numeric_limits<double>::infinity(),0,0),vec::Zero());
  patch.node_indices.push_back(9999);
  const auto c=estimate_curvature(patch,map);
  ASSERT_TRUE(c.valid);
  EXPECT_EQ(c.sample_num,25U);
  EXPECT_LT(c.kappa.norm(),1e-8);
}

TEST(PatchCurvature, RingAndInsufficientSamplesDoNotDetermineQuadratic)
{
  map_type map;
  local_patch patch;
  for (int i=0;i<20;++i) {
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(0.02*std::cos(2*pi*i/20),0.02*std::sin(2*pi*i/20),0),vec::UnitZ());
  }
  EXPECT_FALSE(estimate_curvature(patch,map).valid);
  patch.node_indices.resize(7);
  EXPECT_FALSE(estimate_curvature(patch,map).valid);
}

TEST(PatchCurvature, ConvergedFitStopsEarly)
{
  map_type map;
  local_patch patch;
  for (int i=-5;i<=5;++i) for (int j=-5;j<=5;++j) {
    const double x=0.004*i,y=0.004*j;
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(x,y,3*x*x-y*y),vec::Zero());
  }
  const auto first=estimate_curvature(patch,map);
  ASSERT_TRUE(first.valid);
  EXPECT_LT(first.fit_iter,4U);
  EXPECT_FALSE(first.has_svd_fallback);
}

TEST(PatchCurvature, IllConditionedTwoDimensionalSupportUsesSvdFallback)
{
  map_type map;
  local_patch patch;
  for (int i=-5;i<=5;++i) for (int j=-5;j<=5;++j) {
    const double x=0.004*i,y=0.00006*j;
    patch.node_indices.push_back(map.nodes.size());
    add_node(map,vec(x,y,0),vec::Zero());
  }
  const auto c=estimate_curvature(patch,map);
  ASSERT_TRUE(c.valid);
  EXPECT_TRUE(c.has_svd_fallback);
  EXPECT_LT(c.kappa.norm(),1e-8);
}

TEST(SurfaceModel, UncertainBoundaryIsNotAConfirmedSharpOrSmoothEdge)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int side=0;side<2;++side) for (int i=0;i<7;++i) {
    const double x=0.04*(i%3),y=0.04*(i/3);
    s.planes.clusters[side].node_indices.push_back(s.map.nodes.size());
    add_node(s.map,side ? vec(0,x,y):vec(x,y,0),side ? vec::UnitX():vec::UnitZ());
  }
  edge(s.map,0,7);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.uncertain_edges.size(),1U);
  EXPECT_TRUE(r.sharp_edges.empty());
  EXPECT_TRUE(r.smooth_edges.empty());
  // 不明境界だけを根拠にした、直交する2面の曲面化の禁止。
  for (const auto &region:r.regions) EXPECT_TRUE(region.shape.type=="unknown" || region.shape.type=="plane");
  coverage(r,s.map.nodes.size());
  const auto data=nlohmann::json::parse(serialize(r,s.map,s.planes));
  EXPECT_EQ(data["uncertain_edges"].size(),1U);
}

TEST(SurfaceModel, BoundaryFitBudgetLimitsAdditionalWork)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int side=0;side<2;++side) for (int i=0;i<10;++i) for (int j=0;j<10;++j) {
    s.planes.clusters[side].node_indices.push_back(s.map.nodes.size());
    add_node(s.map,vec((side ? -1:1)*(0.002+0.006*i),0.006*(j-4.5),
      0.003*std::sin(17*i+13*j)),vec::UnitZ());
  }
  edge(s.map,4,104);
  options config;
  config.max_boundary_fits=0;
  const auto disabled=extract(s.map,s.planes,config);
  EXPECT_EQ(disabled.boundary_fit_num,0U);
  EXPECT_EQ(disabled.uncertain_edges.size(),1U);
  config.max_boundary_fits=1;
  const auto limited=extract(s.map,s.planes,config);
  EXPECT_EQ(limited.boundary_fit_num,1U);
  EXPECT_EQ(limited.uncertain_edges.size(),1U);
  config.max_boundary_fits=32;
  const auto full=extract(s.map,s.planes,config);
  EXPECT_EQ(full.boundary_fit_num,2U);
  coverage(full,s.map.nodes.size());
}

TEST(SurfaceModel, FarExtrapolationOfTinyPatchesRemainsUncertain)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int side=0;side<2;++side) for (int i=-1;i<=1;++i) for (int j=-1;j<=1;++j) {
    s.planes.clusters[side].node_indices.push_back(s.map.nodes.size());
    add_node(s.map,side ? vec(0.04,0.001*i,0.001*j):vec(0.001*i,0.001*j,0),
      side ? vec::UnitX():vec::UnitZ());
  }
  edge(s.map,4,13);
  const auto r=extract(s.map,s.planes);
  EXPECT_TRUE(r.sharp_edges.empty());
  EXPECT_EQ(r.uncertain_edges.size(),1U);
}

TEST(SurfaceModel, LocalBoundaryFitProtectsCornerDespiteRemoteNoise)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int side=0;side<2;++side) for (int i=0;i<10;++i) for (int j=0;j<10;++j) {
    const double along=0.002+0.01*i,y=0.006*(j-4.5);
    const double noise=i>=5 ? 0.015*std::sin(13*i+7*j):0;
    s.planes.clusters[side].node_indices.push_back(s.map.nodes.size());
    add_node(s.map,side ? vec(noise,y,along):vec(along,y,noise),
      side ? vec::UnitX():vec::UnitZ());
  }
  edge(s.map,4,104);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.patches.size(),2U);
  for (const auto &p:r.patches) EXPECT_LT(p.curvature.confidence,0.5);
  EXPECT_EQ(r.boundary_fit_num,2U);
  EXPECT_EQ(r.sharp_edges.size(),1U);
  EXPECT_TRUE(r.uncertain_edges.empty());
  EXPECT_TRUE(r.smooth_edges.empty());
  for (const auto &region:r.regions) EXPECT_EQ(region.patch_indices.size(),1U);
}

TEST(SurfaceModel, PositionBoundaryTangentsJoinWideCylinderPatches)
{
  scene s;
  s.planes.clusters.resize(2);
  for (int t=0;t<24;++t) for (int z=0;z<8;++z) {
    const double angle=(t-11.5)*pi/36;
    const vec n(std::cos(angle),std::sin(angle),0);
    const auto idx=s.map.nodes.size();
    s.planes.clusters[t/12].node_indices.push_back(idx);
    // パッチ中心では約60度差、境界ノードでは連続する円筒面。
    add_node(s.map,0.06*n+vec(0,0,0.008*z),((t+z)%2 ? -1:1)*n);
    if (z) edge(s.map,idx,idx-1);
    if (t) edge(s.map,idx,idx-8);
  }
  const auto r=extract(s.map,s.planes);
  EXPECT_TRUE(r.sharp_edges.empty());
  ASSERT_EQ(r.smooth_edges.size(),1U);
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"cylinder");
  // 境界ノード法線だけの乱れによる、位置由来の連続面の二重拒否の防止。
  for (std::size_t i=11*8;i<12*8;++i) {
    s.map.nodes[i].normal.x=0; s.map.nodes[i].normal.y=0; s.map.nodes[i].normal.z=1;
  }
  const auto noisy=extract(s.map,s.planes);
  EXPECT_TRUE(noisy.sharp_edges.empty());
  EXPECT_EQ(noisy.smooth_edges.size(),1U);
  ASSERT_EQ(noisy.regions.size(),1U);
  EXPECT_EQ(noisy.regions[0].shape.type,"cylinder");
}

TEST(SurfaceTracking, small_missing_edges_keep_current_nodes_and_stable_id)
{
  auto s=cylinder(0.1,0.1);
  // 1 cm間隔の小欠損。2 cmの物理上限内での支持領域保持。
  for (auto &node:s.map.nodes) node.pos.z*=0.25;
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  ASSERT_EQ(first.regions.size(),1U);
  s.map.edges.clear();
  s.map.frame_number=s.planes.frame_number=1;
  s.map.nodes[0].pos.z+=0.001;
  const auto next=tracking.update(s.map,s.planes);
  ASSERT_EQ(next.regions.size(),1U);
  EXPECT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(next.regions[0].id,first.regions[0].id);
  EXPECT_EQ(next.model_fits,0U);
  coverage(next,s.map.nodes.size());
  std::set<std::pair<std::string,int>> published;
  make_markers(first,s.map,false,false,published);
  const auto markers=make_markers(next,s.map,false,false,published);
  ASSERT_EQ(markers.markers.size(),1U);
  EXPECT_FLOAT_EQ(markers.markers[0].points[0].z,s.map.nodes[0].pos.z);
  EXPECT_EQ(markers.markers[0].points.size(),s.map.nodes.size());
}

TEST(SurfaceTracking, disabled_support_regions_keep_disconnected_nodes_and_stable_id)
{
  for (bool enable_support_regions:{false,true}) {
    auto s=cylinder(0.1,0.1);
    options config;
    config.enable_support_regions=enable_support_regions;
    tracker tracking;
    const auto first=tracking.update(s.map,s.planes,config);
    ASSERT_EQ(first.regions.size(),1U);
    s.map.edges.clear();
    for (int iter=0;iter<3;++iter) {
      ++s.map.frame_number; s.planes.frame_number=s.map.frame_number;
      const auto next=tracking.update(s.map,s.planes,config);
      coverage(next,s.map.nodes.size());
      if (enable_support_regions) {
        EXPECT_GT(next.regions.size(),1U);
        continue;
      }
      ASSERT_EQ(next.regions.size(),1U);
      EXPECT_TRUE(next.regions[0].is_retained);
      EXPECT_EQ(next.regions[0].id,first.regions[0].id);
      EXPECT_EQ(std::set<std::uint32_t>(next.regions[0].node_indices.begin(),next.regions[0].node_indices.end()),
        std::set<std::uint32_t>(first.regions[0].node_indices.begin(),first.regions[0].node_indices.end()));
      EXPECT_EQ(next.regions[0].shape.q,first.regions[0].shape.q);
      EXPECT_EQ(next.model_fits,0U);
      EXPECT_DOUBLE_EQ(next.support_ms,0.0);
      EXPECT_EQ(next.support_split_num,0U);
      EXPECT_EQ(next.support_gap_links,0U);
    }
  }
}

TEST(SurfaceModel, disabled_support_regions_skip_direct_split)
{
  auto s=cylinder(0.1,0.1);
  options config;
  config.enable_support_regions=false;
  auto surfaces=extract(s.map,s.planes,config);
  ASSERT_EQ(surfaces.regions.size(),1U);
  EXPECT_DOUBLE_EQ(surfaces.support_ms,0.0);
  const auto before=surfaces;
  s.map.edges.clear();
  split_support_regions(surfaces,s.map,config);
  ASSERT_EQ(surfaces.regions.size(),1U);
  EXPECT_EQ(surfaces.regions[0].node_indices,before.regions[0].node_indices);
  EXPECT_EQ(surfaces.regions[0].shape.q,before.regions[0].shape.q);
  EXPECT_EQ(surfaces.patches.size(),before.patches.size());
  EXPECT_DOUBLE_EQ(surfaces.support_ms,0.0);
  EXPECT_EQ(surfaces.support_split_num,0U);
  EXPECT_EQ(surfaces.support_gap_links,0U);
}

TEST(SurfaceTracking, distant_bands_on_same_cylinder_get_stable_separate_ids)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  ASSERT_EQ(first.regions.size(),1U);
  std::vector<std::uint16_t> edges;
  for (std::size_t idx=0;idx+1<s.map.edges.size();idx+=2) {
    const auto a=s.map.edges[idx],b=s.map.edges[idx+1];
    if ((a%6<3)!=(b%6<3)) continue;
    edges.push_back(a); edges.push_back(b);
  }
  s.map.edges=edges;
  for (std::size_t idx=0;idx<s.map.nodes.size();++idx) if (idx%6>=3) s.map.nodes[idx].pos.z+=0.03;
  ++s.map.frame_number; s.planes.frame_number=s.map.frame_number;
  const auto split=tracking.update(s.map,s.planes);
  ASSERT_EQ(split.regions.size(),2U);
  EXPECT_EQ(split.support_split_num,1U);
  EXPECT_NE(split.regions[0].id,split.regions[1].id);
  std::set<std::uint32_t> ids;
  for (const auto &region:split.regions) {
    ids.insert(region.id);
    EXPECT_TRUE(region.is_retained);
    EXPECT_EQ(region.support_parent_id,first.regions[0].id);
    EXPECT_EQ(region.shape.q,first.regions[0].shape.q);
    EXPECT_EQ(region.node_indices.size(),s.map.nodes.size()/2);
    const bool is_upper=region.node_indices.front()%6>=3;
    for (auto idx:region.node_indices) EXPECT_EQ(idx%6>=3,is_upper);
    std::set<std::uint32_t> patch_nodes;
    for (auto idx:region.patch_indices) for (auto node_idx:split.patches[idx].node_indices) patch_nodes.insert(node_idx);
    EXPECT_EQ(patch_nodes,std::set<std::uint32_t>(region.node_indices.begin(),region.node_indices.end()));
  }
  EXPECT_TRUE(ids.count(first.regions[0].id));
  coverage(split,s.map.nodes.size());
  for (int iter=0;iter<3;++iter) {
    ++s.map.frame_number; s.planes.frame_number=s.map.frame_number;
    const auto next=tracking.update(s.map,s.planes);
    std::set<std::uint32_t> next_ids;
    for (const auto &region:next.regions) { next_ids.insert(region.id); EXPECT_TRUE(region.is_retained); }
    EXPECT_EQ(next_ids,ids);
    EXPECT_EQ(next.support_split_num,0U);
    coverage(next,s.map.nodes.size());
  }
}

TEST(SurfaceTracking, free_space_prevents_small_gap_completion)
{
  for (bool has_free_space:{false,true}) {
    auto s=cylinder(0.1,0.1);
    for (auto &node:s.map.nodes) node.pos.z*=0.25;
    tracker tracking;
    const auto first=tracking.update(s.map,s.planes);
    ASSERT_EQ(first.regions.size(),1U);
    std::vector<std::uint16_t> edges;
    for (std::size_t idx=0;idx+1<s.map.edges.size();idx+=2) {
      const auto a=s.map.edges[idx],b=s.map.edges[idx+1];
      if ((a%6<3)!=(b%6<3)) continue;
      edges.push_back(a); edges.push_back(b);
    }
    s.map.edges=edges;
    if (has_free_space) for (auto &node:s.map.nodes)
      node.boundary_evidence=ais_gng_msgs::msg::TopologicalNode::BOUNDARY_FREE_SPACE;
    const auto next=tracking.update(s.map,s.planes);
    EXPECT_EQ(next.regions.size(),has_free_space ? 2U:1U);
    EXPECT_EQ(next.support_gap_links>0,!has_free_space);
    coverage(next,s.map.nodes.size());
  }
}

TEST(SurfaceModel, support_gap_does_not_cross_opposite_cylinder_sides)
{
  scene s;
  result surfaces;
  region surface;
  surface.id=0; surface.shape.type="cylinder";
  surface.shape.q<<1,1,0,0,0,0,0,0,0,-0.005*0.005;
  surface.shape.score=0;
  local_patch patch;
  for (int side=0;side<2;++side) for (int along=0;along<4;++along) for (int row=0;row<4;++row) {
    const double angle=side*pi+0.04*along;
    const vec normal(std::cos(angle),std::sin(angle),0);
    const auto idx=s.map.nodes.size();
    add_node(s.map,0.005*normal+vec(0,0,0.001*row),normal);
    surface.node_indices.push_back(idx); patch.node_indices.push_back(idx);
    if (row) edge(s.map,idx,idx-1);
    if (along) edge(s.map,idx,idx-4);
  }
  surfaces.patches.push_back(patch); surface.patch_indices={0}; surfaces.regions.push_back(surface);
  split_support_regions(surfaces,s.map,{});
  ASSERT_EQ(surfaces.regions.size(),2U);
  EXPECT_NE(surfaces.regions[0].id,surfaces.regions[1].id);
  EXPECT_EQ(surfaces.support_gap_links,0U);
  EXPECT_EQ(surfaces.patches.size(),2U);
  coverage(surfaces,s.map.nodes.size());
}

TEST(SurfaceTracking, local_spacing_limits_gap_completion)
{
  for (double ratio:{1.0,2.5}) {
    auto s=cylinder(0.1,0.1);
    for (auto &node:s.map.nodes) node.pos.z*=0.25;
    tracker tracking;
    tracking.update(s.map,s.planes);
    std::vector<std::uint16_t> edges;
    for (std::size_t idx=0;idx+1<s.map.edges.size();idx+=2) {
      const auto a=s.map.edges[idx],b=s.map.edges[idx+1];
      if ((a%6<3)!=(b%6<3)) continue;
      edges.push_back(a); edges.push_back(b);
    }
    s.map.edges=edges;
    for (std::size_t idx=0;idx<s.map.nodes.size();++idx) if (idx%6>=3) s.map.nodes[idx].pos.z+=0.005;
    options config;
    config.max_support_spacing_ratio=ratio;
    const auto next=tracking.update(s.map,s.planes,config);
    EXPECT_EQ(next.regions.size(),ratio==1.0 ? 2U:1U);
    coverage(next,s.map.nodes.size());
  }
}

TEST(SurfaceModel, invalid_support_options_are_rejected)
{
  const auto s=cylinder(0.1,0.1);
  options config;
  config.max_support_gap=-0.01;
  EXPECT_THROW(extract(s.map,s.planes,config),std::invalid_argument);
  config.max_support_gap=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(extract(s.map,s.planes,config),std::invalid_argument);
  config.max_support_gap=0.02; config.max_support_spacing_ratio=0.5;
  EXPECT_THROW(extract(s.map,s.planes,config),std::invalid_argument);
  config=options{};
  for (double ratio:{-0.1,1.1,std::numeric_limits<double>::quiet_NaN()}) {
    config.min_plane_usage_ratio=ratio;
    EXPECT_THROW(extract(s.map,s.planes,config),std::invalid_argument);
  }
}

TEST(SurfaceModel, small_plane_fractions_are_rejected_after_support_split)
{
  auto s=cylinder(0.1,0.1);
  const auto original=s.map.edges;
  s.map.edges.clear();
  for (std::size_t i=0;i+1<original.size();i+=2)
    if (original[i]%6==original[i+1]%6) edge(s.map,original[i],original[i+1]);
  for (double ratio:{0.0,0.5}) {
    options config; config.min_plane_usage_ratio=ratio;
    const auto r=extract(s.map,s.planes,config);
    ASSERT_EQ(r.regions.size(),6U);
    for (const auto &surface:r.regions) EXPECT_EQ(surface.shape.type,ratio==0 ? "cylinder":"unknown");
    coverage(r,s.map.nodes.size());
  }
}

TEST(SurfaceTracking, current_plane_usage_requires_one_core_and_releases_track)
{
  for (int extra_num:{24,48}) for (bool has_core:{false,true}) {
    auto s=cylinder(0.1,0.1);
    tracker tracking;
    const auto first=tracking.update(s.map,s.planes);
    ASSERT_EQ(first.regions.size(),1U);
    // 各元平面へ遠方の観測を追加。旧追跡核は全点適合のまま、元平面の使用率だけが低下。
    for (std::size_t i=has_core ? 1:0;i<s.planes.clusters.size();++i) {
      for (int j=0;j<extra_num;++j) {
        s.planes.clusters[i].node_indices.push_back(s.map.nodes.size());
        add_node(s.map,vec(10+i,0.01*(j%8),0.01*(j/8)),vec::UnitX());
      }
    }
    ++s.map.frame_number; s.planes.frame_number=s.map.frame_number;
    const auto r=tracking.update(s.map,s.planes);
    const auto core=std::find_if(r.regions.begin(),r.regions.end(),[](const auto &surface) {
      return std::find(surface.node_indices.begin(),surface.node_indices.end(),0)!=surface.node_indices.end();
    });
    ASSERT_NE(core,r.regions.end());
    const bool can_keep=has_core || extra_num==24;
    EXPECT_EQ(core->is_retained,can_keep);
    EXPECT_EQ(core->shape.type,can_keep ? "cylinder":"unknown");
    coverage(r,s.map.nodes.size());
    if (!can_keep) {
      const auto next=tracking.update(s.map,s.planes);
      for (const auto &surface:next.regions) EXPECT_FALSE(surface.is_retained);
    }
  }
}

TEST(SurfaceModel, insufficient_support_is_not_a_visible_curve)
{
  auto s=cylinder(0.1,0.1);
  auto surfaces=extract(s.map,s.planes);
  s.map.edges.clear();
  options config;
  config.max_support_gap=0;
  split_support_regions(surfaces,s.map,config);
  EXPECT_EQ(surfaces.regions.size(),s.map.nodes.size());
  for (const auto &region:surfaces.regions) EXPECT_EQ(region.shape.type,"unknown");
  coverage(surfaces,s.map.nodes.size());
}

TEST(SurfaceTracking, OutlierNodeIsRemovedAndCanRejoin)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  const auto position=s.map.nodes[0].pos;
  s.map.nodes[0].pos.x+=0.03;
  const auto next=tracking.update(s.map,s.planes);
  ASSERT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(next.regions[0].id,first.regions[0].id);
  EXPECT_EQ(next.regions[0].node_indices.size(),s.map.nodes.size()-1);
  EXPECT_EQ(next.regions[0].rejected_node_num,1U);
  EXPECT_EQ(std::count(next.regions[0].node_indices.begin(),next.regions[0].node_indices.end(),0),0);
  coverage(next,s.map.nodes.size());
  s.map.nodes[0].pos=position;
  const auto restored=tracking.update(s.map,s.planes);
  ASSERT_EQ(restored.regions.size(),1U);
  EXPECT_TRUE(restored.regions[0].is_retained);
  EXPECT_EQ(restored.regions[0].rejected_node_num,0U);
  coverage(restored,s.map.nodes.size());
}

TEST(SurfaceTracking, ConnectedNewNonplaneNodesJoinWithoutRefitting)
{
  auto s=cylinder(0.1,0.1);
  for (auto &node:s.map.nodes) node.pos.z*=0.25;
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  ASSERT_EQ(first.regions.size(),1U);
  const auto begin=s.map.nodes.size();
  add_node(s.map,vec(0.1,0,0.045),vec::UnitX());
  add_node(s.map,vec(0.1,0,0.065),vec::UnitX());
  add_node(s.map,vec(0.13,0,0.065),vec::UnitX());
  add_node(s.map,vec(0.1,0,0.085),vec::UnitZ());
  add_node(s.map,vec(-0.1,0,0.045),-vec::UnitX());
  edge(s.map,5,begin); edge(s.map,begin,begin+1);
  edge(s.map,begin+1,begin+2); edge(s.map,begin+1,begin+3);
  const auto next=tracking.update(s.map,s.planes);
  const auto retained=std::find_if(next.regions.begin(),next.regions.end(),[](const auto &r) {return r.is_retained;});
  ASSERT_NE(retained,next.regions.end());
  EXPECT_EQ(retained->id,first.regions[0].id);
  EXPECT_EQ(retained->node_indices.size(),begin+2);
  EXPECT_NE(std::find(retained->node_indices.begin(),retained->node_indices.end(),begin+1),retained->node_indices.end());
  EXPECT_EQ(retained->shape.q,first.regions[0].shape.q);
  EXPECT_EQ(retained->rejected_node_num,0U);
  coverage(next,s.map.nodes.size());
  // 追加点は接続消失で除外、元の追跡核は接続消失だけでは破棄なし。
  s.map.edges.clear();
  const auto disconnected=tracking.update(s.map,s.planes);
  const auto core=std::find_if(disconnected.regions.begin(),disconnected.regions.end(),[](const auto &r) {return r.is_retained;});
  ASSERT_NE(core,disconnected.regions.end());
  EXPECT_EQ(core->node_indices.size(),begin);
  EXPECT_EQ(core->id,first.regions[0].id);
  // 維持側のRMS設定が厳しい場合にも、追加点による条件逸脱なし。
  s.map.nodes[begin].pos.x=0.101;
  edge(s.map,5,begin);
  retention_options strict;
  strict.max_rms=0.0001;
  const auto limited=tracking.update(s.map,s.planes,{},strict);
  const auto strict_core=std::find_if(limited.regions.begin(),limited.regions.end(),[](const auto &r) {return r.is_retained;});
  ASSERT_NE(strict_core,limited.regions.end());
  EXPECT_EQ(strict_core->node_indices.size(),begin);
}

TEST(SurfaceTracking, AdditionalNodesDoNotRescueLostReferenceSupport)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  const auto begin=s.map.nodes.size();
  for (int i=0;i<20;++i) {
    add_node(s.map,vec(0.1,0,0.105+0.001*i),vec::UnitX());
    edge(s.map,5,begin+i);
  }
  tracking.update(s.map,s.planes);
  for (std::size_t i=0;i<begin;++i) s.map.nodes[i].pos.x+=0.5;
  const auto next=tracking.update(s.map,s.planes);
  for (const auto &r:next.regions) {
    EXPECT_FALSE(r.is_retained);
    EXPECT_NE(r.id,first.regions[0].id);
  }
}

TEST(SurfaceTracking, NodeNormalAndNonfinitePositionRejectOnlyThoseNodes)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  tracking.update(s.map,s.planes);
  s.map.nodes[0].normal.x=0; s.map.nodes[0].normal.z=1;
  s.map.nodes[1].pos.x=std::numeric_limits<float>::quiet_NaN();
  const auto next=tracking.update(s.map,s.planes);
  ASSERT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(next.regions[0].rejected_node_num,2U);
  coverage(next,s.map.nodes.size()-1);
}

TEST(SurfaceTracking, MajorityDeviationReleasesModelAndDeletesMarker)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  const auto id=first.regions[0].id;
  std::set<std::pair<std::string,int>> published;
  make_markers(first,s.map,false,false,published);
  for (std::size_t i=0; i<s.map.nodes.size()/2; ++i) s.map.nodes[i].pos.x+=0.2;
  const auto next=tracking.update(s.map,s.planes);
  for (const auto &r:next.regions) { EXPECT_FALSE(r.is_retained); EXPECT_NE(r.id,id); }
  const auto markers=make_markers(next,s.map,false,false,published);
  EXPECT_TRUE(std::any_of(markers.markers.begin(),markers.markers.end(),[&](const auto &m) {
    return m.id==static_cast<int>(id) && m.ns=="surface_nodes" && m.action==m.DELETE;
  }));
  coverage(next,s.map.nodes.size());
}

TEST(SurfaceTracking, GlobalRmsAlsoLimitsRetention)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  tracking.update(s.map,s.planes);
  for (auto &node:s.map.nodes) { node.pos.x*=1.09; node.pos.y*=1.09; }
  const auto next=tracking.update(s.map,s.planes);
  for (const auto &r:next.regions) EXPECT_FALSE(r.is_retained);
}

TEST(SurfaceTracking, ArrayReorderingUsesNodeIdsInsteadOfIndices)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  const auto first=tracking.update(s.map,s.planes);
  std::reverse(s.map.nodes.begin(),s.map.nodes.end());
  for (auto &idx:s.map.edges) idx=s.map.nodes.size()-1-idx;
  for (auto &p:s.planes.clusters) for (auto &idx:p.node_indices) idx=s.map.nodes.size()-1-idx;
  const auto next=tracking.update(s.map,s.planes);
  ASSERT_EQ(next.regions.size(),1U);
  EXPECT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(next.regions[0].id,first.regions[0].id);
  coverage(next,s.map.nodes.size());
}

TEST(SurfaceTracking, MissingIdsCountAgainstOriginalSupport)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  tracking.update(s.map,s.planes);
  s.map.nodes[0].id=60000;
  auto next=tracking.update(s.map,s.planes);
  ASSERT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(next.regions[0].rejected_node_num,1U);
  for (std::size_t i=1; i<s.map.nodes.size()/2; ++i) s.map.nodes[i].id=60000+i;
  next=tracking.update(s.map,s.planes);
  for (const auto &r:next.regions) EXPECT_FALSE(r.is_retained);
  coverage(next,s.map.nodes.size());
}

TEST(SurfaceTracking, DuplicateIdsDisableTrackingForFrame)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  tracking.update(s.map,s.planes);
  s.map.nodes[0].id=s.map.nodes[1].id;
  const auto next=tracking.update(s.map,s.planes);
  for (const auto &r:next.regions) EXPECT_FALSE(r.is_retained);
  coverage(next,s.map.nodes.size());
}

TEST(SurfaceTracking, CoordinateAndSequenceResetsDoNotReuseOldTrack)
{
  for (int kind=0; kind<4; ++kind) {
    auto s=cylinder(0.1,0.1);
    s.map.frame_number=s.planes.frame_number=10;
    s.map.header.stamp.sec=10;
    tracker tracking;
    const auto first=tracking.update(s.map,s.planes);
    if (kind==0) s.map.header.frame_id=s.planes.header.frame_id="other";
    if (kind==1) s.map.frame_number=s.planes.frame_number=1;
    if (kind==2) s.map.header.stamp.sec=1;
    if (kind==3) { EXPECT_TRUE(tracking.update(map_type{},planes_type{}).regions.empty()); }
    const auto next=tracking.update(s.map,s.planes);
    ASSERT_EQ(next.regions.size(),1U);
    EXPECT_FALSE(next.regions[0].is_retained);
    EXPECT_NE(next.regions[0].id,first.regions[0].id);
  }
}

TEST(SurfaceTracking, EstablishedModelDoesNotRequireTwoCurrentPlanes)
{
  auto s=cylinder(0.1,0.1);
  // 接続消失の補完範囲内での、現在平面数と表示資格の独立性。
  for (auto &node:s.map.nodes) node.pos.z*=0.25;
  tracker tracking;
  tracking.update(s.map,s.planes);
  s.planes.clusters.resize(1);
  s.planes.clusters[0].node_indices.clear();
  for (std::size_t i=0; i<s.map.nodes.size(); ++i) s.planes.clusters[0].node_indices.push_back(i);
  s.map.edges.clear();
  const auto next=tracking.update(s.map,s.planes);
  ASSERT_EQ(next.regions.size(),1U);
  EXPECT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(plane_patch_num(next,next.regions[0]),1U);
  const auto data=nlohmann::json::parse(serialize(next,s.map,s.planes));
  EXPECT_EQ(data["models"][0]["is_display_candidate"],true);
  tracker fresh;
  const auto untracked=fresh.update(s.map,s.planes);
  const auto hidden=nlohmann::json::parse(serialize(untracked,s.map,s.planes));
  EXPECT_EQ(hidden["models"][0]["is_display_candidate"],false);
  const auto repeated=fresh.update(s.map,s.planes);
  EXPECT_FALSE(repeated.regions[0].is_retained);
}

TEST(SurfaceTracking, DisabledRetentionKeepsStatelessBehavior)
{
  auto s=cylinder(0.1,0.1);
  tracker tracking;
  tracking.update(s.map,s.planes);
  s.map.edges.clear();
  retention_options config;
  config.enable_retention=false;
  const auto next=tracking.update(s.map,s.planes,{},config);
  for (const auto &r:next.regions) EXPECT_FALSE(r.is_retained);
  coverage(next,s.map.nodes.size());
}

TEST(SurfaceTracking, ModelFitBudgetDoesNotDiscardRetainedRegion)
{
  auto s=cylinder(0.1,0.1);
  // 小欠損の補完とモデル再フィット予算の独立性。
  for (auto &node:s.map.nodes) node.pos.z*=0.25;
  tracker tracking;
  tracking.update(s.map,s.planes);
  s.map.edges.clear();
  options config;
  config.max_model_fits=1;
  const auto next=tracking.update(s.map,s.planes,config);
  ASSERT_EQ(next.regions.size(),1U);
  EXPECT_TRUE(next.regions[0].is_retained);
  EXPECT_EQ(next.model_fits,0U);
}

TEST(SurfaceTracking, TrackedColorDoesNotShiftWhenAnotherRegionDisappears)
{
  const auto s=cylinder(0.1,0.1);
  tracker tracking;
  auto first=tracking.update(s.map,s.planes);
  auto extra=first.regions[0];
  ++extra.id;
  first.regions.push_back(extra);
  std::set<std::pair<std::string,int>> published;
  const auto markers=make_markers(first,s.map,false,false,published);
  first.regions.erase(first.regions.begin());
  const auto next=make_markers(first,s.map,false,false,published);
  for (const auto &m:markers.markers) if (m.id==static_cast<int>(extra.id) && m.ns=="surface_nodes") {
    ASSERT_EQ(next.markers[0].id,m.id);
    EXPECT_EQ(next.markers[0].color,m.color);
  }
}
