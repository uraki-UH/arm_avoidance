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

TEST(SurfaceModel, NoPlanarSeedIsRequired)
{
  const auto s=cylinder(0.1,0.1,48,false);
  const auto r=extract(s.map,s.planes);
  ASSERT_EQ(r.regions.size(),1U);
  EXPECT_EQ(r.regions[0].shape.type,"cylinder");
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
  const auto r=extract(s.map,s.planes);
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
  const auto r=extract(s.map,s.planes);
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
    EXPECT_NEAR(c.kappa.x(),-10.0,1e-4);
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
      EXPECT_NEAR(c.kappa.x(),-5.0,1e-4);
      EXPECT_NEAR(c.kappa.y(),-5.0,1e-4);
    }
    if (type==2) { EXPECT_LT(c.kappa.x()*c.kappa.y(),-24.0); }
    EXPECT_GT(c.confidence,0.99);
  }
}

TEST(PatchCurvature, MissingNormalsAndLineSupportAreInvalidNotFlat)
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

TEST(SurfaceTracking, DisconnectedEdgesKeepCurrentNodesAndStableId)
{
  auto s=cylinder(0.1,0.1);
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
