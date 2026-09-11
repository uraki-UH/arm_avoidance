// 曲面統合ゲートの比較実験。合成形状・保存済み隣接平面対による採否と実行時間の評価。
#include "ais_gng/topological_plane/surface_model.hpp"
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>

namespace surface = fuzzrobo::surface_model;
using json = nlohmann::json;
using vec = Eigen::Vector3d;
constexpr double pi = 3.14159265358979323846;

struct scene
{
  ais_gng_msgs::msg::TopologicalMap map;
  ais_gng_msgs::msg::PlaneClusterArray planes;
  json description;
};

struct patch_stats
{
  std::size_t num = 0;
  double area = 0;
  double plane_rms = 0;
  double bend = 0;
  double confidence = 0;
  vec normal = vec::Zero();
};

vec position(const geometry_msgs::msg::Point32 &point)
{
  return {point.x,point.y,point.z};
}

void add_node(scene &sample,const vec &point,const vec &normal,int owner)
{
  const auto idx=sample.map.nodes.size();
  ais_gng_msgs::msg::TopologicalNode node;
  node.id=static_cast<std::uint16_t>(idx);
  node.pos.x=point.x(); node.pos.y=point.y(); node.pos.z=point.z();
  node.normal.x=normal.x(); node.normal.y=normal.y(); node.normal.z=normal.z();
  sample.map.nodes.push_back(node);
  if (owner>=0) sample.planes.clusters[owner].node_indices.push_back(idx);
}

void add_edge(scene &sample,std::size_t left,std::size_t right)
{
  sample.map.edges.push_back(left); sample.map.edges.push_back(right);
}

scene generate(const std::string &kind,double size,double angle_deg,int around,
  double noise,bool has_bridge,int seed)
{
  scene sample;
  sample.map.header.frame_id=sample.planes.header.frame_id="map";
  sample.planes.clusters.resize(2);
  sample.planes.clusters[1].id=1;
  std::mt19937 random(seed);
  std::normal_distribution<double> error(0,noise);
  const double angle=angle_deg*pi/180;
  constexpr int rows=6;
  for (int along=0;along<around;++along) for (int row=0;row<rows;++row) {
    const double value=2.0*along/(around-1)-1;
    const double height=size*(2.0*row/(rows-1)-1);
    vec point,normal;
    if (kind=="sharp_corner" || kind=="rounded_corner") {
      // 基準V字形状の丸まり。rounded_cornerの丸まり幅は片側長の8%。
      const double rounding=kind=="rounded_corner" ? 0.08:0;
      const double curved=std::sqrt(value*value+rounding*rounding);
      point=vec(size*value*std::cos(angle/2),size*curved*std::sin(angle/2),height);
      normal=vec(-value/curved*std::sin(angle/2),std::cos(angle/2),0).normalized();
    } else {
      const double phase=value*angle/2;
      const double ratio=kind=="ellipse" ? 0.55:1;
      if (kind=="sphere") {
        const double latitude=(2.0*row/(rows-1)-1)*0.45;
        normal=vec(std::cos(latitude)*std::cos(phase),std::cos(latitude)*std::sin(phase),std::sin(latitude));
        point=size*normal;
      } else {
        point=vec(size*std::cos(phase),size*ratio*std::sin(phase),height);
        normal=vec(std::cos(phase),std::sin(phase)/ratio,0).normalized();
      }
    }
    point+=vec(error(random),error(random),error(random));
    // 入力法線の符号反転に対する独立性。
    if ((along+row)%3==0) normal=-normal;
    const int owner=has_bridge && std::abs(value)<0.16 ? -1:(value<0 ? 0:1);
    add_node(sample,point,normal,owner);
    const auto idx=sample.map.nodes.size()-1;
    if (row) add_edge(sample,idx,idx-1);
    if (along) add_edge(sample,idx,idx-rows);
  }
  sample.description={{"suite","synthetic"},{"kind",kind},{"size_m",size},{"angle_deg",angle_deg},{"around",around},
    {"noise_m",noise},{"has_bridge",has_bridge},{"seed",seed},
    {"is_expected_merge",kind!="sharp_corner" && kind!="rounded_corner"}};
  return sample;
}

std::array<patch_stats,2> measure(const scene &sample,bool enable_curvature)
{
  std::array<patch_stats,2> output;
  for (std::size_t idx=0;idx<2;++idx) {
    auto &stats=output[idx];
    surface::local_patch patch;
    patch.node_indices=sample.planes.clusters[idx].node_indices;
    stats.num=patch.node_indices.size();
    vec center=vec::Zero();
    for (auto node_idx:patch.node_indices) center+=position(sample.map.nodes[node_idx].pos);
    center/=stats.num;
    Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
    for (auto node_idx:patch.node_indices) {
      const vec delta=position(sample.map.nodes[node_idx].pos)-center;
      covariance.noalias()+=delta*delta.transpose();
    }
    covariance/=stats.num;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    stats.normal=solver.eigenvectors().col(0);
    stats.plane_rms=std::sqrt(std::max(0.0,solver.eigenvalues()(0)));
    // PCA接平面上の面積代理量。実面積そのものではなく密度補正用の広がり。
    stats.area=12*std::sqrt(std::max(0.0,solver.eigenvalues()(1)*solver.eigenvalues()(2)));
    if (enable_curvature) {
      const auto curvature=surface::estimate_curvature(patch,sample.map);
      stats.confidence=curvature.valid ? curvature.confidence:0;
      stats.bend=std::sqrt(std::max(0.0,
        (curvature.tensor*curvature.support_cov*curvature.tensor.transpose()).trace()));
    }
  }
  return output;
}

const std::vector<std::string> modes={"baseline","fixed_angle","fixed_residual",
  "node_angle","node_residual","node_both","area_residual",
  "flat_fixed_angle","flat_node_angle","flat_area_angle","flat_area_angle_strict"};

json evaluate(const scene &sample,const std::string &mode)
{
  const auto begin=std::chrono::steady_clock::now();
  surface::options config;
  const bool has_flat_gate=mode.find("flat_")==0;
  std::array<patch_stats,2> stats;
  if (mode=="node_residual") {
    for (std::size_t idx=0;idx<2;++idx) stats[idx].num=sample.planes.clusters[idx].node_indices.size();
  } else if (mode!="baseline" && mode!="fixed_residual") stats=measure(sample,has_flat_gate);
  // 基準支持数24。角度10度、距離倍率0.25の下限による無制限な厳格化の抑止。
  const double node_factor=std::clamp(std::sqrt(24.0/std::max<std::size_t>(1,
    std::min(stats[0].num,stats[1].num))),0.25,1.0);
  const double area_factor=std::clamp(std::sqrt(0.0024/std::max(1e-12,
    std::min(stats[0].area,stats[1].area))),0.25,1.0);
  const double angle_deg=180/pi*std::acos(std::clamp(std::abs(stats[0].normal.dot(stats[1].normal)),0.0,1.0));
  const double max_flat_bend=mode=="flat_area_angle_strict" ? 0.20:0.12;
  const bool has_flat_pair=stats[0].confidence>=0.5 && stats[1].confidence>=0.5 &&
    stats[0].bend<=max_flat_bend && stats[1].bend<=max_flat_bend &&
    stats[0].plane_rms<=0.002 && stats[1].plane_rms<=0.002 &&
    std::min(stats[0].area,stats[1].area)>=0.0004;
  bool is_rejected=false;
  if (mode=="fixed_angle") is_rejected=angle_deg>45;
  if (mode=="node_angle" || mode=="node_both")
    is_rejected=angle_deg>std::max(10.0,45*node_factor);
  if (has_flat_gate) {
    const double factor=mode=="flat_fixed_angle" ? 1.0:(mode=="flat_node_angle" ? node_factor:area_factor);
    is_rejected=has_flat_pair && angle_deg>std::max(10.0,45*factor);
  }
  if (mode=="node_residual" || mode=="node_both" || mode=="area_residual" || mode=="fixed_residual") {
    const double factor=mode=="fixed_residual" ? 0.5:(mode=="area_residual" ? area_factor:node_factor);
    config.max_patch_rms*=factor;
    config.max_point_residual*=factor;
  }
  bool is_merged=false;
  double coverage=0;
  std::size_t fits=0;
  if (!is_rejected) {
    const auto result=surface::extract(sample.map,sample.planes,config);
    fits=result.model_fits;
    for (const auto &region:result.regions) {
      if (region.shape.type=="plane" || region.shape.type=="unknown" ||
        surface::plane_patch_num(result,region)<2) continue;
      is_merged=true;
      coverage=std::max(coverage,static_cast<double>(region.node_indices.size())/sample.map.nodes.size());
    }
  }
  const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  return {{"mode",mode},{"is_merged",is_merged},{"is_rejected",is_rejected},{"coverage",coverage},
    {"elapsed_ms",elapsed},{"model_fits",fits},{"node_factor",node_factor},{"area_factor",area_factor},
    {"angle_deg_measured",angle_deg},{"has_flat_pair",has_flat_pair},
    {"patch_nodes",{stats[0].num,stats[1].num}},{"bend",{stats[0].bend,stats[1].bend}}};
}

void run(const scene &sample,std::size_t case_idx,std::ofstream &output)
{
  // モード順の巡回と3回反復。生成・JSON入出力は計測区間の対象外。
  for (int iter=0;iter<3;++iter) for (std::size_t idx=0;idx<modes.size();++idx) {
    auto result=evaluate(sample,modes[(idx+case_idx+iter)%modes.size()]);
    result.update(sample.description);
    result["case_idx"]=case_idx; result["iter"]=iter; result["nodes"]=sample.map.nodes.size();
    output<<result.dump()<<'\n';
  }
}

void recorded_pairs(const std::string &path,std::size_t &case_idx,std::ofstream &output)
{
  std::ifstream input(path);
  const auto records=json::parse(input);
  for (const auto &record:records.at("models")) {
    if (!record.contains("map") || !record.contains("planes")) continue;
    const auto &map=record.at("map");
    const auto &planes=record.at("planes").at("clusters");
    for (std::size_t left=0;left<planes.size();++left) for (std::size_t right=left+1;right<planes.size();++right) {
      if (planes[left]["node_indices"].size()<12 || planes[right]["node_indices"].size()<12) continue;
      scene sample;
      sample.map.header.frame_id=sample.planes.header.frame_id="map";
      sample.planes.clusters.resize(2);
      std::map<std::size_t,std::size_t> indices;
      std::map<std::size_t,int> owners;
      for (int side=0;side<2;++side) for (std::size_t old_idx:planes[side ? right:left]["node_indices"]) {
        if (indices.count(old_idx)) throw std::runtime_error("overlapping plane nodes");
        indices[old_idx]=sample.map.nodes.size(); owners[old_idx]=side;
        const auto &node=map["nodes"][old_idx];
        const auto read=[](const json &value) { return vec(value.at("x"),value.at("y"),value.at("z")); };
        add_node(sample,read(node.at("pos")),read(node.at("normal")),side);
      }
      bool has_link=false;
      for (std::size_t idx=0;idx+1<map["edges"].size();idx+=2) {
        const std::size_t a=map["edges"][idx],b=map["edges"][idx+1];
        if (!indices.count(a) || !indices.count(b)) continue;
        add_edge(sample,indices[a],indices[b]);
        has_link=has_link || owners[a]!=owners[b];
      }
      if (!has_link) continue;
      sample.description={{"suite","recorded"},{"kind","recorded_pair"},{"recording",path},{"frame",map["frame_number"]},
        {"plane_ids",{planes[left]["id"],planes[right]["id"]}}};
      run(sample,case_idx++,output);
    }
  }
}

int main(int argc,char **argv)
{
  if (argc<2) { std::cerr<<"usage: benchmark_surface_merge output.jsonl [observed.json ...]\n"; return 2; }
  try {
    std::ofstream output(argv[1]);
    if (!output) throw std::runtime_error("cannot open output");
    std::size_t case_idx=0;
    for (const auto &kind:{"sharp_corner","rounded_corner","cylinder","ellipse","sphere"})
      for (double size:{0.03,0.08,0.18}) for (double angle:{45.0,90.0,135.0})
        for (int around:{8,16,32}) for (double noise:{0.0,0.0005,0.002})
          for (bool has_bridge:{false,true}) for (int seed:{1,2})
            run(generate(kind,size,angle,around,noise,has_bridge,seed),case_idx++,output);
    std::cout<<"synthetic_cases="<<case_idx<<'\n';
    // 同一観測の複製による支持数依存性。位置・法線・空間的な広がりの追加なし。
    for (const auto &kind:{"sharp_corner","rounded_corner","cylinder","ellipse","sphere"})
      for (double angle:{45.0,90.0,135.0}) for (bool has_bridge:{false,true}) {
        const auto original=generate(kind,0.08,angle,8,0,has_bridge,1);
        for (int copies:{1,2,4}) {
          auto sample=original;
          for (int copy=1;copy<copies;++copy) {
            const auto base=sample.map.nodes.size();
            for (std::size_t idx=0;idx<original.map.nodes.size();++idx) {
              int owner=-1;
              for (int side=0;side<2;++side) {
                const auto &indices=original.planes.clusters[side].node_indices;
                if (std::find(indices.begin(),indices.end(),idx)!=indices.end()) owner=side;
              }
              add_node(sample,position(original.map.nodes[idx].pos),position(original.map.nodes[idx].normal),owner);
              add_edge(sample,idx,base+idx);
            }
            for (std::size_t idx=0;idx+1<original.map.edges.size();idx+=2)
              add_edge(sample,base+original.map.edges[idx],base+original.map.edges[idx+1]);
          }
          sample.description["suite"]="repeat_support";
          sample.description["copies"]=copies;
          run(sample,case_idx++,output);
        }
      }
    for (int idx=2;idx<argc;++idx) recorded_pairs(argv[idx],case_idx,output);
    std::cout<<"all_cases="<<case_idx<<'\n';
  } catch (const std::exception &error) {
    std::cerr<<error.what()<<'\n'; return 1;
  }
}
