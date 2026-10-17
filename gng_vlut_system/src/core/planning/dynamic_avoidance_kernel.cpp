// 保存姿勢の球包絡と観測球の一括照会。ボクセル登録・将来状態の参照なし。
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

extern "C" int gng_dynamic_kernel_version() { return 1; }

extern "C" int gng_dynamic_internal_clear(
    size_t num_poses, size_t num_spheres, size_t num_pairs, const int64_t* pairs,
    const double* centers, const double* radii, double margin, uint8_t* can_pass) noexcept {
  if (!std::isfinite(margin) || margin < 0) return -1;
  for (size_t pose = 0; pose < num_poses; ++pose) {
    can_pass[pose] = 1;
    for (size_t pair = 0; pair < num_pairs; ++pair) {
      const int64_t first = pairs[pair * 2], second = pairs[pair * 2 + 1];
      if (first < 0 || second < 0 || first >= static_cast<int64_t>(num_spheres) ||
          second >= static_cast<int64_t>(num_spheres)) return -1;
      double squared = 0;
      for (int axis = 0; axis < 3; ++axis) {
        const double delta = centers[(pose * num_spheres + first) * 3 + axis] -
                             centers[(pose * num_spheres + second) * 3 + axis];
        squared += delta * delta;
      }
      const double required = radii[first] + radii[second] + margin;
      if (!std::isfinite(squared) || squared < required * required) { can_pass[pose] = 0; break; }
    }
  }
  return 0;
}

extern "C" int gng_dynamic_node_gaps(
    size_t num_nodes, size_t num_spheres, const double* centers, const double* radii,
    const double* lower, const double* upper, const double* obstacle, double radius,
    double danger_margin, double* gaps, uint8_t* labels) noexcept {
  if (!num_nodes || !num_spheres || !std::isfinite(radius) || radius <= 0 ||
      !std::isfinite(danger_margin) || danger_margin < 0) return -1;
  for (int axis = 0; axis < 3; ++axis) if (!std::isfinite(obstacle[axis])) return -1;
  for (size_t node = 0; node < num_nodes; ++node) {
    double squared = 0;
    for (int axis = 0; axis < 3; ++axis) {
      const double delta = std::max({lower[node * 3 + axis] - obstacle[axis],
                                     obstacle[axis] - upper[node * 3 + axis], 0.});
      squared += delta * delta;
    }
    double gap = std::sqrt(squared) - radius;
    if (gap <= danger_margin) {
      gap = 1e6;
      for (size_t sphere = 0; sphere < num_spheres; ++sphere) {
        const double* center = centers + (node * num_spheres + sphere) * 3;
        double norm = 0;
        for (int axis = 0; axis < 3; ++axis) {
          const double delta = center[axis] - obstacle[axis];
          norm += delta * delta;
        }
        gap = std::min(gap, std::sqrt(norm) - radii[sphere] - radius);
        if (gap < 0) break;
      }
    }
    gaps[node] = gap;
    labels[node] = gap < 0 ? 2 : gap < danger_margin ? 3 : 1;
  }
  return 0;
}

#ifdef GNG_ENABLE_VLUT
#include "safety_engine/indexing/dense_spatial_index.hpp"
#include "safety_engine/vlut/safety_vlut_mapper.hpp"
#include <memory>
#include <string>

namespace {
thread_local std::string vlut_error;
struct vlut_session {
  std::shared_ptr<robot_sim::analysis::DenseSpatialIndex> index;
  robot_sim::analysis::SafetyVlutMapper mapper;
  size_t num_nodes;
  std::vector<long> occupied, danger;
};
}

extern "C" const char* gng_vlut_error() noexcept { return vlut_error.c_str(); }

extern "C" void* gng_vlut_open(const char* filename, size_t num_nodes) noexcept {
  try {
    if (!filename || !num_nodes || num_nodes>1000000) throw std::invalid_argument("VLUT入力の不正");
    auto session = std::make_unique<vlut_session>();
    session->index = std::make_shared<robot_sim::analysis::DenseSpatialIndex>(
        .02,Eigen::Vector3d::Constant(-1.),Eigen::Vector3d::Constant(1.));
    if (!session->index->load(filename)) throw std::runtime_error("VLUT読込み失敗");
    session->num_nodes = num_nodes;
    session->mapper.initialize(num_nodes,session->index);
    vlut_error.clear();
    return session.release();
  } catch (const std::exception& error) { vlut_error=error.what();return nullptr; }
}

extern "C" void gng_vlut_close(void* handle) noexcept { delete static_cast<vlut_session*>(handle); }

extern "C" int gng_vlut_update(void* handle,size_t num_segments,const double* segments,
    double radius,double margin,uint8_t* labels,uint64_t* stats) noexcept {
  try {
    if (!handle || !segments || !labels || !stats || !num_segments || num_segments>16 ||
        !std::isfinite(radius) || radius<=0 || radius>1. || !std::isfinite(margin) || margin<0 || margin>1.) {
      throw std::invalid_argument("VLUT観測の不正");
    }
    auto& session = *static_cast<vlut_session*>(handle);
    session.occupied.clear();session.danger.clear();
    const double size=session.index->getVoxelSize(),cell_radius=std::sqrt(3.)*size/2.;
    const double collision_radius=radius+cell_radius,danger_radius=collision_radius+margin;
    for (size_t idx=0;idx<num_segments;++idx) {
      const Eigen::Map<const Eigen::Vector3d> start(segments+idx*6),end(segments+idx*6+3);
      if (!start.allFinite() || !end.allFinite() || start.cwiseAbs().maxCoeff()>100 || end.cwiseAbs().maxCoeff()>100) {
        throw std::invalid_argument("VLUT観測座標の不正");
      }
      const auto delta=(end-start).eval();const double squared=delta.squaredNorm();
      const Eigen::Vector3i lower=((start.cwiseMin(end).array()-danger_radius)/size).floor().cast<int>();
      const Eigen::Vector3i upper=((start.cwiseMax(end).array()+danger_radius)/size).floor().cast<int>();
      if ((upper-lower+Eigen::Vector3i::Ones()).cast<double>().prod()>2000000) {
        throw std::length_error("VLUT観測範囲の上限超過");
      }
      for (int z=lower.z();z<=upper.z();++z) for (int y=lower.y();y<=upper.y();++y) for (int x=lower.x();x<=upper.x();++x) {
        const Eigen::Vector3i cell(x,y,z);
        const Eigen::Vector3d point=(cell.cast<double>().array()+.5)*size;
        const double fraction=squared>1e-18?std::clamp((point-start).dot(delta)/squared,0.,1.):0.;
        const double dist_squared=(point-start-fraction*delta).squaredNorm();
        if (dist_squared>danger_radius*danger_radius) continue;
        const long id=session.index->getGrid().getFlatVoxelId(cell);
        session.danger.push_back(id);
        if (dist_squared<=collision_radius*collision_radius) session.occupied.push_back(id);
      }
    }
    // 共通mapperによる重複除去と追加・削除差分。ノード形状のオンライン再生成なし。
    session.mapper.updateFromVoxels(session.occupied,session.danger);
    const auto& collision=session.mapper.getCollisionCounts();const auto& danger=session.mapper.getDangerCounts();
    for (size_t idx=0;idx<session.num_nodes;++idx) labels[idx]=collision[idx]>0?2:danger[idx]>0?3:1;
    stats[0]=session.mapper.getPrevOccupiedVoxels().size();stats[1]=session.mapper.getPrevDangerVoxels().size();
    stats[2]=session.mapper.getAddedOccupied().size()+session.mapper.getRemovedOccupied().size();
    stats[3]=session.mapper.getAddedDanger().size()+session.mapper.getRemovedDanger().size();
    vlut_error.clear();return 0;
  } catch (const std::exception& error) { vlut_error=error.what();return -1; }
}
#endif
