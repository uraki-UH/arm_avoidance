// センサ点群の単一格子登録と、観測時刻TFによる自己形状キャッシュの配置。
#include "robot_model/robot_voxelizer.hpp"
#include "robot_model/urdf_loader.hpp"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace {
thread_local std::string last_error;
using matrix = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

struct observation_model {
  GNG::Analysis::IndexVoxelGrid grid;
  std::vector<simulation::LinkVoxelData> links;
  std::vector<std::int64_t> mask;
  std::vector<double> last_transforms;
  std::size_t max_num_mask_cells{0};

  observation_model(const char *urdf, const char *resources, const char *meshes,
                    double step, const std::int64_t *layout) : grid(step) {
    if (!std::isfinite(step) || step < .001 || step > 1. ||
        layout[0] != 42 || layout[1] != 21 || layout[2] != 0 ||
        layout[3] < 0 || layout[3] >= (1 << 21)) {
      throw std::invalid_argument("観測格子の不正");
    }
    grid.setIndexingParams(layout[0], layout[1], layout[2], layout[3]);
    auto model = simulation::loadRobotFromUrdf(urdf, resources, meshes);
    std::vector<std::string> names;
    for (const auto &[name, link] : model.getLinks()) {
      if (!link.collisions.empty()) {names.push_back(name);}
    }
    links = simulation::RobotVoxelizer::build(model, names, grid);
    if (links.size() != names.size() || links.empty()) {
      throw std::runtime_error("自己形状の一部または全体の構築失敗");
    }
    for (const auto &link : links) {max_num_mask_cells += link.local_voxel_centers.size();}
    if (max_num_mask_cells > 2000000) {throw std::runtime_error("自己形状のセル予算超過");}
  }

  std::int64_t cell_id(const Eigen::Vector3d &point) const {
    Eigen::Vector3d scaled = (point / grid.getVoxelSize()).array().floor();
    if (!scaled.allFinite() || (scaled.array() + grid.getOffset() < 0).any() ||
        (scaled.array() + grid.getOffset() >= (1 << 21)).any()) {
      throw std::invalid_argument("観測座標の索引範囲外");
    }
    return grid.getFlatVoxelId(scaled.cast<int>());
  }

  static matrix checked_transform(const double *values) {
    matrix result = Eigen::Map<const matrix>(values);
    if (!result.allFinite() || !result.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) ||
        !(result.topLeftCorner<3, 3>().transpose() * result.topLeftCorner<3, 3>()).isApprox(
            Eigen::Matrix3d::Identity(), 1e-6) ||
        std::abs(result.topLeftCorner<3, 3>().determinant() - 1.) > 1e-6) {
      throw std::invalid_argument("観測TFの剛体変換不正");
    }
    return result;
  }

  void update_mask(const double *transforms) {
    if (last_transforms.size() == links.size() * 16 &&
        std::equal(last_transforms.begin(), last_transforms.end(), transforms)) {return;}
    std::unordered_set<std::int64_t> ids;
    ids.reserve(max_num_mask_cells);
    for (std::size_t idx = 0; idx < links.size(); ++idx) {
      const matrix tf = checked_transform(transforms + idx * 16);
      for (const auto &point : links[idx].local_voxel_centers) {
        ids.insert(cell_id(tf.topLeftCorner<3, 3>() * point + tf.topRightCorner<3, 1>()));
      }
    }
    mask.assign(ids.begin(), ids.end());
    last_transforms.assign(transforms, transforms + links.size() * 16);
  }
};
}

extern "C" {
const char *gng_observation_error() {return last_error.c_str();}

void *gng_observation_create(const char *urdf, const char *resources, const char *meshes,
                             double step, const std::int64_t *layout) {
  try {return new observation_model(urdf, resources, meshes, step, layout);}
  catch (const std::exception &error) {last_error = error.what(); return nullptr;}
}

void gng_observation_destroy(void *handle) {delete static_cast<observation_model *>(handle);}
std::size_t gng_observation_num_links(void *handle) {
  return static_cast<observation_model *>(handle)->links.size();
}
const char *gng_observation_link(void *handle, std::size_t idx) {
  auto &links = static_cast<observation_model *>(handle)->links;
  return idx < links.size() ? links[idx].name.c_str() : nullptr;
}
std::size_t gng_observation_mask_capacity(void *handle) {
  return static_cast<observation_model *>(handle)->max_num_mask_cells;
}

int gng_observation_build(void *handle, std::size_t num_points, const double *points,
                          const double *sensor_tf, const double *link_tf, const double *bounds,
                          std::int64_t *roi, std::size_t roi_capacity, std::int64_t *mask,
                          std::size_t mask_capacity, std::int64_t *counts) {
  try {
    auto &model = *static_cast<observation_model *>(handle);
    if (num_points > 2000000 || roi_capacity < num_points || mask_capacity < model.max_num_mask_cells) {
      throw std::invalid_argument("観測配列の容量不足");
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(bounds[axis]) || !std::isfinite(bounds[axis + 3]) ||
          bounds[axis] >= bounds[axis + 3]) {throw std::invalid_argument("観測ROI範囲の不正");}
    }
    const matrix tf = observation_model::checked_transform(sensor_tf);
    model.update_mask(link_tf);
    std::unordered_set<std::int64_t> ids;
    ids.reserve(num_points);
    // 生点群の座標変換・ROI抽出・登録の一回走査。同一格子の再登録なし。
    for (std::size_t idx = 0; idx < num_points; ++idx) {
      const Eigen::Map<const Eigen::Vector3d> source(points + idx * 3);
      if (!source.allFinite()) {throw std::invalid_argument("非有限の観測点");}
      const Eigen::Vector3d point = tf.topLeftCorner<3, 3>() * source + tf.topRightCorner<3, 1>();
      bool is_inside = true;
      for (int axis = 0; axis < 3; ++axis) {
        is_inside &= point[axis] >= bounds[axis] && point[axis] < bounds[axis + 3];
      }
      if (is_inside) {ids.insert(model.cell_id(point));}
    }
    std::copy(ids.begin(), ids.end(), roi);
    std::copy(model.mask.begin(), model.mask.end(), mask);
    counts[0] = ids.size();
    counts[1] = model.mask.size();
    return 0;
  } catch (const std::exception &error) {last_error = error.what(); return -1;}
}
}
