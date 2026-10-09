#pragma once

#include <ais_gng/point_selection.hpp>
#include <pointcloud_sampling/stratified.hpp>
#include <voxel_msgs/msg/voxel.hpp>
#include <voxel_idx.hpp>
#include <point_cloud_store.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <unordered_set>

namespace fuzzrobo::self_point_filter {

// 幾何判定と共有セル判定に共通の抽出方針
template<class AcceptPoint>
inline std::vector<uint32_t> select_if(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed,
    PointSamplingMode mode, AcceptPoint accept_point, bool enable_full_scan) {
  if (mode == PointSamplingMode::Random)
    return pointcloud_sampling::select_random_if(cloud, max_points, seed, accept_point, enable_full_scan);
  if (mode == PointSamplingMode::Stratified)
    return pointcloud_sampling::select_stratified_if(cloud, max_points, seed, accept_point);
  auto selected = pointcloud_sampling::select_stratified_if(cloud, 0, seed, accept_point);
  if (max_points && selected.size() > max_points) {
    if (mode == PointSamplingMode::Uniform) {
      const uint64_t num_valid = selected.size();
      for (uint32_t idx = 0; idx < max_points; ++idx)
        selected[idx] = selected[(2ULL * idx + 1) * num_valid / (2ULL * max_points)];
    }
    selected.resize(max_points);
  }
  return selected;
}

// ROI所属番号の参照のみ。GNG側の再ボクセル化・自己形状照合なし
inline std::vector<uint32_t> select_shared_points(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed,
    PointSamplingMode mode, const voxel_idx::roi_point_membership &membership,
    std::vector<uint8_t> *labels = nullptr) {
  const pointcloud_sampling::detail::xyz_reader reader(cloud);
  if (membership.point_cells.size() != reader.num_points)
    throw std::invalid_argument("共有ROIの元点数不一致");
  if (labels) labels->assign(reader.num_points, 2);
  return select_if(cloud, max_points, seed, mode,
    [&](uint32_t idx, const std::array<double, 3> &) {
      const bool is_self = membership.is_self_point(idx);
      if (labels) (*labels)[idx] = is_self ? 1 : 0;
      return !is_self;
    }, labels != nullptr);
}

// 粗い自己セルの受信時索引。環境側グリッド・点群XYZ複製・最近傍探索なし
struct mask_snapshot {
  voxel_idx::VoxelIndexingSchema schema;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point received_at{std::chrono::steady_clock::now()};
  Eigen::Vector3d origin;
  Eigen::Vector3d min_pos{Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())};
  Eigen::Vector3d max_pos{-min_pos};
  Eigen::Vector3i min_cell{Eigen::Vector3i::Zero()};
  std::array<uint64_t, 3> num_axis_cells{};
  std::vector<uint64_t> bits;
  std::unordered_set<uint64_t> ids;
  std::size_t num_cells{0};

  explicit mask_snapshot(const voxel_msgs::msg::Voxel &message) {
    schema = {message.x_shift, message.y_shift, message.z_shift, message.offset, message.voxel_size};
    header = message.header;
    origin = {message.origin_x, message.origin_y, message.origin_z};
    if (!schema.isValid() || schema.x_shift >= 63 || !std::isfinite(schema.voxel_size) ||
        schema.voxel_size <= 0 || !origin.allFinite() || header.frame_id.empty() ||
        (header.stamp.sec == 0 && header.stamp.nanosec == 0) || schema.offset > INT32_MAX) {
      throw std::invalid_argument("Invalid self voxel mask schema or stamp");
    }
    Eigen::Vector3i lower_cell = Eigen::Vector3i::Constant(INT32_MAX);
    Eigen::Vector3i upper_cell = Eigen::Vector3i::Constant(INT32_MIN);
    for (const int64_t value : message.data) {
      const auto id = static_cast<uint64_t>(value);
      const auto idx = schema.unpack(id);
      if (schema.pack(idx) != id || idx.x < -schema.offset || idx.y < -schema.offset || idx.z < -schema.offset)
        throw std::invalid_argument("Invalid self voxel id");
      const Eigen::Vector3d lower = origin + Eigen::Vector3d(idx.x, idx.y, idx.z) * schema.voxel_size;
      min_pos = min_pos.cwiseMin(lower);
      max_pos = max_pos.cwiseMax((lower.array() + schema.voxel_size).matrix());
      lower_cell = lower_cell.cwiseMin(Eigen::Vector3i(idx.x, idx.y, idx.z));
      upper_cell = upper_cell.cwiseMax(Eigen::Vector3i(idx.x, idx.y, idx.z));
    }
    if (message.data.empty()) return;
    // 自己領域のみの上限2 MiBビットマスク。巨大・疎な外接箱ではハッシュ照合へ切替
    constexpr uint64_t max_mask_cells = 2ULL * 1024 * 1024 * 8;
    uint64_t num_box_cells = 1;
    min_cell = lower_cell;
    for (int axis = 0; axis < 3; ++axis) {
      num_axis_cells[axis] = int64_t(upper_cell[axis]) - lower_cell[axis] + 1;
      if (num_box_cells > max_mask_cells / num_axis_cells[axis]) num_box_cells = max_mask_cells + 1;
      else num_box_cells *= num_axis_cells[axis];
    }
    if (num_box_cells <= max_mask_cells) {
      bits.assign((num_box_cells + 63) / 64, 0);
      for (const int64_t id : message.data) {
        const auto idx = schema.unpack(static_cast<uint64_t>(id));
        const uint64_t flat = dense_id(idx);
        const uint64_t bit = uint64_t{1} << (flat % 64);
        if (!(bits[flat / 64] & bit)) ++num_cells;
        bits[flat / 64] |= bit;
      }
    } else {
      ids.reserve(message.data.size());
      for (const int64_t id : message.data) ids.insert(static_cast<uint64_t>(id));
      num_cells = ids.size();
    }
  }

  uint64_t dense_id(const voxel_idx::VoxelIndex &idx) const {
    return (uint64_t(int64_t(idx.x) - min_cell.x()) * num_axis_cells[1] +
        uint64_t(int64_t(idx.y) - min_cell.y())) * num_axis_cells[2] + uint64_t(int64_t(idx.z) - min_cell.z());
  }

  bool contains(const Eigen::Vector3d &point) const {
    // 軸ごとの外接箱判定。NaN・無限値も同じ比較で除外
    for (int axis = 0; axis < 3; ++axis)
      if (!(point[axis] >= min_pos[axis] && point[axis] < max_pos[axis])) return false;
    Eigen::Vector3i cell;
    for (int axis = 0; axis < 3; ++axis) {
      const double value = (point[axis] - origin[axis]) / schema.voxel_size;
      if (!(value >= INT32_MIN && value < double(INT32_MAX) + 1.0)) return false;
      // 整数変換と負側の端数補正によるfloor。逆数乗算への置換による境界変更なし
      const int truncated = static_cast<int>(value);
      cell[axis] = truncated - (value < truncated ? 1 : 0);
    }
    const voxel_idx::VoxelIndex idx{cell.x(), cell.y(), cell.z()};
    if (!bits.empty()) {
      for (int axis = 0; axis < 3; ++axis) {
        const int64_t local_cell = int64_t(cell[axis]) - min_cell[axis];
        if (local_cell < 0 || uint64_t(local_cell) >= num_axis_cells[axis]) return false;
      }
      const uint64_t flat = dense_id(idx);
      return (bits[flat / 64] & (uint64_t{1} << (flat % 64))) != 0;
    }
    return ids.count(schema.pack(idx)) != 0;
  }
};

// 全点ラベルは要求時のみ。0=自己マスク外、1=自己候補、2=非有限XYZ
inline std::vector<uint32_t> select_points(
    const sensor_msgs::msg::PointCloud2 &cloud, uint32_t max_points, uint32_t seed,
    PointSamplingMode mode, const mask_snapshot &mask, const Eigen::Isometry3d &mask_from_cloud,
    std::vector<uint8_t> *labels = nullptr) {
  if (!mask_from_cloud.matrix().allFinite()) throw std::invalid_argument("Invalid self mask transform");
  // センサ座標の保守的な外接箱。遠方点は座標変換・セル照合の前に除外
  Eigen::Vector3d min_cloud = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_cloud = -min_cloud;
  const bool is_rigid = (mask_from_cloud.linear().transpose() * mask_from_cloud.linear()).isApprox(
      Eigen::Matrix3d::Identity(), 64 * std::numeric_limits<double>::epsilon());
  if (mask.num_cells && is_rigid) {
    const auto cloud_from_mask = mask_from_cloud.inverse();
    for (uint32_t corner = 0; corner < 8; ++corner) {
      Eigen::Vector3d point;
      for (int axis = 0; axis < 3; ++axis)
        point[axis] = corner & (1U << axis) ? mask.max_pos[axis] : mask.min_pos[axis];
      const Eigen::Vector3d transformed = cloud_from_mask * point;
      min_cloud = min_cloud.cwiseMin(transformed);
      max_cloud = max_cloud.cwiseMax(transformed);
    }
    const double scale = std::max({1.0, min_cloud.cwiseAbs().maxCoeff(), max_cloud.cwiseAbs().maxCoeff(),
        mask.min_pos.cwiseAbs().maxCoeff(), mask.max_pos.cwiseAbs().maxCoeff()});
    const double padding = 1e-6 + 64 * std::numeric_limits<double>::epsilon() * scale;
    min_cloud.array() -= padding; max_cloud.array() += padding;
  }
  const bool can_use_cloud_bounds = min_cloud.allFinite() && max_cloud.allFinite();
  const auto accept_point = [&](uint32_t idx, const std::array<double, 3> &xyz) {
    bool can_match = mask.num_cells != 0;
    if (can_use_cloud_bounds) for (int axis = 0; axis < 3 && can_match; ++axis)
      can_match = xyz[axis] >= min_cloud[axis] && xyz[axis] <= max_cloud[axis];
    const bool is_self = can_match && mask.contains(mask_from_cloud * Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
    if (labels) (*labels)[idx] = is_self ? 1 : 0;
    return !is_self;
  };
  // 配列確保より前のレイアウト検証は共通サンプラー側。ラベル容量のみの上限検査
  const uint64_t num_points = uint64_t(cloud.width) * cloud.height;
  if (num_points > UINT32_MAX || (num_points && (!cloud.point_step || num_points > cloud.data.size() / cloud.point_step)))
    throw std::invalid_argument("Invalid self filter cloud size");
  if (labels) labels->assign(num_points, 2);
  return select_if(cloud, max_points, seed, mode, accept_point, labels != nullptr);
}

inline sensor_msgs::msg::PointCloud2 make_labelled_cloud(
    const sensor_msgs::msg::PointCloud2 &source, const std::vector<uint8_t> &labels) {
  if (labels.size() != uint64_t(source.width) * source.height)
    throw std::invalid_argument("Self label count mismatch");
  auto fields = source.fields;
  auto field = std::find_if(fields.begin(), fields.end(), [](const auto &item) { return item.name == "self_candidate"; });
  uint32_t label_offset = source.point_step;
  if (field != fields.end()) {
    if (field->datatype != sensor_msgs::msg::PointField::UINT8 || field->count != 1 || field->offset >= source.point_step)
      throw std::invalid_argument("Invalid existing self_candidate field");
    label_offset = field->offset;
  } else {
    sensor_msgs::msg::PointField label;
    label.name = "self_candidate"; label.offset = label_offset;
    label.datatype = sensor_msgs::msg::PointField::UINT8; label.count = 1;
    fields.push_back(label);
  }
  const uint64_t point_step = uint64_t(source.point_step) + (label_offset == source.point_step ? 1 : 0);
  if (point_step > UINT32_MAX || point_step * source.width > UINT32_MAX)
    throw std::invalid_argument("Labelled cloud row overflow");
  sensor_msgs::msg::PointCloud2 output;
  output.header = source.header; output.width = source.width; output.height = source.height;
  output.fields = std::move(fields); output.is_bigendian = source.is_bigendian; output.is_dense = source.is_dense;
  output.point_step = point_step; output.row_step = point_step * source.width;
  output.data.resize(uint64_t(output.row_step) * output.height);
  for (std::size_t idx = 0; idx < labels.size(); ++idx) {
    const auto offset = (idx / source.width) * source.row_step + (idx % source.width) * source.point_step;
    if (offset + source.point_step > source.data.size()) throw std::invalid_argument("Invalid labelled cloud layout");
    auto *point = output.data.data() + idx * output.point_step;
    std::memcpy(point, source.data.data() + offset, source.point_step);
    point[label_offset] = labels[idx];
  }
  return output;
}

}  // namespace fuzzrobo::self_point_filter
