// 連続占有領域の反復計算用C ABI。ROS・Python実行環境への依存なし。
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

extern "C" int gng_region_kernel_version() noexcept { return 1; }

// 親ID・葉ビットの差分集計。全投稿配列の展開・点群の二重登録なし。
extern "C" int gng_region_accumulate_leaf_counts(
    std::size_t num_selected, std::size_t num_voxels, std::size_t num_relations,
    std::size_t num_parents, std::size_t num_leaves, const std::int64_t* selected,
    const std::int64_t* pointers, const std::int64_t* parents, const std::uint16_t* leaf_bits,
    const std::int64_t* parent_pointers, const std::int64_t* parent_leaves,
    int sign, std::int64_t* counts) noexcept {
  if (!pointers || !parent_pointers || (num_selected && !selected) ||
      (num_relations && (!parents || !leaf_bits)) || (num_leaves && (!parent_leaves || !counts)) ||
      (sign != 1 && sign != -1)) return -1;
  for (std::size_t idx = 0; idx < num_selected; ++idx) {
    const auto row = selected[idx];
    if (row < 0 || static_cast<std::size_t>(row) >= num_voxels) return -1;
    const auto begin = pointers[row], end = pointers[row+1];
    if (begin < 0 || end < begin || static_cast<std::size_t>(end) > num_relations) return -1;
    for (auto relation_idx = begin; relation_idx < end; ++relation_idx) {
      const auto parent = parents[relation_idx];
      if (parent < 0 || static_cast<std::size_t>(parent) >= num_parents) return -1;
      const auto first = parent_pointers[parent], last = parent_pointers[parent+1];
      if (first < 0 || last <= first || last-first > 16 || static_cast<std::size_t>(last) > num_leaves ||
          !leaf_bits[relation_idx] || (static_cast<std::uint32_t>(leaf_bits[relation_idx]) >> (last-first))) return -1;
      auto bits = static_cast<std::uint32_t>(leaf_bits[relation_idx]);
      while (bits) {
        const auto bit = __builtin_ctz(bits);
        bits &= bits-1;
        const auto leaf = parent_leaves[first+bit];
        if (leaf < 0 || static_cast<std::size_t>(leaf) >= num_leaves) return -1;
        if (sign < 0 && counts[leaf] == 0) return -1;
        counts[leaf] += sign;
      }
    }
  }
  return 0;
}

// 登録済みCSRの差分走査。入力点群の再登録・関係配列の展開なし。
extern "C" int gng_region_accumulate_counts(
    std::size_t num_selected, std::size_t num_voxels, std::size_t num_relations,
    std::size_t num_regions, const std::int64_t* selected,
    const std::int64_t* pointers, const std::int64_t* regions,
    const std::uint8_t* is_inner, int sign, std::int64_t* counts) noexcept {
  if (!pointers || (num_selected && !selected) || (num_relations && (!regions || !is_inner)) ||
      (num_regions && !counts) || (sign != 1 && sign != -1)) return -1;
  for (std::size_t idx = 0; idx < num_selected; ++idx) {
    const auto row = selected[idx];
    if (row < 0 || static_cast<std::size_t>(row) >= num_voxels) return -1;
    const auto begin = pointers[row], end = pointers[row+1];
    if (begin < 0 || end < begin || static_cast<std::size_t>(end) > num_relations) return -1;
    for (auto relation_idx = begin; relation_idx < end; ++relation_idx) {
      const auto region_idx = regions[relation_idx];
      if (region_idx < 0 || static_cast<std::size_t>(region_idx) >= num_regions || is_inner[relation_idx] > 1) return -1;
      counts[region_idx] += sign;
      counts[num_regions+region_idx] += sign*is_inner[relation_idx];
    }
  }
  return 0;
}

extern "C" int gng_region_coefficients(
    std::size_t num_spheres, std::size_t num_joints, std::size_t num_operations,
    const std::int64_t* operation_joints, const std::int64_t* operation_kinds,
    const double* multipliers, const std::int64_t* path_starts,
    const std::int64_t* path_operations, const double* reaches,
    const double* centers, const double* axes, const double* origins,
    const std::uint8_t* is_line_preserved, const double* global_bounds,
    const double* max_dev, double* local, double* operation_coefficients) noexcept {
  if (!num_spheres || !num_joints || !num_operations || !operation_joints ||
      !operation_kinds || !multipliers || !path_starts || !path_operations ||
      !reaches || !centers || !axes || !origins || !is_line_preserved ||
      !global_bounds || !max_dev || !local || !operation_coefficients) return -1;
  try {
    std::fill_n(local, num_spheres*num_joints, 0.);
    std::fill_n(operation_coefficients, num_spheres*num_operations, 0.);
    std::vector<std::uint8_t> has_descendant(num_joints);
    for (std::size_t sphere_idx = 0; sphere_idx < num_spheres; ++sphere_idx) {
      std::fill(has_descendant.begin(), has_descendant.end(), 0);
      for (auto path_idx = path_starts[sphere_idx]; path_idx < path_starts[sphere_idx+1]; ++path_idx) {
        const auto operation_idx = path_operations[path_idx];
        if (operation_idx < 0 || static_cast<std::size_t>(operation_idx) >= num_operations) return -1;
        const auto joint_idx = operation_joints[operation_idx];
        if (joint_idx < 0 || static_cast<std::size_t>(joint_idx) >= num_joints) return -1;
        double coefficient = std::abs(multipliers[operation_idx]);
        if (operation_kinds[operation_idx] == 0) {
          const auto* axis = axes+operation_idx*3;
          double delta[3], cross[3];
          for (int idx = 0; idx < 3; ++idx) delta[idx] = centers[sphere_idx*3+idx]-origins[operation_idx*3+idx];
          cross[0] = axis[1]*delta[2]-axis[2]*delta[1];
          cross[1] = axis[2]*delta[0]-axis[0]*delta[2];
          cross[2] = axis[0]*delta[1]-axis[1]*delta[0];
          const double radius = std::sqrt(cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2]);
          double descendant_motion = 0.;
          if (is_line_preserved[operation_idx]) {
            descendant_motion = 1e-12;
          } else {
            for (std::size_t idx = 0; idx < num_joints; ++idx) {
              if (has_descendant[idx]) descendant_motion += global_bounds[sphere_idx*num_joints+idx]*max_dev[idx];
            }
          }
          coefficient *= std::min(reaches[path_idx], radius+descendant_motion);
        } else if (operation_kinds[operation_idx] != 1) {
          return -1;
        }
        local[sphere_idx*num_joints+joint_idx] += coefficient;
        operation_coefficients[sphere_idx*num_operations+operation_idx] = coefficient;
        has_descendant[joint_idx] = 1;
      }
    }
    return 0;
  } catch (...) {
    // C ABI境界での例外遮断。
    return -1;
  }
}

extern "C" int gng_region_points_clear(
    std::size_t num_points, std::size_t num_spheres, std::size_t num_normals,
    const double* points, const double* centers, const double* radii,
    const double* supports, const double* normals, double min_gap, double cell_radius) noexcept {
  if (!points || !centers || !radii || !supports || !normals ||
      !std::isfinite(min_gap) || min_gap < 0. || !std::isfinite(cell_radius) || cell_radius < 0.) return -1;
  bool has_ambiguity = false;
  for (std::size_t point_idx = 0; point_idx < num_points; ++point_idx) {
    for (std::size_t sphere_idx = 0; sphere_idx < num_spheres; ++sphere_idx) {
      double delta[3], squared_dist = 0.;
      for (int idx = 0; idx < 3; ++idx) {
        delta[idx] = points[point_idx*3+idx]-centers[sphere_idx*3+idx];
        squared_dist += delta[idx]*delta[idx];
      }
      const double radius = radii[sphere_idx]+min_gap+cell_radius+1e-9;
      const double squared_radius = radius*radius;
      const double rounding_error_th = 64*std::numeric_limits<double>::epsilon()*std::max({1., squared_dist, squared_radius});
      if (std::abs(squared_dist-squared_radius) <= rounding_error_th) {
        has_ambiguity = true;
        continue;
      }
      if (squared_dist > squared_radius) continue;
      bool is_outside = false, is_ambiguous = false;
      for (std::size_t normal_idx = 0; normal_idx < num_normals; ++normal_idx) {
        const auto* normal = normals+normal_idx*3;
        const double projection = delta[0]*normal[0]+delta[1]*normal[1]+delta[2]*normal[2];
        const double bound = supports[sphere_idx*num_normals+normal_idx]+min_gap+cell_radius+1e-9;
        const double plane_error_th = 64*std::numeric_limits<double>::epsilon()*std::max({1., std::abs(projection), std::abs(bound)});
        if (projection-bound > plane_error_th) {
          is_outside = true;
          break;
        }
        is_ambiguous = is_ambiguous || std::abs(projection-bound) <= plane_error_th;
      }
      if (!is_outside) {
        if (!is_ambiguous) return 0;
        has_ambiguity = true;
      }
    }
  }
  // 丸め誤差範囲内の境界だけ、参照実装への委任。
  return has_ambiguity ? 2 : 1;
}
