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
