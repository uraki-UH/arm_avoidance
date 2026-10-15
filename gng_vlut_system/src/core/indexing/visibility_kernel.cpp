// 同一ROI格子の観測光線照会。0: unknown、1: free、2: obstacle。
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace {
struct grid {
  double step;
  const double* origin;
  const int64_t* layout;

  bool is_valid() const {
    if (!std::isfinite(step) || step < .001 || step > 1.) return false;
    size_t total = 1;
    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(origin[axis]) || layout[axis + 3] <= 0 || layout[axis + 3] > 512) return false;
      total *= static_cast<size_t>(layout[axis + 3]);
    }
    auto shifts = std::array<int64_t, 3>{layout[6], layout[7], layout[8]};
    std::sort(shifts.begin(), shifts.end());
    return total <= 8000000 && shifts == std::array<int64_t, 3>{0, 21, 42};
  }

  size_t size() const { return layout[3] * layout[4] * layout[5]; }

  int64_t idx(const std::array<int64_t, 3>& cell) const {
    for (int axis = 0; axis < 3; ++axis)
      if (cell[axis] < layout[axis] || cell[axis] >= layout[axis] + layout[axis + 3]) return -1;
    return ((cell[0] - layout[0]) * layout[4] + cell[1] - layout[1]) * layout[5] + cell[2] - layout[2];
  }

  std::array<int64_t, 3> cell(const double* point) const {
    std::array<int64_t, 3> result;
    for (int axis = 0; axis < 3; ++axis)
      result[axis] = static_cast<int64_t>(std::floor((point[axis] - origin[axis]) / step));
    return result;
  }

  std::array<int64_t, 3> packed_cell(int64_t id) const {
    std::array<int64_t, 3> result;
    for (int axis = 0; axis < 3; ++axis)
      result[axis] = static_cast<int64_t>((static_cast<uint64_t>(id) >> layout[axis + 6]) & ((1ULL << 21) - 1)) - layout[9];
    return result;
  }

  int64_t packed_idx(int64_t id) const { return idx(packed_cell(id)); }
};

bool has_finite_point(const double* point) {
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]) &&
         std::abs(point[0]) < 1e6 && std::abs(point[1]) < 1e6 && std::abs(point[2]) < 1e6;
}

bool has_sphere_cell(const grid& cells, const std::array<int64_t, 3>& cell, const double* center, double radius) {
  double squared = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const double low = cells.origin[axis] + cell[axis] * cells.step;
    const double gap = std::max({low - center[axis], center[axis] - low - cells.step, 0.});
    squared += gap * gap;
  }
  return squared <= radius * radius;
}

std::array<double, 3> to_sensor(const double* point, const double* transform) {
  std::array<double, 3> result{};
  for (int axis = 0; axis < 3; ++axis)
    for (int row = 0; row < 3; ++row)
      result[axis] += transform[row * 4 + axis] * (point[row] - transform[row * 4 + 3]);
  return result;
}

int cube_pixel(const std::array<double, 3>& point, int resolution, float& depth) {
  int axis = 0;
  if (std::abs(point[1]) > std::abs(point[axis])) axis = 1;
  if (std::abs(point[2]) > std::abs(point[axis])) axis = 2;
  const double forward = std::abs(point[axis]);
  if (forward < 1e-12) return -1;
  const int first = axis == 0 ? 1 : 0;
  const int second = axis == 2 ? 1 : 2;
  const int u = std::clamp(static_cast<int>((point[first] / forward + 1) * .5 * resolution), 0, resolution - 1);
  const int v = std::clamp(static_cast<int>((point[second] / forward + 1) * .5 * resolution), 0, resolution - 1);
  depth = static_cast<float>(forward);
  return ((axis * 2 + (point[axis] < 0)) * resolution + u) * resolution + v;
}

bool add_shadow(const std::array<double, 3>& center, const std::array<double, 3>& half,
                int resolution, std::vector<float>& depths, size_t max_steps, int64_t* counts) {
  // セルの有限な角度幅による影。物体の縁をまたぐ奥行き補間なし。
  for (int face = 0; face < 6; ++face) {
    const int axis = face / 2;
    const int first = axis == 0 ? 1 : 0;
    const int second = axis == 2 ? 1 : 2;
    const double forward = (face % 2 ? -1 : 1) * center[axis];
    const double near = forward - half[axis], far = forward + half[axis];
    if (far <= 0 || std::abs(center[first]) - half[first] > far ||
                    std::abs(center[second]) - half[second] > far) continue;
    int low[2], high[2];
    for (int which = 0; which < 2; ++which) {
      const int side = which == 0 ? first : second;
      double min_ratio = -1, max_ratio = 1;
      if (near > 1e-12) {
        const double a = (center[side] - half[side]) / near;
        const double b = (center[side] - half[side]) / far;
        const double c = (center[side] + half[side]) / near;
        const double d = (center[side] + half[side]) / far;
        min_ratio = std::min({a, b, c, d});
        max_ratio = std::max({a, b, c, d});
      }
      low[which] = std::clamp(static_cast<int>(std::floor((std::max(-1., min_ratio) + 1) * .5 * resolution)), 0, resolution - 1);
      high[which] = std::clamp(static_cast<int>(std::floor((std::min(1., max_ratio) + 1) * .5 * resolution)), 0, resolution - 1);
    }
    for (int u = low[0]; u <= high[0]; ++u)
      for (int v = low[1]; v <= high[1]; ++v) {
        if (static_cast<size_t>(++counts[1]) > max_steps) return false;
        auto& depth = depths[(face * resolution + u) * resolution + v];
        depth = std::min(depth, static_cast<float>(std::max(0., near)));
      }
  }
  ++counts[0];
  return true;
}
}

extern "C" int gng_visibility_kernel_version() { return 3; }

extern "C" int gng_visibility_projection(
    double step, const double* origin, const int64_t* layout, const double* transform,
    const double* limits, int resolution, int32_t* pixels, float* depths) noexcept {
  const grid cells{step, origin, layout};
  if (!cells.is_valid() || resolution < 8 || resolution > 256) return -1;
  size_t idx = 0;
  for (int64_t x = 0; x < layout[3]; ++x)
    for (int64_t y = 0; y < layout[4]; ++y)
      for (int64_t z = 0; z < layout[5]; ++z, ++idx) {
        const double point[3]{origin[0] + (layout[0] + x + .5) * step,
                              origin[1] + (layout[1] + y + .5) * step,
                              origin[2] + (layout[2] + z + .5) * step};
        const auto sensor = to_sensor(point, transform);
        const double horizontal = std::hypot(sensor[0], sensor[1]);
        const double squared = horizontal * horizontal + sensor[2] * sensor[2];
        pixels[idx] = -1;
        depths[idx] = 0;
        if (squared < limits[0] * limits[0]) { pixels[idx] = -2; continue; }
        if (squared > limits[1] * limits[1] || sensor[2] < limits[2] * horizontal ||
            sensor[2] > limits[3] * horizontal) continue;
        pixels[idx] = cube_pixel(sensor, resolution, depths[idx]);
      }
  return 0;
}

extern "C" int gng_visibility_shadow_build(
    size_t num_points, const double* points, size_t num_blocked, const int64_t* blocked,
    size_t num_obstacles, const int64_t* obstacles, double step, const double* origin,
    const int64_t* layout, const double* transform, const double* limits, int resolution,
    const int32_t* pixels, const float* projected_depths, size_t max_steps,
    uint8_t* states, int64_t* counts) noexcept try {
  const grid cells{step, origin, layout};
  if (!cells.is_valid() || resolution < 8 || resolution > 256 || num_points > 2000000) return -1;
  std::fill(counts, counts + 4, 0);
  std::vector<float> depths(6 * resolution * resolution, std::numeric_limits<float>::infinity());
  std::array<double, 3> half{};
  for (int axis = 0; axis < 3; ++axis)
    for (int row = 0; row < 3; ++row) half[axis] += std::abs(transform[row * 4 + axis]) * step * .5;
  double max_dist_squared = 0;
  for (int corner = 0; corner < 8; ++corner) {
    double point[3];
    for (int axis = 0; axis < 3; ++axis)
      point[axis] = origin[axis] + (layout[axis] + ((corner >> axis) & 1) * layout[axis + 3]) * step;
    const auto sensor = to_sensor(point, transform);
    max_dist_squared = std::max(max_dist_squared, sensor[0]*sensor[0] + sensor[1]*sensor[1] + sensor[2]*sensor[2]);
  }
  const double max_occluder_dist = std::sqrt(max_dist_squared) + std::sqrt(3.) * step;
  const auto add = [&](const std::array<double, 3>& center) {
    const double squared = center[0]*center[0] + center[1]*center[1] + center[2]*center[2];
    if (squared < limits[0] * limits[0] || squared > max_occluder_dist * max_occluder_dist) return true;
    return add_shadow(center, half, resolution, depths, max_steps, counts);
  };
  for (size_t idx = 0; idx < num_points; ++idx) {
    const auto* point = points + idx * 3;
    if (has_finite_point(point) && !add({point[0], point[1], point[2]})) return 3;
  }
  // 既存ROI・記録自己マスクのIDだけの投影。点群の格子再登録なし。
  for (size_t idx = 0; idx < num_blocked; ++idx) {
    const auto cell = cells.packed_cell(blocked[idx]);
    double point[3];
    for (int axis = 0; axis < 3; ++axis) point[axis] = origin[axis] + (cell[axis] + .5) * step;
    if (!add(to_sensor(point, transform))) return 3;
  }
  // 視野内は基本free。最短の遮蔽物より奥だけのunknown上書き。
  for (size_t idx = 0; idx < cells.size(); ++idx) {
    const int pixel = pixels[idx];
    states[idx] = pixel >= 0 && projected_depths[idx] < depths[pixel] ? 1 : 0;
    counts[2] += states[idx] == 1;
  }
  for (size_t idx = 0; idx < num_blocked; ++idx) {
    const auto target = cells.packed_idx(blocked[idx]);
    if (target >= 0 && states[target] == 1) { states[target] = 0; --counts[2]; }
  }
  // free・影より観測障害物を優先。前フレームfreeからの昇格制限なし。
  for (size_t idx = 0; idx < num_obstacles; ++idx) {
    const auto target = cells.packed_idx(obstacles[idx]);
    if (target < 0 || states[target] == 2) continue;
    if (states[target] == 1) --counts[2];
    states[target] = 2;
    ++counts[3];
  }
  return 0;
} catch (...) {
  return -2;
}

extern "C" int gng_visibility_build(
    size_t num_points, const double* points, const double* sensor,
    size_t num_blocked, const int64_t* blocked, size_t num_obstacles, const int64_t* obstacles,
    double step, const double* origin, const int64_t* layout, size_t max_steps, int enable_ray_reuse,
    uint8_t* states, int64_t* counts) noexcept try {
  const grid cells{step, origin, layout};
  if (!cells.is_valid() || !has_finite_point(sensor) || num_points > 2000000) return -1;
  std::fill(states, states + cells.size(), 0);
  std::fill(counts, counts + 4, 0);
  std::vector<int64_t> blocked_cells, ray_cells;
  blocked_cells.reserve(num_blocked + num_points);
  ray_cells.reserve(num_points);
  // 生点群の全終端と除去前ROIの遮蔽保持。自己・孤立点の除去による奥側の空き化防止。
  for (size_t idx = 0; idx < num_blocked; ++idx) {
    const auto target = cells.packed_idx(blocked[idx]);
    if (target >= 0) { states[target] = 4; blocked_cells.push_back(target); }
  }
  for (size_t idx = 0; idx < num_points; ++idx) {
    const auto* point = points + idx * 3;
    if (!has_finite_point(point)) continue;
    const auto target = cells.idx(cells.cell(point));
    if (target >= 0) { states[target] = 4; blocked_cells.push_back(target); }
  }
  for (size_t idx = 0; idx < num_obstacles; ++idx) {
    const auto target = cells.packed_idx(obstacles[idx]);
    if (target >= 0 && states[target] != 2) { states[target] = 2; ++counts[3]; }
  }
  for (size_t idx = 0; idx < num_points; ++idx) {
    const auto* point = points + idx * 3;
    if (!has_finite_point(point)) continue;
    double enter = 0., exit = 1.;
    std::array<double, 3> delta;
    for (int axis = 0; axis < 3; ++axis) {
      delta[axis] = point[axis] - sensor[axis];
      const double low = origin[axis] + layout[axis] * step;
      const double high = low + layout[axis + 3] * step;
      if (std::abs(delta[axis]) < 1e-15) {
        if (sensor[axis] < low || sensor[axis] >= high) exit = -1;
      } else {
        double first = (low - sensor[axis]) / delta[axis];
        double second = (high - sensor[axis]) / delta[axis];
        if (first > second) std::swap(first, second);
        enter = std::max(enter, first);
        exit = std::min(exit, second);
      }
    }
    if (enter >= exit) continue;
    // 同じ終端セルへ到達する実光線1本だけの採用。新たなfreeの創作なし、観測量は保守側。
    if (enable_ray_reuse) {
      double terminal[3];
      for (int axis = 0; axis < 3; ++axis) terminal[axis] = sensor[axis] + delta[axis] * (exit - 1e-10);
      const auto target = cells.idx(cells.cell(terminal));
      if (target >= 0) {
        if (states[target] & 8) continue;
        states[target] |= 8;
        ray_cells.push_back(target);
      }
    }
    double position[3];
    for (int axis = 0; axis < 3; ++axis) position[axis] = sensor[axis] + delta[axis] * (enter + 1e-10);
    auto cell = cells.cell(position);
    const auto endpoint = cells.cell(point);
    std::array<double, 3> next, increment;
    std::array<int, 3> direction;
    for (int axis = 0; axis < 3; ++axis) {
      direction[axis] = delta[axis] > 0 ? 1 : -1;
      increment[axis] = delta[axis] == 0 ? std::numeric_limits<double>::infinity() : step / std::abs(delta[axis]);
      next[axis] = delta[axis] == 0 ? increment[axis] :
          (origin[axis] + (cell[axis] + (direction[axis] > 0)) * step - sensor[axis]) / delta[axis];
    }
    ++counts[0];
    while (true) {
      if (static_cast<size_t>(++counts[1]) > max_steps) {
        std::fill(states, states + cells.size(), 0);
        counts[2] = counts[3] = 0;
        return 3;
      }
      const auto target = cells.idx(cell);
      if (target < 0 || cell == endpoint || (states[target] & 6)) break;
      if (!(states[target] & 1)) ++counts[2];
      states[target] |= 1;
      const double crossing = std::min({next[0], next[1], next[2]});
      if (crossing >= exit) break;
      // 稜線・頂点の同時通過。接触だけの隣接セルへの過剰なfree付与防止。
      for (int axis = 0; axis < 3; ++axis) {
        if (next[axis] <= crossing + 1e-12) {
          cell[axis] += direction[axis];
          next[axis] += increment[axis];
        }
      }
    }
  }
  for (const auto target : blocked_cells) if ((states[target] & 7) == 4) states[target] = 0;
  for (const auto target : ray_cells) states[target] &= 7;
  return 0;
} catch (...) {
  return -2;
}

extern "C" int gng_visibility_check(
    const uint8_t* states, double step, const double* origin, const int64_t* layout,
    size_t num_spheres, const double* initial, const double* radii,
    size_t num_poses, const double* centers, const double* padding,
    size_t max_steps, int64_t* counts) noexcept {
  const grid cells{step, origin, layout};
  if (!cells.is_valid() || num_spheres > 10000 || num_poses > 4096) return -1;
  counts[0] = 0;
  counts[1] = -1;
  for (size_t pose = 0; pose < num_poses; ++pose) {
    for (size_t sphere = 0; sphere < num_spheres; ++sphere) {
      const auto* center = centers + (pose * num_spheres + sphere) * 3;
      const double radius = radii[sphere] + padding[pose * num_spheres + sphere];
      if (!has_finite_point(center) || !std::isfinite(radius) || radius < 0 || radius > 2) return -1;
      std::array<int64_t, 3> low, high;
      for (int axis = 0; axis < 3; ++axis) {
        low[axis] = static_cast<int64_t>(std::floor((center[axis] - radius - origin[axis]) / step));
        high[axis] = static_cast<int64_t>(std::floor((center[axis] + radius - origin[axis]) / step));
      }
      for (int64_t x = low[0]; x <= high[0]; ++x)
        for (int64_t y = low[1]; y <= high[1]; ++y)
          for (int64_t z = low[2]; z <= high[2]; ++z) {
            if (static_cast<size_t>(++counts[0]) > max_steps) return 3;
            const std::array<int64_t, 3> cell{x, y, z};
            if (!has_sphere_cell(cells, cell, center, radius)) continue;
            const auto target = cells.idx(cell);
            if (target < 0) return 4;
            if (states[target] == 2) { counts[1] = target; return 2; }
            if (states[target] == 1) continue;
            bool is_initial = false;
            for (size_t before = 0; before < num_spheres; ++before) {
              if (has_sphere_cell(cells, cell, initial + before * 3, radii[before])) {
                is_initial = true;
                break;
              }
            }
            // 開始姿勢の身体内部だけのunknown免除。観測障害物への免除なし。
            if (!is_initial) { counts[1] = target; return 1; }
          }
    }
  }
  return 0;
}
