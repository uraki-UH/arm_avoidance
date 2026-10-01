#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simulation {

// 関節角の線形補間に沿った離散検査。連続時間の非干渉証明は対象外。
template <typename angles_type, typename collision_predicate>
bool has_joint_segment_collision(const angles_type &first, const angles_type &second,
                                 double max_joint_step, collision_predicate is_colliding) {
  if (first.size() != second.size() || first.size() == 0 ||
      !first.allFinite() || !second.allFinite() ||
      !std::isfinite(max_joint_step) || max_joint_step <= 0.0) {
    throw std::invalid_argument("Invalid joint-segment collision input");
  }
  const double span = (second - first).cwiseAbs().maxCoeff();
  const double steps = std::ceil(span / max_joint_step);
  if (steps > 1000000.0) throw std::invalid_argument("Joint segment exceeds sample budget");
  const int num_steps = std::max(1, static_cast<int>(steps));
  // 両端は補間の丸め誤差を含まない保存角そのものによる検査
  if (is_colliding(first)) return true;
  for (int idx = 1; idx < num_steps; ++idx) {
    const auto ratio = static_cast<typename angles_type::Scalar>(idx) / num_steps;
    const angles_type angles = first + (second - first) * ratio;
    if (is_colliding(angles)) return true;
  }
  return is_colliding(second);
}

} // namespace simulation
