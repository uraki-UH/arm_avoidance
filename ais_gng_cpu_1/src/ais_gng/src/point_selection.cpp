#include <ais_gng/point_selection.hpp>

#include <algorithm>
#include <numeric>

namespace fuzzrobo
{

std::optional<PointSamplingMode> parsePointSamplingMode(const std::string & value)
{
  if (value == "head") {
    return PointSamplingMode::Head;
  }
  if (value == "uniform") {
    return PointSamplingMode::Uniform;
  }
  return std::nullopt;
}

std::vector<uint32_t> selectPointIndices(
  uint32_t point_count,
  uint32_t max_points,
  PointSamplingMode mode)
{
  const uint32_t selected_count = std::min(point_count, max_points);
  std::vector<uint32_t> indices(selected_count);
  if (selected_count == 0) {
    return indices;
  }

  if (mode == PointSamplingMode::Head || selected_count == point_count) {
    std::iota(indices.begin(), indices.end(), 0U);
    return indices;
  }

  const uint64_t denominator = 2ULL * selected_count;
  for (uint32_t i = 0; i < selected_count; ++i) {
    const uint64_t numerator = (2ULL * i + 1ULL) * point_count;
    indices[i] = static_cast<uint32_t>(numerator / denominator);
  }
  return indices;
}

}  // namespace fuzzrobo
