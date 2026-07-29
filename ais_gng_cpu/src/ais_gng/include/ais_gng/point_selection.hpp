#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fuzzrobo
{

enum class PointSamplingMode
{
  Head,
  Uniform,
};

std::optional<PointSamplingMode> parsePointSamplingMode(const std::string & value);

std::vector<uint32_t> selectPointIndices(
  uint32_t point_count,
  uint32_t max_points,
  PointSamplingMode mode);

}  // namespace fuzzrobo
