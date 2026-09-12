#pragma once
#include <std_msgs/msg/color_rgba.hpp>
#include <array>
#include <cmath>
#include <cstdint>

namespace arrow_visualization {
// 候補矢印の未評価・範囲内・範囲外の共通sRGB配色。範囲内は明るい薄緑水色
inline constexpr std::array<std::uint32_t, 3> state_colors_srgb{{0xf3da59, 0x7ceeb6, 0xc4c4c4}};

inline std_msgs::msg::ColorRGBA state_color(std::uint8_t state, float alpha = 1.0F) {
  const auto rgb = state_colors_srgb[state < state_colors_srgb.size() ? state : 0];
  const auto linear = [](std::uint32_t channel) {
    const double value = channel / 255.0;
    return static_cast<float>(value <= 0.04045 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4));
  };
  std_msgs::msg::ColorRGBA color;
  color.r = linear((rgb >> 16) & 255);
  color.g = linear((rgb >> 8) & 255);
  color.b = linear(rgb & 255);
  color.a = alpha;
  return color;
}
}  // arrow_visualization 名前空間
