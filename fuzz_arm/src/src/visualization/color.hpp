#pragma once

#include <cstdint>

namespace robot_sim {
namespace visualization {

/**
 * @brief RGBA color representation
 *
 * Values are in the range [0.0, 1.0] for compatibility with OpenGL/DrawStuff
 */
struct Color {
  float r; ///< Red component [0.0, 1.0]
  float g; ///< Green component [0.0, 1.0]
  float b; ///< Blue component [0.0, 1.0]
  float a; ///< Alpha component [0.0, 1.0]

  /**
   * @brief Create color from 8-bit RGB values
   * @param r Red [0, 255]
   * @param g Green [0, 255]
   * @param b Blue [0, 255]
   * @param a Alpha [0.0, 1.0]
   */
  static Color fromRGB(int r, int g, int b, float a = 1.0f) {
    return {r / 255.0f, g / 255.0f, b / 255.0f, a};
  }

  /**
   * @brief Create color from hex value
   * @param hex Hex color value (e.g., 0xFF0000 for red)
   * @param a Alpha [0.0, 1.0]
   */
  static Color fromHex(uint32_t hex, float a = 1.0f) {
    return {((hex >> 16) & 0xFF) / 255.0f, ((hex >> 8) & 0xFF) / 255.0f,
            (hex & 0xFF) / 255.0f, a};
  }

  // Predefined colors
  static Color White() { return {1.0f, 1.0f, 1.0f, 1.0f}; }
  static Color Black() { return {0.0f, 0.0f, 0.0f, 1.0f}; }
  static Color Red() { return {1.0f, 0.0f, 0.0f, 1.0f}; }
  static Color Green() { return {0.0f, 1.0f, 0.0f, 1.0f}; }
  static Color Blue() { return {0.0f, 0.0f, 1.0f, 1.0f}; }
  static Color Yellow() { return {1.0f, 1.0f, 0.0f, 1.0f}; }
  static Color Magenta() { return {1.0f, 0.0f, 1.0f, 1.0f}; }
  static Color Cyan() { return {0.0f, 1.0f, 1.0f, 1.0f}; }
  static Color Gray() { return {0.5f, 0.5f, 0.5f, 1.0f}; }
  static Color Gold() { return fromRGB(255, 215, 0); }
  static Color Purple() { return fromRGB(128, 0, 128); }
  static Color Orange() { return fromRGB(255, 165, 0); }
  static Color Brown() { return fromRGB(165, 42, 42); }
};

} // namespace visualization
} // namespace robot_sim
