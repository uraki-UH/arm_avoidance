// JavaScript版と同じ倍精度の演算順による深度・点群・着色の生成。
// 入出力領域は呼出し側で確保。サンプル間引き・近似演算なし。
namespace {
double round_pixel(double value) {
  const double lower = __builtin_floor(value);
  return value - lower < 0.5 ? lower : lower + 1.0;
}
double max_value(double left, double right) { return left > right ? left : right; }
}
extern "C" void process_points(
    const double* calibration, const float* raw, const float* color_depth,
    const unsigned char* rgba, const float* right_depth, const float* target_depth,
    float* depth, unsigned short* z16, float* xyz, unsigned char* colors,
    unsigned char* color_valid, unsigned int* pixels, double* statistics) {
  const unsigned int width = static_cast<unsigned int>(calibration[0]);
  const unsigned int height = static_cast<unsigned int>(calibration[1]);
  const double fx = calibration[2], fy = calibration[3];
  const double ppx = calibration[4], ppy = calibration[5];
  const unsigned int color_width = static_cast<unsigned int>(calibration[6]);
  const unsigned int color_height = static_cast<unsigned int>(calibration[7]);
  const double color_fx = calibration[8], color_fy = calibration[9];
  const double color_ppx = calibration[10], color_ppy = calibration[11];
  const double baseline = calibration[12], scale = calibration[13];
  const double min_depth = calibration[14], max_depth = calibration[15];
  const bool enable_direct_depth = calibration[16] != 0;
  const double* matrix = calibration + 17;
  const double color_tolerance = .75 / (color_fx < color_fy ? color_fx : color_fy);
  unsigned int num_valid = 0, num_colored = 0, num_stereo_rejected = 0;
  double min_z = __builtin_inf(), max_z = 0;
  __builtin_memset(depth, 0, width * height * sizeof(float));
  __builtin_memset(z16, 0, width * height * sizeof(unsigned short));
  for (unsigned int v = 0; v < height; ++v) {
    const unsigned int row = (enable_direct_depth ? height - 1 - v : v) * width;
    for (unsigned int u = 0; u < width; ++u) {
      const unsigned int source_idx = row + u, idx = v * width + u;
      double z = raw[source_idx];
      if (target_depth && target_depth[source_idx] != z) continue;
      if (!__builtin_isfinite(z) || z < min_depth || z > max_depth) continue;
      if (right_depth) {
        const double right_u = round_pixel(u - fx * baseline / z);
        const double right_z = right_u >= 0 && right_u < width
          ? right_depth[row + static_cast<unsigned int>(right_u)] : 0;
        if (right_z == 0 || right_z != right_z || __builtin_fabs(right_z - z) > max_value(.002, z / fx)) {
          ++num_stereo_rejected; continue;
        }
        z = round_pixel(z / scale) * scale;
        if (z < min_depth || z > max_depth) continue;
      }
      depth[idx] = static_cast<float>(z);
      const double unit_z = max_value(1, round_pixel(z / scale));
      z16[idx] = static_cast<unsigned short>(unit_z < 65535 ? unit_z : 65535);
      const double x = (u - ppx) * z / fx, y = (v - ppy) * z / fy;
      const unsigned int point_idx = num_valid * 3;
      xyz[point_idx] = static_cast<float>(x); xyz[point_idx + 1] = static_cast<float>(y); xyz[point_idx + 2] = static_cast<float>(z);
      pixels[num_valid] = idx;
      const double color_x = matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12];
      const double color_y = matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13];
      const double color_z = matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14];
      const double color_u = round_pixel(color_fx * color_x / color_z + color_ppx);
      const double color_v = round_pixel(color_fy * color_y / color_z + color_ppy);
      bool has_color = false;
      if (color_z > 0 && color_u >= 0 && color_u < color_width && color_v >= 0 && color_v < color_height) {
        const unsigned int cu = static_cast<unsigned int>(color_u), cv = static_cast<unsigned int>(color_v);
        const double visible_z = color_depth[(enable_direct_depth ? color_height - 1 - cv : cv) * color_width + cu];
        has_color = visible_z > 0 && __builtin_fabs(visible_z - color_z) <= max_value(.001, color_tolerance * color_z);
        if (has_color) {
          const unsigned int color_idx = (cv * color_width + cu) * 4;
          colors[point_idx] = rgba[color_idx]; colors[point_idx + 1] = rgba[color_idx + 1]; colors[point_idx + 2] = rgba[color_idx + 2]; ++num_colored;
        }
      }
      if (!has_color) { colors[point_idx] = 155; colors[point_idx + 1] = 165; colors[point_idx + 2] = 175; }
      color_valid[num_valid] = has_color ? 1 : 0;
      if (z < min_z) min_z = z;
      if (z > max_z) max_z = z;
      ++num_valid;
    }
  }
  statistics[0] = num_valid; statistics[1] = num_colored; statistics[2] = num_stereo_rejected;
  statistics[3] = num_valid ? min_z : 0; statistics[4] = num_valid ? max_z : 0;
}
