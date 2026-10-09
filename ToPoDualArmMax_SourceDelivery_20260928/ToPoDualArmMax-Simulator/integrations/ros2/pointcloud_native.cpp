// ROS点群の配列処理。リトルエンディアン形式とPython floatの演算順序の保持。
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace {
constexpr Py_ssize_t max_points = 2073600;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

struct buffer_view {
  Py_buffer value{};
  ~buffer_view() { if (value.obj) PyBuffer_Release(&value); }
  bool acquire(PyObject* object, bool allow_write = false) {
    return PyObject_GetBuffer(object, &value, allow_write ? PyBUF_WRITABLE : PyBUF_SIMPLE) == 0;
  }
  const unsigned char* data() const { return static_cast<const unsigned char*>(value.buf); }
  unsigned char* output() const { return static_cast<unsigned char*>(value.buf); }
};

float read_float(const unsigned char* data) {
  const std::uint32_t bits = std::uint32_t(data[0]) | (std::uint32_t(data[1]) << 8) |
      (std::uint32_t(data[2]) << 16) | (std::uint32_t(data[3]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
void write_bits(unsigned char* data, std::uint32_t bits) {
  for (int idx = 0; idx < 4; ++idx) data[idx] = static_cast<unsigned char>(bits >> (idx * 8));
}
void write_float(unsigned char* data, float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  write_bits(data, bits);
}
void write_color(unsigned char* output, const unsigned char* color) {
  output[12] = color[2]; output[13] = color[1]; output[14] = color[0];
  output[15] = 0; output[16] = color[3];
  output[17] = 0; output[18] = 0; output[19] = 0;
}
PyObject* invalid(const char* message) { PyErr_SetString(PyExc_ValueError, message); return nullptr; }

PyObject* validate_payload(PyObject*, PyObject* args) {
  PyObject* object;
  Py_ssize_t num_points, num_pixels;
  int has_color;
  if (!PyArg_ParseTuple(args, "Onnp", &object, &num_points, &num_pixels, &has_color)) return nullptr;
  buffer_view input;
  if (!input.acquire(object)) return nullptr;
  if (num_points < 0 || num_points > max_points || num_pixels < 0 || num_pixels > max_points ||
      input.value.len != num_points * (has_color ? 16 : 12) + num_pixels * 4)
    return invalid("点群・深度画像のデータ長が不正です");
  const auto* depth = input.data() + num_points * 12;
  const auto* colors = depth + num_pixels * 4;
  const char* error = nullptr;
  Py_BEGIN_ALLOW_THREADS
  Py_ssize_t num_valid = 0;
  for (Py_ssize_t idx = 0; idx < num_pixels; ++idx) {
    const float z = read_float(depth + idx * 4);
    if (!std::isfinite(z) || z < 0) { error = "深度値が不正です"; break; }
    num_valid += z > 0;
  }
  if (!error && has_color) {
    for (Py_ssize_t idx = 0; idx < num_points; ++idx)
      if (colors[idx * 4 + 3] > 1) { error = "色の有効フラグが不正です"; break; }
    if (!error && num_pixels && num_valid != num_points)
      error = "深度と色付き点群の有効点数が一致しません";
  }
  if (!error) {
    for (Py_ssize_t idx = 0; idx < num_points * 3; ++idx)
      if (!std::isfinite(read_float(input.data() + idx * 4))) { error = "座標に非有限値があります"; break; }
  }
  Py_END_ALLOW_THREADS
  if (error) return invalid(error);
  Py_RETURN_NONE;
}

PyObject* build_depth(PyObject*, PyObject* args) {
  PyObject *depth_object, *color_object, *output_object;
  int width, height;
  double fx, fy, ppx, ppy;
  if (!PyArg_ParseTuple(args, "OOiiddddO", &depth_object, &color_object, &width, &height,
                       &fx, &fy, &ppx, &ppy, &output_object)) return nullptr;
  buffer_view depth, colors, output;
  const bool has_color = color_object != Py_None;
  if (!depth.acquire(depth_object) || !output.acquire(output_object, true) ||
      (has_color && !colors.acquire(color_object))) return nullptr;
  if (width < 1 || width > 1920 || height < 1 || height > 1080)
    return invalid("深度画像の寸法が不正です");
  const Py_ssize_t num_pixels = Py_ssize_t(width) * height;
  const int stride = has_color ? 20 : 12;
  if (depth.value.len != num_pixels * 4 || output.value.len != num_pixels * stride ||
      (has_color && colors.value.len % 4)) return invalid("深度・色・出力のデータ長が不正です");
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(ppx) || !std::isfinite(ppy) || fx <= 0 || fy <= 0)
    return invalid("内部パラメータが不正です");
  const char* error = nullptr;
  bool has_overflow = false;
  Py_BEGIN_ALLOW_THREADS
  Py_ssize_t color_offset = 0;
  for (int row = 0; row < height && !error; ++row) {
    for (int column = 0; column < width; ++column) {
      const Py_ssize_t idx = Py_ssize_t(row) * width + column;
      const double z = read_float(depth.data() + idx * 4);
      auto* point = output.output() + idx * stride;
      if (!std::isfinite(z) || z < 0) { error = "深度値が不正です"; break; }
      if (z == 0) {
        // struct.pack('<f', float('nan'))と同じ無効画素・色・パディング。
        for (int axis = 0; axis < 3; ++axis) write_bits(point + axis * 4, 0x7fc00000);
        if (has_color) std::memset(point + 12, 0, 8);
        continue;
      }
      const double x = (column - ppx) * z / fx, y = (row - ppy) * z / fy;
      const float point_x = static_cast<float>(x), point_y = static_cast<float>(y);
      if (!std::isfinite(point_x) || !std::isfinite(point_y)) {
        error = "逆投影結果がfloat32の範囲外です"; has_overflow = true; break;
      }
      write_float(point, point_x); write_float(point + 4, point_y);
      write_float(point + 8, static_cast<float>(z));
      if (has_color) {
        if (color_offset + 4 > colors.value.len) { error = "点群と色の点数が一致しません"; break; }
        if (colors.data()[color_offset + 3] > 1) { error = "色の有効フラグが不正です"; break; }
        write_color(point, colors.data() + color_offset); color_offset += 4;
      }
    }
  }
  if (!error && has_color && color_offset != colors.value.len) error = "点群と色の点数が一致しません";
  Py_END_ALLOW_THREADS
  if (error) { PyErr_SetString(has_overflow ? PyExc_OverflowError : PyExc_ValueError, error); return nullptr; }
  Py_RETURN_NONE;
}

PyObject* colorize(PyObject*, PyObject* args) {
  PyObject *xyz_object, *color_object, *depth_object, *output_object;
  if (!PyArg_ParseTuple(args, "OOOO", &xyz_object, &color_object, &depth_object, &output_object)) return nullptr;
  buffer_view xyz, colors, depth, output;
  const bool has_depth = depth_object != Py_None;
  if (!xyz.acquire(xyz_object) || !colors.acquire(color_object) || !output.acquire(output_object, true) ||
      (has_depth && !depth.acquire(depth_object))) return nullptr;
  const Py_ssize_t num_points = xyz.value.len / 12;
  if (xyz.value.len % 12 || num_points > max_points || colors.value.len % 4 ||
      output.value.len != num_points * 20 || (has_depth && depth.value.len != num_points * 4))
    return invalid("点群・色・深度・出力のデータ長が不正です");
  const char* error = nullptr;
  Py_BEGIN_ALLOW_THREADS
  Py_ssize_t color_offset = 0;
  for (Py_ssize_t idx = 0; idx < num_points; ++idx) {
    auto* point = output.output() + idx * 20;
    std::memcpy(point, xyz.data() + idx * 12, 12);
    std::memset(point + 12, 0, 8);
    if (has_depth) {
      const float z = read_float(depth.data() + idx * 4);
      if (!std::isfinite(z) || z < 0) { error = "深度値が不正です"; break; }
      if (z == 0) continue;
    }
    if (color_offset + 4 > colors.value.len) { error = "点群と色の点数が一致しません"; break; }
    if (colors.data()[color_offset + 3] > 1) { error = "色の有効フラグが不正です"; break; }
    write_color(point, colors.data() + color_offset); color_offset += 4;
  }
  if (!error && color_offset != colors.value.len) error = "点群と色の点数が一致しません";
  Py_END_ALLOW_THREADS
  if (error) return invalid(error);
  Py_RETURN_NONE;
}

PyMethodDef methods[] = {
  {"validate_payload", validate_payload, METH_VARARGS, "点群・深度・色の境界検証。"},
  {"build_depth", build_depth, METH_VARARGS, "画素順の逆投影と色付け。"},
  {"colorize", colorize, METH_VARARGS, "XYZとRGB・有効フラグの結合。"},
  {nullptr, nullptr, 0, nullptr}
};
PyModuleDef module = {PyModuleDef_HEAD_INIT, "_pointcloud_native", "点群配列のC++処理。", -1, methods,
                     nullptr, nullptr, nullptr, nullptr};
}
PyMODINIT_FUNC PyInit__pointcloud_native() { return PyModule_Create(&module); }
