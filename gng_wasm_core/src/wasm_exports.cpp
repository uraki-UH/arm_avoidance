#include "gng_wasm_core/gng_kernel.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#define GNG_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define GNG_EXPORT
#endif

namespace {
gng_wasm_core::GngKernel &kernel() {
  static gng_wasm_core::GngKernel instance;
  return instance;
}

std::vector<gng_wasm_core::Point3f> &points_buffer() {
  static std::vector<gng_wasm_core::Point3f> points;
  return points;
}

std::vector<std::uint8_t> &labels_buffer() {
  static std::vector<std::uint8_t> labels;
  return labels;
}

std::string &json_buffer() {
  static std::string json;
  return json;
}
}

extern "C" {

struct GngWasmConfig {
  std::uint32_t max_nodes;
  std::uint32_t iterations;
  std::uint32_t lambda;
  std::uint32_t max_age;
  float eb;
  float en;
  std::uint32_t seed;
  float node_grid;
  float node_interval;
};

GNG_EXPORT std::uint32_t gng_wasm_abi_version() {
  return 3;
}

GNG_EXPORT void *gng_wasm_create() {
  kernel().reset();
  return &kernel();
}

GNG_EXPORT void gng_wasm_destroy(void *) {
}

GNG_EXPORT void gng_wasm_reset() {
  kernel().reset();
  points_buffer().clear();
  labels_buffer().clear();
  json_buffer().clear();
}

GNG_EXPORT void gng_wasm_set_config(const GngWasmConfig *config) {
  if (!config) return;
  gng_wasm_core::Config cfg = kernel().config();
  cfg.max_nodes = config->max_nodes;
  cfg.iterations = config->iterations;
  cfg.lambda = config->lambda;
  cfg.max_age = config->max_age;
  cfg.eb = config->eb;
  cfg.en = config->en;
  cfg.seed = config->seed;
  cfg.grid = config->node_grid;
  cfg.interval[0] = config->node_interval;
  kernel().set_config(cfg);
}

GNG_EXPORT std::int32_t gng_wasm_set_parameter(const char *name, std::uint32_t index, float value) {
  return kernel().set_parameter(name, index, value) ? 1 : 0;
}

GNG_EXPORT void gng_wasm_set_points(const float *xyz, std::uint32_t point_count) {
  auto &buffer = points_buffer();
  buffer.clear();
  if (!xyz || point_count == 0) {
    kernel().set_points(buffer);
    return;
  }
  buffer.reserve(point_count);
  for (std::uint32_t i = 0; i < point_count; ++i) {
    const std::size_t base = static_cast<std::size_t>(i) * 3;
    buffer.push_back(gng_wasm_core::Point3f{xyz[base], xyz[base + 1], xyz[base + 2]});
  }
  kernel().set_points(buffer);
}

GNG_EXPORT void gng_wasm_update_points(const float *xyz, std::uint32_t point_count) {
  auto &buffer = points_buffer();
  buffer.clear();
  if (!xyz || point_count == 0) {
    kernel().update_points(buffer);
    return;
  }
  buffer.reserve(point_count);
  for (std::uint32_t i = 0; i < point_count; ++i) {
    const std::size_t base = static_cast<std::size_t>(i) * 3;
    buffer.push_back(gng_wasm_core::Point3f{xyz[base], xyz[base + 1], xyz[base + 2]});
  }
  kernel().update_points(buffer);
}

GNG_EXPORT void gng_wasm_set_point_labels(const std::uint8_t *label_data, std::uint32_t count) {
  auto &buf = labels_buffer();
  buf.clear();
  if (!label_data || count == 0) {
    kernel().set_point_labels(buf);
    return;
  }
  buf.assign(label_data, label_data + count);
  kernel().set_point_labels(buf);
}

GNG_EXPORT std::int32_t gng_wasm_run() {
  return kernel().run() ? 1 : 0;
}

GNG_EXPORT std::int32_t gng_wasm_exec(std::uint32_t steps) {
  return kernel().exec(steps) ? 1 : 0;
}

GNG_EXPORT void gng_wasm_update_graph() {
  auto &k = kernel();
  k.prune_isolated_nodes_public();
  k.update_normals_public();
  k.assign_fuzzy_labels_public();
  k.build_clusters_public();
}

GNG_EXPORT std::uint32_t gng_wasm_get_graph_json_size() {
  json_buffer() = kernel().to_json();
  return static_cast<std::uint32_t>(json_buffer().size() + 1);
}

GNG_EXPORT std::uint32_t gng_wasm_write_graph_json(char *dst, std::uint32_t capacity) {
  json_buffer() = kernel().to_json();
  if (!dst || capacity == 0) return 0;
  const std::size_t size = json_buffer().size();
  const std::size_t copy_size = size < (capacity - 1) ? size : (capacity - 1);
  std::memcpy(dst, json_buffer().data(), copy_size);
  dst[copy_size] = '\0';
  return static_cast<std::uint32_t>(copy_size);
}

GNG_EXPORT std::uint32_t gng_wasm_node_count() {
  return static_cast<std::uint32_t>(kernel().nodes().size());
}

GNG_EXPORT std::uint32_t gng_wasm_edge_count() {
  return static_cast<std::uint32_t>(kernel().edges().size());
}

GNG_EXPORT std::uint32_t gng_wasm_cluster_count() {
  return static_cast<std::uint32_t>(kernel().clusters().size());
}

GNG_EXPORT std::uint32_t gng_wasm_iteration() {
  return kernel().iteration();
}

}  // extern "C"
