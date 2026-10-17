#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-${root_dir}/dist}"
gng_cpu_dir="${root_dir}/../../src/gng_cpu"

if ! command -v em++ >/dev/null 2>&1; then
  echo "em++ not found. Install Emscripten first." >&2
  exit 1
fi

mkdir -p "${output_dir}"

em++ \
  -std=c++20 \
  -fexceptions \
  -O3 \
  -I"${root_dir}/include" \
  -I"${gng_cpu_dir}/include" \
  -I"${gng_cpu_dir}/src/cpu" \
  -I"${gng_cpu_dir}/src/utils" \
  "${root_dir}/src/gng_kernel.cpp" \
  "${root_dir}/src/wasm_exports.cpp" \
  "${gng_cpu_dir}/src/cpu/cugng.cpp" \
  "${gng_cpu_dir}/src/cpu/voxel_grid.cpp" \
  "${gng_cpu_dir}/src/utils/node.cpp" \
  "${gng_cpu_dir}/src/utils/param.cpp" \
  "${gng_cpu_dir}/src/utils/vec3f.cpp" \
  -DGNG_VERSION=0 \
  -sMODULARIZE=1 \
  -sEXPORT_ES6=0 \
  -sEXPORT_NAME='GngWasmCore' \
  -sSINGLE_FILE=1 \
  -sENVIRONMENT=web,worker,node \
  -sALLOW_MEMORY_GROWTH=1 \
  -sNO_EXIT_RUNTIME=1 \
  -sEXPORTED_FUNCTIONS='["_gng_wasm_create","_gng_wasm_destroy","_gng_wasm_reset","_gng_wasm_set_points","_gng_wasm_update_points","_gng_wasm_set_point_labels","_gng_wasm_set_config","_gng_wasm_set_parameter","_gng_wasm_run","_gng_wasm_exec","_gng_wasm_update_graph","_gng_wasm_get_graph_json_size","_gng_wasm_write_graph_json","_gng_wasm_node_count","_gng_wasm_edge_count","_gng_wasm_cluster_count","_gng_wasm_iteration","_malloc","_free"]' \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","stringToUTF8","lengthBytesUTF8","UTF8ToString","HEAPU8","stackSave","stackAlloc","stackRestore"]' \
  -o "${output_dir}/gng_wasm_core.js"

echo "WASM生成先: ${output_dir}"
