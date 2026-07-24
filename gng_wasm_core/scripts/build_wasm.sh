#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-${ROOT_DIR}/dist}"

if ! command -v em++ >/dev/null 2>&1; then
  echo "em++ not found. Install Emscripten first." >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

em++ \
  -std=c++17 \
  -O3 \
  -I"${ROOT_DIR}/include" \
  "${ROOT_DIR}/src/gng_kernel.cpp" \
  "${ROOT_DIR}/src/wasm_exports.cpp" \
  -sMODULARIZE=1 \
  -sEXPORT_ES6=0 \
  -sEXPORT_NAME='GngWasmCore' \
  -sSINGLE_FILE=1 \
  -sENVIRONMENT=web,worker,node \
  -sALLOW_MEMORY_GROWTH=1 \
  -sNO_EXIT_RUNTIME=1 \
  -sEXPORTED_FUNCTIONS='["_gng_wasm_create","_gng_wasm_destroy","_gng_wasm_reset","_gng_wasm_set_points","_gng_wasm_set_point_labels","_gng_wasm_set_config","_gng_wasm_set_parameter","_gng_wasm_run","_gng_wasm_exec","_gng_wasm_update_graph","_gng_wasm_get_graph_json_size","_gng_wasm_write_graph_json","_gng_wasm_node_count","_gng_wasm_edge_count","_gng_wasm_cluster_count","_gng_wasm_iteration","_malloc","_free"]' \
  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","stringToUTF8","lengthBytesUTF8","UTF8ToString","HEAPU8","stackSave","stackAlloc","stackRestore"]' \
  -o "${OUT_DIR}/gng_wasm_core.js"

echo "WASM artifacts were generated under: ${OUT_DIR}"
