const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

async function main() {
  const workspace_dir = path.resolve(__dirname, '../..');
  const html = fs.readFileSync(path.join(workspace_dir, 'ToPo-FUZZY_Manipulation_v1.html'), 'utf8');
  const match = html.match(/script\.src='([^']*gng_wasm_core\.js[^']*)'/);
  assert.ok(match, 'HTMLのWASM読み込み先');
  const bundled_path = match[1].split('?')[0];
  assert.equal(bundled_path, 'gng_web_tools/wasm/dist/gng_wasm_core.js');
  assert.ok(fs.existsSync(path.join(workspace_dir, bundled_path)));
  const module_path = process.argv[2] ? path.resolve(process.argv[2]) :
    path.join(workspace_dir, bundled_path);
  const create_module = require(module_path);
  const module = await create_module();
  assert.equal(module.ccall('gng_wasm_abi_version', 'number', [], []), 3);
  assert.equal(typeof module.cwrap, 'function');
  module.ccall('gng_wasm_reset', null, [], []);
  for (const [name, value] of [
    ['node.num_max', 64], ['edge.age_max', 80],
    ['node.eta_s1', 0.055], ['node.eta_s2', 0.006],
  ]) {
    assert.equal(module.ccall('gng_wasm_set_parameter', 'number',
      ['string', 'number', 'number'], [name, 0, value]), 1);
  }

  // 実体化したWASMへの点群入力と有限回数の学習。
  const points = new Float32Array([
    -0.1, -0.1, 0, -0.1, 0, 0, -0.1, 0.1, 0,
    0, -0.1, 0, 0, 0, 0.05, 0, 0.1, 0,
    0.1, -0.1, 0, 0.1, 0, 0, 0.1, 0.1, 0,
  ]);
  const points_ptr = module._malloc(points.byteLength);
  assert.notEqual(points_ptr, 0);
  try {
    new Float32Array(module.HEAPU8.buffer, points_ptr, points.length).set(points);
    module.ccall('gng_wasm_set_points', null, ['number', 'number'],
      [points_ptr, points.length / 3]);
  } finally {
    module._free(points_ptr);
  }
  assert.equal(module.ccall('gng_wasm_exec', 'number', ['number'], [500]), 1);
  assert.equal(module.ccall('gng_wasm_iteration', 'number', [], []), 500);
  const num_nodes = module.ccall('gng_wasm_node_count', 'number', [], []);
  const num_edges = module.ccall('gng_wasm_edge_count', 'number', [], []);
  assert.ok(num_nodes >= 2 && num_nodes <= 64);
  assert.ok(num_edges > 0);
  const json_size = module.ccall('gng_wasm_get_graph_json_size', 'number', [], []);
  const json_ptr = module._malloc(json_size);
  assert.notEqual(json_ptr, 0);
  try {
    assert.equal(module.ccall('gng_wasm_write_graph_json', 'number',
      ['number', 'number'], [json_ptr, json_size]), json_size - 1);
    const graph = JSON.parse(module.UTF8ToString(json_ptr));
    assert.equal(graph.nodes.length, num_nodes);
    assert.equal(graph.iterations, 500);
  } finally {
    module._free(json_ptr);
    module.ccall('gng_wasm_reset', null, [], []);
  }
  console.log(`WASM load OK: nodes=${num_nodes} edges=${num_edges} iterations=500`);
}

main().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
