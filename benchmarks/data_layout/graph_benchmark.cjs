const fs = require('node:fs');
const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const { performance } = require('node:perf_hooks');

// 実デコーダーによる非整列パケットの反復処理と、保持中の配列容量の比較
function packet(seed) {
    const num_nodes = 20000, num_edges = 200000, num_clusters = 200;
    const tag = new TextEncoder().encode('/topological_map');
    const header_end = 36 + tag.length + 1;
    const buffer = new ArrayBuffer(header_end + num_nodes * 96 + num_edges * 2 + num_clusters * 280);
    const view = new DataView(buffer);
    view.setUint32(0, 0x31474d54, true); view.setUint16(4, 2, true);
    view.setUint32(8, tag.length, true); view.setUint32(12, 1, true);
    view.setUint32(20, num_nodes, true); view.setUint32(24, num_edges, true);
    view.setUint32(28, num_clusters, true); view.setUint32(32, buffer.byteLength - 36, true);
    new Uint8Array(buffer, 36, tag.length).set(tag); view.setUint8(36 + tag.length, 119);
    for (let idx = 0; idx < num_nodes; ++idx) {
        const base = header_end + idx * 96;
        view.setUint16(base, idx * 3, true);
        view.setFloat32(base + 24, idx % 100 * .01, true);
        view.setFloat32(base + 28, Math.floor(idx / 100) * .01, true);
        view.setFloat32(base + 32, 1, true);
        view.setUint32(base + 84, 12345678, true);
    }
    let offset = header_end + num_nodes * 96;
    for (let idx = 0; idx < num_edges; ++idx) view.setUint16(offset + idx * 2, (idx + seed) % num_nodes, true);
    offset += num_edges * 2;
    for (let idx = 0; idx < num_clusters; ++idx) {
        view.setUint32(offset, idx + 1, true); view.setFloat32(offset + 48, 1, true);
        view.setUint32(offset + 76, 100, true);
        for (let member = 0; member < 100; ++member) view.setUint16(offset + 80 + member * 2, idx * 100 + member, true);
        offset += 280;
    }
    assert.equal(offset, buffer.byteLength);
    return buffer;
}

const [module_path, seed_text, output_path] = process.argv.slice(2);
const { deserializeTopologicalMap, validate_graph } = require(module_path);
const buffer = packet(Number(seed_text));
for (let iter = 0; iter < 5; ++iter) deserializeTopologicalMap(buffer);
global.gc();
const before = process.memoryUsage();
let graph;
const begin = performance.now();
for (let iter = 0; iter < 30; ++iter) graph = deserializeTopologicalMap(buffer).graph;
const total_ms = performance.now() - begin;
global.gc();
const after = process.memoryUsage();
let checksum = 0;
for (const idx of graph.edges) checksum += idx;
for (const cluster of graph.clusters) {
    for (const id of cluster.nodeIds) checksum += id;
}
// スナップショット経由でも、描画前検査に通る入力表現の保持
const snapshot = structuredClone(graph);
validate_graph(snapshot);
assert.deepEqual(Array.from(snapshot.edges), Array.from(graph.edges));
// 格納幅だけの比較における、ノード属性・所属・メタデータ全体の一致条件
const { graph_json_data } = require(module_path);
const graph_json = graph_json_data ? graph_json_data(graph) : { ...graph,
    edges: Array.from(graph.edges), clusters: graph.clusters.map(cluster => ({ ...cluster, nodeIds: Array.from(cluster.nodeIds) })) };
const graph_hash = Number.parseInt(createHash('sha256').update(JSON.stringify(graph_json)).digest('hex').slice(0, 8), 16);
const heap_bytes = after.heapUsed - before.heapUsed;
const array_bytes = after.arrayBuffers - before.arrayBuffers;
fs.writeFileSync(output_path, JSON.stringify({ total_ms, decode_ms: total_ms / 30,
    heap_bytes, array_bytes, retained_bytes: heap_bytes + array_bytes, checksum, graph_hash }));
