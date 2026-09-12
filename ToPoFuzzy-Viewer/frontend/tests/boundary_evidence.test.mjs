import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { build } from 'esbuild';

const temporary_directory = await mkdtemp(resolve('tests/.boundary-evidence-'));
try {
    const output = resolve(temporary_directory, 'test.mjs');
    await build({ stdin: { contents: "export * from './src/utils/topologicalMapProtocol'; export * from './src/features/visualization/graphLayerSettings';",
        resolveDir: process.cwd() }, outfile: output, bundle: true, platform: 'node', format: 'esm' });
    const module = await import(pathToFileURL(output).href);
    const buffer = new ArrayBuffer(36+84);
    const view = new DataView(buffer);
    view.setUint32(0, 0x31474d54, true); view.setUint16(4, 1, true);
    view.setUint32(20, 1, true); view.setUint32(32, 84, true);
    view.setUint8(36+5, 1);
    for (let flags = 0; flags < 8; ++flags) {
        view.setUint8(36+6, flags);
        const node = module.deserializeTopologicalMap(buffer).graph.nodes[0];
        assert.equal(node.boundary_evidence, flags);
        for (const [id, bit] of [['boundary_occlusion', 1], ['boundary_free_space', 2], ['boundary_fov', 4]]) {
            const definition = module.node_label_definitions.find((item) => item.id === id);
            assert.equal(definition.is_match(node), (flags & bit) !== 0);
            assert.equal(definition.is_match({ ...node, is_boundary_candidate: false }), false);
            const active = module.get_active_node_labels({ node_label_priority: [id] });
            if (flags & bit) assert.equal(module.resolve_node_label(node, active).id, id);
        }
        const unknown = module.node_label_definitions.find((item) => item.id === 'boundary_unknown');
        assert.equal(unknown.is_match(node), flags === 0);
    }
    assert.equal(module.node_label_definitions.find((item) => item.id === 'boundary_unknown')
        .is_match({ is_boundary_candidate: true }), true);
    const settings = module.normalize_node_label_settings();
    assert.deepEqual(module.get_node_label_groups(settings.node_label_priority).map((item) => item.id), ['boundary', 'handle']);
    const node = { is_boundary_candidate: true, boundary_evidence: 7, semanticLabel: 1 };
    const select = (options, value = node) => module.resolve_node_label(value, module.get_active_node_labels(options));
    assert.equal(select({}).id, 'boundary_fov');
    assert.equal(select({ node_label_visibility: { boundary: false } }).id, 'handle');
    assert.equal(select({ node_label_visibility: { boundary: false, handle: false } }), undefined);
    assert.equal(select({ node_label_visibility: { boundary_fov: false, boundary_occlusion: false } }).id, 'boundary_free_space');
    assert.equal(select({}, { is_boundary_candidate: true, boundary_evidence: 2 }).color, '#2196f3');
    assert.equal(select({ node_label_visibility: {
        boundary_fov: false, boundary_occlusion: false, boundary_free_space: false, handle: false,
    } }), undefined);
    assert.equal(select({}, { is_boundary_candidate: true }).id, 'boundary_unknown');
    assert.equal(select({ node_label_visibility: { boundary_unknown: false } }, { is_boundary_candidate: true }), undefined);
    const reordered = module.move_node_label_group(settings.node_label_priority, 'handle', -1);
    assert.equal(select({ node_label_priority: reordered }).id, 'handle');
    assert.deepEqual(reordered.filter((id) => id.startsWith('boundary_')),
        settings.node_label_priority.filter((id) => id.startsWith('boundary_')));
    assert.equal(select({ node_label_colors: { boundary_fov: '#123456' } }).color, '#123456');
    assert.equal(select({ node_label_colors: { boundary_fov: 'invalid' } }).color, '#b388ff');
    assert.equal(select({ enable_boundary_highlight: false, visibleSemanticLabels: { handle: false } }), undefined);
    assert.equal(select({ overlap_label_priority: 'handle' }).id, 'handle');
    assert.deepEqual(module.insert_node_label(['a', 'b', 'c', 'd'], 'd', 'b'), ['a', 'd', 'b', 'c']);
    assert.deepEqual(module.insert_node_label(['a', 'b', 'c'], 'a', null), ['b', 'c', 'a']);
    assert.deepEqual(module.insert_node_label(['a', 'b'], 'a', 'a'), ['a', 'b']);
    assert.deepEqual(module.insert_node_label(['a', 'b'], 'x', 'b'), ['a', 'b']);
    assert.deepEqual(module.insert_node_label(['a', 'b'], 'a', 'x'), ['a', 'b']);
    const child_order = ['boundary_free_space', 'boundary_unknown', 'boundary_fov', 'boundary_occlusion'];
    const priority = module.reorder_node_label_subset(reordered, child_order);
    assert.deepEqual(module.get_node_label_groups(priority).map((item) => item.id), ['handle', 'boundary']);
    assert.deepEqual(priority.filter((id) => id.startsWith('boundary_')), child_order);
    const overlap = (id, target) => module.insert_node_label(['a', 'b', 'c', 'd'], id,
        module.get_node_label_overlap_target(['a', 'b', 'c', 'd'], id, target));
    assert.deepEqual(overlap('a', 'b'), ['b', 'a', 'c', 'd']);
    assert.deepEqual(overlap('a', 'c'), ['b', 'c', 'a', 'd']);
    assert.deepEqual(overlap('a', 'd'), ['b', 'c', 'd', 'a']);
    assert.deepEqual(overlap('d', 'b'), ['a', 'd', 'b', 'c']);
    assert.deepEqual(overlap('b', 'a'), ['b', 'a', 'c', 'd']);
    assert.deepEqual(overlap('b', 'b'), ['a', 'b', 'c', 'd']);
    assert.deepEqual(overlap('b', 'unknown'), ['a', 'b', 'c', 'd']);
    console.log('boundary_evidence_protocol_labels=passed');
} finally {
    await rm(temporary_directory, { recursive: true, force: true });
}
