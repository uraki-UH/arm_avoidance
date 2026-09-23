import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';

function make_packet(version, tag = '/topological_map') {
    const tag_bytes = new TextEncoder().encode(tag);
    const node_size = version === 1 ? 84 : 96;
    const buffer = new ArrayBuffer(36 + tag_bytes.length + 4 * node_size + 2 * (80 + 2));
    const view = new DataView(buffer);
    view.setUint32(0, 0x31474d54, true);
    view.setUint16(4, version, true);
    view.setUint32(8, tag_bytes.length, true);
    view.setUint32(20, 4, true);
    view.setUint32(28, 2, true);
    view.setUint32(32, buffer.byteLength - 36, true);
    new Uint8Array(buffer, 36, tag_bytes.length).set(tag_bytes);
    let offset = 36 + tag_bytes.length;
    // IDと添字が重なる並びでの、別ノードへの誤対応防止。
    [2, 99, 0, 7].forEach((id, idx) => {
        view.setUint16(offset, id, true);
        view.setUint8(offset + 2, 3);
        view.setFloat32(offset + 24, idx, true);
        offset += node_size;
    });
    for (const [id, label, member] of [[20, 4, 0], [30, 5, 2]]) {
        view.setUint32(offset, id, true);
        view.setUint8(offset + 4, label);
        view.setFloat32(offset + 48, 1, true);
        for (const pos of [24, 28, 32]) view.setFloat32(offset + pos, 1, true);
        view.setUint32(offset + 76, 1, true);
        view.setUint16(offset + 80, member, true);
        offset += 82;
    }
    return buffer;
}

test('人・車クラスタの受信所属・描画色・更新・表示切替・既存属性の保持', async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    const directory = await mkdtemp(resolve('tests/.human-car-display-'));
    let root;
    try {
        const outfile = resolve(directory, 'display.mjs');
        await build({ stdin: { contents: `
            export * from './src/features/visualization/GraphRenderer';
            export * from './src/features/visualization/GngLayerControls';
            export * from './src/features/visualization/graphLayerSettings';
            export * from './src/utils/topologicalMapProtocol';
        `, resolveDir: resolve('.') }, outfile, bundle: true, packages: 'external',
        platform: 'node', format: 'esm', jsx: 'automatic' });
        const helpers = await import(pathToFileURL(outfile).href);
        for (const version of [1, 2]) {
            const graph = helpers.deserializeTopologicalMap(make_packet(version)).graph;
            assert.deepEqual(graph.clusters.map(cluster => cluster.nodeIds), [[2], [0]]);
            const other = helpers.deserializeTopologicalMap(make_packet(version, '/nonplane_components')).graph;
            assert.deepEqual(other.clusters.map(cluster => cluster.nodeIds), [[0], [2]]);
        }
        const raw = { nodes: [{ id: 12 }], clusters: [{ nodeIds: [0, 999, -1, 0.5] }] };
        const original_raw = structuredClone(raw);
        assert.deepEqual(helpers.normalize_environment_cluster_ids('/topological_map', raw).clusters[0].nodeIds, [12]);
        assert.deepEqual(raw, original_raw);
        assert.equal(helpers.normalize_environment_cluster_ids('/template/Tmap', raw), raw);

        const graph = helpers.deserializeTopologicalMap(make_packet(2)).graph;
        const original = structuredClone(graph);
        const settings = { ...helpers.createDefaultGraphLayerSettings('/topological_map', graph),
            showEdges: false, node_label_visibility: { boundary: false, grasp_labels: false } };
        assert.equal(settings.showClusters, false);
        const scene = new THREE.Scene();
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        const renderer = { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas };
        root = createRoot(canvas);
        root.configure({ gl: renderer, scene, frameloop: 'never',
            size: { width: 100, height: 100, top: 0, left: 0 } });
        const render = async (data = graph, display = settings, extra = {}) => {
            await act(async () => { root.render(React.createElement(helpers.GraphRenderer, {
                tag: '/topological_map', data, settings: display, ...extra,
            })); });
            const colors = new Map();
            const color = new THREE.Color();
            scene.traverse(object => {
                if (!object.isInstancedMesh || !object.userData.pick_nodes) return;
                for (let idx = 0; idx < object.count; idx++) {
                    object.getColorAt(idx, color);
                    colors.set(object.userData.pick_nodes[idx].id, color.getHexString());
                    assert.equal(object.userData.pick_nodes[idx].label, 3, '受信ノードの幾何ラベル保持');
                }
            });
            return colors;
        };
        assert.deepEqual(await render(), new Map([[2, 'd946ef'], [0, '8b5cf6'], [99, 'ffff00'], [7, 'ffff00']]));
        assert.deepEqual(graph, original);
        const hidden = { ...settings, visibleLabels: { ...settings.visibleLabels, 4: false } };
        assert.equal((await render(graph, hidden)).has(2), false);
        assert.equal((await render(graph, hidden)).get(0), '8b5cf6');
        // 座標・ノード配列・timestampが同一でも、クラスタ更新だけで通常色へ復帰。
        const expired = { ...graph, clusters: [] };
        assert.equal((await render(expired)).get(2), 'ffff00');
        const switched = { ...graph, clusters: graph.clusters.map(cluster => ({ ...cluster, label: 5 })) };
        assert.equal((await render(switched)).get(2), '8b5cf6');
        assert.equal((await render(graph, settings, { uniform_node_color: '#123456' })).get(2), '123456');
        const semantic = { ...graph, nodes: graph.nodes.map(node => ({ ...node, semanticLabel: 1 })) };
        assert.equal((await render(semantic, { ...settings, node_label_visibility: { boundary: false } })).get(2), '00d1ff');

        // 実UIのClusters操作による設定更新と、円柱・ボックスの描画切替。
        let updated;
        const controls = helpers.GngLayerControls({ tag: '/topological_map', graphData: graph, settings,
            onUpdate: value => { updated = value; }, onRemove() {} });
        function find_toggle(element) {
            if (!React.isValidElement(element)) return undefined;
            if (element.props.label === 'Clusters') return element;
            return React.Children.toArray(element.props.children).map(find_toggle).find(Boolean);
        }
        const toggle = find_toggle(controls);
        assert.ok(toggle, 'Clusters表示切替');
        toggle.props.onToggle();
        assert.deepEqual(updated, { showClusters: true });
        await render(graph, { ...settings, ...updated });
        const shapes = [];
        scene.traverse(object => {
            if (object.userData.inspection_selection?.kind === 'cluster') {
                shapes.push([object.geometry.type, object.material.color.getHexString()]);
            }
        });
        assert.deepEqual(shapes, [['CylinderGeometry', 'd946ef'], ['BoxGeometry', '8b5cf6']]);
        await render(graph, settings);
        scene.traverse(object => assert.notEqual(object.userData.inspection_selection?.kind, 'cluster'));
    } finally {
        if (root) await act(async () => { root.unmount(); });
        await rm(directory, { recursive: true, force: true });
    }
});
