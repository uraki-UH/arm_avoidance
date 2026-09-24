import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';

test('クラスタ一括描画の姿勢・色・選択・容量再利用・速度矢印・表示解除', async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    const directory = await mkdtemp(resolve('tests/.cluster-batch-'));
    let root;
    try {
        const outfile = resolve(directory, 'batch.mjs');
        await build({ stdin: { contents: `
            export * from './src/features/visualization/ClusterBatch';
            export * from './src/features/visualization/graphLayerSettings';
            export * from './src/features/visualization/arrows';
            export { LAYER_COLORS } from './src/types';
        `, resolveDir: process.cwd() }, outfile, bundle: true, packages: 'external',
        platform: 'node', format: 'esm', jsx: 'automatic', logLevel: 'silent' });
        const helpers = await import(pathToFileURL(outfile).href);
        const scene = new THREE.Scene();
        const canvas = { width: 200, height: 200, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        root.configure({ gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            scene, frameloop: 'never', size: { width: 200, height: 200, top: 0, left: 0 } });
        const make_cluster = id => ({ id, label: id % 6, semanticLabel: 0, pos: [id, id % 4, 2],
            quat: new THREE.Quaternion().setFromEuler(new THREE.Euler(0.1, 0.2, id * 0.03)).toArray(),
            scale: [0.2, 0.5, 0.8], velocity: [id % 3 === 0 ? 0 : 0.05 * (id % 7), 0, 0], nodeIds: [] });
        const props = { graph: { timestamp: 0, nodes: [], edges: [], clusters: [] }, source_id: '/test',
            active_labels: helpers.get_active_node_labels({}), node_opacity: 0.8,
            selected_cluster_id: null, enable_velocity: true };
        const render = () => act(async () => { root.render(React.createElement(helpers.ClusterBatch, props)); });
        const meshes = () => {
            const result = [];
            scene.traverse(object => { if (object.isInstancedMesh && object.userData.pick_clusters) result.push(object); });
            return result;
        };
        const near = (actual, expected) => actual.forEach((value, idx) =>
            assert.ok(Math.abs(value - expected[idx]) < 1e-4, `${idx}: ${value} != ${expected[idx]}`));
        for (const num of [0, 12, 1000, 23]) {
            const previous = meshes();
            props.graph = { ...props.graph, clusters: Array.from({ length: num }, (_, idx) => make_cluster(idx)) };
            await render();
            const current = meshes();
            assert.equal(current.length, 4);
            assert.equal(current.reduce((sum, mesh) => sum + mesh.count, 0), num);
            for (const mesh of current) {
                assert.equal(mesh.__r3f.eventCount, 0, '選択無効時にイベント登録なし');
                if (num === 23) assert.equal(mesh, previous[current.indexOf(mesh)], '縮小時に容量再利用');
                mesh.userData.pick_clusters.forEach((cluster, idx) => {
                    const parent = new THREE.Object3D(), child = new THREE.Object3D();
                    parent.position.fromArray(cluster.pos); parent.quaternion.fromArray(cluster.quat);
                    child.scale.fromArray(cluster.scale);
                    if (cluster.label === 4) {
                        child.rotation.x = Math.PI / 2;
                        child.scale.set(cluster.scale[0], cluster.scale[2], cluster.scale[1]);
                    }
                    parent.add(child); parent.updateMatrixWorld(true);
                    const matrix = new THREE.Matrix4(); mesh.getMatrixAt(idx, matrix);
                    near(matrix.elements, child.matrixWorld.elements);
                    const color = new THREE.Color(); mesh.getColorAt(idx, color);
                    near(color.toArray(), new THREE.Color(helpers.LAYER_COLORS[cluster.label]).toArray());
                });
            }
            const arrows = scene.getObjectByName('common-arrows');
            const expected = props.graph.clusters.flatMap(cluster => helpers.build_arrow_parts(
                { position: cluster.pos, direction: cluster.velocity },
                helpers.resolve_arrow_style(helpers.velocity_arrow_style(Math.hypot(...cluster.velocity)))));
            assert.equal(arrows.children[0].count, expected.length);
            for (const [idx, part] of expected.entries()) {
                for (const [mesh, value] of [[arrows.children[0], part.shaft], [arrows.children[1], part.head]]) {
                    const matrix = new THREE.Matrix4(); mesh.getMatrixAt(idx, matrix); near(matrix.elements, value.elements);
                }
            }
        }
        const selected = [];
        props.on_select = id => selected.push(id);
        props.selected_cluster_id = 4;
        await render();
        const selected_mesh = meshes().find(mesh => mesh.material.opacity === 0.1 && mesh.count > 0);
        assert.equal(selected_mesh.userData.pick_clusters[0].id, 4);
        const color = new THREE.Color(); selected_mesh.getColorAt(0, color);
        assert.equal(color.getHexString(), 'ffffff');
        const click = (mesh, delta = 0) => mesh.__r3f.handlers.onClick({ instanceId: 0, button: 0, delta, stopPropagation() {} });
        click(selected_mesh); assert.deepEqual(selected, [null]);
        click(selected_mesh, 9); assert.deepEqual(selected, [null]);
        const other = meshes().find(mesh => mesh.material.opacity !== 0.1 && mesh.count > 0);
        click(other); assert.equal(selected.at(-1), other.userData.pick_clusters[0].id);
        props.visible_labels = { 0: false, 1: false, 2: false, 3: false, 4: true, 5: false };
        props.enable_velocity = false;
        await render();
        assert.ok(meshes().flatMap(mesh => mesh.userData.pick_clusters).every(cluster => cluster.label === 4));
        assert.equal(scene.getObjectByName('common-arrows'), undefined);
        await act(async () => root.render(null));
        assert.equal(scene.getObjectByName('cluster-batch'), undefined);
    } finally {
        if (root) await act(async () => root.unmount());
        await rm(directory, { recursive: true, force: true });
        delete globalThis.IS_REACT_ACT_ENVIRONMENT;
    }
});
