import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React, { useLayoutEffect } from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';

test('球リストの初回・個数・色・位置変更で描画前の直径を維持', async () => {
    // 実際のReact Three Fiber更新順序を使用、GPU描画のみ代替
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    const temporary_directory = await mkdtemp(resolve('node_modules/.marker-renderer-test-'));
    let root;
    try {
        const output_file = resolve(temporary_directory, 'renderer.mjs');
        await build({
            entryPoints: ['src/features/visualization/MarkerArrayRenderer.tsx'],
            outfile: output_file,
            bundle: true,
            packages: 'external',
            platform: 'node',
            format: 'esm',
            jsx: 'automatic',
            logLevel: 'silent',
        });
        const { MarkerArrayRenderer } = await import(pathToFileURL(output_file).href);
        const scene = new THREE.Scene();
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        const renderer = { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas };
        root = createRoot(canvas);
        root.configure({ gl: renderer, scene, frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 } });
        let num_checks = 0;
        let previous_mesh;
        let previous_line;
        let num_disposed = 0;
        function observe({ marker }) {
            // 子のlayout effect後・passive effect前の表示状態
            useLayoutEffect(() => {
                const meshes = [];
                scene.traverse((object) => { if (object.isInstancedMesh) meshes.push(object); });
                assert.equal(meshes.length, 1);
                const mesh = meshes[0];
                assert.equal(mesh.count, marker.points.length);
                const lines = [];
                scene.traverse((object) => { if (object.isLineSegments) lines.push(object); });
                assert.equal(lines.length, 1);
                const line = lines[0];
                assert.equal(line.geometry.drawRange.count, marker.points.length * 2);
                assert.equal(mesh.material.color.getHexString(), new THREE.Color(...marker.color.slice(0, 3)).getHexString());
                assert.equal(line.material.color.getHexString(), mesh.material.color.getHexString());
                if (previous_mesh) {
                    assert.equal(mesh.geometry, previous_mesh.geometry);
                    assert.equal(mesh.material, previous_mesh.material);
                    assert.equal(line.material, previous_line.material);
                    if (marker.points.length <= previous_mesh.instanceMatrix.count) {
                        assert.equal(mesh, previous_mesh);
                        assert.equal(line.geometry, previous_line.geometry);
                    }
                } else {
                    mesh.geometry.addEventListener('dispose', () => { num_disposed++; });
                    mesh.material.addEventListener('dispose', () => { num_disposed++; });
                    line.material.addEventListener('dispose', () => { num_disposed++; });
                }
                assert.equal(num_disposed, 0);
                previous_mesh = mesh;
                previous_line = line;
                for (let idx = 0; idx < mesh.count; ++idx) {
                    const matrix = new THREE.Matrix4();
                    mesh.getMatrixAt(idx, matrix);
                    const scale = new THREE.Vector3().setFromMatrixScale(matrix);
                    const position = new THREE.Vector3().setFromMatrixPosition(matrix);
                    for (const value of scale.toArray()) assert.ok(Math.abs(value - marker.scale[0]) < 1e-6);
                    assert.deepEqual(position.toArray(), marker.points[idx]);
                }
                num_checks++;
            }, [marker]);
            return React.createElement(MarkerArrayRenderer, {
                tag: '/curved_surface_clusters/markers', data: { markers: [marker, {
                    ...marker, ns: 'surface_edges', type: 'line_list',
                    points: marker.points.flatMap((p) => [p, p.map((v) => v + 1)]),
                }] }, transforms: {},
            });
        }
        let marker = {
            id: 1, ns: 'nonplane_components', type: 'sphere_list', action: 0,
            frameId: 'world', pos: [0, 0, 0], quat: [0, 0, 0, 1],
            scale: [0.012, 0.012, 0.012], color: [1, 0, 0, 1], points: [[1, 2, 3]],
        };
        const updates = [
            {},
            { points: [[1, 2, 3], [4, 5, 6]] },
            { points: [[1, 2, 3], [4, 5, 6], [7, 8, 9]] },
            { color: [0, 1, 0, 1] },
            { points: [[7, 8, 9]] },
            { points: [] },
            { points: [[2, 3, 4]], scale: [0.024, 0.024, 0.024] },
        ];
        for (const update of updates) {
            marker = { ...marker, ...update };
            await act(async () => { root.render(React.createElement(observe, { marker })); });
        }
        assert.equal(num_checks, updates.length);
        await act(async () => { root.unmount(); });
        root = undefined;
        await new Promise((resolve) => setTimeout(resolve, 600));
        assert.equal(num_disposed, 3);
    } finally {
        if (root) await act(async () => { root.unmount(); });
        await rm(temporary_directory, { recursive: true, force: true });
        delete globalThis.IS_REACT_ACT_ENVIRONMENT;
    }
});
