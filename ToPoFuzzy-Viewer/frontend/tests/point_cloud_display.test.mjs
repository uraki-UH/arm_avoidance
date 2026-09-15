import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';
import ts from 'typescript';

test('トピック別設定の分離・不透明度0・共通設定への復帰', async () => {
    const source = ts.createSourceFile('App.tsx', await readFile('src/App.tsx', 'utf8'),
        ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX);
    function find(node) {
        if (ts.isVariableDeclaration(node) && node.name.getText(source) === 'update_point_cloud_display') return node;
        return ts.forEachChild(node, find);
    }
    const declaration = find(source);
    assert.ok(declaration);
    const code = ts.transpileModule(`return (${declaration.initializer.getText(source)});`, {
        compilerOptions: { target: ts.ScriptTarget.ES2022 },
    }).outputText;
    let settings = {};
    const update = new Function('set_point_cloud_display', code)(callback => { settings = callback(settings); });
    const first = { mode: 'simple', pointSize: 0.008, simpleColor: '#ff0000', opacity: 0 };
    const second = { mode: 'height', pointSize: 0.03, min: -1, max: 2, opacity: 0.5 };
    update('/points_a', first);
    update('/points_b', second);
    assert.equal(settings['/points_a'], first);
    assert.equal(settings['/points_b'], second);
    const previous = settings;
    update('/points_a', null);
    assert.equal(settings['/points_a'], undefined);
    assert.equal(settings['/points_b'], second);
    assert.equal(previous['/points_a'], first);
    // ストリーム同期から独立した設定と、実Rendererへの接続。
    assert.match(source.text, /heatmapSettings=\{point_cloud_display\[pc.id\] \?\? heatmapSettings\}/);
    assert.match(source.text, /opacity=\{point_cloud_display\[pc.id\]\?\.opacity\}/);
    assert.match(source.text, /update_point_cloud_display\(id, null\)/);
});

test('2点群の色・サイズ・透明度を独立反映、表示変更で頂点バッファ再転送なし', async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    const directory = await mkdtemp(resolve('tests/.point-cloud-display-'));
    let root;
    try {
        const output = resolve(directory, 'renderer.mjs');
        await build({ entryPoints: ['src/features/visualization/PointCloudRenderer.tsx'], outfile: output,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', jsx: 'automatic', logLevel: 'silent' });
        const { PointCloudRenderer } = await import(pathToFileURL(output).href);
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        const scene = new THREE.Scene();
        root.configure({ gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            scene, frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 } });
        const clouds = ['a', 'b'].map(id => ({ id, name: id, count: 1,
            points: new Float32Array([1, 2, 3]), opacity: 1 }));
        let common = { mode: 'simple', min: 0, max: 3, colorScheme: 'viridis',
            simpleColor: '#00ff00', pointSize: 0.02 };
        let overrides = { ...common, simpleColor: '#ff0000', pointSize: 0.008, opacity: 0 };
        const render = async () => act(async () => root.render(React.createElement(React.Fragment, null,
            ...clouds.map((cloud, idx) => React.createElement(PointCloudRenderer, {
                key: cloud.id, data: cloud, heatmapSettings: idx === 0 ? overrides ?? common : common,
                opacity: idx === 0 ? overrides?.opacity : undefined,
            })))));
        await render();
        const meshes = [];
        scene.traverse(object => { if (object.isPoints) meshes.push(object); });
        assert.equal(meshes.length, 2);
        assert.equal(meshes[0].material.size, 0.008);
        assert.equal(meshes[0].material.color.getHexString(), 'ff0000');
        assert.equal(meshes[0].material.opacity, 0);
        assert.equal(meshes[1].material.size, 0.02);
        assert.equal(meshes[1].material.color.getHexString(), '00ff00');
        assert.equal(meshes[1].material.opacity, 1);
        common = { ...common, pointSize: 0.04, simpleColor: '#0000ff' };
        await render();
        assert.equal(meshes[0].material.size, 0.008);
        assert.equal(meshes[0].material.opacity, 0);
        assert.equal(meshes[0].material.color.getHexString(), 'ff0000');
        assert.equal(meshes[1].material.size, 0.04);
        assert.equal(meshes[1].material.color.getHexString(), '0000ff');
        const geometry = meshes[0].geometry;
        const position = geometry.getAttribute('position');
        const version = position.version;
        overrides = { ...overrides, mode: 'height', min: -2, max: 4, opacity: 0.25 };
        await render();
        assert.equal(meshes[0].material.uniforms.uOpacity.value, 0.25);
        assert.equal(meshes[0].material.uniforms.uMin.value, -2);
        assert.equal(meshes[0].material.uniforms.uMax.value, 4);
        assert.equal(meshes[0].geometry, geometry);
        assert.equal(position.version, version);
        assert.equal(meshes[1].material.opacity, 1);
        overrides = null;
        await render();
        assert.equal(meshes[0].material.size, 0.04);
        assert.equal(meshes[0].material.opacity, 1);
        assert.equal(meshes[0].material.color.getHexString(), '0000ff');
    } finally {
        if (root) await act(async () => root.unmount());
        await rm(directory, { recursive: true, force: true });
    }
});
