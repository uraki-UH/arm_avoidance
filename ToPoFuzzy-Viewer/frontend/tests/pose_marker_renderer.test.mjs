import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';

test('候補PoseのZ軸・TF適用・空配列・TF欠落を描画ツリーで確認', async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    const temp_dir = await mkdtemp(resolve('tests/.pose-renderer-'));
    let root;
    try {
        const output = resolve(temp_dir, 'renderer.mjs');
        await build({
            entryPoints: ['src/features/visualization/MarkerArrayRenderer.tsx'],
            outfile: output, bundle: true, packages: 'external', platform: 'node',
            format: 'esm', jsx: 'automatic', logLevel: 'silent',
        });
        const { MarkerArrayRenderer } = await import(pathToFileURL(output).href);
        const scene = new THREE.Scene();
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        root.configure({
            gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            scene, frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 },
        });
        const marker = {
            id: 0, ns: 'pose_array', type: 'arrow', action: 0, frameId: 'table',
            pos: [0, 0, 0], quat: [0, 0, 0, 1], points: [[1, 2, 3], [1, 2, 2.92]],
            scale: [0.008, 0.016, 0.02], color: [0.15, 0.8, 1, 1],
        };
        const transforms = { table: { pos: [10, 0, 0], quat: [0, 0, 0, 1] } };
        async function render(markers, tf) {
            await act(async () => {
                root.render(React.createElement(MarkerArrayRenderer, {
                    tag: '/grasp_pose_cands', data: { source_type: 'pose_array', markers }, transforms: tf,
                }));
            });
            scene.updateMatrixWorld(true);
            const arrows = [];
            scene.traverse(object => { if (object instanceof THREE.ArrowHelper) arrows.push(object); });
            return arrows;
        }
        assert.equal((await render([marker], {})).length, 0);
        const arrows = await render([marker], transforms);
        assert.equal(arrows.length, 1);
        assert.deepEqual(arrows[0].getWorldPosition(new THREE.Vector3()).toArray(), [11, 2, 3]);
        const axis = new THREE.Vector3(0, 1, 0).applyQuaternion(
            arrows[0].getWorldQuaternion(new THREE.Quaternion()));
        assert.ok(axis.distanceTo(new THREE.Vector3(0, 0, -1)) < 1e-9);
        for (const color of [[0.9, 0.7, 0.1, 1], [0.2, 0.85, 0.25, 1], [0.55, 0.55, 0.55, 1]]) {
            const updated = await render([{ ...marker, color }], transforms);
            assert.equal(updated.length, 1);
            assert.equal(updated[0], arrows[0]);
            assert.equal(updated[0].line.material.color.getHexString(), new THREE.Color(...color.slice(0, 3)).getHexString());
        }
        assert.equal((await render([], transforms)).length, 0);
        assert.equal((await render([marker], transforms)).length, 1);
        assert.equal((await render([marker], {})).length, 0);
    } finally {
        if (root) await act(async () => root.unmount());
        await new Promise(resolve => setTimeout(resolve, 600));
        await rm(temp_dir, { recursive: true, force: true });
        delete globalThis.IS_REACT_ACT_ENVIRONMENT;
    }
});
