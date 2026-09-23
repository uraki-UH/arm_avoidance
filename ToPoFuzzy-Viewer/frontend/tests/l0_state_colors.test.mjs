import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import * as THREE from 'three';

test('L0状態割合の描画・件数更新・既存ラベルと旧バイナリの保持', async () => {
    const directory = await mkdtemp(resolve('tests/.l0-state-test-'));
    const geometry = new THREE.SphereGeometry(1, 4, 3);
    const material = new THREE.MeshStandardMaterial();
    const mesh = new THREE.InstancedMesh(geometry, material, 1);
    try {
        const outfile = resolve(directory, 'helpers.mjs');
        await build({ stdin: { contents: `
            export * from './src/features/visualization/gngGraphics';
            export * from './src/utils/topologicalMapProtocol';
        `, resolveDir: resolve('.') }, outfile, bundle: true, packages: 'external', platform: 'node', format: 'esm' });
        const module = await import(pathToFileURL(outfile));
        const palette = ['#808080', '#00ff00', '#ff0000', '#ffff00'];
        const node = { x: 0, y: 0, z: 0, label: 1, num_safe_states: 1, num_danger_states: 0, num_collision_states: 99 };
        const color = new THREE.Color(module.resolveGraphNodeColor(node, palette));
        assert.ok(color.r > 0.98 && color.g < 0.02);
        assert.equal(node.label, 1);
        assert.equal(module.get_node_state_counts(node).num_total, 100);
        for (const [safe, danger, collision, expected] of [[100, 0, 0, '#00ff00'], [0, 100, 0, '#ffff00'], [0, 0, 100, '#ff0000']]) {
            assert.equal(module.resolve_state_ratio_color({ ...node, num_safe_states: safe, num_danger_states: danger,
                num_collision_states: collision }, palette), expected);
        }
        const matrix = mesh.instanceMatrix;
        module.updateNodeInstances(mesh, [node], 0.01, { palette });
        mesh.getColorAt(0, color);
        assert.ok(color.r > 0.98 && color.g < 0.02);
        module.updateNodeInstances(mesh, [{ ...node, num_safe_states: 99, num_collision_states: 1 }], 0.01, { palette });
        mesh.getColorAt(0, color);
        assert.ok(color.g > 0.98 && color.r < 0.02);
        assert.equal(mesh.instanceMatrix, matrix);
        module.updateNodeInstances(mesh, [node], 0.01, { colorMode: 'uniform', uniformColor: '#a855f7' });
        mesh.getColorAt(0, color);
        assert.equal(color.getHexString(), 'a855f7');
        for (const value of [undefined, -1, NaN, Infinity, 0.5]) {
            const invalid = { ...node, num_safe_states: value };
            assert.equal(module.get_node_state_counts(invalid), undefined);
            assert.equal(module.resolveGraphNodeColor(invalid, palette), '#00ff00');
        }
        assert.equal(module.resolve_state_ratio_color({ ...node, num_safe_states: 0, num_collision_states: 0 }), undefined);

        // バイナリv1は従来属性、v2は表示用件数を追加。元labelは両方で保持
        for (const version of [1, 2]) {
            const size = version === 1 ? 84 : 96;
            const buffer = new ArrayBuffer(36 + size);
            const view = new DataView(buffer);
            view.setUint32(0, 0x31474d54, true); view.setUint16(4, version, true);
            view.setUint32(20, 1, true); view.setUint32(32, size, true);
            view.setUint8(36 + 2, 1);
            if (version === 2) {
                view.setUint32(36 + 84, 1, true);
                view.setUint32(36 + 88, 4, true);
                view.setUint32(36 + 92, 95, true);
            }
            const decoded = module.deserializeTopologicalMap(buffer).graph.nodes[0];
            assert.equal(decoded.label, 1);
            assert.equal(decoded.num_safe_states, version === 2 ? 1 : undefined);
            assert.equal(decoded.num_danger_states, version === 2 ? 4 : undefined);
            assert.equal(decoded.num_collision_states, version === 2 ? 95 : undefined);
            assert.throws(() => module.deserializeTopologicalMap(buffer.slice(0, -1)));
        }
    } finally {
        mesh.dispose(); geometry.dispose(); material.dispose();
        await rm(directory, { recursive: true, force: true });
    }
});
