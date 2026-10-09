import assert from 'node:assert/strict';
import { readFile, mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

test('同一URLのメッシュ改訂・候補間共有・旧失敗と新キャッシュの分離', async () => {
    const temporary_dir = await mkdtemp(resolve('node_modules/.robot-cache-test-'));
    try {
        // React描画から独立した、本番キャッシュ処理そのものの検証
        const source = await readFile('src/features/visualization/RobotRenderer.tsx', 'utf8');
        const output = resolve(temporary_dir, 'cache.mjs');
        await build({ stdin: { contents: "import * as THREE from 'three';\n" +
            source.slice(source.indexOf('type MeshLoadFunction'), source.indexOf('function RobotInstanceRenderer')) +
            '\nexport { loadCachedMesh };', loader: 'ts', resolveDir: process.cwd() },
            outfile: output, bundle: true, packages: 'external', platform: 'node', format: 'esm' });
        const { loadCachedMesh } = await import(pathToFileURL(output).href);
        const manager = new THREE.LoadingManager();
        let num_loads = 0;
        const load = (path, unused_manager, complete) => {
            num_loads++;
            complete(new THREE.Mesh(new THREE.BoxGeometry(num_loads, 1, 1), new THREE.MeshBasicMaterial()));
        };
        const get = (revision, loader = load) => new Promise((done, reject) =>
            loadCachedMesh('/meshes/cover.stl', revision, manager, loader,
                (object, error) => error ? reject(error) : done(object)));
        const [first, candidate] = await Promise.all([get('old'), get('old')]);
        assert.equal(num_loads, 1);
        assert.equal(first.geometry, candidate.geometry);
        assert.notEqual(first.material, candidate.material);
        const updated = await get('new');
        assert.equal(num_loads, 2);
        assert.notEqual(updated.geometry, first.geometry);
        assert.equal((await get('new')).geometry, updated.geometry);
        let fail_old;
        const delayed = get('delayed', (path, unused_manager, complete) => { fail_old = complete; });
        const expected_failure = assert.rejects(delayed, /old failure/);
        const latest = await get('latest');
        fail_old(new THREE.Object3D(), new Error('old failure'));
        await expected_failure;
        assert.equal((await get('latest')).geometry, latest.geometry);
        assert.equal(num_loads, 3);
    } finally {
        await rm(temporary_dir, { recursive: true, force: true });
    }
});

test('ROS Viewerと同じSTLLoader・片面材質で青色カバーの全周に欠損なし', async () => {
    const bytes = await readFile('../../urdf/topo_dual_arm_max_long/meshes/chest_lidar_color_0.stl');
    const geometry = new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    const material = new THREE.MeshPhongMaterial();
    const mesh = new THREE.Mesh(geometry, material);
    const ray = new THREE.Raycaster();
    let num_rays = 0;
    for (const z of [18, 24, 30]) for (let deg = 0.37; deg < 360; deg += 10) {
        const angle = deg * Math.PI / 180;
        ray.set(new THREE.Vector3(40 * Math.cos(angle), 40 * Math.sin(angle), z),
            new THREE.Vector3(-Math.cos(angle), -Math.sin(angle), 0));
        const hits = ray.intersectObject(mesh, false);
        assert.ok(hits.length, `欠損: z=${z}, angle=${deg}`);
        assert.ok(Math.abs(40 - hits[0].distance - Math.sqrt(22 ** 2 - (z - 13.48) ** 2)) < 0.08);
        num_rays++;
    }
    assert.equal(num_rays, 108);
    geometry.dispose();
    material.dispose();
});
