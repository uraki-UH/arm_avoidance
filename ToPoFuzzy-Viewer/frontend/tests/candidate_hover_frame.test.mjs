import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';

for (const source_id of ['/grasp_pose_cands/Tmap', '/nonplane_components']) test(
    `トピック別フラグとAABB判定・遅延応答の無効化: ${source_id}`, async () => {
    const saved_globals = new Map(['window', 'document', 'performance', 'IS_REACT_ACT_ENVIRONMENT']
        .map(name => [name, Object.getOwnPropertyDescriptor(globalThis, name)]));
    const temporary_directory = await mkdtemp(resolve('tests/.hover-frame-test-'));
    let root;
    let now_ms = 1000;
    let timer;
    const pending = [];
    const requests = [];
    const selections = [];
    const geometry = new THREE.SphereGeometry(0.06, 12, 8);
    const material = new THREE.MeshBasicMaterial();
    try {
        const output_file = resolve(temporary_directory, 'hover.mjs');
        await build({
            stdin: { contents: "export * from './src/features/visualization/CandidateHoverFrame'; export * from './src/features/visualization/graphLayerSettings';",
                resolveDir: process.cwd(), loader: 'tsx' },
            outfile: output_file, bundle: true, packages: 'external', platform: 'node',
            format: 'esm', jsx: 'automatic', logLevel: 'silent',
        });
        const { CandidateHoverFrame, createDefaultGraphLayerSettings } = await import(pathToFileURL(output_file).href);
        assert.equal(createDefaultGraphLayerSettings(source_id, { mode: 'dynamic' }).enable_bounding_box, false);
        const canvas = new EventTarget();
        canvas.width = canvas.height = 200;
        canvas.style = { cursor: 'crosshair' };
        canvas.getBoundingClientRect = () => ({ left: 0, top: 0, width: 200, height: 200 });
        const window_stub = new EventTarget();
        window_stub.setInterval = callback => { timer = callback; return 1; };
        window_stub.clearInterval = () => { timer = undefined; };
        let top_element = canvas;
        Object.defineProperties(globalThis, {
            window: { configurable: true, value: window_stub },
            document: { configurable: true, value: { elementFromPoint: () => top_element } },
            performance: { configurable: true, value: { now: () => now_ms } },
            IS_REACT_ACT_ENVIRONMENT: { configurable: true, value: true },
        });
        extend(THREE);
        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100);
        camera.position.set(0, 0, 5);
        camera.updateMatrixWorld(true);
        const mesh = new THREE.InstancedMesh(geometry, material, 2);
        mesh.setMatrixAt(0, new THREE.Matrix4());
        mesh.setMatrixAt(1, new THREE.Matrix4().makeTranslation(0.4, 0, 0));
        mesh.userData = { inspection_source: source_id, inspection_revision: {}, pick_nodes: [{ id: 11 }, { id: 12 }] };
        mesh.raycast = () => { throw new Error('ノードへのraycastは禁止'); };
        const other = new THREE.Mesh(geometry, material);
        other.position.set(1.4, 0, 0);
        other.userData = { inspection_source: source_id, inspection_revision: {}, inspection_selection: { kind: 'cluster', id: 8 } };
        other.raycast = () => { throw new Error('球へのraycastは禁止'); };
        // 名前によらず、フラグ未指定のトピックは範囲取得・当たり判定の対象外
        for (const source of ['/ToPoDualArm/Tmap_static', '/nonplane_components', '/nonplane_components/Tmap',
            '/topological_map', '/normals', '/markers', '/grasp_pose_cands', '/grasp_pose_cands_other/Tmap', '/other/grasp_pose_cands/Tmap']) {
            if (source === source_id) continue;
            const excluded = new THREE.Mesh(geometry, material);
            excluded.userData = { inspection_source: source, inspection_revision: {} };
            scene.add(excluded);
        }
        scene.add(mesh, other);
        const renderer = { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas };
        root = createRoot(canvas);
        root.configure({ gl: renderer, scene, camera, frameloop: 'never', size: { width: 200, height: 200, top: 0, left: 0 } });
        const props = {
            is_enabled: true, transforms: {}, layer_settings: {},
            on_inspect: (source, selection) => { selections.push({ source, selection }); },
            get_bounds: source => {
                assert.equal(source, source_id);
                requests.push({ source });
                return new Promise((resolve, reject) => { pending.push({ resolve, reject }); });
            },
        };
        const bounds = { source_id, frame_id: 'world', selection: { kind: 'cluster', id: 7 },
            min_position: [-0.1, -0.2, -0.1], max_position: [0.5, 0.2, 0.1] };
        const other_bounds = { ...bounds, selection: { ...bounds.selection, id: 8 },
            min_position: [1.3, -0.2, -0.1], max_position: [1.5, 0.2, 0.1] };
        const frame = () => scene.getObjectByName('candidate-hover-frame');
        const emit = (type, values = {}) => canvas.dispatchEvent(Object.assign(new Event(type), values));
        const move = (x, y = 0) => {
            const projected = new THREE.Vector3(x, y, 0).project(camera);
            emit('pointermove', { clientX: (projected.x + 1) * 100, clientY: (1 - projected.y) * 100,
                buttons: 0, isPrimary: true });
        };
        const tick = async (ms = 100) => {
            now_ms += ms;
            scene.updateMatrixWorld(true);
            await act(async () => { timer?.(); });
            scene.updateMatrixWorld(true);
        };
        const respond = async (result = [bounds, other_bounds]) => {
            assert.ok(pending.length);
            await act(async () => { pending.shift().resolve(result); });
            scene.updateMatrixWorld(true);
            await tick();
        };
        await act(async () => { root.render(React.createElement(CandidateHoverFrame, props)); });
        move(0.2);
        await tick();
        assert.equal(requests.length, 0);
        assert.equal(frame(), undefined);
        assert.equal(timer, undefined);
        props.layer_settings = { [source_id]: { enable_bounding_box: true } };
        await act(async () => { root.render(React.createElement(CandidateHoverFrame, props)); });
        move(0.2);
        await tick();
        assert.equal(requests.length, 1);
        await respond();

        // 点のない枠内からのクリックと、ドラッグ後の誤選択防止
        const gap = new THREE.Vector3(0.2, 0, 0).project(camera);
        const event_position = { clientX: (gap.x + 1) * 100, clientY: (1 - gap.y) * 100, button: 0, isPrimary: true };
        await act(async () => {
            emit('pointerdown', event_position);
            emit('pointerup', { ...event_position, buttons: 0 });
            emit('click', event_position);
        });
        assert.deepEqual(selections, [{ source: source_id, selection: bounds.selection }]);
        await act(async () => {
            emit('pointerdown', event_position);
            emit('pointermove', { ...event_position, clientX: event_position.clientX + 20, buttons: 1 });
            emit('pointerup', { ...event_position, buttons: 0 });
            emit('click', event_position);
        });
        assert.equal(selections.length, 1);
        move(0.2);
        await tick();
        const initial_frame = frame();
        assert.ok(initial_frame);
        assert.equal(canvas.style.cursor, 'pointer');

        // 同じ物体の点間・別ノードへの移動で枠を維持し、再取得を抑止
        for (const x of [0.2, 0.4, 0.25]) {
            move(x);
            await tick(300);
            assert.equal(frame(), initial_frame);
        }
        assert.equal(requests.length, 1);

        // 更新中・一過性の取得失敗も枠を維持。同じrevisionでも再試行
        mesh.userData.inspection_revision = {};
        await tick(300);
        assert.deepEqual(requests.at(-1), { source: source_id });
        assert.ok(frame());
        await act(async () => { pending.shift().reject(new Error('temporary failure')); });
        assert.equal(frame(), initial_frame);
        await tick(300);
        assert.equal(pending.length, 1);
        await respond();
        assert.equal(frame(), initial_frame);

        // 境界の短い外れからの復帰と、十分に離れた場合の解除
        move(0.7);
        await tick(100);
        assert.equal(frame(), initial_frame);
        move(0.2);
        await tick(100);
        assert.equal(frame(), initial_frame);
        move(0.7);
        await tick(400);
        assert.equal(frame(), undefined);
        assert.equal(canvas.style.cursor, 'crosshair');

        move(0);
        await tick(300);
        const previous_frame = frame();
        move(1.4);
        await tick(300);
        assert.equal(frame(), previous_frame);
        assert.ok(Math.abs(frame().position.x - 1.4) < 1e-6);

        // 重なった枠では、カメラに近い候補の選択
        mesh.userData.inspection_revision = {};
        await tick(300);
        move(0.2);
        await respond([bounds, other_bounds, { ...bounds, selection: { ...bounds.selection, id: 9 },
            min_position: [-0.1, -0.2, 1], max_position: [0.5, 0.2, 1.2] }]);
        assert.ok(Math.abs(frame().position.z - 1.1) < 1e-6);

        // カーソル退出後の遅延応答による枠の復活防止
        mesh.userData.inspection_revision = {};
        await tick(300);
        await act(async () => { emit('pointerleave'); });
        await respond();
        assert.equal(frame(), undefined);

        move(0);
        await tick(300);
        await act(async () => { emit('pointerdown'); });
        assert.equal(frame(), undefined);
        move(0);
        await tick(300);
        mesh.visible = other.visible = false;
        const request_num = requests.length;
        const selection_num = selections.length;
        await tick();
        assert.equal(frame(), undefined);
        await act(async () => {
            emit('pointerdown', event_position);
            emit('pointerup', { ...event_position, buttons: 0 });
            emit('click', event_position);
        });
        await tick(500);
        assert.equal(requests.length, request_num);
        assert.equal(selections.length, selection_num);
        assert.equal(canvas.style.cursor, 'crosshair');

        mesh.visible = other.visible = true;
        move(0);
        await tick(300);
        await respond();

        // TF・回転・非一様スケールを含む描画位置と判定位置の一致
        mesh.userData.inspection_revision = {};
        await tick(300);
        const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
        await act(async () => { root.render(React.createElement(CandidateHoverFrame, { ...props,
            transforms: { sensor: { pos: [-1, 0.5, 0], quat: q.toArray() } },
            layer_settings: { [source_id]: { enable_bounding_box: true,
                graphTransform: { position: [0.4, 0, 0], rotation: [0, 0, 0], scale: [2, 1, 1] } } },
        })); });
        move(-1, 1.3);
        await respond([{ ...bounds, frame_id: 'sensor' }]);
        assert.ok(frame());
        const displayed_position = frame().getWorldPosition(new THREE.Vector3());
        assert.ok(displayed_position.distanceTo(new THREE.Vector3(-1, 1.3, 0)) < 1e-6);
        top_element = {};
        await tick();
        assert.equal(frame(), undefined);
        assert.equal(canvas.style.cursor, 'crosshair');
        // 取得待機中のOFF、遅延応答の破棄、再ON時の再取得
        top_element = canvas;
        await act(async () => { root.render(React.createElement(CandidateHoverFrame, props)); });
        mesh.userData.inspection_revision = {};
        move(0.2);
        await tick(300);
        const num_before_disable = requests.length;
        const selections_before_disable = selections.length;
        const disabled_props = { ...props, layer_settings: { [source_id]: { enable_bounding_box: false } } };
        await act(async () => {
            root.render(React.createElement(CandidateHoverFrame, disabled_props));
        });
        assert.equal(frame(), undefined);
        await act(async () => {
            emit('pointerdown', event_position);
            emit('pointerup', { ...event_position, buttons: 0 });
            emit('click', event_position);
        });
        assert.equal(selections.length, selections_before_disable);
        await respond();
        assert.equal(frame(), undefined);
        assert.equal(requests.length, num_before_disable);
        await act(async () => { root.render(React.createElement(CandidateHoverFrame, props)); });
        move(0.2);
        await tick(300);
        assert.equal(requests.length, num_before_disable + 1);
        await respond();
        assert.ok(frame());
        await act(async () => { root.render(React.createElement(CandidateHoverFrame, disabled_props)); });
        assert.equal(frame(), undefined);
        assert.equal(canvas.style.cursor, 'crosshair');
    } finally {
        if (root) await act(async () => { root.unmount(); });
        await new Promise(resolve => setTimeout(resolve, 600));
        geometry.dispose();
        material.dispose();
        for (const [name, descriptor] of saved_globals) {
            if (descriptor) Object.defineProperty(globalThis, name, descriptor);
            else delete globalThis[name];
        }
        await rm(temporary_directory, { recursive: true, force: true });
    }
});
