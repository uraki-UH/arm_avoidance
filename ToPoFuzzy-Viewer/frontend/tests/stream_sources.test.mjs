import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot } from '@react-three/fiber';
import * as THREE from 'three';

// 実際のhookとReact更新順序の試験。WebSocketと描画時刻のみ代替。
test('入力選択・遅延受信・再接続とロボットのモデル保持', async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    const directory = await mkdtemp(resolve('tests/.stream-sources-'));
    const previous_window = globalThis.window, previous_socket = globalThis.WebSocket;
    const frames = new Map();
    let frame_id = 0, socket, root, api;
    let raw_sources = [], is_read_only = true, url = 'ws://test/observe';
    globalThis.window = { setTimeout, clearTimeout, setInterval, clearInterval,
        requestAnimationFrame(callback) { frames.set(++frame_id, callback); return frame_id; },
        cancelAnimationFrame(id) { frames.delete(id); } };
    class test_socket extends EventTarget {
        static OPEN = 1;
        readyState = 0;
        sent = [];
        constructor() { super(); socket = this; }
        open() { this.readyState = 1; this.onopen?.(); }
        receive(payload) {
            const event = new MessageEvent('message', { data: payload instanceof ArrayBuffer ? payload : JSON.stringify(payload) });
            this.onmessage?.(event); this.dispatchEvent(event);
        }
        send(message) {
            const request = JSON.parse(message); this.sent.push(request);
            if (!request.method) return;
            queueMicrotask(() => this.receive({ id: request.id, ok: true,
                result: request.method === 'sources.list' ? { sources: raw_sources } : {
                    success: true, sourceId: request.params.sourceId, active: request.params.active } }));
        }
        close() { this.readyState = 3; this.onclose?.(); this.dispatchEvent(new Event('close')); }
    }
    globalThis.WebSocket = test_socket;
    try {
        const output = resolve(directory, 'hook.mjs');
        await build({ entryPoints: ['src/hooks/useWebSocket.ts'], outfile: output, bundle: true,
            packages: 'external', platform: 'node', format: 'esm', logLevel: 'silent' });
        const { useWebSocket } = await import(pathToFileURL(output).href);
        function probe() { api = useWebSocket(url, { is_read_only }); return null; }
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        root.configure({ gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            scene: new THREE.Scene(), frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 } });
        const render = () => act(async () => root.render(React.createElement(probe)));
        const flush = () => act(async () => { const callbacks = [...frames.values()]; frames.clear(); callbacks.forEach(callback => callback(0)); });
        await render();
        await act(async () => { api.connect(); socket.open(); });
        const entries = [['/pc', 'pointcloud', 'pointClouds'], ['/graph', 'topological_map', 'graphData'],
            ['/marker', 'marker', 'markerData'], ['/voxel', 'voxel', 'voxelData']];
        raw_sources = entries.map(([id, type]) => ({ id, type, name: id, active: true }));
        await act(async () => api.getSources());
        const pc = new ArrayBuffer(36);
        new Uint8Array(pc).set([3, ...new TextEncoder().encode('/pc')]);
        const view = new DataView(pc, 4);
        view.setUint32(0, 0x50434458, true); view.setUint32(4, 1, true);
        view.setUint32(8, 1, true); view.setUint8(12, 1); view.setUint32(16, 12, true);
        const graph = { nodes: [], edges: [], clusters: [], frameId: 'world' };
        const robot = { urdf: '<robot name="test"/>', frameId: 'world', jointNames: ['joint'], jointValues: [0], positions: [], orientations: [], timestamp: 1 };
        const publish = () => {
            socket.receive(pc);
            socket.receive({ type: 'stream.graph', tag: '/graph', graph });
            socket.receive({ type: 'stream.marker_array', tag: '/marker', markers: [] });
            socket.receive({ type: 'stream.voxel', tag: '/voxel', data: ['1'], sequence: 1, layout: { voxelSize: .1 } });
        };
        await act(async () => { publish(); socket.receive({ type: 'stream.robot.description', tag: 'test_robot', robot }); });
        await flush();
        entries.forEach(([id, , field]) => assert.ok(api[field][id], field));
        assert.equal(api.robotData.test_robot, undefined);
        assert.equal(api.sources.find(source => source.id === 'robot:test_robot').active, false);
        await act(async () => api.subscribeSource('robot:test_robot'));
        assert.equal(api.robotData.test_robot.urdf, robot.urdf);
        await act(async () => socket.receive({ type: 'stream.capabilities', marker_array_applied: true }));
        // 描画待ちの状態で解除。後着パケットと一覧更新でも復活しないことの確認。
        await act(async () => { publish(); for (const [id] of entries) await api.unsubscribeSource(id, true);
            await api.unsubscribeSource('robot:test_robot', true); });
        for (let idx = 0; idx < 3; ++idx) {
            await act(async () => { publish();
                socket.receive({ type: 'stream.robot.pose', tag: 'test_robot', robot: { jointValues: [idx + 1] } });
                socket.receive({ id: 'sync_sources', ok: true, result: { sources: raw_sources } }); });
            await flush();
            entries.forEach(([id, , field]) => assert.equal(api[field][id], undefined, field));
            assert.equal(api.robotData.test_robot, undefined);
            assert.ok(api.sources.every(source => !source.active));
        }
        assert.ok(socket.sent.some(item => item.type === 'stream.marker_array.applied'));
        assert.ok(!socket.sent.some(item => item.method === 'sources.setActive' && !item.params.active));
        // 非選択バイナリGNGのACK返却と、本体展開の省略。
        const tag = new TextEncoder().encode('/graph'), packet = new ArrayBuffer(36 + tag.length);
        const header = new DataView(packet);
        header.setUint32(0, 0x31474d54, true); header.setUint16(4, 2, true);
        header.setUint32(8, tag.length, true); header.setUint32(20, 100, true); header.setUint32(32, tag.length, true);
        new Uint8Array(packet, 36).set(tag);
        await act(async () => socket.receive(packet));
        assert.ok(socket.sent.some(item => item.type === 'stream.topological_map.applied' && item.topic === '/graph'));
        await act(async () => { for (const [id] of entries) await api.subscribeSource(id);
            await api.subscribeSource('robot:test_robot'); publish(); });
        await flush();
        entries.forEach(([id, , field]) => assert.ok(api[field][id], field));
        assert.equal(api.robotData.test_robot.urdf, robot.urdf);
        assert.deepEqual(api.robotData.test_robot.jointNames, ['joint']);
        assert.deepEqual(api.robotData.test_robot.jointValues, [3]);
        assert.ok(!socket.sent.some(item => item.params?.sourceId?.startsWith('robot:')));
        // 配信元が一覧から消えた後の後着データも除外。再検出後は再開可能。
        const all_sources = raw_sources;
        raw_sources = raw_sources.filter(source => source.id !== '/graph');
        await act(async () => { await api.getSources(); publish(); }); await flush();
        assert.equal(api.graphData['/graph'], undefined);
        raw_sources = all_sources;
        await act(async () => { await api.getSources(); publish(); }); await flush();
        assert.ok(api.graphData['/graph']);
        await act(async () => { socket.receive({ type: 'stream.robot.delete', tag: 'test_robot' });
            socket.receive({ type: 'stream.robot.pose', tag: 'test_robot', robot: { jointValues: [8] } }); });
        await flush();
        assert.equal(api.robotData.test_robot, undefined);
        assert.ok(!api.sources.some(source => source.type === 'robot'));
        // 同じ接続先への再接続は解除を保持。旧接続のパケットは無効。
        await act(async () => api.unsubscribeSource('/pc', true));
        const old_socket = socket;
        await act(async () => { api.connect(); socket.open(); publish(); old_socket.receive({ type: 'stream.graph', tag: '/old', graph }); });
        await flush();
        assert.equal(api.pointClouds['/pc'], undefined); assert.equal(api.graphData['/old'], undefined);
        // 接続先変更は選択状態とモデルキャッシュを初期化。
        url = 'ws://other/observe'; await render();
        await act(async () => { api.connect(); socket.open(); publish(); }); await flush();
        assert.ok(api.pointClouds['/pc']);
        // 通常Viewerでも解除と後着データを処理。
        is_read_only = false; await render();
        await act(async () => api.unsubscribeSource('/pc', true));
        await act(async () => socket.receive(pc)); await flush();
        assert.equal(api.pointClouds['/pc'], undefined);
        assert.ok(socket.sent.some(item => item.method === 'sources.setActive' && item.params.active === false));
        await act(async () => api.disconnect());
        entries.forEach(([, , field]) => assert.deepEqual(api[field], {}));
    } finally {
        await act(async () => root?.unmount()); socket?.close();
        globalThis.window = previous_window; globalThis.WebSocket = previous_socket;
        await rm(directory, { recursive: true, force: true });
    }
});
