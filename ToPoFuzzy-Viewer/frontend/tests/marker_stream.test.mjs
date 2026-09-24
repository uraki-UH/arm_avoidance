import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot } from '@react-three/fiber';

test('Markerの最新反映・描画後ACK・旧配信の辞書継承・削除・再接続', async () => {
    const directory = await mkdtemp(resolve('tests/.marker-stream-'));
    const previous_window = globalThis.window, previous_websocket = globalThis.WebSocket;
    const callbacks = new Map(), sent = [];
    let next_id = 0, socket, state, root;
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    globalThis.window = {
        requestAnimationFrame(callback) { callbacks.set(++next_id, callback); return next_id; },
        cancelAnimationFrame(id) { callbacks.delete(id); }, setTimeout, clearTimeout,
    };
    globalThis.WebSocket = class {
        static OPEN = 1;
        readyState = 1;
        constructor() { socket = this; }
        send(value) { sent.push(JSON.parse(value)); }
        close() {}
    };
    try {
        const output = resolve(directory, 'hook.mjs');
        await build({ entryPoints: ['src/hooks/useWebSocket.ts'], outfile: output,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', logLevel: 'silent' });
        const { useWebSocket } = await import(pathToFileURL(output).href);
        function Probe() { state = useWebSocket('ws://marker-test'); return null; }
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        root.configure({ gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 } });
        await act(async () => root.render(React.createElement(Probe)));
        await act(async () => state.connect());
        const emit = value => socket.onmessage({ data: JSON.stringify(value) });
        const frame = async () => act(async () => {
            const pending = [...callbacks.values()]; callbacks.clear();
            pending.forEach(callback => callback());
        });
        const topic = '/plane_clusters/markers/nodes';
        const marker = id => ({ type: 'stream.marker_array', tag: topic,
            markers: [{ ns: 'plane', id, type: 'line_list', action: 0, points: [[id, 0, 0], [id, 1, 0]] }] });
        // 旧gatewayの連続配信も1描画につき最新1件。省略された矢印辞書は継承。
        await act(async () => {
            emit({ ...marker(0), arrow_styles: { shared: { scale: [1, 1, 1] } } });
            for (let id = 1; id <= 20; id++) emit(marker(id));
        });
        assert.equal(state.markerData[topic], undefined);
        await frame();
        assert.equal(state.markerData[topic].markers[0].id, 20);
        assert.deepEqual(state.markerData[topic].arrow_styles.shared.scale, [1, 1, 1]);
        assert.equal(sent.length, 0, '対応能力の通知がない旧gatewayへACKを送信しない');

        emit({ type: 'stream.capabilities', marker_array_applied: true });
        emit({ type: 'stream.capabilities', marker_array_applied: true });
        assert.deepEqual(sent, [{ type: 'stream.marker_array.ready' }]);
        await act(async () => emit(marker(21)));
        await frame();
        assert.equal(state.markerData[topic].markers[0].id, 21);
        assert.equal(sent.length, 1, '反映前のACK送信なし');
        await frame();
        assert.deepEqual(sent.at(-1), { type: 'stream.marker_array.applied', topic });

        await act(async () => {
            emit(marker(22));
            emit({ type: 'stream.delete', topic });
        });
        await frame();
        assert.equal(state.markerData[topic], undefined);
        assert.equal(sent.length, 2, '削除された待機Markerの復活・ACKなし');

        await act(async () => emit({ ...marker(23), markers: [] }));
        await frame();
        assert.deepEqual(state.markerData[topic].markers, []);
        const previous_socket = socket;
        await act(async () => state.connect());
        assert.notEqual(previous_socket, socket);
        await frame();
        assert.equal(sent.length, 2, '旧接続の描画完了通知を新接続へ送信しない');
        await act(async () => emit(marker(24)));
        await frame(); await frame();
        assert.equal(sent.length, 2, '再接続後は対応能力の通知を再確認');
    } finally {
        if (root) await act(async () => root.unmount());
        globalThis.window = previous_window;
        globalThis.WebSocket = previous_websocket;
        delete globalThis.IS_REACT_ACT_ENVIRONMENT;
        await rm(directory, { recursive: true, force: true });
    }
});
