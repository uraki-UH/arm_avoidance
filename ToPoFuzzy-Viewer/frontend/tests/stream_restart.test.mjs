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

test('停止イベントでScene Layers用状態と描画待ちを消去、同名ストリームの復帰', async () => {
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    const directory = await mkdtemp(resolve('tests/.stream-restart-'));
    const previous_window = globalThis.window;
    const previous_websocket = globalThis.WebSocket;
    const frames = new Map();
    let frame_id = 0;
    let socket;
    let state;
    let graph_settings;
    let root;
    globalThis.window = {
        requestAnimationFrame(callback) { frames.set(++frame_id, callback); return frame_id; },
        cancelAnimationFrame(id) { frames.delete(id); },
        setTimeout, clearTimeout,
    };
    globalThis.WebSocket = class {
        static OPEN = 1;
        readyState = 1;
        constructor() { socket = this; }
        send() {}
        close() {}
    };
    try {
        const output = resolve(directory, 'hook.mjs');
        await build({ stdin: { contents: `
            export { useWebSocket } from './src/hooks/useWebSocket';
            export { useGraphLayerSettings } from './src/features/visualization/graphLayerSettings';
        `, resolveDir: process.cwd() }, outfile: output,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', logLevel: 'silent' });
        const { useWebSocket, useGraphLayerSettings } = await import(pathToFileURL(output).href);
        function probe() {
            state = useWebSocket('ws://test');
            graph_settings = useGraphLayerSettings(state.graphData);
            return null;
        }
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        root.configure({ gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 } });
        await act(async () => root.render(React.createElement(probe)));
        await act(async () => state.connect());
        const emit = (value) => socket.onmessage({ data: JSON.stringify(value) });
        const flush = async () => act(async () => {
            const callbacks = [...frames.values()]; frames.clear();
            callbacks.forEach((callback) => callback());
        });
        const graph = (x) => ({ type: 'stream.graph', tag: '/restart/Tmap',
            graph: { nodes: [{ id: 1, x, y: 0, z: 0 }], edges: [], clusters: [] } });
        const marker = (id) => ({ type: 'stream.marker_array', tag: '/restart/markers',
            markers: [{ id, type: 2, action: 0, frameId: 'map' }] });
        await act(async () => {
            emit({ id: 'sync_sources', ok: true, result: { sources: [
                { id: '/restart/Tmap', active: true }, { id: '/restart/markers', active: true }] } });
            emit(graph(1)); emit(marker(1));
        });
        await flush();
        assert.equal(state.graphData['/restart/Tmap'].nodes[0].x, 1);
        assert.equal(state.markerData['/restart/markers'].markers[0].id, 1);
        await act(async () => graph_settings.updateLayerSettings('/restart/Tmap', {
            visible: false, nodeColor: '#123456', visibleLabels: [3],
            showManipulabilityEllipsoids: true,
            graphTransform: { position: [1, 2, 3], rotation: [0, 0, 1], scale: [2, 2, 2] },
        }));
        const saved_settings = graph_settings.layerSettings['/restart/Tmap'];
        await act(async () => {
            // フレーム描画前の旧データを残した状態での停止通知。
            emit(graph(9));
            emit({ type: 'stream.delete', topic: '/restart/Tmap' });
            emit({ type: 'stream.delete', topic: '/restart/markers' });
        });
        await flush();
        assert.deepEqual(state.graphData, {});
        assert.deepEqual(state.markerData, {});
        assert.ok(state.sources.every((source) => source.active));
        assert.equal(graph_settings.layerSettings['/restart/Tmap'], saved_settings);
        await act(async () => { emit(graph(2)); emit(marker(2)); });
        await flush();
        assert.equal(state.graphData['/restart/Tmap'].nodes[0].x, 2);
        assert.equal(state.markerData['/restart/markers'].markers[0].id, 2);
        assert.equal(graph_settings.layerSettings['/restart/Tmap'], saved_settings);
    } finally {
        if (root) await act(async () => root.unmount());
        globalThis.window = previous_window;
        globalThis.WebSocket = previous_websocket;
        await rm(directory, { recursive: true, force: true });
    }
});

test('実Appの点群再同期は旧バッファを破棄し、表示OFF・透明度0・手動変換を保持', async () => {
    // Appの実際のeffectを評価。React描画・ROS通信のみ対象外。
    const source = ts.createSourceFile('App.tsx', await readFile('src/App.tsx', 'utf8'),
        ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX);
    function find(node) {
        if (ts.isCallExpression(node) && node.expression.getText(source) === 'useEffect'
            && node.arguments[1]?.getText(source).includes('wsPointClouds')) return node;
        return ts.forEachChild(node, find);
    }
    const effect = find(source);
    assert.ok(effect);
    const code = ts.transpileModule(`return (${effect.arguments[0].getText(source)});`, {
        compilerOptions: { target: ts.ScriptTarget.ES2022 },
    }).outputText;
    const settings = { current: new Map() };
    let clouds = [];
    const sync = (received) => {
        const scope = {
            setPointClouds: (update) => { clouds = update(clouds); },
            point_cloud_view_settings_ref: settings,
            wsPointClouds: received, disabledSourceIds: new Set(),
            isEditMode: false, editLayerId: null, pointCloudOpacity: 1,
        };
        new Function(...Object.keys(scope), code)(...Object.values(scope))();
    };
    const cloud = (x) => ({ id: '/points', name: '/points', frameId: 'map',
        points: new Float32Array([x, 0, 1]), count: 1 });
    sync({ '/points': cloud(1) });
    const presentation = { visible: false, opacity: 0, position: [1, 2, 3],
        rotation: [0, 0, 0.5], scale: [2, 2, 2], matrix: new THREE.Matrix4() };
    clouds[0] = { ...clouds[0], ...presentation };
    sync({});
    assert.deepEqual(clouds, []);
    assert.deepEqual(settings.current.get('/points'), presentation);
    assert.equal('points' in settings.current.get('/points'), false);
    const next = { ...cloud(2), frameId: 'world' };
    sync({ '/points': next });
    assert.equal(clouds[0].points, next.points);
    assert.equal(clouds[0].frameId, 'world');
    for (const [key, value] of Object.entries(presentation)) assert.deepEqual(clouds[0][key], value);
});
