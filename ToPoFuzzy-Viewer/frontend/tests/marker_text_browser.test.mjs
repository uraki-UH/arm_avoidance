import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { build } from 'esbuild';

const profile = await mkdtemp(join(tmpdir(), 'marker-text-browser-'));
let browser;
let session_id;
let next_id = 0;
const pending = new Map();
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
try {
    const bundle = await build({ stdin: { resolveDir: process.cwd(), contents: `
        import React, {useState} from 'react';
        import {createRoot} from 'react-dom/client';
        import {Canvas, useThree} from '@react-three/fiber';
        import {MarkerArrayRenderer} from './src/features/visualization/MarkerArrayRenderer';
        import * as THREE from 'three';
        function Probe() { window.scene_state=useThree(); return null; }
        function App() {
            const [data, set_data]=useState({markers: []});
            window.set_data=set_data;
            return <Canvas camera={{position:[0,0,3],fov:40}} gl={{preserveDrawingBuffer:true}}
                frameloop="demand" onCreated={({gl})=>gl.setClearColor('#101018')}>
                <Probe /><MarkerArrayRenderer tag="/standard_text" data={data}
                    transforms={{sensor:{pos:[0.2,0,0],quat:[0,0,0,1]}}}/>
            </Canvas>;
        }
        const root=createRoot(document.getElementById('app'));
        root.render(<App />);
        window.disposed=0;
        const observed=new Set();
        window.snapshot=()=>{
            const {scene,gl,camera}=window.scene_state;
            gl.render(scene,camera);
            const sprites=[];
            scene.traverse(o=>{
                if (!o.isSprite) return;
                const texture=o.material.map;
                if (!observed.has(texture.uuid)) {
                    observed.add(texture.uuid);
                    texture.addEventListener('dispose',()=>window.disposed++);
                }
                sprites.push({texture:texture.uuid,position:o.getWorldPosition(new THREE.Vector3()).toArray(),
                    scale:o.scale.toArray(),opacity:o.material.opacity,color:o.material.color.toArray(),
                    width:texture.image.width,height:texture.image.height});
            });
            const context=gl.getContext();
            const pixels=new Uint8Array(context.drawingBufferWidth*context.drawingBufferHeight*4);
            context.readPixels(0,0,context.drawingBufferWidth,context.drawingBufferHeight,context.RGBA,context.UNSIGNED_BYTE,pixels);
            let lit=0;
            for (let i=0;i<pixels.length;i+=4) if (Math.max(pixels[i],pixels[i+1],pixels[i+2])>100) lit++;
            return {sprites,lit,disposed:window.disposed};
        };
        window.unmount=()=>root.unmount();
    `, loader: 'tsx' }, bundle: true, write: false, format: 'iife', jsx: 'automatic',
    define: { 'process.env.NODE_ENV': '"production"' } });
    // 専用プロファイル・CDPパイプで既存ブラウザから分離した実WebGL描画検証
    const command = ['/opt/google/chrome/chrome', '--headless=new', '--use-gl=angle', '--use-angle=swiftshader',
        '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check',
        '--remote-debugging-pipe', `--user-data-dir=${profile}`, 'about:blank'];
    console.log('START', command.join(' '));
    browser = spawn(command[0], command.slice(1), { stdio: ['ignore', 'ignore', 'ignore', 'pipe', 'pipe'], detached: true });
    console.log('Chrome PID:', browser.pid);
    let incoming = '';
    browser.stdio[4].on('data', data => {
        incoming += data;
        let end;
        while ((end = incoming.indexOf('\0')) >= 0) {
            const message = JSON.parse(incoming.slice(0, end)); incoming = incoming.slice(end + 1);
            const value = pending.get(message.id);
            if (!value) continue;
            pending.delete(message.id);
            if (message.error) value.reject(new Error(JSON.stringify(message.error)));
            else value.resolve(message.result);
        }
    });
    const call = (method, params = {}, session = session_id) => new Promise((resolve, reject) => {
        const id = ++next_id;
        const timer = setTimeout(() => { pending.delete(id); reject(new Error('CDP timeout: ' + method)); }, 10000);
        pending.set(id, { resolve: value => { clearTimeout(timer); resolve(value); }, reject: error => { clearTimeout(timer); reject(error); } });
        browser.stdio[3].write(JSON.stringify({ id, method, params, sessionId: session }) + '\0');
    });
    const { targetId } = await call('Target.createTarget', { url: 'about:blank' });
    session_id = (await call('Target.attachToTarget', { targetId, flatten: true })).sessionId;
    await call('Page.enable');
    await call('Emulation.setDeviceMetricsOverride', { width: 960, height: 540, deviceScaleFactor: 1, mobile: false });
    const { frameTree } = await call('Page.getFrameTree');
    await call('Page.setDocumentContent', { frameId: frameTree.frame.id,
        html: '<html><body style="margin:0"><div id="app" style="width:960px;height:540px"></div></body></html>' });
    const evaluate = async expression => {
        const response = await call('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
        if (response.exceptionDetails) throw new Error(JSON.stringify(response.exceptionDetails));
        return response.result.value;
    };
    await evaluate(bundle.outputFiles[0].text);
    for (let iter = 0; iter < 50 && !await evaluate('Boolean(window.scene_state)'); iter++) await pause(100);
    assert.ok(await evaluate('Boolean(window.scene_state)'), 'Canvas初期化');
    let marker = { ns: 'test', id: 1, type: 'text', action: 0, frameId: 'sensor',
        pos: [-0.2, 0, 0], quat: [0, 0, 1, 0], scale: [0, 0, 0.06], color: [1, 1, 1, 1],
        text: '#1 local_point_budget\n接触未確認' };
    const render = async (data = { markers: [marker] }) => {
        await evaluate(`window.set_data(${JSON.stringify(data)})`);
        await pause(120);
        return evaluate('window.snapshot()');
    };
    const initial = await render();
    assert.equal(initial.sprites.length, 1);
    assert.deepEqual(initial.sprites[0].position, [0, 0, 0]);
    assert.ok(initial.lit > 100, '文字の実ピクセル出力');
    marker = { ...marker, pos: [0, 0, 0], color: [0, 1, 0, 0.5], scale: [0, 0, 0.12] };
    const changed = await render();
    assert.equal(changed.sprites[0].texture, initial.sprites[0].texture);
    assert.equal(changed.sprites[0].opacity, 0.5);
    assert.equal(changed.sprites[0].scale[1], initial.sprites[0].scale[1] * 2);
    assert.deepEqual(changed.sprites[0].position, [0.2, 0, 0]);
    await evaluate('window.scene_state.camera.position.set(2,1,3);window.scene_state.camera.lookAt(0,0,0)');
    assert.ok((await evaluate('window.snapshot()')).lit > 100, '視点変更後の文字出力');
    marker = { ...marker, text: '#2 insufficient_contact_support', scale: [0, 0, 0.045], color: [1, 1, 1, 1] };
    const replacement = await render();
    assert.notEqual(replacement.sprites[0].texture, initial.sprites[0].texture);
    assert.ok(replacement.disposed >= 1);
    if (process.env.MARKER_SCREENSHOT) {
        const shot = await call('Page.captureScreenshot', { format: 'png' });
        await writeFile(process.env.MARKER_SCREENSHOT, Buffer.from(shot.data, 'base64'));
    }
    assert.equal((await render({ markers: [marker], visible: false })).sprites.length, 0);
    assert.ok((await render()).lit > 100);
    assert.equal((await render({ markers: [{ ...marker, color: [1, 1, 1, 0] }] })).lit, 0);
    for (const value of [{ ...marker, action: 2 }, { ...marker, action: 3 }, { ...marker, text: '' },
        { ...marker, scale: [1, 1, 0] }]) assert.equal((await render({ markers: [value] })).sprites.length, 0);
    marker = { ...marker, text: 'long '.repeat(2000) };
    const long = await render();
    assert.ok(long.sprites[0].width <= 2048 && long.sprites[0].height <= 2048);
    assert.equal((await render({ markers: [] })).sprites.length, 0);
    if (process.env.MARKER_FIXTURE) {
        const live = JSON.parse(await readFile(process.env.MARKER_FIXTURE, 'utf8'));
        const active = live.markers.filter(m => m.action === 0);
        const labels = active.filter(m => m.type === 'text');
        assert.ok(active.length > 0 && active.every(m => m.frameId === 'world'));
        assert.ok(labels.every(m => m.text));
        const center = active.reduce((sum, m) => sum.map((v, idx) => v + m.pos[idx] / active.length), [0, 0, 0]);
        const has_geometry = active.length !== labels.length;
        await render(live);
        await evaluate(`window.scene_state.camera.position.set(${center[0] + (has_geometry ? 0.4 : 0)},
            ${center[1] - (has_geometry ? 0.6 : 0)},${center[2] + (has_geometry ? 0.5 : 0.9)});
            window.scene_state.camera.lookAt(${center.join(',')})`);
        const actual = await evaluate('window.snapshot()');
        assert.equal(actual.sprites.length, labels.length);
        assert.ok(actual.lit > 100);
        if (process.env.MARKER_SCREENSHOT) {
            const shot = await call('Page.captureScreenshot', { format: 'png' });
            await writeFile(process.env.MARKER_SCREENSHOT, Buffer.from(shot.data, 'base64'));
        }
        console.log('実ROS Marker描画:', active.length, '文字:', labels.length);
    }
    await evaluate('window.unmount()');
    console.log('PASS: 文字ピクセル・TF・色・透明度・高さ・視点・更新・削除・解放・テクスチャ上限');
} finally {
    for (const value of pending.values()) value.reject(new Error('検証終了'));
    pending.clear();
    if (browser?.pid) {
        for (const signal of ['SIGTERM', 'SIGKILL']) {
            try { process.kill(-browser.pid, signal); } catch (error) { if (error.code !== 'ESRCH') throw error; }
            await pause(150);
        }
    }
    await rm(profile, { recursive: true, force: true });
    console.log('Chrome停止済み・専用プロファイル削除済み');
}
