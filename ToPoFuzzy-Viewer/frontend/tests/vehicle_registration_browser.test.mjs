import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { build } from 'esbuild';

const profile = await mkdtemp(join(tmpdir(), 'vehicle-registration-browser-'));
const pending = new Map();
let browser;
let session_id;
let next_id = 0;
const pause = ms => new Promise(done => setTimeout(done, ms));
try {
    const evidence = resolve('../../benchmarks/vehicle_registration_20260924');
    const synthetic = JSON.parse(await readFile(join(evidence, 'synthetic_partial.json'), 'utf8')).snapshot;
    const live = JSON.parse(await readFile(join(evidence, 'live_57.json'), 'utf8')).snapshot;
    const bundle = await build({ stdin: { resolveDir: process.cwd(), loader: 'tsx', contents: `
        import React,{useState} from 'react';
        import {createRoot} from 'react-dom/client';
        import {ClusterDetailPanel} from './src/features/visualization/ClusterDetailPanel';
        import {useWebSocket} from './src/hooks/useWebSocket';
        function App() {
            const api=useWebSocket('ws://127.0.0.1:9001'); window.api=api;
            const [snapshot,set_snapshot]=useState(${JSON.stringify(synthetic)}); window.change_snapshot=set_snapshot;
            return <div style={{position:'relative',width:700,height:850}}><ClusterDetailPanel snapshot={snapshot}
                onClose={()=>{}} on_refresh={()=>set_snapshot({...snapshot})} is_loading={false} error={null}
                register_vehicle={api.register_vehicle}/></div>;
        }
        const root=createRoot(document.getElementById('app')); root.render(<App/>);
        window.unmount=()=>root.unmount();
    ` }, bundle: true, write: false, format: 'iife', jsx: 'automatic',
        define: { 'process.env.NODE_ENV': '"production"' } });
    const command = ['/opt/google/chrome/chrome', '--headless=new', '--use-gl=angle', '--use-angle=swiftshader',
        '--enable-unsafe-swiftshader', '--no-first-run', '--no-default-browser-check', '--remote-debugging-pipe',
        `--user-data-dir=${profile}`, 'about:blank'];
    console.log('START', command.join(' '));
    browser = spawn(command[0], command.slice(1), { stdio: ['ignore', 'ignore', 'ignore', 'pipe', 'pipe'], detached: true });
    console.log('Chrome PID:', browser.pid);
    let incoming = '';
    browser.stdio[4].on('data', data => {
        incoming += data;
        let end;
        while ((end = incoming.indexOf('\0')) >= 0) {
            const message = JSON.parse(incoming.slice(0, end)); incoming = incoming.slice(end + 1);
            const waiter = pending.get(message.id);
            if (!waiter) continue;
            pending.delete(message.id);
            if (message.error) waiter.reject(new Error(JSON.stringify(message.error)));
            else waiter.resolve(message.result);
        }
    });
    const call = (method, params = {}, session = session_id) => new Promise((done, fail) => {
        const id = ++next_id;
        const timer = setTimeout(() => { pending.delete(id); fail(new Error('CDP timeout: ' + method)); }, 30000);
        pending.set(id, { resolve: value => { clearTimeout(timer); done(value); }, reject: error => { clearTimeout(timer); fail(error); } });
        browser.stdio[3].write(JSON.stringify({ id, method, params, sessionId: session }) + '\0');
    });
    const { targetId } = await call('Target.createTarget', { url: 'about:blank' });
    session_id = (await call('Target.attachToTarget', { targetId, flatten: true })).sessionId;
    await call('Page.enable');
    await call('Emulation.setDeviceMetricsOverride', { width: 700, height: 850, deviceScaleFactor: 1, mobile: false });
    const { frameTree } = await call('Page.getFrameTree');
    await call('Page.setDocumentContent', { frameId: frameTree.frame.id,
        html: '<html><body style="margin:0;background:#111827;color:#e5e7eb;font-family:sans-serif"><div id="app"></div></body></html>' });
    const evaluate = async expression => {
        const result = await call('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
        if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
        return result.result.value;
    };
    await evaluate(bundle.outputFiles[0].text);
    const { readdir } = await import('node:fs/promises');
    const css_file = (await readdir('dist/assets')).find(name=>name.endsWith('.css'));
    const css = await readFile(join('dist/assets',css_file),'utf8');
    await evaluate(`document.head.appendChild(Object.assign(document.createElement('style'),{textContent:${JSON.stringify(css)}}))`);
    const until = async predicate => {
        for(let iter=0;iter<250;iter++) { if(await evaluate(predicate)) return; await pause(100); }
        throw new Error('待機時間超過: '+predicate+' / '+await evaluate('document.body.innerText'));
    };
    await until('Boolean(window.api)');
    await evaluate('window.api.connect()');
    await until('Boolean(window.api?.isConnected)');
    await evaluate("[...document.querySelectorAll('button')].find(b=>b.textContent==='車モデルを比較').click()");
    await until("Boolean(document.querySelector('table'))");
    assert.equal(await evaluate("document.querySelectorAll('tbody tr').length"),4);
    assert.match(await evaluate('document.body.innerText'),/形状候補: 乗用車（セダン）/);
    assert.equal(await evaluate('document.querySelectorAll("canvas").length'),1);
    let shot=await call('Page.captureScreenshot',{format:'png'});
    await writeFile(join(evidence,'synthetic_registration.png'),Buffer.from(shot.data,'base64'));
    await evaluate("document.querySelectorAll('tbody button')[3].click()");
    assert.equal(await evaluate("document.querySelectorAll('tbody button')[3].getAttribute('aria-pressed')"),'true');
    await evaluate(`[...document.querySelectorAll('button')].find(b=>b.textContent==='車モデルを比較').click();window.change_snapshot(${JSON.stringify(live)})`);
    await until("!document.querySelector('table')");
    await pause(5000);
    assert.equal(await evaluate("Boolean(document.querySelector('table'))"),false,'古いクラスタの計算結果を表示しない');
    await evaluate("[...document.querySelectorAll('button')].find(b=>b.textContent==='車モデルを比較').click()");
    await until("Boolean(document.querySelector('table'))");
    assert.match(await evaluate('document.body.innerText'),/判定保留/);
    shot=await call('Page.captureScreenshot',{format:'png'});
    await writeFile(join(evidence,'intersection_registration.png'),Buffer.from(shot.data,'base64'));
    console.log('PASS 実RPC・4候補・姿勢照合・候補切替・選択変更中の古い結果破棄・実交差点の判定保留');
    await evaluate('window.unmount()');
} finally {
    for (const waiter of pending.values()) waiter.reject(new Error('検証終了'));
    pending.clear();
    if (browser?.pid) {
        for (const signal of ['SIGINT', 'SIGTERM', 'SIGKILL']) {
            try { process.kill(-browser.pid, signal); } catch (error) { if (error.code !== 'ESRCH') throw error; }
            await pause(150);
        }
    }
    await rm(profile, { recursive: true, force: true });
    console.log('Chrome停止済み・専用プロファイル削除済み');
}
