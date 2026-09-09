import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, readFile, readdir, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { build } from 'esbuild';

const profile = await mkdtemp(join(tmpdir(), 'label-priority-browser-'));
let browser;
const pending = new Map();
let next_id = 0;
let session_id;
let errors = '';
const pause = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
try {
    const bundle = await build({ stdin: { resolveDir: process.cwd(), contents: `
        import React, {useState} from 'react';
        import {createRoot} from 'react-dom/client';
        import {GngLabelModal} from './src/features/visualization/GngLabelModal';
        function App() {
            const [settings, set_settings] = useState({});
            window.settings = settings;
            return <GngLabelModal open visibleLabels={{0:false,1:false,2:false,3:false,4:false,5:false}}
                label_settings={settings} onClose={()=>{}} onUpdate={(value)=>set_settings(current=>({...current,...value}))} />;
        }
        const root = createRoot(document.getElementById('app'));
        window.unmount = () => root.unmount();
        root.render(<App />);
    `, loader: 'tsx' }, bundle: true, write: false, format: 'iife', jsx: 'automatic', define: { 'process.env.NODE_ENV': '"production"' } });
    // ポート公開なしのCDPパイプと専用一時プロファイル。既存ブラウザとの分離。
    const command = ['/opt/google/chrome/chrome', '--headless=new', '--disable-gpu', '--no-first-run',
        '--no-default-browser-check', '--remote-debugging-pipe', `--user-data-dir=${profile}`, 'about:blank'];
    console.log('起動コマンド:', command.join(' '));
    browser = spawn(command[0], command.slice(1), { stdio: ['ignore', 'ignore', 'pipe', 'pipe', 'pipe'], detached: true });
    console.log('検証Chrome PID:', browser.pid);
    browser.stderr.on('data', (data) => { errors += data; });
    browser.on('error', (error) => { for (const value of pending.values()) value.reject(error); });
    browser.on('exit', () => { for (const value of pending.values()) value.reject(new Error('Chrome終了: ' + errors.slice(-1200))); });
    let incoming = '';
    browser.stdio[4].on('data', (data) => {
        incoming += data;
        let end;
        while ((end = incoming.indexOf('\0')) >= 0) {
            const message = JSON.parse(incoming.slice(0, end)); incoming = incoming.slice(end + 1);
            const value = pending.get(message.id);
            if (value) {
                pending.delete(message.id);
                if (message.error) value.reject(new Error(JSON.stringify(message.error)));
                else value.resolve(message.result);
            }
        }
    });
    const call = (method, params = {}, session = session_id) => new Promise((resolve, reject) => {
        const id = ++next_id;
        const timer = setTimeout(() => { pending.delete(id); reject(new Error('CDP時間超過: ' + method)); }, 10000);
        pending.set(id, { resolve: (value) => { clearTimeout(timer); resolve(value); }, reject: (error) => { clearTimeout(timer); reject(error); } });
        browser.stdio[3].write(JSON.stringify({ id, method, params, sessionId: session }) + '\0');
    });
    await call('Browser.getVersion');
    const { targetId } = await call('Target.createTarget', { url: 'about:blank' });
    session_id = (await call('Target.attachToTarget', { targetId, flatten: true })).sessionId;
    await call('Page.enable');
    await call('Emulation.setDeviceMetricsOverride', { width: 440, height: 900, deviceScaleFactor: 1, mobile: false });
    const css_name = (await readdir('dist/assets')).find((name) => name.endsWith('.css'));
    const css = await readFile(join('dist/assets', css_name), 'utf8');
    const { frameTree } = await call('Page.getFrameTree');
    await call('Page.setDocumentContent', { frameId: frameTree.frame.id,
        html: `<html><head><style>${css}</style></head><body><div id="app"></div></body></html>` });
    const evaluate = async (expression) => {
        const response = await call('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
        if (response.exceptionDetails) throw new Error(JSON.stringify(response.exceptionDetails));
        return response.result.value;
    };
    await evaluate(bundle.outputFiles[0].text);
    await pause(100);
    const selector = (name) => `[aria-label="${name}"]`;
    const point = (name) => evaluate(`(()=>{const e=document.querySelector(${JSON.stringify(selector(name))});const r=e.getBoundingClientRect();return {x:r.left+r.width/2,y:r.top+(e.dataset.priorityId ? 16 : r.height/2)}})()`);
    const order = (name) => evaluate(`Array.from(document.querySelector(${JSON.stringify(selector(name))}).children).map(x=>x.dataset.priorityId).filter(Boolean)`);
    const mouse = (type, p) => call('Input.dispatchMouseEvent', { type, ...p, button: 'left', buttons: type === 'mouseReleased' ? 0 : 1, clickCount: 1 });
    const drag = async (name, target, hold = 420, is_escape = false, source = null) => {
        await mouse('mousePressed', source ?? await point(name + 'を長押しして移動'));
        await pause(hold);
        if (hold >= 350 && !is_escape) assert.ok(await evaluate('document.querySelector("[data-insertion-marker]") !== null'), '移動前の長押し成立');
        if (is_escape) await call('Input.dispatchKeyEvent', { type: 'keyDown', key: 'Escape', code: 'Escape', windowsVirtualKeyCode: 27 });
        await mouse('mouseMoved', target);
        if (hold >= 350 && !is_escape && target.x > 20) assert.ok(await evaluate('document.querySelector("[data-insertion-marker]") !== null'));
        await mouse('mouseReleased', target);
        await pause(30);
        assert.equal(await evaluate('document.querySelector("[data-insertion-marker]") !== null'), false);
    };
    const root_name = '重複ラベルの優先順位';
    const frame_style = () => evaluate(`(()=>{
        const style=getComputedStyle(document.querySelector('[data-priority-id="handle"]'));
        return {border:style.borderTopWidth,color:style.borderTopColor,background:style.backgroundColor};
    })()`);
    const idle_style = await frame_style();
    assert.equal(idle_style.border, '1px');
    assert.notEqual(idle_style.background, 'rgba(0, 0, 0, 0)');
    const frame_point = await point('HANDLEを長押しして移動');
    frame_point.x = await evaluate('document.querySelector("[data-priority-id=handle]").getBoundingClientRect().left + 10');
    await call('Input.dispatchMouseEvent', { type: 'mouseMoved', ...frame_point, buttons: 0 });
    await pause(200);
    const hover_style = await frame_style();
    assert.notEqual(hover_style.color, idle_style.color);
    await mouse('mousePressed', frame_point);
    await pause(200);
    const pressed_style = await frame_style();
    assert.notEqual(pressed_style.color, hover_style.color);
    await mouse('mouseReleased', frame_point);
    await pause(200);
    const button_styles = await evaluate(`['境界候補','HANDLE'].map(name=>{
        const style=getComputedStyle(document.querySelector('[aria-label="'+name+'を先頭へ移動"]'));
        return {border:style.borderTopWidth,background:style.backgroundColor,opacity:style.opacity};
    })`);
    for (const style of button_styles) {
        assert.equal(style.border, '1px');
        assert.equal(style.opacity, '1');
        assert.notEqual(style.background, 'rgba(0, 0, 0, 0)');
    }
    assert.notEqual(button_styles[0].background, button_styles[1].background);
    const check_overlap = async () => {
        const lower_target = await point('HANDLEを長押しして移動'); lower_target.y -= 12;
        await drag('境界候補', lower_target);
        assert.deepEqual(await order(root_name), ['handle', 'boundary'], '下の項目の上端付近への重なり');
        const upper_target = await point('HANDLEを長押しして移動'); upper_target.y += 12;
        await drag('境界候補', upper_target);
        assert.deepEqual(await order(root_name), ['boundary', 'handle'], '上の項目の下端付近への重なり');
    };
    const check_frame = async () => {
        for (const edge of ['left', 'right', 'top', 'bottom']) {
            const source = await evaluate(`(()=>{
                const r=document.querySelector('[data-priority-id="handle"]').getBoundingClientRect();
                const edge=${JSON.stringify(edge)};
                return {x:edge==='left'?r.left+0.5:edge==='right'?r.right-0.5:r.left+r.width/2,
                    y:edge==='top'?r.top+0.5:edge==='bottom'?r.bottom-0.5:r.top+r.height/2};
            })()`);
            await drag('HANDLE', await point('境界候補を長押しして移動'), 420, false, source);
            assert.deepEqual(await order(root_name), ['handle', 'boundary'], edge + '枠線からの長押し');
            await mouse('mousePressed', await point('境界候補を先頭へ移動'));
            await mouse('mouseReleased', await point('境界候補を先頭へ移動'));
            await pause(30);
        }
        const summary_point = await evaluate(`(()=>{const r=document.querySelector('summary').getBoundingClientRect();return {x:r.left+r.width/2,y:r.top+r.height/2}})()`);
        await drag('境界候補', await point('HANDLEを長押しして移動'), 420, false, summary_point);
        assert.deepEqual(await order(root_name), ['handle', 'boundary'], '枠内の展開見出しからの長押し');
        assert.equal(await evaluate('document.querySelector("details").open'), false, 'ドラッグでの誤展開なし');
        await mouse('mousePressed', await point('境界候補を先頭へ移動'));
        await mouse('mouseReleased', await point('境界候補を先頭へ移動'));
        await pause(30);
        await mouse('mousePressed', summary_point);
        await mouse('mouseReleased', summary_point);
        await pause(30);
        assert.equal(await evaluate('document.querySelector("details").open'), true, '見出しの短いクリックによる展開');
        await mouse('mousePressed', summary_point);
        await mouse('mouseReleased', summary_point);
        await pause(30);
        assert.equal(await evaluate('document.querySelector("details").open'), false, '見出しの短いクリックによる折り畳み');
    };
    assert.deepEqual(await order(root_name), ['boundary', 'handle']);
    await mouse('mousePressed', await point('HANDLEの色分け'));
    await mouse('mouseReleased', await point('HANDLEの色分け'));
    await pause(30);
    assert.equal(await evaluate('window.settings.node_label_visibility.handle'), false);
    await mouse('mousePressed', await point('HANDLEの色分け'));
    await mouse('mouseReleased', await point('HANDLEの色分け'));
    await pause(30);
    assert.equal(await evaluate('window.settings.node_label_visibility.handle'), true);
    const icon_point = await evaluate(`(()=>{const r=document.querySelector('[aria-label="HANDLEを長押しして移動"] [data-drag-indicator]').getBoundingClientRect();return {x:r.left+r.width/2,y:r.top+r.height/2}})()`);
    await drag('HANDLE', await point('境界候補を長押しして移動'), 30, false, icon_point);
    assert.deepEqual(await order(root_name), ['handle', 'boundary'], '押しながらすぐ動かす操作の取りこぼし');
    await mouse('mousePressed', await point('境界候補を先頭へ移動'));
    await mouse('mouseReleased', await point('境界候補を先頭へ移動'));
    await pause(30);
    await drag('HANDLE', await point('境界候補を長押しして移動'));
    assert.deepEqual(await order(root_name), ['handle', 'boundary']);
    assert.equal(await evaluate('window.settings.node_label_visibility.handle'), true);
    await mouse('mousePressed', await point('境界候補を先頭へ移動'));
    await mouse('mouseReleased', await point('境界候補を先頭へ移動'));
    await pause(30);
    assert.deepEqual(await order(root_name), ['boundary', 'handle']);
    await check_overlap();
    await check_frame();
    await evaluate('document.querySelector("summary").click()');
    const child_name = '境界候補の原因別優先順位';
    const target = await point('遮蔽の証拠を長押しして移動'); target.y -= 8;
    await drag('原因不明', target);
    assert.deepEqual(await order(child_name), ['boundary_fov', 'boundary_unknown', 'boundary_occlusion', 'boundary_free_space']);
    await drag('HANDLE', { x: 2, y: 2 });
    assert.deepEqual(await order(root_name), ['boundary', 'handle']);
    await drag('HANDLE', await point('境界候補を長押しして移動'), 420, true);
    assert.deepEqual(await order(root_name), ['boundary', 'handle']);
    assert.equal(await evaluate('document.querySelector("[role=dialog]").scrollWidth > document.querySelector("[role=dialog]").clientWidth'), false);
    const touch_start = await point('HANDLEを長押しして移動');
    const touch_end = await point('境界候補を長押しして移動');
    await call('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ ...touch_start, id: 1 }] });
    await pause(420);
    await call('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ ...touch_end, id: 1 }] });
    await call('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await pause(30);
    assert.deepEqual(await order(root_name), ['handle', 'boundary']);
    await call('Emulation.setDeviceMetricsOverride', { width: 320, height: 900, deviceScaleFactor: 1, mobile: false });
    assert.equal(await evaluate('document.querySelector("[role=dialog]").scrollWidth > document.querySelector("[role=dialog]").clientWidth'), false);
    await mouse('mousePressed', await point('HANDLEを長押しして移動'));
    await evaluate('window.unmount()');
    await pause(420);
    await mouse('mouseReleased', { x: 2, y: 2 });
    assert.equal(await evaluate('document.querySelector("[role=dialog]")'), null);
    if (process.argv.includes('--live')) {
        // 配信中のViteモジュールを直接使用。継続的な親再描画下の追加検査。
        await call('Page.navigate', { url: 'http://localhost:5173' });
        await pause(1000);
        await evaluate(`(async()=>{
            const {default:React}=await import('/node_modules/.vite/deps/react.js');
            const {default:ReactDOM}=await import('/node_modules/.vite/deps/react-dom_client.js');
            const {GngLabelModal}=await import('/src/features/visualization/GngLabelModal.tsx');
            const host=document.createElement('div');document.body.appendChild(host);
            function App(){
                const [settings,set_settings]=React.useState({});
                const [tick,set_tick]=React.useState(0);
                React.useEffect(()=>{const timer=setInterval(()=>set_tick(n=>n+1),30);return()=>clearInterval(timer)},[]);
                window.settings=settings;window.render_tick=tick;
                return React.createElement(GngLabelModal,{open:true,visibleLabels:{0:false,1:false,2:false,3:false,4:false,5:false},
                    label_settings:settings,onClose:()=>{},onUpdate:value=>set_settings(current=>({...current,...value}))});
            }
            const root=ReactDOM.createRoot(host);window.unmount=()=>{root.unmount();host.remove()};root.render(React.createElement(App));
        })()`);
        await pause(150);
        assert.deepEqual(await order(root_name), ['boundary', 'handle']);
        await drag('HANDLE', await point('境界候補を長押しして移動'), 30);
        assert.deepEqual(await order(root_name), ['handle', 'boundary']);
        await mouse('mousePressed', await point('境界候補を先頭へ移動'));
        await mouse('mouseReleased', await point('境界候補を先頭へ移動'));
        await pause(30);
        await drag('HANDLE', await point('境界候補を長押しして移動'));
        assert.deepEqual(await order(root_name), ['handle', 'boundary']);
        assert.ok(await evaluate('window.render_tick') > 10);
        await mouse('mousePressed', await point('境界候補を先頭へ移動'));
        await mouse('mouseReleased', await point('境界候補を先頭へ移動'));
        await pause(30);
        await check_overlap();
        await check_frame();
        await evaluate('window.unmount()');
        console.log('label_priority_live_modules=passed');
    }
    console.log('label_priority_browser=passed');
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
    console.log('検証Chrome: 停止済み・一時プロファイル削除済み');
}
