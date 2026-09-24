import assert from 'node:assert/strict';
import { execFileSync, spawn } from 'node:child_process';
import { mkdtemp, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { build } from 'esbuild';

const base_ref = '51c154c2df675601242547b0eed3ac72660be0ed';
const profile = await mkdtemp(join(tmpdir(), 'cluster-render-browser-'));
const pending = new Map();
let browser;
let session_id;
let next_id = 0;
const pause = ms => new Promise(done => setTimeout(done, ms));
try {
    const baseline = execFileSync('git', ['show', `${base_ref}:ToPoFuzzy-Viewer/frontend/src/features/visualization/GraphRenderer.tsx`], { encoding: 'utf8' });
    const bundle = await build({ stdin: { resolveDir: process.cwd(), loader: 'tsx', contents: `
        import React from 'react';
        import {createRoot,extend,flushSync} from '@react-three/fiber';
        import * as THREE from 'three';
        import {GraphRenderer as Before} from 'cluster-baseline';
        import {GraphRenderer as After} from './src/features/visualization/GraphRenderer';
        import {createDefaultGraphLayerSettings} from './src/features/visualization/graphLayerSettings';
        extend(THREE);
        const canvas=document.createElement('canvas');document.body.appendChild(canvas);
        const renderer=new THREE.WebGLRenderer({canvas,antialias:false,preserveDrawingBuffer:true});
        renderer.setSize(400,400); renderer.setClearColor('#101820');
        const scene=new THREE.Scene(),camera=new THREE.OrthographicCamera(-1,33,33,-1,0.1,100);
        camera.manual=true; camera.position.set(0,0,20);camera.lookAt(0,0,0);camera.updateMatrixWorld(true);
        const root=createRoot(canvas);root.configure({gl:renderer,scene,camera,frameloop:'never',size:{width:400,height:400,top:0,left:0}});
        const props={tag:'/topological_map',enableClusterSelection:false};
        const make_graph=(num,frame)=>({timestamp:frame,nodes:[],edges:[],clusters:Array.from({length:num},(_,id)=>({
            id,label:id%6,pos:[id%32+frame%2*0.002,Math.floor(id/32),0],scale:[0.7,0.6,0.5],
            quat:[0,0,0,1],velocity:[id%3===0?0:0.05*(id%7),0,0],nodeIds:[]}))});
        let store;
        const context=renderer.getContext();
        function draw(){renderer.render(scene,camera);context.finish();}
        function update(component,graph,settings){flushSync(()=>{store=root.render(React.createElement(component,{...props,data:graph,settings}));});}
        window.measure=async(mode,num,enable_velocity)=>{
            flushSync(()=>root.render(null));await new Promise(done=>setTimeout(done,50));
            const component=mode==='before'?Before:After;
            let graph=make_graph(num,0);
            const settings={...createDefaultGraphLayerSettings(props.tag,graph),showNodes:false,showEdges:false,
                showClusters:true,showNormals:false,showVelocity:enable_velocity};
            update(component,graph,settings);await new Promise(done=>setTimeout(done,50));draw();
            const updates=[],renders=[];
            for(let iter=0;iter<40;iter++){
                graph=make_graph(num,iter);
                let start=performance.now();update(component,graph,settings);let update_ms=performance.now()-start;
                start=performance.now();draw();let render_ms=performance.now()-start;
                if(iter>=10){updates.push(update_ms);renders.push(render_ms);}
            }
            update(component,make_graph(num,0),settings);draw();
            const pixels=new Uint8Array(400*400*4);context.readPixels(0,0,400,400,context.RGBA,context.UNSIGNED_BYTE,pixels);
            let num_colored_pixels=0;
            for(let idx=0;idx<pixels.length;idx+=4) if(pixels[idx]!==pixels[0] || pixels[idx+1]!==pixels[1] || pixels[idx+2]!==pixels[2]) num_colored_pixels++;
            let pixel_dev=0,max_pixel_dev=0;
            if(mode==='before')window.reference_pixels=pixels;
            else for(let idx=0;idx<pixels.length;idx++){
                const dev=Math.abs(pixels[idx]-window.reference_pixels[idx]);pixel_dev+=dev;max_pixel_dev=Math.max(max_pixel_dev,dev);
            }
            let num_meshes=0;scene.traverse(object=>{if(object.isMesh&&object.visible)num_meshes++;});
            const median=values=>[...values].sort((a,b)=>a-b)[Math.floor(values.length/2)];
            return {mode,num,enable_velocity,update_ms:median(updates),render_ms:median(renders),
                calls:renderer.info.render.calls,num_meshes,num_cluster_interactions:store.getState().internal.interaction.filter(object=>
                    object.userData.inspection_selection?.kind==='cluster'||object.userData.pick_clusters).length,num_colored_pixels,
                mean_pixel_dev:pixel_dev/pixels.length,max_pixel_dev};
        };
        window.cleanup=()=>{root.unmount();renderer.dispose();renderer.forceContextLoss();};
    ` }, bundle: true, write: false, format: 'iife', jsx: 'automatic',
        define: { 'process.env.NODE_ENV': '"production"' },
        plugins: [{ name: 'baseline', setup(plugin) {
            plugin.onResolve({ filter: /^cluster-baseline$/ }, () => ({ path: 'baseline', namespace: 'baseline' }));
            plugin.onLoad({ filter: /.*/, namespace: 'baseline' }, () => ({ contents: baseline, loader: 'tsx',
                resolveDir: resolve('src/features/visualization') }));
        } }] });
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
        const timer = setTimeout(() => { pending.delete(id); fail(new Error('CDP timeout: ' + method)); }, 45000);
        pending.set(id, { resolve: value => { clearTimeout(timer); done(value); }, reject: error => { clearTimeout(timer); fail(error); } });
        browser.stdio[3].write(JSON.stringify({ id, method, params, sessionId: session }) + '\0');
    });
    const { targetId } = await call('Target.createTarget', { url: 'about:blank' });
    session_id = (await call('Target.attachToTarget', { targetId, flatten: true })).sessionId;
    const evaluate = async expression => {
        const result = await call('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
        if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
        return result.result.value;
    };
    await evaluate(bundle.outputFiles[0].text);
    const results = [];
    for (const num of [100, 1000]) for (const enable_velocity of [false, true]) {
        const before = await evaluate(`window.measure('before',${num},${enable_velocity})`);
        const after = await evaluate(`window.measure('after',${num},${enable_velocity})`);
        console.log(JSON.stringify({ before, after }));
        assert.equal(before.num_cluster_interactions, num);
        assert.equal(after.num_cluster_interactions, 0);
        assert.ok(before.num_colored_pixels > 100);
        assert.ok(after.num_colored_pixels > 100);
        assert.ok(after.calls <= 6);
        assert.ok(after.calls < before.calls);
        assert.ok(after.mean_pixel_dev < 0.5, '分離配置の表示色・形状を維持');
        results.push({ before, after });
    }
    if (process.argv[2]) await writeFile(process.argv[2], JSON.stringify({ base_ref,
        conditions: 'Chrome headless / SwiftShader / 400x400 / 合成100・1000クラスタ / 30更新の中央値 / ノード・エッジOFF', results }, null, 2) + '\n');
    await evaluate('window.cleanup()');
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
