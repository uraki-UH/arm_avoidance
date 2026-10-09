// 標準・軽量描画の切替と全ノード・辺・選択対象の維持確認。
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { build } from 'esbuild';

const profile = await mkdtemp(join(tmpdir(), 'graph-geometry-browser-'));
const pending = new Map();
let browser;
let session_id;
let next_id = 0;
const pause = ms => new Promise(done => setTimeout(done, ms));
try {
    const bundle = await build({ stdin: { resolveDir: process.cwd(), loader: 'tsx', contents: `
        import React from 'react';
        import {createRoot,extend,flushSync} from '@react-three/fiber';
        import * as THREE from 'three';
        import {GraphRenderer} from './src/features/visualization/GraphRenderer';
        import {createDefaultGraphLayerSettings} from './src/features/visualization/graphLayerSettings';
        extend(THREE);
        const canvas=document.createElement('canvas');document.body.appendChild(canvas);
        const renderer=new THREE.WebGLRenderer({canvas,antialias:false,preserveDrawingBuffer:true});
        renderer.setSize(400,400);renderer.setClearColor('#101820');
        const scene=new THREE.Scene(),camera=new THREE.OrthographicCamera(-1,5,4,-1,.1,100);
        camera.manual=true;camera.position.set(0,0,20);camera.lookAt(0,0,0);camera.updateMatrixWorld(true);
        const root=createRoot(canvas);root.configure({gl:renderer,scene,camera,frameloop:'never',size:{width:400,height:400,top:0,left:0}});
        const tag='/test/graph';
        let graph={timestamp:0,frameId:'world',nodes:Array.from({length:20},(_,idx)=>({id:idx,x:idx%5,y:Math.floor(idx/5),z:0,nx:0,ny:0,nz:1,label:1,age:0})),edges:Array.from({length:19},(_,idx)=>[idx,idx+1]).flat(),clusters:[]};
        window.measure=async(enable_simple_graph,enable_motion=false)=>{
            if(enable_motion)graph={...graph,timestamp:1,nodes:graph.nodes.map((node,idx)=>idx===0?{...node,x:node.x+.4}:node)};
            const settings={...createDefaultGraphLayerSettings(tag,graph),nodeScale:.1,edgeWidth:.04,enable_simple_graph};
            flushSync(()=>root.render(React.createElement(GraphRenderer,{tag,data:graph,settings})));
            await new Promise(done=>setTimeout(done,100));renderer.render(scene,camera);renderer.getContext().finish();
            let num_instances=0,num_triangles=0;const matrices=[],colors=[],node_ids=[];
            scene.traverse(object=>{if(!object.isInstancedMesh)return;num_instances+=object.count;num_triangles+=object.count*object.geometry.index.count/3;
                matrices.push(...object.instanceMatrix.array.slice(0,object.count*16));if(object.instanceColor)colors.push(...object.instanceColor.array.slice(0,object.count*3));
                for(const node of object.userData.pick_nodes??[])node_ids.push(node.id);
            });
            const pixels=new Uint8Array(400*400*4),context=renderer.getContext();context.readPixels(0,0,400,400,context.RGBA,context.UNSIGNED_BYTE,pixels);
            let num_colored_pixels=0;for(let idx=0;idx<pixels.length;idx+=4)if(pixels[idx]!==pixels[0]||pixels[idx+1]!==pixels[1]||pixels[idx+2]!==pixels[2])num_colored_pixels++;
            let is_pixel_equal=true;if(!window.reference_pixels)window.reference_pixels=pixels;else is_pixel_equal=pixels.every((value,idx)=>value===window.reference_pixels[idx]);
            return {num_instances,num_triangles,matrices,colors,node_ids,num_colored_pixels,is_pixel_equal};
        };
        window.cleanup=()=>{root.unmount();renderer.dispose();renderer.forceContextLoss();};
    ` }, bundle:true,nodePaths:[resolve('node_modules')],write:false,format:'iife',jsx:'automatic',define:{'process.env.NODE_ENV':'"production"'} });
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
    const standard=await evaluate('window.measure(false)'),compact=await evaluate('window.measure(true)'),restored=await evaluate('window.measure(false)');
    assert.equal(standard.num_instances,39);assert.equal(compact.num_instances,39);
    assert.ok(compact.num_triangles<standard.num_triangles*.5);
    assert.deepEqual(compact.matrices,standard.matrices);assert.deepEqual(compact.colors,standard.colors);assert.deepEqual(compact.node_ids,standard.node_ids);
    assert.ok(standard.num_colored_pixels>100&&compact.num_colored_pixels>100);
    assert.ok(restored.is_pixel_equal,'標準表示へ戻した際のピクセル不一致');
    const moved=await evaluate('window.measure(false,true)');assert.notDeepEqual(moved.matrices,standard.matrices);assert.equal(moved.is_pixel_equal,false);
    const summary={standard_triangles:standard.num_triangles,compact_triangles:compact.num_triangles,num_instances:compact.num_instances,is_pixel_equal:restored.is_pixel_equal};
    console.log(JSON.stringify(summary));if(process.argv[2])await writeFile(process.argv[2],JSON.stringify(summary,null,2)+'\n');
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
    await rm(profile, { recursive: true, force: true, maxRetries:5, retryDelay:200 });
    console.log('Chrome停止済み・専用プロファイル削除済み');
}
