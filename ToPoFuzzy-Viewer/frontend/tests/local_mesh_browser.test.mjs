import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { build } from 'esbuild';

const profile = await mkdtemp(join(tmpdir(), 'local-mesh-browser-'));
const pending = new Map();
let browser;
let session_id;
let next_id = 0;
const pause = ms => new Promise(done => setTimeout(done, ms));
try {
    const bundle = await build({ stdin: { resolveDir: process.cwd(), loader: 'tsx', contents: `
        import React from 'react';
        import {createRoot} from 'react-dom/client';
        import {Canvas,useThree} from '@react-three/fiber';
        import {OrbitControls} from '@react-three/drei';
        import * as THREE from 'three';
        import {useLocalMeshes} from './src/features/meshes/use_local_meshes';
        import {LocalMeshPanel} from './src/features/meshes/LocalMeshPanel';
        import {LocalMeshRenderer} from './src/features/meshes/LocalMeshRenderer';
        function Probe() { window.scene_state=useThree(); return null; }
        function App() {
            const meshes=useLocalMeshes(); window.meshes=meshes;
            return <><div style={{width:360,height:600,overflow:'auto',float:'left'}}><LocalMeshPanel meshes={meshes}/></div>
                <div style={{width:720,height:600,float:'left'}}><Canvas frameloop="demand"
                    camera={{position:[5,5,5],up:[0,0,1],fov:50}} gl={{preserveDrawingBuffer:true}}
                    onCreated={({gl})=>gl.setClearColor('#101018')}>
                    <Probe/><ambientLight intensity={0.3}/><pointLight position={[10,10,10]} intensity={0.5}/>
                    <pointLight position={[-10,-10,-10]} intensity={0.3}/>
                    {meshes.items.map(item=><LocalMeshRenderer key={item.id} item={item}
                        focus_req={meshes.focus?.id===item.id?meshes.focus.req:undefined}/>)}
                    <OrbitControls makeDefault/>
                </Canvas></div></>;
        }
        const root=createRoot(document.getElementById('app')); root.render(<App/>);
        window.unmount=()=>root.unmount();
        window.snapshot=()=>{
            const {gl,scene,camera}=window.scene_state; gl.render(scene,camera);
            const context=gl.getContext(), pixels=new Uint8Array(720*600*4);
            context.readPixels(0,0,720,600,context.RGBA,context.UNSIGNED_BYTE,pixels);
            let lit=0; for(let idx=0;idx<pixels.length;idx+=4) if(Math.max(...pixels.subarray(idx,idx+3))>60) lit++;
            const geometry=[]; scene.traverse(o=>{if(o.isMesh) geometry.push({
                pos:o.getWorldPosition(new THREE.Vector3()).toArray(),
                scale:o.getWorldScale(new THREE.Vector3()).toArray(),
                has_texture:(Array.isArray(o.material)?o.material:[o.material]).some(m=>Boolean(m.map))
            });});
            return {lit,geometry,memory:gl.info.memory,error:window.meshes.error,count:window.meshes.items.length,
                camera:camera.position.toArray(),target:window.scene_state.controls?.target.toArray(),
                models:window.meshes.items.map(item=>{const box=new THREE.Box3().setFromObject(item.asset.object);
                    return {bounds:[box.min.toArray(),box.max.toArray()],num_triangles:item.asset.num_triangles};})};
        };
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
    await call('Emulation.setDeviceMetricsOverride', { width: 1080, height: 600, deviceScaleFactor: 1, mobile: false });
    const { frameTree } = await call('Page.getFrameTree');
    await call('Page.setDocumentContent', { frameId: frameTree.frame.id,
        html: '<html><body style="margin:0"><div id="app"></div></body></html>' });
    const evaluate = async expression => {
        const result = await call('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
        if (result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
        return result.result.value;
    };
    await evaluate(bundle.outputFiles[0].text);
    for (let iter=0; iter<100 && !await evaluate('Boolean(window.scene_state)'); iter++) await pause(50);
    assert.ok(await evaluate('Boolean(window.scene_state)'), 'Canvas初期化');
    const load_files = async paths => {
        const { root } = await call('DOM.getDocument');
        const { nodeId } = await call('DOM.querySelector', { nodeId: root.nodeId, selector: 'input[aria-label="メッシュファイル選択"]' });
        await call('DOM.setFileInputFiles', { nodeId, files: paths });
        await pause(80);
        await evaluate("[...document.querySelectorAll('button')].find(b=>b.textContent==='追加').click()");
        for (let iter=0; iter<200; iter++) {
            await pause(100);
            if (await evaluate('!window.meshes.is_loading')) return evaluate('window.snapshot()');
        }
        throw new Error('メッシュ読み込みの制限時間超過');
    };
    const fixture = join(profile, 'fixture.obj');
    await writeFile(fixture, 'v 0 0 0\nv 1000 0 0\nv 0 1000 0\nf 1 2 3\n');
    const first = await load_files([fixture]);
    assert.equal(first.count, 1);
    assert.ok(first.lit > 100, '面のピクセル描画');
    await evaluate("window.meshes.update(window.meshes.items[0].id,{unit_scale:0.001,transform:{position:[2,3,4],rotation:[0,0,0],scale:[1,1,1]}})");
    await pause(80);
    const changed = await evaluate('window.snapshot()');
    assert.deepEqual(changed.geometry[0].pos, [2,3,4]);
    assert.deepEqual(changed.geometry[0].scale, [0.001,0.001,0.001]);
    assert.equal(await evaluate('window.meshes.items[0].asset.object.children[0].geometry.attributes.position.array[3]'), 1000, '原本頂点の保持');
    await evaluate('window.meshes.update(window.meshes.items[0].id,{transform:{position:[2,3,4],rotation:[0,0,Math.PI/2],scale:[1,1,1]}})');
    await pause(80);
    const rotated = await evaluate('window.meshes.items[0].asset.object.localToWorld(window.meshes.items[0].asset.object.position.clone().set(1000,0,0)).toArray()');
    rotated.forEach((value, idx) => assert.ok(Math.abs(value - [2,4,4][idx]) < 1e-6, '表示回転と単位換算'));
    await evaluate('window.meshes.focus_item(window.meshes.items[0].id)');
    await pause(80);
    assert.ok((await evaluate('window.snapshot()')).lit > 100);
    await evaluate('window.meshes.update(window.meshes.items[0].id,{is_visible:false})');
    await pause(80);
    assert.equal((await evaluate('window.snapshot()')).lit, 0, '非表示');
    const clear = async () => {
        await evaluate('window.meshes.items.forEach(item=>window.meshes.remove(item.id))');
        await pause(100);
        const state = await evaluate('window.snapshot()');
        assert.equal(state.count, 0);
        assert.equal(state.geometry.length, 0);
        assert.equal(state.memory.geometries, 0, 'GPU形状の解放');
        assert.equal(state.memory.textures, 0, 'GPU画像の解放');
        return state;
    };
    await clear();
    // 外部BINを伴うglTFと不足ファイルの検証
    const positions = new Float32Array([0,0,0, 1,0,0, 0,1,0]);
    const gltf_path = join(profile, 'triangle.gltf');
    const bin_path = join(profile, 'triangle.bin');
    await writeFile(bin_path, Buffer.from(positions.buffer));
    await writeFile(gltf_path, JSON.stringify({ asset:{version:'2.0'}, scene:0,
        scenes:[{nodes:[0]}], nodes:[{mesh:0}], meshes:[{primitives:[{attributes:{POSITION:0}}]}],
        buffers:[{uri:'triangle.bin',byteLength:36}], bufferViews:[{buffer:0,byteLength:36}],
        accessors:[{bufferView:0,componentType:5126,count:3,type:'VEC3',min:[0,0,0],max:[1,1,0]}] }));
    const gltf_state = await load_files([gltf_path, bin_path]);
    assert.equal(gltf_state.count, 1);
    assert.ok(gltf_state.lit > 100, 'glTF外部BINの実描画');
    await clear();
    const missing = await load_files([gltf_path]);
    assert.equal(missing.count, 0);
    assert.match(missing.error, /triangle.bin/);
    console.log('PASS glTF外部BIN・不足ファイルのエラー表示');
    if (process.env.VEHICLE_MODELS) {
        const base = process.env.VEHICLE_MODELS;
        for (const paths of [
            ['opengameart/fancy_motorcycle/bike.obj'],
            ['poly_pizza/bicycle/bicycle.glb'],
            ['mirageym/road_bike/RoadBike_SubDiv.fbx'],
            ['artec3d/motorbike/ply/Motorbike_ply.ply'],
            ['artec3d/motorbike/obj/Bike.obj', 'artec3d/motorbike/obj/Bike.mtl', 'artec3d/motorbike/obj/Bike_0.png'],
        ]) {
            const state = await load_files(paths.map(path => resolve(base, path)));
            assert.equal(state.error, '', paths[0]);
            assert.equal(state.count, 1, paths[0]);
            if (state.lit <= 100) {
                console.log('描画診断', paths[0], { ...state, geometry: state.geometry.slice(0, 3) });
                console.log(await evaluate(`window.meshes.items.map(item=>{
                    const materials=[]; item.asset.object.traverse(o=>{if(o.isMesh) (Array.isArray(o.material)?o.material:[o.material]).forEach(m=>materials.push({type:m.type,color:m.color.toArray(),opacity:m.opacity,visible:m.visible,side:m.side}));});
                    return {materials:materials.slice(0,3)};})`));
                const shot = await call('Page.captureScreenshot', { format: 'png' });
                await writeFile('/tmp/local_mesh_failure.png', Buffer.from(shot.data, 'base64'));
            }
            assert.ok(state.lit > 100, paths[0] + ' 実ピクセル描画');
            if (paths.length > 1) assert.ok(state.geometry.some(mesh => mesh.has_texture), 'OBJ材質・PNGの適用');
            console.log('PASS 実モデル描画', paths[0], state.lit, 'pixels', state.memory);
            if (process.env.MESH_SCREENSHOT && paths.length > 1) {
                const shot = await call('Page.captureScreenshot', { format: 'png' });
                await writeFile(process.env.MESH_SCREENSHOT, Buffer.from(shot.data, 'base64'));
            }
            await clear();
        }
        const large = await load_files([resolve(base, 'artec3d/semi_trailer_truck/extracted/truck/Truck.obj')]);
        assert.equal(large.count, 0);
        assert.match(large.error, /サイズ上限/);
        console.log('PASS 大容量トラックの読込前拒否');
    }
    // 原本から生成した軽量GLBの単位・座標・材質・単独ファイル読込の検証
    if (process.env.MESH_PREVIEW) {
        const preview_path = resolve(process.env.MESH_PREVIEW);
        const manifest = JSON.parse(await readFile(join(dirname(preview_path), 'preview.json'), 'utf8'));
        const state = await load_files([preview_path]);
        assert.equal(state.error, '');
        assert.equal(state.count, 1);
        assert.ok(state.lit > 100, '軽量GLBの実描画');
        assert.ok(state.geometry.some(mesh => mesh.has_texture), 'GLB内包テクスチャ');
        assert.equal(state.models[0].num_triangles, manifest.preview_num_faces);
        state.models[0].bounds.flat().forEach((value, idx) =>
            assert.ok(Math.abs(value - manifest.preview_bounds.flat()[idx]) < 1e-4, '表示用GLBの座標範囲'));
        assert.ok(Math.hypot(...state.camera) < 100, 'm単位の表示・遠方へのカメラ移動なし');
        console.log('PASS 軽量GLBの実描画・内包画像・座標・面数', state.models[0], state.lit, 'pixels');
        if (process.env.MESH_PREVIEW_SCREENSHOT) {
            const shot = await call('Page.captureScreenshot', { format: 'png' });
            await writeFile(process.env.MESH_PREVIEW_SCREENSHOT, Buffer.from(shot.data, 'base64'));
        }
        await clear();
    }
    await evaluate('window.unmount()');
    console.log('PASS メッシュGUI・単位・位置・非表示・削除・GPU解放');
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
