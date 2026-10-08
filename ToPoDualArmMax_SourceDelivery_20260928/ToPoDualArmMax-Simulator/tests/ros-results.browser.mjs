import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import fs from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

// 専用サーバー・Chrome・ROSドメインによる表示統合と効率の試験。
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const directory=await fs.mkdtemp('/tmp/topo-ros-ui-');
const output_directory=process.argv[2]?path.dirname(path.resolve(process.argv[2])):directory;
await fs.mkdir(output_directory,{recursive:true});
const is_mesa=process.env.TOPO_TEST_GPU==='mesa';
const chrome_env={...process.env};
if(is_mesa){delete chrome_env.LD_LIBRARY_PATH;Object.assign(chrome_env,{__NV_PRIME_RENDER_OFFLOAD:'0',__GLX_VENDOR_LIBRARY_NAME:'mesa',__EGL_VENDOR_LIBRARY_FILENAMES:'/usr/share/glvnd/egl_vendor.d/50_mesa.json'});}
const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');
const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
const server=spawn(process.execPath,['app/server.mjs'],{cwd:root,env:{...process.env,PORT:String(port)},stdio:'ignore'});
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let chrome,ws,fixture;
let fixture_log='';
const stopped=async process=>{
 if(!process||process.exitCode!==null||process.signalCode!==null)return;
 const exited=once(process,'exit');process.kill('SIGTERM');
 let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,5000);})]);clearTimeout(timer);
 if(process.exitCode===null&&process.signalCode===null){process.kill('SIGKILL');await exited;}
};
try{
 for(let i=0;i<50;i++){try{if((await fetch(`http://127.0.0.1:${port}/api/health`)).ok)break;}catch{}await pause(100);}
 chrome=spawn(process.env.CHROME_BIN||'/usr/bin/google-chrome',[
  '--headless=new','--disable-background-timer-throttling','--disable-renderer-backgrounding','--no-first-run','--disable-dev-shm-usage',...(is_mesa?['--use-gl=angle','--use-angle=gl-egl']:['--use-angle=swiftshader','--enable-unsafe-swiftshader']),'--window-size=1440,1000','--remote-debugging-port=0',
  '--user-data-dir='+directory,`http://127.0.0.1:${port}/?model=long`],{stdio:'ignore',env:chrome_env});
 let pages;
 for(let i=0;i<100;i++){try{const debug_port=(await fs.readFile(directory+'/DevToolsActivePort','utf8')).split('\n')[0];pages=await(await fetch(`http://127.0.0.1:${debug_port}/json`)).json();if(pages.some(p=>p.type==='page'))break;}catch{}await pause(100);}
 assert.ok(pages,'Chromeの起動失敗');
 ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
 const measure_frames=async()=>evaluate(`new Promise(resolve=>{const values=[];let last=performance.now();const before=simulator.renderer.info.memory.geometries;const frame=()=>{const now=performance.now();values.push(now-last);last=now;if(values.length<${is_mesa?240:16})requestAnimationFrame(frame);else{values.sort((a,b)=>a-b);resolve({frame_p50_ms:values[Math.floor(values.length*.5)],frame_p95_ms:values[Math.floor(values.length*.95)],geometry_growth:simulator.renderer.info.memory.geometries-before,num_frames:values.length});}};requestAnimationFrame(frame);})`);
 const exceptions=[];let seq=0;const requests=new Map();
 ws.onmessage=event=>{const value=JSON.parse(event.data);if(value.method==='Runtime.exceptionThrown')exceptions.push(value.params);if(value.id){requests.get(value.id)?.(value);requests.delete(value.id);}};
 const call=(method,params={})=>new Promise((resolve,reject)=>{
  const id=++seq,timer=setTimeout(()=>{requests.delete(id);reject(Error('CDP時間超過: '+method));},45000);
  requests.set(id,value=>{clearTimeout(timer);value.error?reject(Error(JSON.stringify(value.error))):resolve(value.result);});
  ws.send(JSON.stringify({id,method,params}));
 });
 const evaluate=async expression=>{
  let result;try{result=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});}catch(error){throw Error(String(error)+' expression='+expression.slice(0,180));}
  if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;
 };
 let is_ready=false;
 for(let i=0;i<150;i++){is_ready=await evaluate('!!window.simulator?.diagnostics.ready');if(is_ready)break;await pause(200);}
 assert.ok(is_ready,'アプリ起動失敗');console.log('READY: browser');
 const renderer_name=await evaluate(`(()=>{const gl=simulator.renderer.getContext(),ext=gl.getExtension('WEBGL_debug_renderer_info');return ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):'unknown';})()`);
 console.log('RENDERER: '+renderer_name);
 if(is_mesa)assert.ok(!/SwiftShader|llvmpipe|softpipe|unknown/i.test(renderer_name),'実GPU描画が利用できません');

 const baseline=await measure_frames();
 const probe_ws=net.createServer();probe_ws.listen(0,'127.0.0.1');await once(probe_ws,'listening');
 const ws_port=probe_ws.address().port;await new Promise(resolve=>probe_ws.close(resolve));
 const origin=`http://127.0.0.1:${port}`;
 fixture=spawn('docker',['exec','-i','-e','ROS_DOMAIN_ID=187','-e','ROS_LOCALHOST_ONLY=1','-e','PYTHONDONTWRITEBYTECODE=1','gng_cpu_container','bash','-lc',
  `source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/tests/ros_results_fixture.py --port ${ws_port} --origin ${origin}`],{stdio:['pipe','pipe','pipe']});
 fixture.stdout.on('data',data=>fixture_log+=data);fixture.stderr.on('data',data=>fixture_log+=data);
 await call('Runtime.enable');
 await pause(2500);
 const wait_for=async(expression)=>{for(let idx=0;idx<100;idx++){const value=await evaluate(expression);if(value)return value;await pause(100);}throw Error('待機超過: '+expression+' '+fixture_log);};
 await evaluate(`document.querySelector('[data-panel=ros]').click();simulator.ros_results.open()`);
 await wait_for('!!simulator.ros_results.api');
 await evaluate(`(()=>{const input=document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="ROS結果の接続先"]');input.value='ws://127.0.0.1:${ws_port}/observe';input.dispatchEvent(new FocusEvent('focusout',{bubbles:true,composed:true}));})()`);
 await wait_for(`simulator.ros_results.api.isConnected&&simulator.ros_results.api.sources.some(source=>source.id==='/test/Tmap_robot')`);
 console.log('READY: Viewer sources');
 // 実際のTopicsチェックボックスで購読開始。
 await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelectorAll('input[type=checkbox][aria-label^="Topic: "]').forEach(input=>{if(!input.checked)input.click();})`);
 const summary=await wait_for(`(()=>{const a=simulator.ros_results.api;return Object.keys(a.graphData).length>=6&&Object.keys(a.pointClouds).length&&Object.keys(a.markerData).length>=2&&Object.keys(a.voxelData).length&&Object.keys(a.robotData).length?{graphs:Object.keys(a.graphData),points:Object.keys(a.pointClouds),markers:Object.keys(a.markerData),voxels:Object.keys(a.voxelData),robots:Object.keys(a.robotData)}:null})()`);
 await wait_for(`!!simulator.ros_results.scene.root_scene.getObjectByName('/topological_map')?.userData.has_transform`);
 await pause(700);
 const initial=await evaluate(`(()=>{const s=simulator.ros_results.scene.root_scene;const objects=[];s.traverse(o=>{if(o.isMesh||o.isPoints||o.isLine||o.isSprite)objects.push({type:o.type,mask:o.layers.mask});});return {objects,transform:s.getObjectByName('/topological_map').matrix.elements[12],canvas_count:document.querySelectorAll('#canvas-host canvas').length};})()`);
 assert.equal(initial.transform,.25);assert.equal(initial.canvas_count,1);
 assert.ok(initial.objects.length>10,JSON.stringify(initial));
 assert.ok(initial.objects.every(o=>o.mask===8),JSON.stringify(initial));
 await wait_for(`(()=>{const group=simulator.ros_results.scene.root_scene.getObjectByName('/test/Tmap_robot');const link=simulator.robot.links.base_link;return group?.visible&&group.userData.has_transform&&group.matrix.elements.every((value,idx)=>Math.abs(value-link.matrixWorld.elements[idx])<1e-8);})()`);
 console.log('PASS: namespaced robot map aligned to the selected Simulator model');
 console.log('PASS: graph, point cloud, markers, poses, voxel, robot, TF and sensor-layer isolation');
 const robot_variants=await evaluate(`(()=>{const layer=document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="表示: test_robot"]').closest('.surface-muted');const labels=['Collision','Manip'];for(const label of labels){const button=[...layer.querySelectorAll('button')].find(b=>b.textContent.trim()===label);if(!button)throw Error(label);button.click();}return labels;})()`);
 await pause(300);console.log('PASS: robot '+robot_variants.join(', '));

 // Connection & StreamsのTopicsだけで表示を切替。Scene Layersは従来の目アイコン。
 assert.equal(await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelectorAll('input[type="checkbox"][aria-label^="表示:"]').length`),0);
 for (const [topic, name] of [['/topological_map','/topological_map'],['/test/points','/test/points'],['/test/markers','/test/markers-markers'],['/test/voxels','/test/voxels']]) {
  await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="Topic: ${topic}"]').click()`);
  await wait_for(`!simulator.ros_results.scene.root_scene.getObjectByName('${name}')`);
  await evaluate('simulator.ros_results.api.getSources()');
  assert.equal(await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="Topic: ${topic}"]').checked`),false);
  assert.ok(await evaluate(`!!simulator.ros_results.scene.root_scene.getObjectByName('/grasp_pose_cands/Tmap')`));
  await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="Topic: ${topic}"]').click()`);
  await wait_for(`!!simulator.ros_results.scene.root_scene.getObjectByName('${name}')`);
 }
 console.log('PASS: Connection & Streams topic checkboxes, refresh and per-topic visibility');
 const variants=await evaluate(`(()=>{const root=document.querySelector('#ros-results').shadowRoot;const layer=root.querySelector('[aria-label="表示: /topological_map"]').closest('.surface-muted');const labels=['Normals','Clusters','Ellipses'];for(const name of labels){const button=[...layer.querySelectorAll('button')].find(b=>b.textContent.trim()===name);if(!button)throw Error(name);button.click();}return labels;})()`);
 await pause(300);console.log('PASS: Scene Layers display settings: '+variants.join(', '));
 // 読取専用の詳細計算。永続データへの編集操作は別途拒否確認。
 const inspection=await evaluate(`simulator.ros_results.api.inspect_graph('/grasp_pose_cands/Tmap',{kind:'cluster',id:7},simulator.ros_results.api.graphData['/grasp_pose_cands/Tmap'])`);
 assert.equal(inspection.graph.nodes.length,64);
 const registration=await evaluate(`simulator.ros_results.api.register_vehicle(${JSON.stringify(inspection)},.25,.35)`);
 assert.equal(registration.candidates.length,4);console.log('PASS: inspection and geometric model matching');
 // 画面上の候補クリックによる共有詳細パネルの表示と終了。
 const pointer=await evaluate(`(async()=>{const THREE=await import('three');const p=new THREE.Vector3(.425,.115,.2).project(simulator.camera);const rect=simulator.renderer.domElement.getBoundingClientRect();return {x:rect.left+(p.x+1)*rect.width/2,y:rect.top+(1-p.y)*rect.height/2};})()`);
 await call('Input.dispatchMouseEvent',{type:'mouseMoved',...pointer});await pause(1800);
 await call('Input.dispatchMouseEvent',{type:'mousePressed',...pointer,button:'left',clickCount:1});
 await call('Input.dispatchMouseEvent',{type:'mouseReleased',...pointer,button:'left',clickCount:1});
 await wait_for(`!!document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="候補の独立3Dビュー"] canvas')`);
 await pause(1200);
 const cluster_shot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output_directory,'cluster.png'),Buffer.from(cluster_shot.data,'base64'));
 await evaluate(`[...document.querySelector('#ros-results').shadowRoot.querySelectorAll('button')].find(button=>button.textContent==='車モデルを比較').click()`);
 await wait_for(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="車両候補の比較"]')?.querySelectorAll('tbody tr').length===4`);
 await pause(1600);
 const detail_shot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output_directory,'detail.png'),Buffer.from(detail_shot.data,'base64'));
 await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="候補ビューを閉じる"]').click()`);
 await wait_for(`!document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="候補の独立3Dビュー"]')`);
 assert.equal(await evaluate('simulator.renderer.getContext().isContextLost()'),false);
 console.log('PASS: candidate picking and independent detail canvas');

 const denial=await evaluate(`new Promise((resolve,reject)=>{const socket=new WebSocket('ws://127.0.0.1:${ws_port}/observe');const timer=setTimeout(()=>{socket.close();reject(Error('拒否応答待ち'));},3000);socket.onopen=()=>socket.send(JSON.stringify({id:'denied',method:'edit.commit',params:{}}));socket.onmessage=e=>{if(typeof e.data!=='string')return;const value=JSON.parse(e.data);if(value.id==='denied'){clearTimeout(timer);socket.close();resolve(value);}};})`);
 assert.equal(denial.error.code,'READ_ONLY');
 // 非表示は共有購読を停止しないことの確認。
 const hidden=await evaluate(`simulator.ros_results.api.unsubscribeSource('/test/points',true)`);assert.equal(hidden.success,true);
 await evaluate(`simulator.ros_results.api.subscribeSource('/test/points')`);
 // ホバー枠の出現を定常描画の資源増加へ混ぜないため、計測前にシーン外へ移動。
 await call('Input.dispatchMouseEvent',{type:'mouseMoved',x:30,y:30});
 await pause(2000);
 await call('HeapProfiler.collectGarbage');
 const heap_before=await call('Runtime.getHeapUsage');
 if(process.env.TOPO_PROFILE){await call('Profiler.enable');await call('Profiler.start');}
 const metrics=await measure_frames();metrics.baseline_p50_ms=baseline.frame_p50_ms;metrics.baseline_p95_ms=baseline.frame_p95_ms;
 if(process.env.TOPO_PROFILE){const profile=await call('Profiler.stop');await fs.writeFile(path.join(output_directory,'cpu-profile.json'),JSON.stringify(profile));}
 await call('HeapProfiler.collectGarbage');
 const heap_after=await call('Runtime.getHeapUsage');metrics.heap_delta_bytes=heap_after.usedSize-heap_before.usedSize;
 console.log('MEASURED: '+JSON.stringify(metrics));
 assert.equal(metrics.geometry_growth,0);
 await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="表示: /topological_map"]').scrollIntoView({block:'center'})`);
 const shot=await call('Page.captureScreenshot',{format:'png'});await fs.writeFile(path.join(output_directory,'browser.png'),Buffer.from(shot.data,'base64'));
 // TF未接続の地図を明示選択し、遠方でも実ピクセルに描画されたことの確認。
 assert.equal(await evaluate(`simulator.ros_results.scene.root_scene.getObjectByName('/test/Tmap_remote').visible`),false);
 await evaluate(`[...document.querySelector('#ros-results').shadowRoot.querySelectorAll('button')].find(button=>button.textContent==='map を基準に表示').click()`);
 await wait_for(`simulator.ros_results.scene.root_scene.getObjectByName('/test/Tmap_remote')?.visible`);
 await pause(500);
 const framing=await evaluate(`(()=>{const a=simulator.ros_results.api,s=simulator.ros_results.scene;const graph=a.graphData['/test/Tmap_remote'];const group=s.root_scene.getObjectByName('/test/Tmap_remote');const meshes=[];group.traverse(object=>{if(object.isMesh)meshes.push({count:object.count,fog:object.material.fog});});return {num_inside:graph.nodes.filter(node=>{const point=simulator.camera.position.clone().set(node.x,node.y,node.z).applyMatrix4(group.matrixWorld).project(simulator.camera);return Math.abs(point.x)<1&&Math.abs(point.y)<1&&Math.abs(point.z)<1;}).length,num_nodes:graph.nodes.length,meshes};})()`);
 assert.equal(framing.num_inside,framing.num_nodes);assert.ok(framing.meshes.some(mesh=>mesh.count>0));assert.ok(framing.meshes.every(mesh=>!mesh.fog));
 // 150 m幅の試験地図向けに、共有Viewerの表示サイズ設定を使用。
 await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('[aria-label="表示: /test/Tmap_remote"]').closest('.surface-muted').querySelector('[title="Graph colors"]').click()`);
 await pause(200);
 for(const [label,value] of [['Node Size','.3'],['Edge Width','.03']]){
  await evaluate(`(()=>{const root=document.querySelector('#ros-results').shadowRoot;const label=[...root.querySelectorAll('label')].find(label=>label.textContent==='${label}');const input=label.parentElement.parentElement.querySelector('input[type=range]');Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set.call(input,'${value}');input.dispatchEvent(new Event('input',{bubbles:true,composed:true}));input.dispatchEvent(new Event('change',{bubbles:true,composed:true}));})()`);
  await pause(100);
 }
 await evaluate(`document.querySelector('#ros-results').shadowRoot.querySelector('h2').closest('.surface-panel').querySelector('button').click()`);
 await pause(500);
 const visible_shot=await call('Page.captureScreenshot',{format:'png'});
 await evaluate(`simulator.ros_results.scene.root_scene.visible=false`);await pause(100);
 const hidden_shot=await call('Page.captureScreenshot',{format:'png'});
 await evaluate(`simulator.ros_results.scene.root_scene.visible=true`);
 const num_pixels=await evaluate(`(async()=>{const rect=simulator.renderer.domElement.getBoundingClientRect();const read=async source=>{const bitmap=await createImageBitmap(await(await fetch(source)).blob());const canvas=new OffscreenCanvas(bitmap.width,bitmap.height);const context=canvas.getContext('2d');context.drawImage(bitmap,0,0);bitmap.close();return context.getImageData(rect.left,rect.top+110,rect.width,rect.height-220).data;};const a=await read('data:image/png;base64,${visible_shot.data}'),b=await read('data:image/png;base64,${hidden_shot.data}');let count=0;for(let idx=0;idx<a.length;idx+=4)if(Math.abs(a[idx]-b[idx])+Math.abs(a[idx+1]-b[idx+1])+Math.abs(a[idx+2]-b[idx+2])>30)count++;return count;})()`);
 assert.ok(num_pixels>100,'遠方GNGの描画ピクセル: '+num_pixels);
 await fs.writeFile(path.join(output_directory,'remote-map.png'),Buffer.from(visible_shot.data,'base64'));
 console.log('PASS: missing TF guidance, reference selection, camera framing and visible remote map pixels '+num_pixels);
 await evaluate('simulator.ros_results.api.disconnect()');
 await wait_for(`Object.keys(simulator.ros_results.api.graphData).length===0`);
 await pause(200);
 assert.equal(await evaluate(`(()=>{let n=0;simulator.ros_results.scene.root_scene.traverse(o=>{if(o.isMesh||o.isPoints||o.isLine||o.isSprite)n++;});return n;})()`),0);
 await evaluate('simulator.ros_results.dispose()');await pause(700);
 assert.equal(await evaluate('simulator.renderer.getContext().isContextLost()'),false);
 assert.equal(await evaluate('simulator.diagnostics.errors.length'),0);
 assert.deepEqual(exceptions,[]);
 if(process.argv[2])await fs.writeFile(process.argv[2],JSON.stringify(metrics,null,2));
 console.log(JSON.stringify({metrics,summary,renderer_name}));
}finally{
 try{
 if(fixture&&fixture.exitCode===null&&fixture.signalCode===null){fixture.stdin.end();const exited=once(fixture,'exit');let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,120000);})]);clearTimeout(timer);assert.ok(fixture.exitCode!==null||fixture.signalCode!==null,'ROS試験プロセスが未停止');console.log(fixture_log.trim());}
 }finally{
  ws?.close();await stopped(chrome);await stopped(server);await fs.rm(directory,{recursive:true,force:true,maxRetries:5,retryDelay:200});
  console.log('STOPPED: test server and Chrome');
 }
}
