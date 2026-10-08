import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import fs from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

// 専用サーバー・Chromeでの外部環境の表示、保存復元、センサ計測の検証。
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const directory=await fs.mkdtemp('/tmp/topo-environment-ui-');
const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');
const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
const server=spawn(process.execPath,['app/server.mjs'],{cwd:root,env:{...process.env,PORT:String(port)},stdio:'ignore'});
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let chrome,ws;
const stopped=async child=>{
 if(!child||child.exitCode!==null||child.signalCode!==null)return;
 const exited=once(child,'exit');child.kill('SIGTERM');
 let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,5000);})]);clearTimeout(timer);
 if(child.exitCode===null&&child.signalCode===null){child.kill('SIGKILL');await exited;}
};
const deadline=setTimeout(()=>{console.error('検証時間上限: 240秒');ws?.close();void stopped(chrome);void stopped(server);},240000);
try{
 for(let iter=0;iter<50;iter++){try{if((await fetch(`http://127.0.0.1:${port}/api/health`)).ok)break;}catch{}await pause(100);}
 chrome=spawn(process.env.CHROME_BIN||'/usr/bin/google-chrome',[
  '--headless=new','--no-first-run','--disable-dev-shm-usage','--use-angle=swiftshader',
  '--enable-unsafe-swiftshader','--window-size=1440,1000','--remote-debugging-port=0',
  '--user-data-dir='+directory,`http://127.0.0.1:${port}/?model=long`],{stdio:'ignore'});
 let pages;
 for(let iter=0;iter<100;iter++){try{const debug_port=(await fs.readFile(directory+'/DevToolsActivePort','utf8')).split('\n')[0];pages=await(await fetch(`http://127.0.0.1:${debug_port}/json`)).json();if(pages.some(page=>page.type==='page'))break;}catch{}await pause(100);}
 assert.ok(pages,'Chromeの起動');
 ws=new WebSocket(pages.find(page=>page.type==='page').webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
 let seq=0;const requests=new Map();
 ws.onmessage=event=>{const value=JSON.parse(event.data);if(value.id){requests.get(value.id)?.(value);requests.delete(value.id);}};
 const call=(method,params={})=>new Promise((resolve,reject)=>{
  const id=++seq,timer=setTimeout(()=>{requests.delete(id);reject(Error('CDP時間超過: '+method));},60000);
  requests.set(id,value=>{clearTimeout(timer);value.error?reject(Error(JSON.stringify(value.error))):resolve(value.result);});
  ws.send(JSON.stringify({id,method,params}));
 });
 const evaluate=async expression=>{
  const result=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
  if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;
 };
 let is_ready=false;
 for(let iter=0;iter<100;iter++){is_ready=await evaluate('!!window.simulator?.diagnostics.ready');if(is_ready)break;await pause(200);}
 assert.ok(is_ready,'アプリの起動');
 await evaluate(`(async()=>{window.THREE=await import('three');window.env=simulator.workspace;window.assets=env.environment_assets;window.before=JSON.stringify(env.getState());window.pose=JSON.stringify(simulator.robot.getPose());document.querySelector('[data-panel=environment]').click();document.getElementById('environment-asset').value='littlest_tokyo';document.getElementById('environment-asset-apply').click();})()`);
 for(let iter=0;iter<150;iter++){if(await evaluate('!document.getElementById("environment-asset-apply").disabled'))break;await pause(200);}
 const result=await evaluate(`(()=>{
  const box=new THREE.Box3().setFromObject(assets.group),size=box.getSize(new THREE.Vector3());let num_meshes=0,num_triangles=0,num_textures=0;
  assets.group?.traverse(mesh=>{if(mesh.isMesh){num_meshes++;num_triangles+=(mesh.geometry.index?.count??mesh.geometry.attributes.position.count)/3;for(const material of [].concat(mesh.material))if(material.map?.image)num_textures++;}});
  const state=env.getState();window.saved=structuredClone(state);state.environment_asset='none';
  return {asset_id:assets.asset_id,num_meshes,num_triangles,num_textures,size:size.toArray(),min:box.min.toArray(),has_preserved:JSON.stringify(state)===before&&JSON.stringify(simulator.robot.getPose())===pose,has_scene:assets.group?.parent===simulator.scene,status:document.getElementById('environment-asset-status').textContent};
 })()`);
 assert.equal(result.asset_id,'littlest_tokyo',result.status);assert.ok(result.num_meshes>0&&result.num_textures>0);assert.ok(result.has_preserved&&result.has_scene);
 assert.ok(Math.abs(Math.max(...result.size.slice(0,2))-12)<1e-4&&Math.abs(result.min[0]-1.5)<1e-4&&Math.abs(result.min[2]+.14)<1e-4);
 console.log('PASS 読込・テクスチャ・座標変換・既存配置保持',JSON.stringify(result));
 const sensor=await evaluate(`(async()=>{
  const lidar=await import('./lidar-core.js'),geometries=new Map(),meshes=[];
  simulator.scene.updateMatrixWorld(true);
  assets.group.traverse(mesh=>{if(!mesh.isMesh)return;const geometry=mesh.geometry,id=geometry.uuid;
   if(!geometries.has(id))geometries.set(id,lidar.buildGeometry({position:lidar.packedPositions(geometry.attributes.position),index:geometry.index?.array}));
   meshes.push({id:meshes.length+1,geometry:id,matrix:mesh.matrixWorld.toArray()});
  });
  const frame=lidar.scan({...lidar.defaultLidarConfig(),scanPattern:'low-discrepancy',beams:2000},new THREE.Matrix4().makeTranslation(0,0,1).toArray(),lidar.prepareMeshes(meshes,geometries));
  for(const geometry of geometries.values())geometry.dispose();
  const {RGBDSensor}=await import('./rgbd-core.js'),sensor=new RGBDSensor(simulator.renderer,simulator.scene);
  const k={width:128,height:96,fx:65,fy:65,ppx:63.5,ppy:47.5};
  const world=new THREE.Matrix4().makeBasis(new THREE.Vector3(0,-1,0),new THREE.Vector3(0,0,-1),new THREE.Vector3(1,0,0)).setPosition(0,0,1);
  const with_asset=sensor.render(k,world,'environment_test',true);assets.group.visible=false;
  let without_asset;try{without_asset=sensor.render(k,world,'environment_test',true);}finally{assets.group.visible=true;for(const target of Object.values(sensor.targets))target.dispose();sensor.depthMaterial.dispose();}
  let num_changed=0;for(let idx=0;idx<with_asset.length;idx++)if(with_asset[idx]>0&&Math.abs(with_asset[idx]-without_asset[idx])>.001)num_changed++;
  return {lidar_points:frame.count,depth_pixels:num_changed};
 })()`);
 assert.ok(sensor.lidar_points>0&&sensor.depth_pixels>0,JSON.stringify(sensor));console.log('PASS 外部環境のLiDAR・深度計測',sensor);
 const restored=await evaluate(`(async()=>{const group=assets.group;await assets.set('none');const has_removed=!group.parent&&env.getState().environment_asset==='none';await env.load(saved);let has_rejected=false;try{await env.load({...saved,environment_asset:'unknown_asset'});}catch{has_rejected=true;}return {has_removed,has_rejected,has_restored:assets.group===group&&group.parent===simulator.scene,state:env.getState(),saved};})()`);
 assert.ok(restored.has_removed&&restored.has_rejected&&restored.has_restored,'環境解除・未対応アセット拒否');
 assert.deepEqual(restored.state,restored.saved,'保存復元時の配置保持');
 assert.equal(await evaluate(`(async()=>{const old=structuredClone(saved);delete old.environment_asset;await env.load(old);const ok=assets.asset_id==='none'&&!assets.group;await env.load(saved);return ok;})()`),true,'従来のシーンJSONの互換性');
 console.log('PASS 環境解除・保存復元・不正入力の拒否・旧形式互換性');
 await evaluate('document.getElementById("environment-asset-focus").click()');await pause(500);
 const screenshot=await call('Page.captureScreenshot',{format:'png'});
 await fs.writeFile('/tmp/topo-environment-littlest-tokyo.png',Buffer.from(screenshot.data,'base64'));
 assert.deepEqual(await evaluate('simulator.diagnostics.errors'),[]);
 console.log('Screenshot: /tmp/topo-environment-littlest-tokyo.png');
}finally{
 clearTimeout(deadline);ws?.close();await stopped(chrome);await stopped(server);
 await fs.rm(directory,{recursive:true,force:true});
 console.log('検証用Chrome・サーバー停止済み、専用プロファイル削除済み');
}
