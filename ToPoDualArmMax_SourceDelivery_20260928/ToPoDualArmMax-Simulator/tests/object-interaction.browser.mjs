import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import fs from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

// 専用サーバー・専用Chromeプロファイルによる実ポインター操作試験。
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const directory=await fs.mkdtemp('/tmp/topo-object-ui-');
const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');
const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
const server=spawn(process.execPath,['app/server.mjs'],{cwd:root,env:{...process.env,PORT:String(port)},stdio:'ignore'});
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let chrome,ws;
const stopped=async process=>{
 if(!process||process.exitCode!==null||process.signalCode!==null)return;
 const exited=once(process,'exit');process.kill('SIGTERM');
 let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,5000);})]);clearTimeout(timer);
 if(process.exitCode===null&&process.signalCode===null){process.kill('SIGKILL');await exited;}
};
try{
 for(let i=0;i<50;i++){try{if((await fetch(`http://127.0.0.1:${port}/api/health`)).ok)break;}catch{}await pause(100);}
 chrome=spawn(process.env.CHROME_BIN||'/usr/bin/google-chrome',[
  '--headless=new','--no-first-run','--disable-dev-shm-usage','--use-angle=swiftshader',
  '--enable-unsafe-swiftshader','--window-size=1440,1000','--remote-debugging-port=0',
  '--user-data-dir='+directory,`http://127.0.0.1:${port}/?model=long`],{stdio:'ignore'});
 let pages;
 for(let i=0;i<100;i++){try{const debug_port=(await fs.readFile(directory+'/DevToolsActivePort','utf8')).split('\n')[0];pages=await(await fetch(`http://127.0.0.1:${debug_port}/json`)).json();if(pages.some(p=>p.type==='page'))break;}catch{}await pause(100);}
 assert.ok(pages,'Chromeの起動失敗');
 ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
 let seq=0;const requests=new Map();
 ws.onmessage=event=>{const value=JSON.parse(event.data);if(value.id){requests.get(value.id)?.(value);requests.delete(value.id);}};
 const call=(method,params={})=>new Promise((resolve,reject)=>{
  const id=++seq,timer=setTimeout(()=>{requests.delete(id);reject(Error('CDP時間超過: '+method));},45000);
  requests.set(id,value=>{clearTimeout(timer);value.error?reject(Error(JSON.stringify(value.error))):resolve(value.result);});
  ws.send(JSON.stringify({id,method,params}));
 });
 const evaluate=async expression=>{
  const result=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
  if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;
 };
 let is_ready=false;
 for(let i=0;i<150;i++){is_ready=await evaluate('!!window.simulator?.diagnostics.ready');if(is_ready)break;await pause(200);}
 assert.ok(is_ready,'アプリ起動失敗');
 await evaluate(`(async()=>{window.THREE=await import('three');window.env=simulator.workspace;window.ui=env.object_interaction;window.bottle=env.items.find(x=>x.type==='bottle');document.querySelector('[data-panel=environment]').click();document.querySelector('[data-view=top]').click();env.state.yaw=.35;env.buildTable();window.original=bottle.group.matrixWorld.clone();window.camera_before=simulator.camera.position.toArray();window.screenPoint=point=>{const p=point.clone().project(simulator.camera),r=simulator.renderer.domElement.getBoundingClientRect();return {x:r.left+(p.x+1)*r.width/2,y:r.top+(1-p.y)*r.height/2};};})()`);
 await evaluate('simulator.camera.updateMatrixWorld(true)');
 const source=await evaluate('screenPoint(new THREE.Box3().setFromObject(bottle.group,true).getCenter(new THREE.Vector3()))');
 const target=await evaluate('screenPoint(new THREE.Vector3(.17,.25,.001).applyMatrix4(env.root.matrixWorld))');
 const mouse=async(type,p,buttons=0)=>call('Input.dispatchMouseEvent',{type,x:p.x,y:p.y,button:type==='mouseMoved'?'none':'left',buttons,clickCount:type==='mouseMoved'?0:1});
 await mouse('mouseMoved',source);
 await pause(200);
 assert.equal(await evaluate('ui.hover?.id===bottle.id && ui.hover_box.visible'),true,'ホバー枠');
 await mouse('mousePressed',source,1);await mouse('mouseReleased',source);
 assert.equal(await evaluate('env.selected===bottle && !ui.pending'),true,'クリック選択');
 await mouse('mousePressed',source,1);await mouse('mouseMoved',target,1);await mouse('mouseReleased',target);
 assert.equal(await evaluate('!!ui.pending && ui.status.dataset.state === "preview"'),true,'ドラッグ候補');
 assert.equal(await evaluate('bottle.group.matrixWorld.equals(original)'),true,'確定前の原物体保持');
 assert.equal(await evaluate('simulator.camera.position.toArray().every((v,i)=>Math.abs(v-camera_before[i])<1e-8)'),true,'視点回転との分離');
 assert.equal(await evaluate('ui.pending.ghost.parent === env.overlay'),true,'センサー描画との分離');
 assert.equal(await evaluate('ui.pending.materials.every(m=>m.isMeshBasicMaterial && m.transparent && m.opacity<1)'),true,'照明不要の半透明プレビュー');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'t',code:'KeyT'});
 await call('Input.dispatchKeyEvent',{type:'keyUp',key:'t',code:'KeyT'});
 assert.equal(await evaluate('Math.abs(ui.pending.ghost.quaternion.angleTo(bottle.group.getWorldQuaternion(new THREE.Quaternion()))-Math.PI/12)<1e-6'),true,'回転');
 await evaluate('window.goal=ui.pending.ghost.matrixWorld.clone()');
 await pause(300);
 const screenshot=await call('Page.captureScreenshot',{format:'png'});
 const screenshot_path=process.env.SCREENSHOT_PATH||'/tmp/topo-object-preview.png';await fs.writeFile(screenshot_path,Buffer.from(screenshot.data,'base64'));
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Enter',code:'Enter'});
 assert.equal(await evaluate('!ui.pending && ui.status.dataset.state === "placed"'),true,'配置確定');
 assert.equal(await evaluate('bottle.group.matrixWorld.elements.every((v,i)=>Math.abs(v-goal.elements[i])<1e-8)'),true,'回転した親座標系での配置');
 console.log('PASS hover / click / drag / ghost / camera / rotate / placement / parent frame');
 assert.equal(await evaluate(`(()=>{ui.begin(bottle);const select=ui.panel.querySelector('select');select.value='pick_place';select.dispatchEvent(new Event('change'));const before=bottle.group.matrixWorld.clone();ui.apply();const ok=ui.apply_button.disabled&&ui.status.textContent.includes('未接続')&&bottle.group.matrixWorld.equals(before);ui.cancel();select.value='direct';select.dispatchEvent(new Event('change'));return ok;})()`),true,'未接続の実行防止');
 await evaluate('ui.begin(bottle);ui.rotate(15)');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Escape',code:'Escape'});
 assert.equal(await evaluate('!ui.pending && ui.status.dataset.state === "cancelled" && bottle.group.matrixWorld.elements.every((v,i)=>Math.abs(v-goal.elements[i])<1e-8)'),true,'取消');
 assert.equal(await evaluate(`(()=>{ui.begin(bottle);ui.pending.has_surface=false;ui.refresh();const ok=ui.apply_button.disabled;ui.cancel();return ok;})()`),true,'配置面なし');
 assert.equal(await evaluate(`(()=>{ui.begin(bottle);bottle.group.position.x+=.01;ui.refresh();const ok=ui.apply_button.disabled&&ui.reason().includes('変更');ui.cancel();return ok;})()`),true,'元配置変更');
 assert.equal(await evaluate(`(()=>{ui.begin(bottle);const item=env.items.pop();if(item!==bottle)env.items=env.items.filter(x=>x!==bottle);ui.refresh();const ok=ui.apply_button.disabled;ui.cancel();return ok;})()`),true,'削除済み対象');
 assert.deepEqual(await evaluate('simulator.diagnostics.errors'),[]);
 console.log('PASS unavailable planner / cancel / no surface / stale source / deleted source');
 console.log('Screenshot: '+screenshot_path);
}finally{
 ws?.close();await stopped(chrome);await stopped(server);
 await fs.rm(directory,{recursive:true,force:true});
 console.log('検証用Chrome・サーバー停止済み、専用プロファイル削除済み');
}
