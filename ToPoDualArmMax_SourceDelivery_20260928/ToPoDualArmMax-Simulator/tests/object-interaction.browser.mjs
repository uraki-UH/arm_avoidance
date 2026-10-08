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
 assert.equal(await evaluate(`(async()=>{
  const panel=simulator.physics_panel,robot=simulator.robot,saved=robot.getPose(),prepare=panel.prepare_motion;
  let num_attempts=0;panel.prepare_motion=async()=>{num_attempts++;throw Error('試験用の未接続ブリッジ');};
  try{for(const id of ['camera-aim','sensor-aim']){
   robot.setPose({...saved,neck_pan_joint:0,neck_tilt_joint:0});
   document.getElementById(id).click();
   for(let iter=0;iter<100&&Math.abs(robot.getPose().neck_tilt_joint)<.05;iter++)await new Promise(resolve=>setTimeout(resolve,200));
   if(Math.abs(robot.getPose().neck_tilt_joint)<.05||panel.socket||num_attempts)throw Error(JSON.stringify({id,pose:robot.getPose(),has_socket:!!panel.socket,num_attempts,frames:simulator.diagnostics.frames,toast:document.getElementById('toast').textContent}));
  }return true;}finally{panel.prepare_motion=prepare;simulator.reset();robot.setPose(saved);}
 })()`),true,'物理ブリッジなしでカメラ・RGB-Dのテーブル注視');

 assert.equal(await evaluate(`(async()=>{
  const s=simulator,p=s.physics_panel,robot=s.robot,pose=robot.getPose(),frames=[...s.keyframes],prepare=p.prepare_motion;
  let num_attempts=0;p.prepare_motion=async()=>{num_attempts++;throw Error('試験用の未接続ブリッジ');};
  const wait_for=async test=>{for(let iter=0;iter<100&&!test();iter++)await new Promise(resolve=>setTimeout(resolve,200));return test();};
  try{
   s.reset();const from=robot.getPose().L_joint2;document.querySelector('[data-pose=ready]').click();
   if(!await wait_for(()=>Math.abs(robot.getPose().L_joint2-from)>.05))return false;
   s.reset();s.keyframes.splice(0,s.keyframes.length,robot.getPose(),{...robot.getPose(),neck_pan_joint:.4});document.getElementById('duration').value='.5';document.getElementById('play').click();
   if(!await wait_for(()=>robot.getPose().neck_pan_joint>.05))return false;
   s.reset();document.getElementById('ai-demo').click();
   if(!await wait_for(()=>Math.abs(robot.getPose().neck_tilt_joint)>.05))return false;
   document.getElementById('ai-demo').click();return num_attempts===0&&!p.socket;
  }finally{p.prepare_motion=prepare;s.reset();robot.setPose(pose);s.keyframes.splice(0,s.keyframes.length,...frames);}
 })()`),true,'物理ブリッジなしでプリセット・ポーズ再生・VM AI動作');
 assert.equal(await evaluate(`(async()=>{
  const s=simulator,p=s.physics_panel,robot=s.robot;s.reset();
  const actual=robot.getPose(),goal={...actual,L_joint2:actual.L_joint2+.5};
  robot.setPose(goal);s.syncTargets();robot.setPose(actual);
  const before=s.getState().tcp.L.errorMm;
  p.robot_model=robot;p.enable_dynamics=true;p.actual={...actual};p.actual_joints=null;p.targets={...goal};p.latest=null;
  p.base_signature=JSON.stringify([robot.position.toArray(),robot.quaternion.toArray()]);p.socket={readyState:3,close(){}};
  document.querySelector('[data-panel=environment]').click();document.querySelector('[data-panel=robot]').click();
  try{
   for(let iter=0;iter<100&&s.getState().tcp.L.errorMm>.001;iter++)await new Promise(resolve=>setTimeout(resolve,200));
   const state=s.getState();return before>10&&state.tcp.L.errorMm<.001&&state.tcp.R.errorMm<.001&&p.targets.L_joint2===goal.L_joint2&&robot.getPose().L_joint2===actual.L_joint2;
  }finally{p.stop();s.reset();}
 })()`),true,'物理実姿勢への手先表示追従と内部目標の保持');
 assert.deepEqual(await evaluate(`Array.from(document.querySelectorAll('.workspace-tabs [data-panel]'),b=>b.dataset.panel)`),['robot','environment','sensors','physics','ai','ros']);
 assert.equal(await evaluate(`(()=>{
  const $=id=>document.getElementById(id),click=name=>document.querySelector('[data-panel='+name+']').click();
  const is_visible=name=>$(name+'-panel').getClientRects().length>0;
  const sensors=[simulator.rgbd,simulator.color_camera_panel,simulator.lidar];
  const input=$('camera-panel').querySelector('input[type=number]'),value=input.value;
  input.value='123';click('sensors');
  if(!is_visible('sensor')||is_visible('camera')||is_visible('lidar'))return false;
  for(const name of ['camera','lidar','sensor']){
   document.querySelector('[data-sensor-panel='+name+']').click();
   if(['sensor','camera','lidar'].some(other=>is_visible(other)!==(other===name)))return false;
   if(document.querySelector('[data-sensor-panel='+name+']').getAttribute('aria-pressed')!=='true')return false;
  }
  document.querySelector('[data-sensor-panel=camera]').click();click('environment');
  if(is_visible('camera')||$('environment-panel').contains($('physics-start')))return false;
  click('physics');if(!is_visible('physics')||!$('physics-panel').contains($('physics-start'))||is_visible('environment'))return false;
  click('sensors');if(!is_visible('camera')||input.value!=='123'||is_visible('physics'))return false;
  const settings=$('camera-capture-settings'),preview=$('camera-left'),once=$('camera-once');
  if(settings.open||preview.getBoundingClientRect().bottom>settings.getBoundingClientRect().top||once.getBoundingClientRect().bottom>preview.getBoundingClientRect().top)return false;
  settings.open=true;if(preview.getBoundingClientRect().bottom>settings.getBoundingClientRect().top)return false;
  settings.open=false;input.value=value;click('robot');
  return sensors.every((panel,i)=>panel===[simulator.rgbd,simulator.color_camera_panel,simulator.lidar][i]);
 })()`),true,'センサ子タブ・物理独立タブ・設定保持');
 console.log('PASS sensor subtabs / standalone physics / retained settings');
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
 const ctrl_click=async point=>{for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:point.x,y:point.y,button:'left',buttons:type==='mousePressed'?1:0,clickCount:1,modifiers:2});};
 const box_point=await evaluate('screenPoint(new THREE.Box3().setFromObject(env.items.find(x=>x.type==="box").group,true).getCenter(new THREE.Vector3()))');
 await ctrl_click(box_point);
 assert.equal(await evaluate('env.selected_ids.size===2 && env.selection_boxes.size===1 && !ui.pending && !ui.press'),true,'Ctrl追加選択と個別枠');
 assert.equal(await evaluate('document.getElementById("object-x").disabled && !document.getElementById("object-delete").disabled'),true,'複数選択中の単体編集禁止');
 await ctrl_click(box_point);
 assert.equal(await evaluate('env.selected_ids.size===1 && env.selected===bottle'),true,'Ctrl選択解除');
 await ctrl_click(box_point);await mouse('mousePressed',source,1);await mouse('mouseReleased',source);
 assert.equal(await evaluate('env.selected_ids.size===1 && env.selected===bottle && !document.getElementById("object-x").disabled'),true,'通常クリックの単体選択');

 await mouse('mousePressed',source,1);await mouse('mouseMoved',target,1);await mouse('mouseReleased',target);
 assert.equal(await evaluate('!!ui.pending && ui.status.dataset.state === "preview"'),true,'ドラッグ候補');
 assert.equal(await evaluate('bottle.group.matrixWorld.equals(original)'),true,'確定前の原物体保持');
 assert.equal(await evaluate('simulator.camera.position.toArray().every((v,i)=>Math.abs(v-camera_before[i])<1e-8)'),true,'視点回転との分離');
 assert.equal(await evaluate('ui.pending.ghost.parent === env.overlay'),true,'センサ描画との分離');
 assert.equal(await evaluate('ui.pending.materials.every(m=>m.isMeshBasicMaterial && m.transparent && m.opacity<1)'),true,'照明不要の半透明プレビュー');
 assert.equal(await evaluate(`(()=>{const source=[],preview=[];bottle.group.traverse(x=>{if(x.isMesh)source.push(x);});ui.pending.ghost.traverse(x=>{if(x.isMesh)preview.push(x);});return source.every((mesh,i)=>{const originals=[].concat(mesh.material),copies=[].concat(preview[i].material);return originals.every((m,j)=>{const copy=copies[j];return copy!==m&&copy.color.equals(m.color)&&copy.map===m.map&&copy.alphaMap===m.alphaMap&&Math.abs(copy.opacity-m.opacity*.32)<1e-9;});});})()`),true,'元の色・テクスチャの保持と透過率の適用');

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
 assert.equal(await evaluate(`(()=>{ui.begin(bottle);env.items=env.items.filter(x=>x!==bottle);ui.refresh();const ok=ui.apply_button.disabled;ui.cancel();return ok;})()`),true,'削除済み対象');
 await evaluate('env.items.push(bottle);env.updateList()');
 // 右クリック時の選択保持と、選択対象だけの一括削除
 await evaluate('env.select(bottle.id)');await ctrl_click(box_point);
 await mouse('mouseMoved',box_point);
 assert.equal(await evaluate('ui.hover_box.visible && !!ui.hover'),true,'削除前のホバー枠');
 for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:box_point.x,y:box_point.y,button:'right',buttons:type==='mousePressed'?2:0,clickCount:1});
 assert.equal(await evaluate('env.selected_ids.size===2 && !document.querySelector(".object-context-menu").hidden && document.querySelector(".object-context-menu [data-action=delete]").textContent.includes("2個")'),true,'右クリックで複数選択を保持');
 assert.equal(await evaluate(`(()=>{const section=document.querySelector('.object-context-physics');return !section.querySelector('details').open&&section.querySelector('select').getClientRects().length===0;})()`),true,'通常メニューでモード一覧を非表示');
 await evaluate('document.querySelector("[data-action=toggle-physics]").click()');
 assert.equal(await evaluate('env.items.filter(x=>env.selected_ids.has(x.id)).every(x=>x.physics.mode==="dynamic") && !env.physics_panel.socket'),true,'一操作で動的物理を有効化');
 for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:box_point.x,y:box_point.y,button:'right',buttons:type==='mousePressed'?2:0,clickCount:1});
 await evaluate('document.querySelector("[data-action=toggle-physics]").click()');
 assert.equal(await evaluate('env.items.filter(x=>env.selected_ids.has(x.id)).every(x=>x.physics.mode==="none")'),true,'一操作で物理を無効化');
 for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:box_point.x,y:box_point.y,button:'right',buttons:type==='mousePressed'?2:0,clickCount:1});
 await evaluate('document.querySelector(".object-context-physics details").open=true');
 assert.equal(await evaluate(`(()=>{
  const panel=env.physics_panel,select=document.querySelector('.object-context-physics select');
  const untouched=env.items.find(x=>x.type==='mug'),before=JSON.stringify(untouched.physics);
  bottle.physics={mode:'hinge',mass:.7,constraint:{axis:[0,0,1],pivot:[0,0,0],range:[-1,1]}};
  let num_closed=0;panel.socket={close(){num_closed++;}};
  select.value='dynamic';select.dispatchEvent(new Event('change'));document.querySelector('[data-action=apply-physics]').click();
  return env.items.filter(x=>env.selected_ids.has(x.id)).every(x=>x.physics.mode==='dynamic')&&bottle.physics.mass===.7&&!bottle.physics.constraint&&JSON.stringify(untouched.physics)===before&&num_closed===1&&!panel.socket&&document.getElementById('physics-status').textContent.includes('再開始');
 })()`),true,'選択対象だけ物理適用・質量保持・古い拘束削除・停止');
 for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:box_point.x,y:box_point.y,button:'right',buttons:type==='mousePressed'?2:0,clickCount:1});
 await evaluate('document.querySelector("[data-action=open-physics]").click()');
 assert.equal(await evaluate('!document.getElementById("physics-panel").hidden && env.selected.type==="box"'),true,'右クリック対象の物理タブへ移動');
 assert.equal(await evaluate(`(()=>{
  const panel=env.physics_panel,item=env.selected;
  for(const mode of ['none','static','dynamic','kinematic','hinge','slide']){
   const select=document.getElementById('physics-mode');select.value=mode;select.dispatchEvent(new Event('change'));
   if(item.physics.mode!==mode)return false;
  }
  item.physics.constraint={axis:[1,0,0],pivot:[0,0,0],range:[-.1,.1]};const before=JSON.stringify(item.physics);
  if(!panel.set_object_mode([item],'slide')||JSON.stringify(item.physics)!==before)return false;
  if(panel.set_object_mode([item],'invalid')||panel.set_object_mode([{}],'dynamic'))return false;
  return JSON.stringify(item.physics)===before&&!panel.socket;
 })()`),true,'共通更新・全6モード・同一モードの拘束保持・不正対象拒否');
 console.log('PASS context physics / selected-only / mass / constraint / stop / no auto-start / settings tab');
 await evaluate('document.querySelector("[data-panel=environment]").click();env.select(bottle.id)');await ctrl_click(box_point);
 for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:box_point.x,y:box_point.y,button:'right',buttons:type==='mousePressed'?2:0,clickCount:1});
 assert.equal(await evaluate('document.querySelector(".object-context-physics select").value==="" && document.querySelector("[data-action=apply-physics]").disabled'),true,'異なる物理設定の混在表示');
 await evaluate('document.querySelector(".object-context-menu [data-action=delete]").click()');
 assert.equal(await evaluate('env.items.length===1 && env.items[0].type==="mug" && env.selected_ids.size===0 && env.selection_boxes.size===0'),true,'選択物体だけ削除と枠解放');
 assert.equal(await evaluate('!env.selection.visible && !ui.hover_box.visible && !ui.preview_box.visible && !ui.hover && !ui.pending'),true,'右クリック削除後の全枠解除');
 await evaluate('env.select(env.items[0].id);document.getElementById("object-x").focus()');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Delete',code:'Delete'});
 assert.equal(await evaluate('env.items.length'),1,'入力中のDelete保護');
 await evaluate('document.activeElement.blur()');
 const mug_point=await evaluate('screenPoint(new THREE.Box3().setFromObject(env.items[0].group,true).getCenter(new THREE.Vector3()))');
 await mouse('mouseMoved',mug_point);
 assert.equal(await evaluate('ui.hover_box.visible && !!ui.hover'),true,'Delete前のホバー枠');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Delete',code:'Delete'});
 assert.equal(await evaluate('env.items.length'),0,'Deleteキー削除');
 assert.equal(await evaluate('!env.selection.visible && env.selection_boxes.size===0 && !ui.hover_box.visible && !ui.preview_box.visible && !ui.hover && !ui.pending'),true,'Delete後の全枠解除');
 console.log('PASS Ctrl select / toggle / single select / context batch delete / input guard / Delete');
 await evaluate('window.protected_item=env.add("cube",[.1,.1]);window.protected_state=JSON.stringify(protected_item.physics);window.robot_pose=JSON.stringify(simulator.robot.getPose())');
 const robot_point=await evaluate(`(()=>{
  let point=null;simulator.robot.updateWorldMatrix(true,true);
  simulator.robot.traverseVisible(mesh=>{
   if(point||!mesh.isMesh)return;
   const candidate=screenPoint(new THREE.Box3().setFromObject(mesh).getCenter(new THREE.Vector3()));
   if(document.elementFromPoint(candidate.x,candidate.y)!==simulator.renderer.domElement)return;
   const hit=ui.cast({clientX:candidate.x,clientY:candidate.y})[0];
   for(let node=hit?.object;node;node=node.parent)if(node===simulator.robot){point=candidate;break;}
  });return point;
 })()`);
 assert.ok(robot_point,'ロボット表面のクリック位置');
 await mouse('mouseMoved',robot_point);
 assert.equal(await evaluate(`ui.hover_robot===simulator.robot&&ui.hover_box.visible&&!ui.hover&&!ui.pending&&ui.hover_box.material.color.getHex()===0x38bdf8&&document.querySelector('.object-context-menu').hidden`),true,'クリック前のロボットホバー枠');
 assert.equal(await evaluate(`ui.hover_box.box.equals(new THREE.Box3().setFromObject(simulator.robot))&&JSON.stringify(simulator.robot.getPose())===robot_pose&&env.selected===protected_item`),true,'全体BBox・姿勢と既存選択の保持');
 await mouse('mouseMoved',{x:10,y:10});
 assert.equal(await evaluate('!ui.hover_robot&&!ui.hover_box.visible'),true,'キャンバス外への移動でホバー枠解除');
 const protected_point=await evaluate('screenPoint(new THREE.Box3().setFromObject(protected_item.group,true).getCenter(new THREE.Vector3()))');
 await mouse('mouseMoved',protected_point);
 assert.equal(await evaluate('ui.hover===protected_item&&!ui.hover_robot&&ui.hover_box.visible&&ui.hover_box.material.color.getHex()===0xffce45'),true,'物体への移動で黄色のホバー枠へ切替');
 await mouse('mouseMoved',robot_point);
 const right_robot=async()=>{for(const type of ['mousePressed','mouseReleased'])await call('Input.dispatchMouseEvent',{type,x:robot_point.x,y:robot_point.y,button:'right',buttons:type==='mousePressed'?2:0,clickCount:1});};
 await right_robot();
 assert.equal(await evaluate(`(()=>{const menu=document.querySelector('.object-context-menu');return !menu.hidden&&menu.dataset.targetKind==='robot'&&menu.querySelector('[data-action=delete]').hidden&&env.selected_ids.size===0;})()`),true,'ロボット選択と物体選択・削除の分離');
 assert.deepEqual(await evaluate(`Array.from(document.querySelector('.object-context-physics select').options,x=>x.value)`),['dynamic','kinematic']);
 const robot_screenshot=await call('Page.captureScreenshot',{format:'png'});
 await fs.writeFile('/tmp/topo-robot-physics-menu.png',Buffer.from(robot_screenshot.data,'base64'));
 assert.equal(await evaluate(`(()=>{
  const select=document.querySelector('.object-context-physics select');select.value='kinematic';select.dispatchEvent(new Event('change'));document.querySelector('[data-action=apply-physics]').click();
  return document.getElementById('physics-robot-mode').value==='kinematic'&&!env.physics_panel.socket&&JSON.stringify(protected_item.physics)===protected_state&&JSON.stringify(simulator.robot.getPose())===robot_pose;
 })()`),true,'ロボットへの適用・物体と関節姿勢の保持・自動起動なし');
 await right_robot();
 assert.equal(await evaluate(`(()=>{
  let num_closed=0;env.physics_panel.socket={close(){num_closed++;}};
  const select=document.querySelector('.object-context-physics select');select.value='dynamic';select.dispatchEvent(new Event('change'));document.querySelector('[data-action=apply-physics]').click();
  return num_closed===1&&!env.physics_panel.socket&&document.getElementById('physics-robot-mode').value==='dynamic'&&document.getElementById('physics-status').textContent.includes('再開始');
 })()`),true,'動力学適用と再開始案内');
 await right_robot();await evaluate('document.querySelector("[data-action=open-physics]").click()');
 assert.equal(await evaluate('!document.getElementById("physics-panel").hidden&&document.activeElement.id==="physics-robot-mode"'),true,'ロボットの物理設定へ移動');
 assert.equal(await evaluate(`(()=>{const panel=env.physics_panel,select=document.getElementById('physics-robot-mode');select.value='kinematic';select.dispatchEvent(new Event('change'));return select.value==='kinematic'&&!panel.set_robot_mode({},'dynamic')&&!panel.set_robot_mode(simulator.robot,'none')&&select.value==='kinematic';})()`),true,'共通設定経路・旧モデルや非対応モードの拒否');
 await evaluate('document.activeElement.blur()');await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Delete',code:'Delete'});
 assert.equal(await evaluate('env.items.includes(protected_item)'),true,'ロボット選択後の誤削除防止');
 console.log('PASS robot hover / pointer leave / object hover switch / context / robot-only modes / object isolation / stop / no auto-start / settings / stale target');
 assert.equal(await evaluate(`(()=>{
  const panel=env.physics_panel,robot=simulator.robot,original=robot.getPose();
  panel.robot_model=robot;panel.enable_dynamics=true;panel.actual={...original};panel.targets={...original};panel.actual_joints=null;panel.socket={close(){}};
  const name='neck_pan_joint',goal=original[name]+.2;
  robot.setJoint(name,goal);panel.sync_robot_pose();
  const first=robot.getPose()[name]===original[name]&&panel.targets[name]===goal;
  panel.sync_robot_pose();const second=panel.targets[name]===goal;
  panel.stop();robot.setPose(original);return first&&second;
 })()`),true,'物理未受信フレームの指令回収と計算済み姿勢保持');
 const empty_point=await evaluate(`(()=>{const canvas=simulator.renderer.domElement,rect=canvas.getBoundingClientRect();for(let y=rect.top+30;y<rect.bottom-30;y+=50)for(let x=rect.left+30;x<rect.right-30;x+=50){if(document.elementFromPoint(x,y)===canvas&&!ui.pick({clientX:x,clientY:y}))return {x,y};}return null;})()`);
 assert.ok(empty_point,'選択対象のないクリック位置');
 await evaluate('env.select(protected_item.id);window.extra_item=env.add("cube",[-.2,.1]);env.select(protected_item.id,true)');
 await mouse('mouseMoved',empty_point);await mouse('mousePressed',empty_point,1);await mouse('mouseReleased',empty_point);
 assert.equal(await evaluate('!env.selected&&env.selected_ids.size===0&&!env.selection.visible&&env.selection_boxes.size===0&&!ui.hover_box.visible&&!ui.preview_box.visible'),true,'空き領域クリックによる複数選択と全枠の解除');
 await evaluate('env.select(protected_item.id);ui.begin(protected_item)');
 await mouse('mousePressed',empty_point,1);await mouse('mouseReleased',empty_point);
 assert.equal(await evaluate('!env.selected&&!ui.pending&&!ui.preview_box.visible&&JSON.stringify(protected_item.physics)===protected_state'),true,'空き領域クリックによるプレビュー取消');
 await evaluate('env.select(protected_item.id);env.select(extra_item.id,true)');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Escape',code:'Escape'});
 assert.equal(await evaluate('!env.selected&&env.selected_ids.size===0&&env.selection_boxes.size===0&&!env.selection.visible'),true,'Escによる複数選択の解除');
 await evaluate('env.select(protected_item.id);ui.begin(protected_item);ui.rotate(15)');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Escape',code:'Escape'});
 assert.equal(await evaluate('!env.selected&&!ui.pending&&!ui.hover_box.visible&&!ui.preview_box.visible&&!env.selection.visible'),true,'Escによるプレビューと選択の同時解除');
 await evaluate('document.querySelector("[data-panel=environment]").click();env.select(protected_item.id);document.getElementById("object-x").focus()');
 assert.equal(await evaluate('document.activeElement.id'), 'object-x', '表示中の入力欄へフォーカス');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Escape',code:'Escape'});
 assert.equal(await evaluate('env.selected===protected_item'),true,'入力欄編集中のEscによる誤解除防止');
 await evaluate('document.activeElement.blur()');await right_robot();
 await evaluate('document.querySelector(".object-context-physics select").focus()');
 await call('Input.dispatchKeyEvent',{type:'keyDown',key:'Escape',code:'Escape'});
 assert.equal(await evaluate('document.querySelector(".object-context-menu").hidden&&!ui.hover_box.visible&&env.overlay.children.filter(x=>x.isBox3Helper).every(x=>!x.visible)'),true,'EscによるロボットメニューとBBoxの解除');
 await evaluate('document.activeElement.blur();env.select(protected_item.id)');
 await mouse('mousePressed',empty_point,1);await mouse('mouseMoved',{x:empty_point.x+20,y:empty_point.y+20},1);await mouse('mouseMoved',empty_point,1);await mouse('mouseReleased',empty_point);
 assert.equal(await evaluate('env.selected===protected_item&&!ui.pending'),true,'元の座標へ戻るカメラドラッグでも選択維持');
 console.log('PASS empty click / Escape / multi deselect / preview cancel / input guard / robot menu dismiss / camera drag');
 assert.deepEqual(await evaluate('simulator.diagnostics.errors'),[]);
 console.log('PASS unavailable planner / cancel / no surface / stale source / deleted source');
 console.log('Screenshot: '+screenshot_path);
}finally{
 ws?.close();await stopped(chrome);await stopped(server);
 await fs.rm(directory,{recursive:true,force:true});
 console.log('検証用Chrome・サーバー停止済み、専用プロファイル削除済み');
}
