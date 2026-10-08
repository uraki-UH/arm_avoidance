import * as THREE from 'three';
const $=id=>document.getElementById(id);
export const robot_physics_modes={dynamic:'関節動力学（基台固定）',kinematic:'姿勢指定（接触のみ）'};
export const physics_modes={none:'物理なし',static:'固定（接触あり）',dynamic:'動的（落下・接触）',kinematic:'姿勢指定（接触あり）',hinge:'ヒンジ（世界に固定した回転軸）',slide:'スライド（世界に固定した直線軸）'};

// 表示メッシュごとの箱近似。外観・センサ用メッシュとは独立した衝突形状
function body_description(id,group,mode,mass=.2){
 group.updateWorldMatrix(true,true);
 const position=new THREE.Vector3(),rotation=new THREE.Quaternion(),scale=new THREE.Vector3();group.matrixWorld.decompose(position,rotation,scale);
 const rigid=new THREE.Matrix4().compose(position,rotation,new THREE.Vector3(1,1,1)),inverse=rigid.clone().invert(),geoms=[];
 group.traverse(mesh=>{
  if(!mesh.isMesh||!mesh.geometry.attributes.position)return;
  mesh.geometry.computeBoundingBox();const box=mesh.geometry.boundingBox,center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());
  const matrix=inverse.clone().multiply(mesh.matrixWorld),p=new THREE.Vector3(),q=new THREE.Quaternion(),s=new THREE.Vector3();matrix.decompose(p,q,s);center.applyMatrix4(matrix);size.multiply(s).multiplyScalar(.5);
  geoms.push({position:center.toArray(),quaternion:q.toArray(),size:size.toArray().map(x=>Math.max(.0005,Math.abs(x)))});
 });
 return {id,mode,mass,position:position.toArray(),quaternion:rotation.toArray(),geoms};
}

export class PhysicsPanel {
 constructor({environment,robot,scene}){
  this.environment=environment;this.robot=robot;this.scene=scene;this.socket=null;this.latest=null;this.last_ms=0;this.robot_model=null;this.kinematic=[];
  const controls=document.createElement('section');controls.innerHTML=`<h2>MuJoCo</h2><label>ロボット<select id="physics-robot-mode"><option value="dynamic">関節動力学（基台固定）</option><option value="kinematic">姿勢指定（接触のみ）</option></select></label><details><summary>関節固定・自己接触</summary><label><input type="checkbox" id="physics-self-collision">自己接触を計算</label><div id="physics-locks"></div></details><label>選択物体のモード<select id="physics-mode">${Object.entries(physics_modes).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select></label><label>質量 kg<input id="physics-mass" type="number" min="0.001" max="1000" step="0.1" value="0.2"></label><details id="physics-constraint"><summary>物体の拘束軸・範囲</summary><label>ローカル軸<select id="physics-axis"><option>X</option><option>Y</option><option selected>Z</option></select></label><label>支点 x y z（物体ローカル m）<input id="physics-pivot" value="0 0 0"></label><label>下限・上限（ヒンジ deg / スライド m）<input id="physics-range" value="-90 90"></label><button id="physics-constraint-apply">拘束を適用</button></details><div class="row-actions"><button id="physics-start">物理を開始</button><button id="physics-stop">停止</button><button id="physics-demo">落下する箱を追加</button></div><pre id="physics-status">物理OFF</pre><p class="sub-note">物体は箱近似、ロボットはURDF衝突メッシュの凸包。関節動力学はURDFの慣性・可動範囲・トルク上限を使用。固定した関節は開始時の角度を保持。物理なしの物体もセンサには映ります。編集後は停止して再開始。停止時の配置を保持。</p>`;
  $('physics-panel').append(controls);environment.physics_panel=this;
  $('physics-mode').onchange=()=>{const item=environment.selected;if(item)this.set_object_mode([item],$('physics-mode').value);};
  $('physics-mass').onchange=()=>{const item=environment.selected;const mass=Number($('physics-mass').value);if(item&&Number.isFinite(mass)&&mass>=.001&&mass<=1000){this.stop();item.physics={...(item.physics||{}),mode:item.physics?.mode||'none',mass};}};
  $('physics-constraint-apply').onclick=()=>{const item=environment.selected;if(!item)return;const pivot=$('physics-pivot').value.trim().split(/\s+/).map(Number),range=$('physics-range').value.trim().split(/\s+/).map(Number),axis=[0,0,0];axis[$('physics-axis').selectedIndex]=1;if(pivot.length!==3||range.length!==2||![...pivot,...range].every(Number.isFinite)||range[0]>=range[1]||range[0]>0||range[1]<0){$('physics-status').textContent='拘束値が不正です。範囲は0を含む昇順で指定してください';return;}this.stop();item.physics.constraint={axis,pivot,range:range.map(x=>item.physics.mode==='hinge'?x*Math.PI/180:x)};};
  $('physics-robot-mode').onchange=()=>this.set_robot_mode(this.robot(),$('physics-robot-mode').value);$('physics-self-collision').onchange=()=>this.stop();
  $('physics-start').onclick=()=>this.start();$('physics-stop').onclick=()=>this.stop();
  $('physics-demo').onclick=()=>{this.stop();const item=environment.add('box',[0,0]);if(item){item.group.position.z=.35;item.physics={mode:'dynamic',mass:.2};environment.syncObject();environment.changed();this.start();}};
  const previous=environment.onChange;environment.onChange=()=>{previous?.();if(this.socket&&JSON.stringify(environment.getState())!==this.scene_signature)this.stop('配置変更のため停止。再開始で反映します');};
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.stop();});
 }
 set_robot_mode(robot,mode){
  if(robot!==this.robot()||!robot?.parent||!Object.hasOwn(robot_physics_modes,mode))return false;
  const has_running=!!this.socket;
  this.stop();$('physics-robot-mode').value=mode;
  $('physics-status').textContent=`ロボットに「${robot_physics_modes[mode]}」を設定。${has_running?'物理を停止しました。再開始で反映。':'「物理を開始」で反映。'}`;
  return true;
 }
 set_object_mode(items,mode){
  if(!Object.hasOwn(physics_modes,mode)||!items.length||items.some(item=>!this.environment.items.includes(item)))return false;
  const has_running=!!this.socket;
  this.stop();
  for(const item of items){
   const previous=item.physics||{};
   item.physics={...previous,mode,mass:previous.mass??.2};
   if(previous.mode!==mode)delete item.physics.constraint;
  }
  this.environment.changed();
  $('physics-status').textContent=`${items.length}個に「${physics_modes[mode]}」を設定。${has_running?'物理を停止しました。再開始で反映。':'「物理を開始」で反映。'}`;
  if(this.environment.selected&&items.includes(this.environment.selected))$('physics-mode').value=mode;
  return true;
 }
 stop(message='物理OFF'){
  const socket=this.socket;this.socket=null;if(socket)socket.close();this.latest=null;this.scene.userData.physics_time_sec=null;$('physics-status').textContent=message;
  this.environment.object_interaction?.refresh();
 }
 start(){
  this.stop();
  try{
   const environment=this.environment,robot=this.robot();this.robot_model=robot;this.targets=robot.getPose();this.actual={...this.targets};this.base_signature=JSON.stringify([robot.position.toArray(),robot.quaternion.toArray()]);
   environment.setEditing(false);const bodies=[];this.kinematic=[];
   if(environment.table.visible)bodies.push(body_description('table',environment.table,'static'));
   for(const item of environment.items){const mode=item.physics?.mode||'none';if(mode==='none')continue;const desc=body_description('object_'+item.id,item.group,mode,item.physics?.mass??.2);desc.constraint=item.physics?.constraint;bodies.push(desc);if(mode==='kinematic')this.kinematic.push({id:desc.id,group:item.group});}
   robot.updateWorldMatrix(true,true);let idx=0;
   const enable_dynamics=$('physics-robot-mode').value==='dynamic';this.enable_dynamics=enable_dynamics;this.actual_joints=null;
   const robot_config=enable_dynamics?{model:robot.modelId,pose:this.targets,position:robot.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:robot.getWorldQuaternion(new THREE.Quaternion()).toArray(),locked_joints:[...$('physics-locks').querySelectorAll('input:checked')].map(input=>input.value),enable_self_collision:$('physics-self-collision').checked}:null;
   if(!enable_dynamics)robot.traverseVisible(mesh=>{if(!mesh.isMesh||!mesh.geometry.attributes.position)return;const id='robot_'+idx++,desc=body_description(id,mesh,'kinematic');bodies.push(desc);this.kinematic.push({id,group:mesh});});
   const endpoint=new URL($('ros-endpoint').value);endpoint.protocol=endpoint.protocol==='https:'?'wss:':'ws:';endpoint.port=String(Number(endpoint.port||8879)+1);endpoint.pathname='/physics';endpoint.search='';endpoint.hash='';
   this.scene_signature=JSON.stringify(environment.getState());
   const socket=this.socket=new WebSocket(endpoint);
   socket.onopen=()=>{if(this.socket===socket)socket.send(JSON.stringify({type:'start',bodies,robot:robot_config}));};
   socket.onmessage=event=>{if(this.socket!==socket)return;try{const value=JSON.parse(event.data);if(value.type==='error')throw Error(value.error);if(value.type==='physics')this.latest=value;if(value.type==='ready')$('physics-status').textContent='MuJoCo接続済み';}catch(error){this.stop('物理エラー：'+error.message);}};
   socket.onerror=()=>{if(this.socket===socket)this.stop('物理ブリッジに接続できません。一括起動とMuJoCoの導入を確認してください');};
   socket.onclose=()=>{if(this.socket===socket)this.stop('物理接続が切れました');};
   environment.object_interaction?.refresh();
  }catch(error){this.stop('物理エラー：'+error.message);}
 }
 async prepare_motion(){
  const has_physics=this.socket||$('physics-robot-mode').value==='dynamic'||this.environment.items.some(item=>(item.physics?.mode||'none')!=='none');
  if(!has_physics)return false;
  if(!this.socket)this.start();
  const socket=this.socket;
  if(!socket)throw Error($('physics-status').textContent);
  const deadline=performance.now()+15000;
  while(this.socket===socket&&this.scene.userData.physics_time_sec===null&&performance.now()<deadline)await new Promise(resolve=>setTimeout(resolve,50));
  if(this.socket!==socket)throw Error($('physics-status').textContent);
  if(this.scene.userData.physics_time_sec===null){this.stop('物理接続が時間切れです');throw Error('物理接続が時間切れです');}
  return true;
 }
 sync_robot_pose(){
  if(!this.socket||!this.enable_dynamics||this.robot()!==this.robot_model)return;
  const robot=this.robot();
  // 指令値の回収と物理姿勢の維持。受信のない描画フレームでも目標への瞬間移動を防止
  for(const [name,value] of Object.entries(robot.getPose()))if(value!==this.actual[name])this.targets[name]=value;
  robot.setPose(this.actual);
  if(this.actual_joints){for(const [name,value] of Object.entries(this.actual_joints))robot.setJoint(name,value,false);robot.updateMatrixWorld(true);}
 }
 tick(now){
  const current_robot=this.robot();
  if(this.controls_robot!==current_robot){this.controls_robot=current_robot;$('physics-locks').replaceChildren();for(const name of Object.keys(current_robot.getPose())){const label=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.value=name;input.onchange=()=>this.stop();label.append(input,document.createTextNode(name));$('physics-locks').append(label);}}
  const item=this.environment.selected;for(const id of ['physics-mode','physics-mass'])$(id).disabled=!item;
  if(item){if(document.activeElement!==$('physics-mode'))$('physics-mode').value=item.physics?.mode||'none';if(document.activeElement!==$('physics-mass'))$('physics-mass').value=item.physics?.mass??.2;}
  $('physics-constraint').hidden=!item||!['hinge','slide'].includes(item.physics?.mode);
  if(item&&!$('physics-constraint').contains(document.activeElement)){const c=item.physics?.constraint||{axis:[0,0,1],pivot:[0,0,0],range:item.physics?.mode==='hinge'?[-Math.PI/2,Math.PI/2]:[-.2,.2]};$('physics-axis').selectedIndex=c.axis.findIndex(x=>x!==0);$('physics-pivot').value=c.pivot.join(' ');$('physics-range').value=c.range.map(x=>item.physics?.mode==='hinge'?x*180/Math.PI:x).join(' ');}
  if(!this.socket)return;if(this.robot()!==this.robot_model){this.stop('モデル切替のため停止');return;}
  if(this.base_signature!==JSON.stringify([current_robot.position.toArray(),current_robot.quaternion.toArray()])){this.stop('基台配置の変更により停止。再開始で反映します');return;}
  this.sync_robot_pose();
  if(this.latest){
   const frame=this.latest;this.latest=null;
   if(frame.joints){this.actual_joints=frame.joints;for(const [name,value] of Object.entries(frame.joints))current_robot.setJoint(name,value,false);current_robot.updateMatrixWorld(true);this.actual=current_robot.getPose();}
   for(const pose of frame.poses){
    const item=this.environment.items.find(x=>'object_'+x.id===pose.id);
    if(!item){this.stop('物体構成変更のため停止');return;}
    const interaction=this.environment.object_interaction;
    item.group.updateWorldMatrix(true,false);
    const previous_world=interaction?.pending?.item===item?item.group.matrixWorld.clone():null;
    const world=new THREE.Matrix4().compose(new THREE.Vector3(...pose.position),new THREE.Quaternion(...pose.quaternion),item.group.getWorldScale(new THREE.Vector3()));
    item.group.parent.updateWorldMatrix(true,false);
    const local=item.group.parent.matrixWorld.clone().invert().multiply(world);
    local.decompose(item.group.position,item.group.quaternion,item.group.scale);item.group.updateMatrixWorld(true);
    interaction?.sync_physics_pose(item,previous_world);
   }
   this.environment.renderer.shadowMap.needsUpdate=true;this.environment.syncObject();this.environment.refresh_selection();
   this.scene_signature=JSON.stringify(this.environment.getState());
   this.scene.userData.physics_time_sec=frame.time_sec;$('physics-status').textContent=`MuJoCo実行中 · ${frame.time_sec.toFixed(2)} s · 接触 ${frame.contacts}件`;
  }
  if(now-this.last_ms>=33&&this.socket.readyState===WebSocket.OPEN&&!this.socket.bufferedAmount){this.last_ms=now;const poses=this.kinematic.map(({id,group})=>({id,position:group.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:group.getWorldQuaternion(new THREE.Quaternion()).toArray()}));this.socket.send(JSON.stringify({type:'poses',poses,joints:this.targets}));}
 }
}
