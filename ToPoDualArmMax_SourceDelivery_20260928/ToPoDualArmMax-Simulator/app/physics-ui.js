import * as THREE from 'three';
const $=id=>document.getElementById(id);
const modes={none:'物理なし',static:'固定（接触あり）',dynamic:'動的（落下・接触）',kinematic:'姿勢指定（接触あり）'};

// 表示メッシュごとの箱近似。外観・センサー用メッシュとは独立した衝突形状
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
  const controls=document.createElement('section');controls.innerHTML=`<h2>MuJoCo物理</h2><label>選択物体のモード<select id="physics-mode">${Object.entries(modes).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select></label><label>質量 kg<input id="physics-mass" type="number" min="0.001" max="1000" step="0.1" value="0.2"></label><div class="row-actions"><button id="physics-start">物理を開始</button><button id="physics-stop">停止</button><button id="physics-demo">落下する箱を追加</button></div><pre id="physics-status">物理OFF</pre><p class="sub-note">衝突形状はメッシュごとの箱近似。物理なしの物体もセンサーには映ります。双腕は姿勢追従の接触モデルで、関節のトルク・動力学は未対応。編集後は停止して再開始。停止時の配置を保持。</p>`;
  $('environment-panel').append(controls);
  $('physics-mode').onchange=()=>{const item=environment.selected;if(item){this.stop();item.physics={...(item.physics||{}),mode:$('physics-mode').value,mass:Number($('physics-mass').value)};}};
  $('physics-mass').onchange=()=>{const item=environment.selected;const mass=Number($('physics-mass').value);if(item&&Number.isFinite(mass)&&mass>=.001&&mass<=1000){this.stop();item.physics={mode:item.physics?.mode||'none',mass};}};
  $('physics-start').onclick=()=>this.start();$('physics-stop').onclick=()=>this.stop();
  $('physics-demo').onclick=()=>{this.stop();const item=environment.add('box',[0,0]);if(item){item.group.position.z=.35;item.physics={mode:'dynamic',mass:.2};environment.syncObject();environment.changed();this.start();}};
  const previous=environment.onChange;environment.onChange=()=>{previous?.();if(this.socket&&JSON.stringify(environment.getState())!==this.scene_signature)this.stop('配置変更のため停止。再開始で反映します');};
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.stop();});
 }
 stop(message='物理OFF'){
  const socket=this.socket;this.socket=null;if(socket)socket.close();this.latest=null;this.scene.userData.physics_time_sec=null;$('physics-status').textContent=message;
 }
 start(){
  this.stop();
  try{
   const environment=this.environment,robot=this.robot();this.robot_model=robot;
   environment.setEditing(false);const bodies=[];this.kinematic=[];
   if(environment.table.visible)bodies.push(body_description('table',environment.table,'static'));
   for(const item of environment.items){const mode=item.physics?.mode||'none';if(mode==='none')continue;const desc=body_description('object_'+item.id,item.group,mode,item.physics?.mass??.2);bodies.push(desc);if(mode==='kinematic')this.kinematic.push({id:desc.id,group:item.group});}
   robot.updateWorldMatrix(true,true);let idx=0;
   robot.traverseVisible(mesh=>{if(!mesh.isMesh||!mesh.geometry.attributes.position)return;const id='robot_'+idx++,desc=body_description(id,mesh,'kinematic');bodies.push(desc);this.kinematic.push({id,group:mesh});});
   const endpoint=new URL($('ros-endpoint').value);endpoint.protocol=endpoint.protocol==='https:'?'wss:':'ws:';endpoint.port=String(Number(endpoint.port||8879)+1);endpoint.pathname='/physics';endpoint.search='';endpoint.hash='';
   this.scene_signature=JSON.stringify(environment.getState());
   const socket=this.socket=new WebSocket(endpoint);
   socket.onopen=()=>{if(this.socket===socket)socket.send(JSON.stringify({type:'start',bodies}));};
   socket.onmessage=event=>{if(this.socket!==socket)return;try{const value=JSON.parse(event.data);if(value.type==='error')throw Error(value.error);if(value.type==='physics')this.latest=value;if(value.type==='ready')$('physics-status').textContent='MuJoCo接続済み';}catch(error){this.stop('物理エラー：'+error.message);}};
   socket.onerror=()=>{if(this.socket===socket)this.stop('物理ブリッジに接続できません。一括起動とMuJoCoの導入を確認してください');};
   socket.onclose=()=>{if(this.socket===socket)this.stop('物理接続が切れました');};
  }catch(error){this.stop('物理エラー：'+error.message);}
 }
 tick(now){
  const item=this.environment.selected;for(const id of ['physics-mode','physics-mass'])$(id).disabled=!item;
  if(item){if(document.activeElement!==$('physics-mode'))$('physics-mode').value=item.physics?.mode||'none';if(document.activeElement!==$('physics-mass'))$('physics-mass').value=item.physics?.mass??.2;}
  if(!this.socket)return;if(this.robot()!==this.robot_model){this.stop('モデル切替のため停止');return;}
  if(this.latest){
   const frame=this.latest;this.latest=null;
   for(const pose of frame.poses){const item=this.environment.items.find(x=>'object_'+x.id===pose.id);if(!item){this.stop('物体構成変更のため停止');return;}const world=new THREE.Matrix4().compose(new THREE.Vector3(...pose.position),new THREE.Quaternion(...pose.quaternion),item.group.getWorldScale(new THREE.Vector3()));item.group.parent.updateWorldMatrix(true,false);const local=item.group.parent.matrixWorld.clone().invert().multiply(world);local.decompose(item.group.position,item.group.quaternion,item.group.scale);item.group.updateMatrixWorld(true);}
   this.environment.renderer.shadowMap.needsUpdate=true;this.environment.syncObject();if(this.environment.selected)this.environment.selection.box.setFromObject(this.environment.selected.group);
   this.scene_signature=JSON.stringify(this.environment.getState());
   this.scene.userData.physics_time_sec=frame.time_sec;$('physics-status').textContent=`MuJoCo実行中 · ${frame.time_sec.toFixed(2)} s · 接触 ${frame.contacts}件`;
  }
  if(now-this.last_ms>=33&&this.socket.readyState===WebSocket.OPEN&&!this.socket.bufferedAmount){this.last_ms=now;const poses=this.kinematic.map(({id,group})=>({id,position:group.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:group.getWorldQuaternion(new THREE.Quaternion()).toArray()}));this.socket.send(JSON.stringify({type:'poses',poses}));}
 }
}
