import * as THREE from 'three';
const $=id=>document.getElementById(id);
export const physics_modes={static:'固定（接触あり）',dynamic:'動的（落下・接触）',kinematic:'姿勢指定（接触あり）',hinge:'ヒンジ（世界に固定した回転軸）',slide:'スライド（世界に固定した直線軸）'};

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
  this.max_leader_age_ms=1000;this.enable_robot_physics=true;this.saved_modes=new WeakMap();
  const controls=document.createElement('section');controls.innerHTML=`<h2>MuJoCo</h2><div id="physics-target" class="sub-note"></div><label><input id="physics-enabled" type="checkbox">物理を有効にする</label><details id="physics-robot-settings"><summary>関節固定・自己接触・回避</summary><label>回避モジュール<select id="physics-avoidance"><option value="none">なし</option><option value="oscbf">OSCBF（実験・速度フィルタ）</option></select></label><label><input type="checkbox" id="physics-self-collision">自己接触を計算</label><div id="physics-locks"></div></details><label id="physics-mode-field">物理モード<select id="physics-mode"><option value="" disabled>選択物体で異なる設定</option>${Object.entries(physics_modes).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select></label><label id="physics-mass-field">質量 kg<input id="physics-mass" type="number" min="0.001" max="1000" step="0.1" value="0.2"></label><details id="physics-constraint"><summary>物体の拘束軸・範囲</summary><label>ローカル軸<select id="physics-axis"><option>X</option><option>Y</option><option selected>Z</option></select></label><label>支点 x y z（物体ローカル m）<input id="physics-pivot" value="0 0 0"></label><label>下限・上限（ヒンジ deg / スライド m）<input id="physics-range" value="-90 90"></label><button id="physics-constraint-apply">拘束を適用</button></details><div class="row-actions"><button id="physics-start">物理を開始</button><button id="physics-stop">停止</button><button id="physics-demo">落下する箱を追加</button></div><pre id="physics-status">物理OFF</pre><p class="sub-note">設定変更後は物理を再開始</p>`;
  $('physics-panel').append(controls);environment.physics_panel=this;
  const bulk=document.createElement('section');bulk.innerHTML=`<h3>物体の物理を一括設定</h3><label><input id="physics-bulk-enabled" type="checkbox">対象物体の物理を有効にする</label><label>一括適用モード<select id="physics-bulk-mode">${Object.entries(physics_modes).map(([key,label])=>`<option value="${key}"${key==='dynamic'?' selected':''}>${label}</option>`).join('')}</select></label><div class="row-actions"><button id="physics-bulk-apply">対象物体に一括適用</button></div><p id="physics-bulk-status" class="sub-note"></p>`;
  $('physics-start').parentElement.before(bulk);
  $('physics-bulk-enabled').onchange=()=>this.set_physics_enabled(this.get_physics_items(),$('physics-bulk-enabled').checked);
  $('physics-bulk-apply').onclick=()=>this.set_all_object_modes($('physics-bulk-mode').value);
  $('physics-mode').onchange=()=>this.set_object_mode(this.get_selected_targets(),$('physics-mode').value);
  $('physics-mass').onchange=()=>{const item=environment.selected;const mass=Number($('physics-mass').value);if(item&&Number.isFinite(mass)&&mass>=.001&&mass<=1000){this.stop();item.physics={...(item.physics||{}),mode:item.physics?.mode||'none',mass};}};
  $('physics-constraint-apply').onclick=()=>{const item=environment.selected;if(!item)return;const pivot=$('physics-pivot').value.trim().split(/\s+/).map(Number),range=$('physics-range').value.trim().split(/\s+/).map(Number),axis=[0,0,0];axis[$('physics-axis').selectedIndex]=1;if(pivot.length!==3||range.length!==2||![...pivot,...range].every(Number.isFinite)||range[0]>=range[1]||range[0]>0||range[1]<0){$('physics-status').textContent='拘束値が不正です。範囲は0を含む昇順で指定してください';return;}this.stop();item.physics.constraint={axis,pivot,range:range.map(x=>item.physics.mode==='hinge'?x*Math.PI/180:x)};};
  $('physics-enabled').onchange=()=>this.set_physics_enabled(this.get_selected_targets(),$('physics-enabled').checked);$('physics-self-collision').onchange=()=>this.stop();
  $('physics-avoidance').onchange=()=>this.stop();
  $('physics-start').onclick=()=>this.start();$('physics-stop').onclick=()=>this.stop();
  $('physics-demo').onclick=()=>{this.stop();const item=environment.add('box',[0,0]);if(item){item.group.position.z=.35;item.physics={mode:'dynamic',mass:.2};environment.syncObject();environment.changed();this.start();}};
  const previous=environment.onChange;environment.onChange=()=>{previous?.();this.refresh_selected_controls();this.refresh_bulk_controls();if(this.socket&&JSON.stringify(environment.getState())!==this.scene_signature)this.stop('配置変更のため停止。再開始で反映します');};
  this.refresh_selected_controls();this.refresh_bulk_controls();
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.stop();});
 }
 get_selected_targets(){
  const items=this.environment.items.filter(item=>this.environment.selected_ids.has(item.id));
  return items.length?items:[this.robot()].filter(Boolean);
 }
 is_physics_enabled(target){
  return target===this.robot()?this.enable_robot_physics:(target.physics?.mode||'none')!=='none';
 }
 get_object_mode(item){
  return this.is_physics_enabled(item)?item.physics.mode:this.saved_modes.get(item)||'dynamic';
 }
 set_physics_enabled(targets,enable_physics){
  const robot=this.robot();
  if(typeof enable_physics!=='boolean'||!targets.length||targets.some(target=>target===robot?!robot?.parent:!this.environment.items.includes(target)))return false;
  const has_running=!!this.socket;
  this.stop();
  for(const target of targets){
   if(target===robot){this.enable_robot_physics=enable_physics;continue;}
   const previous=target.physics||{mode:'none',mass:.2};
   if(enable_physics){target.physics={...previous,mode:this.get_object_mode(target)};}
   else{
    if(previous.mode!=='none')this.saved_modes.set(target,previous.mode);
    target.physics={...previous,mode:'none'};
   }
  }
  this.environment.changed();this.refresh_robot_controls();
  $('physics-status').textContent=enable_physics?`物理を有効化。${has_running?'物理を停止しました。再開始で反映。':'「物理を開始」で反映。'}`:'描画のみ（力学・接触なし）。';
  return true;
 }
 refresh_selected_controls(){
  const targets=this.get_selected_targets(),is_robot=targets.length===1&&targets[0]===this.robot();
  const states=targets.map(target=>this.is_physics_enabled(target)),enable_physics=states.length>0&&states.every(Boolean);
  $('physics-target').textContent=is_robot?'対象: ロボット全体':targets.length===1?`対象: ${targets[0].group.name}`:`対象: ${targets.length}個の物体`;
  $('physics-enabled').checked=enable_physics;$('physics-enabled').indeterminate=states.some(Boolean)&&!enable_physics;$('physics-enabled').disabled=!targets.length;
  $('physics-robot-settings').hidden=!is_robot;
  $('physics-mode-field').hidden=is_robot;$('physics-mass-field').hidden=is_robot;
  $('physics-mode').disabled=is_robot||!enable_physics;$('physics-mass').disabled=is_robot||!enable_physics||targets.length!==1;
  if(!is_robot){const modes=new Set(targets.map(item=>this.get_object_mode(item)));$('physics-mode').value=modes.size===1?[...modes][0]:'';}
  $('physics-constraint').hidden=is_robot||!enable_physics||targets.length!==1||!['hinge','slide'].includes(targets[0]?.physics?.mode);
 }
 refresh_robot_controls(){
  for(const input of document.querySelectorAll('#physics-avoidance, #physics-self-collision, #physics-locks input'))input.disabled=!this.enable_robot_physics;
 }
 get_physics_items(){
  return this.environment.items.filter(item=>{
   for(let group=item.group;group;group=group.parent)if(!group.visible)return false;
   let num_meshes=0;
   item.group.traverse(mesh=>{if(mesh.isMesh&&mesh.geometry?.attributes.position?.count>0)num_meshes++;});
   return num_meshes>0&&num_meshes<=256;
  });
 }
 refresh_bulk_controls(){
  const items=this.get_physics_items(),num_items=items.length;
  const states=items.map(item=>this.is_physics_enabled(item)),enable_physics=num_items>0&&states.every(Boolean);
  $('physics-bulk-enabled').checked=enable_physics;$('physics-bulk-enabled').indeterminate=states.some(Boolean)&&!enable_physics;$('physics-bulk-enabled').disabled=!num_items;
  $('physics-bulk-mode').disabled=!enable_physics;$('physics-bulk-apply').disabled=!enable_physics;
  $('physics-bulk-status').textContent=`対象: ${num_items}個 / 対象外: ${this.environment.items.length-num_items}個`;
 }
 set_all_object_modes(mode){
  const items=this.get_physics_items();
  this.refresh_bulk_controls();
  if(!items.length){$('physics-status').textContent='一括適用できる物体がありません';return false;}
  return this.set_object_mode(items,mode);
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
  clearInterval(this.leader_timer);this.leader_timer=null;
  this.leader_anchor=null;this.is_leader_stopped=true;
  const socket=this.socket;this.socket=null;if(socket)socket.close();this.enable_dynamics=false;this.latest=null;this.scene.userData.physics_time_sec=null;this.motion_state=null;$('physics-status').textContent=message;
  this.environment.object_interaction?.refresh();
 }
 start(){
  this.stop();
  if(this.robot().pose_source==='ros'&&this.enable_robot_physics){$('physics-status').textContent='ROS追従中です。姿勢の入力元をシミュレータ操作へ切り替えてください';return;}
  if(this.robot().pose_source==='leader'&&!this.enable_robot_physics){$('physics-status').textContent='物理フォロワーにはロボットの力学が必要です';return;}
  try{
   const environment=this.environment,robot=this.robot();this.robot_model=robot;this.targets=robot.getPose();this.actual={...this.targets};this.base_signature=JSON.stringify([robot.position.toArray(),robot.quaternion.toArray()]);
   environment.setEditing(false);const bodies=[];this.kinematic=[];
   if(environment.table.visible)bodies.push(body_description('table',environment.table,'static'));
   for(const item of environment.items){const mode=item.physics?.mode||'none';if(mode==='none'||!item.group.visible)continue;const desc=body_description('object_'+item.id,item.group,mode,item.physics?.mass??.2);desc.constraint=item.physics?.constraint;bodies.push(desc);if(mode==='kinematic')this.kinematic.push({id:desc.id,group:item.group});}
   robot.updateWorldMatrix(true,true);
   const enable_dynamics=this.enable_robot_physics;this.enable_dynamics=enable_dynamics;this.actual_joints=null;
   this.leader_anchor=null;this.leader_stamp_sec=null;this.is_leader_stopped=false;this.leader_received_ms=performance.now();
   const robot_config=enable_dynamics?{model:robot.modelId,pose:this.targets,position:robot.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:robot.getWorldQuaternion(new THREE.Quaternion()).toArray(),locked_joints:[...$('physics-locks').querySelectorAll('input:checked')].map(input=>input.value),enable_self_collision:$('physics-self-collision').checked,enable_leader_follow:robot.pose_source==='leader',max_leader_age_sec:this.max_leader_age_ms/1000}:null;
   const endpoint=new URL($('ros-endpoint').value);endpoint.protocol=endpoint.protocol==='https:'?'wss:':'ws:';endpoint.port=String(Number(endpoint.port||8879)+1);endpoint.pathname='/physics';endpoint.search='';endpoint.hash='';
   this.scene_signature=JSON.stringify(environment.getState());
   const socket=this.socket=new WebSocket(endpoint);
   socket.onopen=()=>{if(this.socket===socket)socket.send(JSON.stringify({type:'start',bodies,robot:robot_config,avoidance:{mode:enable_dynamics?$('physics-avoidance').value:'none'}}));};
   socket.onmessage=event=>{if(this.socket!==socket)return;try{const value=JSON.parse(event.data);if(value.type==='error')throw Error(value.error);if(value.type==='physics'){
    // 物理フォロワーの実測と指令処理は描画周期から独立
    if(robot.pose_source==='leader'&&value.joints){if(!this.actual_joints)this.leader_received_ms=performance.now();this.actual_joints=value.joints;this.actual=Object.fromEntries(robot.actuated.map(joint=>[joint.name,value.joints[joint.name]]));this.motion_state=value.motion??null;if(value.is_leader_stopped&&!this.is_leader_stopped)this.hold_leader('物理側のリーダー入力失効。物理の再開始が必要');}
    this.latest=value;
   }if(value.type==='ready')$('physics-status').textContent='MuJoCo接続済み';}catch(error){this.stop('物理エラー：'+error.message);}};
   socket.onerror=()=>{if(this.socket===socket)this.stop('物理ブリッジに接続できません。一括起動とMuJoCoの導入を確認してください');};
   socket.onclose=()=>{if(this.socket===socket)this.stop('物理接続が切れました');};
   if(robot.pose_source==='leader')this.leader_timer=setInterval(()=>{if(this.actual_joints&&!this.is_leader_stopped&&performance.now()-this.leader_received_ms>this.max_leader_age_ms)this.hold_leader('リーダー入力の失効。物理の再開始が必要');this.send_poses();},33);
   environment.object_interaction?.refresh();
  }catch(error){this.stop('物理エラー：'+error.message);}
 }
 async prepare_motion(){
  if(!this.socket||!this.enable_dynamics)return false;
  const socket=this.socket;
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
  if(robot.pose_source!=='leader')for(const [name,value] of Object.entries(robot.getPose()))if(value!==this.actual[name])this.targets[name]=value;
  robot.set_received_pose(this.actual_joints??this.actual);
 }
 hold_leader(message='リーダー入力停止。物理の再開始が必要'){
  if(this.socket&&this.enable_dynamics&&this.actual)this.targets={...this.actual};
  this.is_leader_stopped=true;this.leader_anchor=null;this.leader_detail=message;
 }
 set_leader_target(pose,now_ms=performance.now()){
  // モータ目標だけの更新。描画姿勢へのリーダー角度の直接代入なし
  if(!this.socket||!this.enable_dynamics||!this.actual_joints||this.robot()!==this.robot_model||this.is_leader_stopped)return false;
  const robot=this.robot(),names=Object.keys(pose);
  if(!names.length||names.some(name=>!robot.actuated.some(j=>j.name===name)||!Number.isFinite(pose[name])))throw Error('リーダーの関節名・角度が不正です');
  if(!this.leader_anchor){this.leader_anchor={...pose};this.follower_anchor={...this.actual};}
  if(names.length!==Object.keys(this.leader_anchor).length||names.some(name=>!Object.hasOwn(this.leader_anchor,name)))throw Error('追従中の関節集合の変更');
  const targets={...this.targets};
  for(const name of names){const joint=robot.joints[name],value=/^(?:L|R)_gripper_joint$/.test(name)?pose[name]:(window.simulator?.ros_points?.robot_panel?.joint_stream?.follow_config?.profiles[window.simulator.ros_points.robot_panel.joint_stream.follow_config.profile]?.follow_mode==='absolute'?pose[name]:this.follower_anchor[name]+pose[name]-this.leader_anchor[name]);if(!Number.isFinite(value)||value<joint.lower||value>joint.upper)throw Error('追従目標の可動域超過');targets[name]=value;}
  this.targets=targets;this.leader_received_ms=now_ms;return true;
 }
 send_poses(){
  if(!this.socket||this.socket.readyState!==WebSocket.OPEN||this.socket.bufferedAmount)return;
  const poses=this.kinematic.map(({id,group})=>({id,position:group.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:group.getWorldQuaternion(new THREE.Quaternion()).toArray()}));
  this.socket.send(JSON.stringify({type:'poses',poses,joints:this.enable_dynamics?this.targets:undefined,leader_stamp_sec:this.leader_stamp_sec}));
 }
 tick(now){
  const current_robot=this.robot();
  if(this.controls_robot!==current_robot){this.controls_robot=current_robot;$('physics-locks').replaceChildren();for(const name of Object.keys(current_robot.getPose())){const label=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.value=name;input.onchange=()=>this.stop();label.append(input,document.createTextNode(name));$('physics-locks').append(label);}this.refresh_robot_controls();this.refresh_selected_controls();}
  const item=this.environment.selected;
  if(item&&document.activeElement!==$('physics-mass'))$('physics-mass').value=item.physics?.mass??.2;
  if(item&&!$('physics-constraint').contains(document.activeElement)){const c=item.physics?.constraint||{axis:[0,0,1],pivot:[0,0,0],range:item.physics?.mode==='hinge'?[-Math.PI/2,Math.PI/2]:[-.2,.2]};$('physics-axis').selectedIndex=c.axis.findIndex(x=>x!==0);$('physics-pivot').value=c.pivot.join(' ');$('physics-range').value=c.range.map(x=>item.physics?.mode==='hinge'?x*180/Math.PI:x).join(' ');}
  if(!this.socket)return;if(this.robot()!==this.robot_model){this.stop('モデル切替のため停止');return;}
  if(this.base_signature!==JSON.stringify([current_robot.position.toArray(),current_robot.quaternion.toArray()])){this.stop('基台配置の変更により停止。再開始で反映します');return;}
  if(current_robot.pose_source==='leader'&&this.actual_joints&&!this.is_leader_stopped&&now-this.leader_received_ms>this.max_leader_age_ms)this.hold_leader('リーダー入力の失効。物理の再開始が必要');
  this.sync_robot_pose();
  if(this.latest){
   const frame=this.latest;this.latest=null;if(frame.avoidance)$('physics-status').textContent=`MuJoCo · OSCBF · ${frame.avoidance.solve_ms.toFixed(1)} ms · 近接 ${frame.avoidance.num_constraints} 組`;
   if(this.enable_dynamics&&frame.joints){if(!this.actual_joints)this.leader_received_ms=now;this.motion_state=frame.motion??null;this.actual_joints=frame.joints;current_robot.set_received_pose(frame.joints);this.actual=current_robot.getPose();}
   if(current_robot.pose_source==='leader'&&frame.is_leader_stopped&&!this.is_leader_stopped)this.hold_leader('物理側のリーダー入力失効。物理の再開始が必要');
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
  if(current_robot.pose_source==='leader'&&this.is_leader_stopped)$('physics-status').textContent=this.leader_detail??'リーダー入力停止。物理の再開始が必要';
  if(current_robot.pose_source!=='leader'&&now-this.last_ms>=33){this.last_ms=now;this.send_poses();}
 }
}
