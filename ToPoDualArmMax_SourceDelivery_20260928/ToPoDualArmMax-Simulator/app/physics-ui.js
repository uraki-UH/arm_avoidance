import * as THREE from 'three';
import {bridge_socket_url,pose_sources} from './robot-ros-state.js';
import {physics_plugins,format_plugin_status,plugin_definition} from './ros-plugins.js';
const $=id=>document.getElementById(id);
export const physics_modes={static:'固定（接触あり）',dynamic:'動的（落下・接触）',kinematic:'姿勢指定（接触あり）',hinge:'ヒンジ（世界に固定した回転軸）',slide:'スライド（世界に固定した直線軸）'};

export function gng_status_text(frame){return format_plugin_status(frame,plugin_definition('gng'));}

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
  this.environment=environment;this.robot=robot;this.scene=scene;this.socket=null;this.latest=null;this.motion_sample=null;this.last_ms=0;this.robot_model=null;this.kinematic=[];
  this.max_leader_age_ms=1000;this.enable_robot_physics=true;this.saved_modes=new WeakMap();this.can_place_objects=false;this.placement_id=0;this.pending_placement=null;
  const controls=document.createElement('section');controls.innerHTML=`<h2>MuJoCo</h2><details id="physics-robot-settings"><summary>関節固定・自己接触・回避</summary><label>回避モジュール<select id="physics-avoidance"><option value="none">なし</option></select></label><div id="physics-plugin-settings"></div><label><input type="checkbox" id="physics-self-collision">自己接触を計算</label><div id="physics-locks"></div></details><label id="physics-mode-field">物理モード<select id="physics-mode"><option value="" disabled>選択物体で異なる設定</option>${Object.entries(physics_modes).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select></label><label id="physics-mass-field">質量 kg<input id="physics-mass" type="number" min="0.001" max="1000" step="0.1" value="0.2"></label><details id="physics-constraint"><summary>物体の拘束軸・範囲</summary><label>ローカル軸<select id="physics-axis"><option>X</option><option>Y</option><option selected>Z</option></select></label><label>支点 x y z（物体ローカル m）<input id="physics-pivot" value="0 0 0"></label><label>下限・上限（ヒンジ deg / スライド m）<input id="physics-range" value="-90 90"></label><button id="physics-constraint-apply">拘束を適用</button></details><div class="row-actions"><button id="physics-start">物理を開始</button><button id="physics-stop">停止</button><button id="physics-demo">落下する箱を追加</button></div><pre id="physics-status">物理OFF</pre>`;
  $('physics-panel').append(controls);environment.physics_panel=this;
  const scene_list=document.createElement('section');scene_list.className='physics-scene';
  scene_list.innerHTML='<h3>シーン内の物理</h3><p class="sub-note">対象名をクリックで設定。変更後は「物理を開始」で反映。</p><ul id="physics-scene-list" aria-label="シーンの物理設定一覧"></ul>';
  $('physics-robot-settings').before(scene_list);this.scene_rows=new Map();
  const bulk=document.createElement('section');bulk.innerHTML=`<h3>物体の物理を一括設定</h3><button id="physics-bulk-apply" type="button" class="wide-button" aria-pressed="false">対象物体に物理を一括適用</button><label>物理モード<select id="physics-bulk-mode"><option value="" disabled>物体ごとに異なる設定</option>${Object.entries(physics_modes).map(([key,label])=>`<option value="${key}"${key==='dynamic'?' selected':''}>${label}</option>`).join('')}</select></label><p id="physics-bulk-status" class="sub-note"></p>`;
  $('physics-start').parentElement.before(bulk);
  $('physics-bulk-apply').onclick=()=>{const items=this.get_physics_items();this.set_physics_enabled(items,!items.every(item=>this.is_physics_enabled(item)));};
  $('physics-bulk-mode').onchange=()=>this.set_all_object_modes($('physics-bulk-mode').value);
  $('physics-mode').onchange=()=>this.set_object_mode(this.get_selected_targets(),$('physics-mode').value);
  $('physics-mass').onchange=()=>{const item=environment.selected;const mass=Number($('physics-mass').value);if(item&&Number.isFinite(mass)&&mass>=.001&&mass<=1000){this.stop();item.physics={...(item.physics||{}),mode:item.physics?.mode||'none',mass};this.refresh_scene_list();}};
  $('physics-constraint-apply').onclick=()=>{const item=environment.selected;if(!item)return;const pivot=$('physics-pivot').value.trim().split(/\s+/).map(Number),range=$('physics-range').value.trim().split(/\s+/).map(Number),axis=[0,0,0];axis[$('physics-axis').selectedIndex]=1;if(pivot.length!==3||range.length!==2||![...pivot,...range].every(Number.isFinite)||range[0]>=range[1]||range[0]>0||range[1]<0){$('physics-status').textContent='拘束値が不正です。範囲は0を含む昇順で指定してください';return;}this.stop();item.physics.constraint={axis,pivot,range:range.map(x=>item.physics.mode==='hinge'?x*Math.PI/180:x)};};
  $('physics-self-collision').onchange=()=>this.stop();
  this.plugins=new physics_plugins(this);
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
  $('physics-status').textContent=enable_physics?`${has_running?'物理を停止しました。再開始で反映。':'「物理を開始」で反映。'}`:'描画のみ（力学・接触なし）。';
  return true;
 }
 refresh_selected_controls(){
  const targets=this.get_selected_targets(),is_robot=targets.length===1&&targets[0]===this.robot();
  const states=targets.map(target=>this.is_physics_enabled(target)),enable_physics=states.length>0&&states.every(Boolean);
  $('physics-robot-settings').hidden=!is_robot;
  $('physics-mode-field').hidden=is_robot;$('physics-mass-field').hidden=is_robot;
  $('physics-mode').disabled=is_robot||!enable_physics;$('physics-mass').disabled=is_robot||!enable_physics||targets.length!==1;
  if(!is_robot){const modes=new Set(targets.map(item=>this.get_object_mode(item)));$('physics-mode').value=modes.size===1?[...modes][0]:'';}
  $('physics-constraint').hidden=is_robot||!enable_physics||targets.length!==1||!['hinge','slide'].includes(targets[0]?.physics?.mode);
  this.refresh_scene_list();
 }
 // シーンと物理設定の対応一覧。行の再利用によるフォーカス・スクロール位置の保持
 refresh_scene_list(){
  const environment=this.environment,robot=this.robot(),list=$('physics-scene-list');
  const is_visible=group=>{for(let node=group;node;node=node.parent)if(!node.visible)return false;return true;};
  const entries=[];
  if(robot)entries.push({key:robot,target:robot,name:'ロボット（'+robot.modelId+'）',is_robot:true,group:robot});
  for(const item of environment.items)entries.push({key:item,target:item,name:item.group.name||'物体 '+item.id,group:item.group});
  if(environment.table)entries.push({key:'table',name:'テーブル',group:environment.table,description:is_visible(environment.table)?'固定（接触あり）':'非表示・計算対象外'});
  entries.push({key:'ground',name:'地面',description:'固定（接触あり）・床の表示設定とは独立'});
  const pedestal=environment.static_surfaces?.pedestal;
  if(pedestal)entries.push({key:'pedestal',name:'台座',group:pedestal,description:'物理未対応（表示・センサ用）'});
  const background=environment.environment_assets?.group;
  if(background)entries.push({key:'environment',name:'外部環境',group:background,description:'物理未対応（表示・センサ用）'});
  const keys=new Set(entries.map(entry=>entry.key));
  for(const [key,row] of this.scene_rows)if(!keys.has(key)){row.element.remove();this.scene_rows.delete(key);}
  for(const [idx,entry] of entries.entries()){
   let row=this.scene_rows.get(entry.key);
   if(!row){
    const element=document.createElement('li'),name=document.createElement(entry.target?'button':'strong'),description=document.createElement('span');
    element.className='physics-scene-row';name.className='physics-scene-name';description.className='physics-scene-description';element.append(name,description);
    row={element,name,description};this.scene_rows.set(entry.key,row);
    if(entry.target){
     element.dataset.targetKind=entry.is_robot?'robot':'object';if(!entry.is_robot)element.dataset.objectId=String(entry.target.id);
     name.type='button';name.onclick=event=>{
      if(entry.is_robot?this.robot()!==entry.target:!environment.items.includes(entry.target))return;
      environment.object_interaction?.cancel();environment.setEditing(false);
      environment.select(entry.is_robot?null:entry.target.id,!entry.is_robot&&(event.ctrlKey||event.metaKey));
     };
     const toggle=document.createElement('button');toggle.type='button';toggle.className='physics-scene-toggle';
     toggle.onclick=()=>this.set_physics_enabled([entry.target],!this.is_physics_enabled(entry.target));
     element.append(toggle);row.toggle=toggle;
    }else element.dataset.targetKind=entry.key;
   }
   if(list.children[idx]!==row.element)list.insertBefore(row.element,list.children[idx]??null);
   if(row.name.textContent!==entry.name)row.name.textContent=entry.name;
   let description=entry.description;
   if(entry.target){
    const enable_physics=this.is_physics_enabled(entry.target),is_selected=entry.is_robot?!environment.selected_ids.size:environment.selected_ids.has(entry.target.id);
    row.name.setAttribute('aria-pressed',String(is_selected));row.name.setAttribute('aria-label',entry.name+'の設定を表示');
    row.element.classList.toggle('is-selected',is_selected);
    const toggle_text=enable_physics?'物理ON':'物理OFF';if(row.toggle.textContent!==toggle_text)row.toggle.textContent=toggle_text;
    row.toggle.setAttribute('aria-pressed',String(enable_physics));row.toggle.setAttribute('aria-label',entry.name+'の物理を'+(enable_physics?'OFF':'ON')+'に切替');
    description=enable_physics?(entry.is_robot?'関節動力学・基台固定':physics_modes[entry.target.physics.mode]??entry.target.physics.mode):'描画のみ';
    if(!entry.is_robot)description+=' · '+(entry.target.physics?.mass??.2)+' kg';
   }
   if(entry.group&&!is_visible(entry.group))description+=(entry.target&&!entry.is_robot?' · 非表示・計算対象外':entry.key==='table'?'':' · 非表示');
   if(row.description.textContent!==description)row.description.textContent=description;
  }
 }
 refresh_robot_controls(){
  for(const input of document.querySelectorAll('#physics-avoidance, #physics-self-collision, #physics-locks input'))input.disabled=!this.enable_robot_physics;
  this.plugins.update();
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
  const button=$('physics-bulk-apply');button.disabled=!num_items;
  button.textContent=enable_physics?'対象物体の物理を一括解除':'対象物体に物理を一括適用';
  button.setAttribute('aria-pressed',enable_physics?'true':states.some(Boolean)?'mixed':'false');
  const modes=new Set(items.map(item=>this.get_object_mode(item)));
  $('physics-bulk-mode').disabled=!enable_physics;$('physics-bulk-mode').value=modes.size===1?[...modes][0]:'';
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
  if(this.pending_placement){const pending=this.pending_placement;this.pending_placement=null;clearTimeout(pending.timer);pending.reject(Error(message));}
  this.can_place_objects=false;
  clearInterval(this.leader_timer);this.leader_timer=null;
  this.leader_anchor=null;this.is_leader_stopped=true;
  const socket=this.socket;this.socket=null;if(socket)socket.close();this.enable_dynamics=false;this.latest=null;this.motion_sample=null;this.scene.userData.physics_time_sec=null;this.motion_state=null;$('physics-status').textContent=message;
  this.environment.object_interaction?.refresh();
 }
 start(){
  if(window.simulator?.avoidance_replay?.has_pose_control&&this.enable_robot_physics){$('physics-status').textContent='実験の姿勢を表示中です。実験終了後に別タブへ切り替えてください';return;}
  this.stop();
  if(this.robot().pose_source===pose_sources.ros&&this.enable_robot_physics){$('physics-status').textContent='ROS追従中です。姿勢の入力元をシミュレータ操作へ切り替えてください';return;}
  if(this.robot().pose_source===pose_sources.leader&&!this.enable_robot_physics){$('physics-status').textContent='物理フォロワーにはロボットの力学が必要です';return;}
  try{
   if(this.enable_robot_physics)this.plugins.validate_source(this.robot().pose_source);
   const avoidance=this.enable_robot_physics?this.plugins.settings():{mode:'none'};
   const environment=this.environment,robot=this.robot();this.robot_model=robot;this.targets=robot.getPose();this.actual={...this.targets};this.base_signature=JSON.stringify([robot.position.toArray(),robot.quaternion.toArray()]);
   environment.setEditing(false);const bodies=[];this.kinematic=[];
   if(environment.table.visible)bodies.push(body_description('table',environment.table,'static'));
   for(const item of environment.items){const mode=item.physics?.mode||'none';if(mode==='none'||!item.group.visible)continue;const desc=body_description('object_'+item.id,item.group,mode,item.physics?.mass??.2);desc.constraint=item.physics?.constraint;bodies.push(desc);if(mode==='kinematic')this.kinematic.push({id:desc.id,group:item.group});}
   robot.updateWorldMatrix(true,true);
   const enable_dynamics=this.enable_robot_physics;this.enable_dynamics=enable_dynamics;this.actual_joints=null;
   this.leader_anchor=null;this.leader_stamp_sec=null;this.is_leader_stopped=false;this.leader_received_ms=performance.now();
   const robot_config=enable_dynamics?{model:robot.modelId,pose:this.targets,position:robot.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:robot.getWorldQuaternion(new THREE.Quaternion()).toArray(),locked_joints:[...$('physics-locks').querySelectorAll('input:checked')].map(input=>input.value),enable_self_collision:$('physics-self-collision').checked,enable_leader_follow:robot.pose_source===pose_sources.leader,max_leader_age_sec:this.max_leader_age_ms/1000}:null;
   const endpoint=bridge_socket_url($('ros-endpoint').value,'/physics');
   this.update_scene_signature();this.placement_id=0;
   const socket=this.socket=new WebSocket(endpoint);
   socket.onopen=()=>{if(this.socket===socket)socket.send(JSON.stringify({type:'start',bodies,robot:robot_config,avoidance}));};
   socket.onmessage=event=>{if(this.socket!==socket)return;try{const value=JSON.parse(event.data);if(value.type==='error')throw Error(value.error);if(value.type==='physics'){
    if((value.placement_id??0)<this.placement_id)return;
    // 全物理モード共通の実測標本。姿勢・速度・駆動トルク・物理時刻の一括取得、描画周期から独立
    if(this.enable_dynamics&&value.joints&&Number.isFinite(value.time_sec))this.motion_sample={model:robot.modelId,pose:value.joints,motion:value.motion??null,time_ms:value.time_sec*1000,received_ms:performance.now(),is_simulation_input:true};
    // 物理フォロワーの実測と指令処理は描画周期から独立
    if(robot.pose_source===pose_sources.leader&&value.joints){if(!this.actual_joints)this.leader_received_ms=performance.now();this.actual_joints=value.joints;this.actual=Object.fromEntries(robot.actuated.map(joint=>[joint.name,value.joints[joint.name]]));this.motion_state=value.motion??null;if(value.is_leader_stopped&&!this.is_leader_stopped)this.hold_leader('物理側のリーダー入力失効。物理の再開始が必要');}
    this.latest=value;
   }if(value.type==='ready'){this.can_place_objects=value.can_place_objects===true;$('physics-status').textContent='MuJoCo接続済み';environment.object_interaction?.refresh();}
   if(value.type==='placed'&&value.placement_id===this.pending_placement?.placement_id){
    const pending=this.pending_placement;this.pending_placement=null;this.placement_id=value.placement_id;
    if((this.latest?.placement_id??0)<this.placement_id)this.latest=null;
    clearTimeout(pending.timer);pending.resolve();
   }
   }catch(error){this.stop('物理エラー：'+error.message);}};
   socket.onerror=()=>{if(this.socket===socket)this.stop('物理ブリッジに接続できません。一括起動とMuJoCoの導入を確認してください');};
   socket.onclose=()=>{if(this.socket===socket)this.stop('物理接続が切れました');};
   if(robot.pose_source===pose_sources.leader)this.leader_timer=setInterval(()=>{if(this.actual_joints&&!this.is_leader_stopped&&performance.now()-this.leader_received_ms>this.max_leader_age_ms)this.hold_leader('リーダー入力の失効。物理の再開始が必要');this.send_poses();},33);
   environment.object_interaction?.refresh();
  }catch(error){this.stop('物理エラー：'+error.message);}
 }
 update_scene_signature(){this.scene_signature=JSON.stringify(this.environment.getState());}
 placement_reason(item){
  if(!this.socket||!item||!this.is_physics_enabled(item))return '';
  if(!this.can_place_objects)return '物理ブリッジの更新を反映してから物理を再開始してください';
  if(this.socket.readyState!==WebSocket.OPEN)return '物理接続を確認してください';
  return this.pending_placement?'配置を反映中です':'';
 }
 // 同一物理セッションへの配置要求。worker完了通知後だけブラウザ配置の確定が可能
 place_object(item,world){
  const reason=this.placement_reason(item);
  if(reason)return Promise.reject(Error(reason));
  const position=new THREE.Vector3(),quaternion=new THREE.Quaternion(),scale=new THREE.Vector3();world.decompose(position,quaternion,scale);
  return new Promise((resolve,reject)=>{
   const placement_id=this.placement_id+1;
   const timer=setTimeout(()=>this.stop('配置の反映が時間切れです'),5000);
   this.pending_placement={item,placement_id,resolve,reject,timer};
   try{this.socket.send(JSON.stringify({type:'place',placement_id,pose:{id:'object_'+item.id,position:position.toArray(),quaternion:quaternion.toArray()}}));}
   catch(error){this.stop('配置送信エラー：'+error.message);}
  });
 }
 // 入力元共通の実測参照。モデル・接続・受信鮮度の確認
 get_motion_sample(now_ms=performance.now()){
  const sample=this.motion_sample;
  return this.socket&&this.enable_dynamics&&this.robot()===this.robot_model&&sample?.model===this.robot_model.modelId&&now_ms-sample.received_ms>=0&&now_ms-sample.received_ms<=1000?sample:null;
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
  if(window.simulator?.avoidance_replay?.has_pose_control)return;
  if(!this.socket||!this.enable_dynamics||this.robot()!==this.robot_model)return;
  const robot=this.robot();
  // 指令値の回収と物理姿勢の維持。受信のない描画フレームでも目標への瞬間移動を防止
  if(robot.pose_source!==pose_sources.leader)for(const [name,value] of Object.entries(robot.getPose()))if(value!==this.actual[name])this.targets[name]=value;
  const actual=this.actual_joints??this.actual;
  // 同じ実測姿勢の再適用なし。通信・描画からの重複呼出し時も関節行列の更新は差分のみ
  if(Object.entries(actual).some(([name,value])=>robot.joints[name]?.q!==value))robot.set_received_pose(actual);
 }
 set_goal_pose(pose){
  if(!this.socket||!this.enable_dynamics||!this.plugins.can_set_goal())return false;
  const robot=this.robot();
  if(robot.pose_source!==pose_sources.simulator)throw Error('目標姿勢はシミュレータ操作で指定してください');
  if(!pose||Object.keys(pose).length!==robot.actuated.length||robot.actuated.some(joint=>!Number.isFinite(pose[joint.name])||pose[joint.name]<joint.lower||pose[joint.name]>joint.upper))throw Error('目標姿勢の関節角が不正です');
  // 名目目標の一括保持。描画実測の書換え・途中の補間目標の再送なし。
  this.targets={...pose};this.send_poses();return true;
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
  const poses=this.kinematic.filter(({id})=>id!=='object_'+this.pending_placement?.item.id).map(({id,group})=>({id,position:group.getWorldPosition(new THREE.Vector3()).toArray(),quaternion:group.getWorldQuaternion(new THREE.Quaternion()).toArray()}));
  this.socket.send(JSON.stringify({type:'poses',poses,joints:this.enable_dynamics?this.targets:undefined,leader_stamp_sec:this.leader_stamp_sec}));
 }
 tick(now){
  const current_robot=this.robot();
  if(this.controls_robot!==current_robot){this.controls_robot=current_robot;$('physics-locks').replaceChildren();for(const name of Object.keys(current_robot.getPose())){const label=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.value=name;input.onchange=()=>this.stop();label.append(input,document.createTextNode(name));$('physics-locks').append(label);}this.refresh_robot_controls();this.refresh_selected_controls();}
  if(!$('physics-panel').hidden&&now-(this.last_scene_list_ms??-Infinity)>=250){this.last_scene_list_ms=now;this.refresh_scene_list();}
  const item=this.environment.selected;
  if(item&&document.activeElement!==$('physics-mass'))$('physics-mass').value=item.physics?.mass??.2;
  if(item&&!$('physics-constraint').contains(document.activeElement)){const c=item.physics?.constraint||{axis:[0,0,1],pivot:[0,0,0],range:item.physics?.mode==='hinge'?[-Math.PI/2,Math.PI/2]:[-.2,.2]};$('physics-axis').selectedIndex=c.axis.findIndex(x=>x!==0);$('physics-pivot').value=c.pivot.join(' ');$('physics-range').value=c.range.map(x=>item.physics?.mode==='hinge'?x*180/Math.PI:x).join(' ');}
  if(!this.socket)return;if(this.robot()!==this.robot_model){this.stop('モデル切替のため停止');return;}
  if(this.base_signature!==JSON.stringify([current_robot.position.toArray(),current_robot.quaternion.toArray()])){this.stop('基台配置の変更により停止。再開始で反映します');return;}
  if(current_robot.pose_source===pose_sources.leader&&this.actual_joints&&!this.is_leader_stopped&&now-this.leader_received_ms>this.max_leader_age_ms)this.hold_leader('リーダー入力の失効。物理の再開始が必要');
  this.sync_robot_pose();
  if(this.latest){
   const frame=this.latest;this.latest=null;if(frame.avoidance){const state=frame.avoidance;$('physics-status').textContent=this.plugins.format(frame);}
   if(this.enable_dynamics&&frame.joints){
    if(!this.actual_joints)this.leader_received_ms=now;this.motion_state=frame.motion??null;this.actual_joints=frame.joints;this.actual={...frame.joints};
    if(!window.simulator?.avoidance_replay?.has_pose_control){current_robot.set_received_pose(frame.joints);this.actual=current_robot.getPose();}
   }
   if(current_robot.pose_source===pose_sources.leader&&frame.is_leader_stopped&&!this.is_leader_stopped)this.hold_leader('物理側のリーダー入力失効。物理の再開始が必要');
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
   this.update_scene_signature();
   this.scene.userData.physics_time_sec=frame.time_sec;if(!frame.avoidance)$('physics-status').textContent=`MuJoCo実行中 · ${frame.time_sec.toFixed(2)} s · 接触 ${frame.contacts}件`;
  }
  if(current_robot.pose_source===pose_sources.leader&&this.is_leader_stopped)$('physics-status').textContent=this.leader_detail??'リーダー入力停止。物理の再開始が必要';
  if(current_robot.pose_source!==pose_sources.leader&&now-this.last_ms>=33){this.last_ms=now;this.send_poses();}
 }
}
