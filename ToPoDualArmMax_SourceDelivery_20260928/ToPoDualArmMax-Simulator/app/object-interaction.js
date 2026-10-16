import * as THREE from 'three';

const labels={direct:'直接配置',pick_place:'把持して移動',push:'押して移動・回転'};
const drag_modes=Object.freeze({select:'select',translate:'translate',rotate:'rotate'});
const rotation_axes=Object.freeze({roll:new THREE.Vector3(1,0,0),pitch:new THREE.Vector3(0,1,0),yaw:new THREE.Vector3(0,0,1)});
const visible=object=>{for(let p=object;p;p=p.parent)if(!p.visible)return false;return true;};
const descendant=(object,parent)=>{for(let p=object;p;p=p.parent)if(p===parent)return true;return false;};
const is_same_transform=(left,right)=>left.elements.every((value,idx)=>Math.abs(value-right.elements[idx])<=1e-8);

// 配置プレビューと直接配置の操作管理。ロボット実行との分離。
export class ObjectInteraction {
 constructor(environment){
  this.environment=environment;this.canvas=environment.renderer.domElement;
  this.has_priority_pick=environment.has_priority_pick??(()=>false);
  this.ray=new THREE.Raycaster();this.hover=null;this.hover_robot=null;this.press=null;this.empty_press=null;this.pending=null;this.mode='direct';this.is_applying=false;
  this.hover_box=new THREE.Box3Helper(new THREE.Box3(),0xffce45);
  this.preview_box=new THREE.Box3Helper(new THREE.Box3(),0x35d5bc);
  for(const helper of [this.hover_box,this.preview_box]){helper.visible=false;helper.material.depthTest=false;helper.renderOrder=1000;environment.overlay.add(helper);}
  this.hover_box.onBeforeRender=()=>{
   if(!this.hover_robot)return;
   if(this.hover_robot!==environment.physics_panel?.robot()||!this.hover_robot.parent||!visible(this.hover_robot)){this.clear_hover();return;}
   this.hover_box.box.setFromObject(this.hover_robot);
  };
  const panel=this.panel=document.createElement('section');panel.id='object-interaction';
  panel.className='object-interaction-toolbar';
  panel.innerHTML=`<div class="object-interaction-row"><select data-role="mode" aria-label="物体操作モード" title="Alt＋左ドラッグで移動、Alt＋右ドラッグでYaw回転">${Object.entries(labels).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select><button data-role="rotate" title="Yaw +15°（T／Shift+Tで逆回転。物体上・仮配置中：Alt＋ホイールでPitch、Alt＋Shift＋ホイールでRoll、Ctrl併用で5°）" aria-label="Yawを15度回転">↻</button><button data-role="apply" title="配置を確定（Enter）">確定</button><button data-role="cancel" title="配置プレビューを取り消す（Esc）" aria-label="配置を取り消す">×</button></div><output data-role="status" role="status" aria-live="polite" hidden></output>`;
  document.getElementById('viewport').append(panel);
  this.status=panel.querySelector('[data-role=status]');this.apply_button=panel.querySelector('[data-role=apply]');this.apply_button.disabled=true;
  panel.querySelector('[data-role=mode]').onchange=event=>{this.mode=event.target.value;this.refresh();};
  panel.querySelector('[data-role=rotate]').onclick=()=>this.rotate(15);
  this.apply_button.onclick=()=>this.apply();
  panel.querySelector('[data-role=cancel]').onclick=()=>this.cancel();
  this.canvas.addEventListener('wheel',event=>this.wheel(event),{capture:true,passive:false});
  this.canvas.addEventListener('pointermove',event=>this.pointer_move(event),true);
  this.canvas.addEventListener('pointerdown',event=>this.pointer_down(event),true);
  this.canvas.addEventListener('dblclick',event=>this.double_click(event),true);
  this.canvas.addEventListener('pointerup',event=>this.pointer_up(event),true);
  this.canvas.addEventListener('pointercancel',()=>this.cancel(),true);
  this.canvas.addEventListener('lostpointercapture',()=>{if(this.press)this.cancel();});
  this.canvas.addEventListener('pointerleave',()=>{this.clear_hover();if(!this.press)this.canvas.style.cursor='';});
  window.addEventListener('blur',()=>this.cancel());
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.cancel();});
  window.addEventListener('keydown',event=>{
   if(event.ctrlKey||event.metaKey||document.querySelector('dialog[open]'))return;
   const key=event.key.toLowerCase();
   if(event.target.closest?.('input,select,textarea,[contenteditable]')&&!(key==='escape'&&event.target.closest('.object-context-menu')))return;
   if(event.altKey){
    if(key==='escape'&&this.pending&&!this.is_applying){this.set_preview_rotation(this.pending.initial_quaternion);event.preventDefault();event.stopImmediatePropagation();}
    return;
   }
   if(key==='escape'&&(this.pending||this.press||this.hover||this.hover_robot||environment.selected_ids.size||document.querySelector('.object-context-menu:not([hidden])'))){this.clear_selection();event.preventDefault();event.stopImmediatePropagation();return;}
   if(key==='enter'&&this.pending){this.apply();event.preventDefault();event.stopImmediatePropagation();}
   if(key==='t'&&(this.pending||environment.selected)){this.rotate(event.shiftKey?-15:15);event.preventDefault();event.stopImmediatePropagation();}
  },true);
 }
 set_status(message,state='idle'){this.status.hidden=state!=='blocked';this.status.textContent=state==='blocked'?message:'';this.status.dataset.state=state;}
 cast(event,root=this.environment.scene){
  const rect=this.canvas.getBoundingClientRect();
  this.ray.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,1-(event.clientY-rect.top)/rect.height*2),this.environment.camera);
  root.updateWorldMatrix(true,true);
  return this.ray.intersectObject(root,true).filter(hit=>hit.object.isMesh&&visible(hit.object));
 }
 pick(event){
  const hit=this.cast(event)[0];
  return hit?this.environment.items.find(item=>descendant(hit.object,item.group)):null;
 }
 pick_preview(event){return this.pending?this.cast(event,this.pending.ghost)[0]??null:null;}
 clear_hover(){this.hover=null;this.hover_robot=null;this.hover_box.visible=false;}
 clear_selection(){this.cancel();this.environment.select(null);this.hide_context_menu?.();}
 pointer_down(event){
  if(this.is_applying)return;
  const env=this.environment;
  this.empty_press=null;
  if(event.button!==0&&!(event.button===2&&event.altKey&&!event.ctrlKey&&!event.metaKey))return;
  if([env.gizmo,window.simulator?.gizmo].some(gizmo=>gizmo?.enabled&&(gizmo.dragging||gizmo.axis)))return;
  const drag_mode=event.button===2?drag_modes.rotate:event.altKey?drag_modes.translate:drag_modes.select;
  // overlay上の仮配置を優先。通常クリックでの候補破棄・背後の物体選択の防止
  if(!env.editing&&!event.ctrlKey&&!event.metaKey&&this.pick_preview(event)){
   const ghost=this.pending.ghost;
   const support_z=this.pending.support_z??new THREE.Box3().setFromObject(ghost,true).min.z;
   const point=this.ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0,0,1),-support_z),new THREE.Vector3());
   const offset=point?new THREE.Vector3(ghost.position.x-point.x,ghost.position.y-point.y,0):null;
   this.start_press(event,this.pending.item,drag_mode,offset);return;
  }
  // 可視ROS候補へのクリック優先。背後の環境物体による入力の遮断なし。
  if(!event.ctrlKey&&!event.metaKey&&this.has_priority_pick(event.clientX,event.clientY))return;
  const item=this.pick(event);
  if(!item){if(event.button===0)this.empty_press={x:event.clientX,y:event.clientY,id:event.pointerId,has_moved:false};return;}
  if(env.editing)return;
  if(!event.altKey||event.ctrlKey||event.metaKey){
   this.cancel(false);
   if(event.ctrlKey||event.metaKey){event.preventDefault();event.stopImmediatePropagation();env.select(item.id,true);}
   else env.select(item.id);
   return;
  }
  // 物体上のAltドラッグを移動・回転へ割当て
  this.cancel(false);env.select(item.id);this.start_press(event,item,drag_mode);
  this.set_status(item.group.name+' を選択。ドラッグで配置先、Tで向きを指定');
 }
 // 新規配置と仮配置の持ち直しで共通のポインター所有・視点操作の復帰管理
 start_press(event,item,drag_mode,offset=null){
  event.preventDefault();event.stopImmediatePropagation();this.release();this.clear_hover();this.hide_context_menu?.();
  this.has_orbit=this.environment.orbit.enabled;this.environment.orbit.enabled=false;
  this.press={item,x:event.clientX,y:event.clientY,last_x:event.clientX,id:event.pointerId,button:event.button,drag_mode,has_moved:false,offset};
  this.canvas.setPointerCapture(event.pointerId);this.canvas.style.cursor=drag_mode!==drag_modes.select?'grabbing':'pointer';
 }
 double_click(event){
  if(event.button!==0||event.ctrlKey||event.metaKey||!this.pick_preview(event))return;
  event.preventDefault();event.stopImmediatePropagation();this.apply();
 }
 pointer_move(event){
  if(this.empty_press&&Math.hypot(event.clientX-this.empty_press.x,event.clientY-this.empty_press.y)>4)this.empty_press.has_moved=true;
  if(this.press){
   event.preventDefault();event.stopImmediatePropagation();
   if(this.press.drag_mode===drag_modes.select)return;
   if(Math.hypot(event.clientX-this.press.x,event.clientY-this.press.y)>4)this.press.has_moved=true;
   if(this.press.has_moved){
    if(!this.pending)this.begin(this.press.item);
    if(this.press.drag_mode===drag_modes.rotate){this.rotate((event.clientX-this.press.last_x)*.5);this.press.last_x=event.clientX;}
    else this.move_preview(event);
    this.refresh();
   }
   return;
  }
  // 視点ドラッグ中の全メッシュ照合の省略。物体配置のドラッグは上段で処理。
  if(event.buttons||this.environment.editing){this.clear_hover();this.canvas.style.cursor='';return;}
  if(this.pick_preview(event)){this.clear_hover();this.canvas.style.cursor=event.altKey?'grab':'pointer';return;}
  const hit=this.cast(event)[0],robot=this.environment.physics_panel?.robot();
  this.hover=hit?this.environment.items.find(item=>descendant(hit.object,item.group))||null:null;
  this.hover_robot=hit&&robot&&descendant(hit.object,robot)?robot:null;
  this.hover_box.visible=!!(this.hover||this.hover_robot);
  this.hover_box.material.color.set(this.hover_robot?0x38bdf8:0xffce45);
  if(this.hover_robot)this.hover_box.box.setFromObject(this.hover_robot);
  else if(this.hover)this.hover_box.box.setFromObject(this.hover.group,true);
  this.canvas.style.cursor=this.hover?(event.altKey?'grab':'pointer'):this.hover_robot?'pointer':'';
 }
 release(){
  const press=this.press;this.press=null;this.empty_press=null;
  if(press){if(this.canvas.hasPointerCapture(press.id))this.canvas.releasePointerCapture(press.id);this.environment.orbit.enabled=this.has_orbit;}
  this.canvas.style.cursor='';
 }
 pointer_up(event){
  if(this.empty_press&&event.pointerId===this.empty_press.id&&event.button===0){
   const press=this.empty_press;this.empty_press=null;
   if(!press.has_moved&&Math.hypot(event.clientX-press.x,event.clientY-press.y)<=4&&!this.pick(event))this.clear_selection();
   return;
  }
  if(!this.press||event.pointerId!==this.press.id||event.button!==this.press.button)return;
  event.preventDefault();event.stopImmediatePropagation();this.release();
  if(this.pending)this.refresh();
 }
 begin(item){
  this.dispose_preview();item.group.updateWorldMatrix(true,true);
  const ghost=item.group.clone(true);
  const materials=[],material_cache=new Map();
  // 元の色・テクスチャを共有した照明不要の半透明材質。元材質への変更なし
  const preview_material=source=>{
   if(!material_cache.has(source)){
    const material=new THREE.MeshBasicMaterial({color:source.color?.clone()??0xffffff,map:source.map??null,alphaMap:source.alphaMap??null,vertexColors:source.vertexColors,transparent:true,opacity:source.opacity*.32,depthWrite:false,side:THREE.DoubleSide,alphaTest:source.alphaTest});
    material.visible=source.visible;material.toneMapped=source.toneMapped;
    material_cache.set(source,material);materials.push(material);
   }
   return material_cache.get(source);
  };
  // 形状の共有と、センサ対象外のoverlayへの配置
  ghost.traverse(object=>{if(object.isMesh){object.material=Array.isArray(object.material)?object.material.map(preview_material):preview_material(object.material);object.castShadow=false;object.receiveShadow=false;}});
  const matrix=item.group.matrixWorld.clone();matrix.decompose(ghost.position,ghost.quaternion,ghost.scale);
  this.environment.overlay.add(ghost);
  // 物理更新用のsourceとは独立した、回転操作の復帰先
  this.pending={item,ghost,materials,source:matrix,initial_quaternion:ghost.quaternion.clone(),has_source_changed:false,has_surface:true,support_z:null};
  this.preview_box.visible=true;this.refresh();
 }
 move_preview(event){
  const pending=this.pending,env=this.environment;
  const hit=this.cast(event).find(hit=>!descendant(hit.object,pending.item.group));
  const normal=hit?.face?.normal.clone().applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld));
  // 床・テーブル・物体上面のみ。ロボットや壁面への吸着なし。
  const has_support=hit&&(descendant(hit.object,env.table)||env.items.some(item=>descendant(hit.object,item.group))||hit.object.geometry.type==='PlaneGeometry');
  pending.has_surface=!!(has_support&&normal&&normal.z>.95);
  if(!pending.has_surface)return;
  const ghost=pending.ghost;ghost.position.x=hit.point.x+(this.press?.offset?.x??0);ghost.position.y=hit.point.y+(this.press?.offset?.y??0);
  ghost.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(ghost,true);
  ghost.position.z+=hit.point.z-box.min.z;pending.support_z=hit.point.z;ghost.updateMatrixWorld(true);
 }
 wheel(event){
  if(!event.altKey||this.is_applying||!Number.isFinite(event.deltaY)||event.deltaY===0)return;
  // 仮配置・ドラッグ対象・ホバー対象の順で操作対象を解決。Altなしの通常ズームは維持
  let item=this.pending?.item??this.press?.item;
  if(!item){
   if(this.environment.editing||[this.environment.gizmo,window.simulator?.gizmo].some(gizmo=>gizmo?.enabled&&(gizmo.dragging||gizmo.axis))||this.has_priority_pick(event.clientX,event.clientY))return;
   item=this.pick(event);if(!item)return;
  }
  event.preventDefault();event.stopImmediatePropagation();
  if(!this.pending){this.hide_context_menu?.();this.environment.select(item.id);this.begin(item);}
  this.rotate(-Math.sign(event.deltaY)*(event.ctrlKey?5:15),event.shiftKey?'roll':'pitch');
 }
 rotate(deg,axis='yaw'){
  if(!this.pending)return;
  this.set_preview_rotation(this.pending.ghost.quaternion.clone().premultiply(new THREE.Quaternion().setFromAxisAngle(rotation_axes[axis],THREE.MathUtils.degToRad(deg))));
 }
 set_preview_rotation(quaternion){
  if(!this.pending||this.is_applying)return;
  const ghost=this.pending.ghost;
  const bottom=new THREE.Box3().setFromObject(ghost,true).min.z;
  ghost.quaternion.copy(quaternion);
  ghost.updateMatrixWorld(true);ghost.position.z+=bottom-new THREE.Box3().setFromObject(ghost,true).min.z;ghost.updateMatrixWorld(true);this.refresh();
 }
 reason(){
  const pending=this.pending,env=this.environment;
  if(!pending)return '物体をAlt＋左ドラッグしてください';
  if(!env.items.includes(pending.item)||!visible(pending.item.group))return '元の物体が削除または非表示になりました';
  pending.item.group.updateWorldMatrix(true,true);
  if(pending.has_source_changed||!is_same_transform(pending.source,pending.item.group.matrixWorld))return '元の配置が変更されています。取消して再選択してください';
  if(!pending.has_surface)return '配置面がありません。床や水平な天板に合わせてください';
  const ghost=pending.ghost;
  if(!ghost.position.toArray().every(v=>Number.isFinite(v)&&Math.abs(v)<=100))return '配置範囲外です（±100 m）';
  return '';
 }
 execution_reason(){
  if(this.mode!=='direct')return labels[this.mode]+'：計画器未接続（目標指定のみ）';
  return this.environment.physics_panel?.placement_reason(this.pending?.item)??'';
 }
 sync_physics_pose(item,previous_world){
  const pending=this.pending;
  if(pending?.item!==item||!previous_world)return;
  if(!is_same_transform(pending.source,previous_world)){pending.has_source_changed=true;return;}
  // 物理更新だけを元姿勢へ反映。目標の世界座標と手動編集の検出は保持
  pending.source.copy(item.group.matrixWorld);
 }
 refresh(){
  this.apply_button.textContent=this.mode==='direct'?'確定':'未接続';
  const execution_reason=this.execution_reason();
  this.apply_button.title=execution_reason||'配置を確定（Enter）';
  if(!this.pending){this.apply_button.disabled=true;this.set_status('');return;}
  const reason=this.reason();this.apply_button.disabled=!!(reason||execution_reason||this.is_applying);
  if(reason)this.apply_button.title=reason;
  this.preview_box.box.setFromObject(this.pending.ghost,true);
  this.preview_box.material.color.set(reason?0xffa330:0x35d5bc);
  this.set_status(reason,reason?'blocked':'preview');
 }
 async apply(){
  if(!this.pending||this.is_applying)return;
  const reason=this.reason()||this.execution_reason();if(reason){this.refresh();this.set_status(reason,'blocked');return;}
  const {item,ghost}=this.pending;
  ghost.updateWorldMatrix(true,true);const world=ghost.matrixWorld.clone(),physics=this.environment.physics_panel;
  this.is_applying=true;this.release();this.refresh();
  try{
   if(physics?.socket&&physics.is_physics_enabled(item))await physics.place_object(item,world);
   item.group.parent.updateWorldMatrix(true,false);
   const local=item.group.parent.matrixWorld.clone().invert().multiply(world);
   local.decompose(item.group.position,item.group.quaternion,item.group.scale);
   item.group.rotation.reorder('ZYX');
   // 受理済みの配置だけをシーン署名へ反映。物理セッションと他物体・関節状態の保持
   if(physics?.socket)physics.update_scene_signature();
   this.environment.select(item.id);this.environment.syncObject();this.environment.changed();
   this.dispose_preview();this.apply_button.disabled=true;this.set_status('直接配置を完了（ロボット動作なし）','placed');
  }catch(error){this.is_applying=false;this.refresh();this.set_status(error.message,'blocked');}
  finally{this.is_applying=false;}
 }
 dispose_preview(){
  if(this.pending){this.pending.ghost.removeFromParent();for(const material of this.pending.materials)material.dispose();this.pending=null;}
  this.preview_box.visible=false;
 }
 cancel(has_message=true){if(this.is_applying)return;this.release();this.dispose_preview();this.clear_hover();this.apply_button.disabled=true;this.set_status('','cancelled');}
}
