import * as THREE from 'three';

const labels={direct:'直接配置',pick_place:'把持して移動',push:'押して移動・回転'};
const visible=object=>{for(let p=object;p;p=p.parent)if(!p.visible)return false;return true;};
const descendant=(object,parent)=>{for(let p=object;p;p=p.parent)if(p===parent)return true;return false;};

// 配置編集のプレビュー専用。ロボット実行や物理的な可否保証との分離。
export class ObjectInteraction {
 constructor(environment){
  this.environment=environment;this.canvas=environment.renderer.domElement;
  this.ray=new THREE.Raycaster();this.hover=null;this.press=null;this.pending=null;this.mode='direct';
  this.hover_box=new THREE.Box3Helper(new THREE.Box3(),0xffce45);
  this.preview_box=new THREE.Box3Helper(new THREE.Box3(),0x35d5bc);
  for(const helper of [this.hover_box,this.preview_box]){helper.visible=false;helper.material.depthTest=false;helper.renderOrder=1000;environment.overlay.add(helper);}
  const panel=this.panel=document.createElement('section');panel.id='object-interaction';
  panel.className='object-interaction-toolbar';
  panel.innerHTML=`<div class="object-interaction-row"><select data-role="mode" aria-label="物体操作モード" title="物体をドラッグして配置">${Object.entries(labels).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select><button data-role="rotate" title="Yaw +15°（T／Shift+Tで逆回転。ドラッグ中はホイール、Shift併用で5°）" aria-label="Yawを15度回転">↻</button><button data-role="apply" title="配置を確定（Enter）">確定</button><button data-role="cancel" title="配置プレビューを取り消す（Esc）" aria-label="配置を取り消す">×</button></div><output data-role="status" role="status" aria-live="polite" hidden></output>`;
  document.getElementById('viewport').append(panel);
  this.status=panel.querySelector('[data-role=status]');this.apply_button=panel.querySelector('[data-role=apply]');this.apply_button.disabled=true;
  panel.querySelector('[data-role=mode]').onchange=event=>{this.mode=event.target.value;this.refresh();};
  panel.querySelector('[data-role=rotate]').onclick=()=>this.rotate(15);
  this.apply_button.onclick=()=>this.apply();
  panel.querySelector('[data-role=cancel]').onclick=()=>this.cancel();
  this.canvas.addEventListener('wheel',event=>this.wheel(event),{capture:true,passive:false});
  this.canvas.addEventListener('pointermove',event=>this.pointer_move(event),true);
  this.canvas.addEventListener('pointerdown',event=>this.pointer_down(event),true);
  this.canvas.addEventListener('pointerup',event=>this.pointer_up(event),true);
  this.canvas.addEventListener('pointercancel',()=>this.cancel(),true);
  this.canvas.addEventListener('lostpointercapture',()=>{if(this.press)this.cancel();});
  this.canvas.addEventListener('pointerleave',()=>{this.hover=null;this.hover_box.visible=false;if(!this.press)this.canvas.style.cursor='';});
  window.addEventListener('blur',()=>this.cancel());
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.cancel();});
  window.addEventListener('keydown',event=>{
   if(event.ctrlKey||event.metaKey||event.altKey||event.target.closest?.('input,select,textarea,[contenteditable=true]')||document.querySelector('dialog[open]'))return;
   const key=event.key.toLowerCase();
   if(key==='escape'&&(this.pending||this.press)){this.cancel();event.preventDefault();event.stopImmediatePropagation();}
   if(key==='enter'&&this.pending){this.apply();event.preventDefault();event.stopImmediatePropagation();}
   if(key==='t'&&(this.pending||environment.selected)){this.rotate(event.shiftKey?-15:15);event.preventDefault();event.stopImmediatePropagation();}
  },true);
 }
 set_status(message,state='idle'){this.status.hidden=state!=='blocked';this.status.textContent=state==='blocked'?message:'';this.status.dataset.state=state;}
 cast(event){
  const rect=this.canvas.getBoundingClientRect();
  this.ray.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,1-(event.clientY-rect.top)/rect.height*2),this.environment.camera);
  this.environment.scene.updateMatrixWorld(true);
  return this.ray.intersectObjects(this.environment.scene.children,true).filter(hit=>hit.object.isMesh&&visible(hit.object));
 }
 pick(event){
  const hit=this.cast(event)[0];
  return hit?this.environment.items.find(item=>descendant(hit.object,item.group)):null;
 }
 pointer_down(event){
  const env=this.environment;
  if(event.button!==0||env.editing||window.simulator?.gizmo?.dragging)return;
  const item=this.pick(event);if(!item)return;
  event.preventDefault();event.stopImmediatePropagation();this.cancel(false);
  this.has_orbit=env.orbit.enabled;env.orbit.enabled=false;
  env.select(item.id);this.press={item,x:event.clientX,y:event.clientY,id:event.pointerId};
  this.canvas.setPointerCapture(event.pointerId);this.canvas.style.cursor='grabbing';
  this.set_status(item.group.name+' を選択。ドラッグで配置先、Tで向きを指定');
 }
 pointer_move(event){
  if(this.press){
   event.preventDefault();event.stopImmediatePropagation();
   if(!this.pending&&Math.hypot(event.clientX-this.press.x,event.clientY-this.press.y)>4)this.begin(this.press.item);
   if(this.pending){this.move_preview(event);this.refresh();}
   return;
  }
  if(this.environment.editing)return;
  this.hover=this.pick(event)||null;this.hover_box.visible=!!this.hover;
  if(this.hover)this.hover_box.box.setFromObject(this.hover.group,true);
  this.canvas.style.cursor=this.hover?'grab':'';
 }
 release(){
  const press=this.press;this.press=null;
  if(press){if(this.canvas.hasPointerCapture(press.id))this.canvas.releasePointerCapture(press.id);this.environment.orbit.enabled=this.has_orbit;}
  this.canvas.style.cursor='';
 }
 pointer_up(event){
  if(!this.press||event.pointerId!==this.press.id)return;
  event.preventDefault();event.stopImmediatePropagation();this.release();
  if(this.pending)this.refresh();
 }
 begin(item){
  this.dispose_preview();item.group.updateWorldMatrix(true,true);
  const ghost=item.group.clone(true);
  const material=new THREE.MeshBasicMaterial({color:0x43dbc7,transparent:true,opacity:.32,depthWrite:false,side:THREE.DoubleSide});
  const materials=[material];
  // 形状の共有、照明不要のプレビュー専用材質。センサー対象外のoverlayへ配置。
  ghost.traverse(object=>{if(object.isMesh){object.material=material;object.castShadow=false;object.receiveShadow=false;}});
  const matrix=item.group.matrixWorld.clone();matrix.decompose(ghost.position,ghost.quaternion,ghost.scale);
  this.environment.overlay.add(ghost);
  this.pending={item,ghost,materials,source:matrix,has_surface:true,support_z:null};
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
  const ghost=pending.ghost;ghost.position.x=hit.point.x;ghost.position.y=hit.point.y;
  ghost.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(ghost,true);
  ghost.position.z+=hit.point.z-box.min.z;pending.support_z=hit.point.z;ghost.updateMatrixWorld(true);
 }
 wheel(event){
  if(!this.press||event.ctrlKey||!Number.isFinite(event.deltaY)||event.deltaY===0)return;
  event.preventDefault();event.stopImmediatePropagation();
  if(!this.pending)this.begin(this.press.item);
  this.rotate(-Math.sign(event.deltaY)*(event.shiftKey?5:15));
 }
 rotate(deg){
  if(!this.pending){if(!this.environment.selected)return;this.begin(this.environment.selected);}
  const ghost=this.pending.ghost;
  const bottom=new THREE.Box3().setFromObject(ghost,true).min.z;
  ghost.quaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1),THREE.MathUtils.degToRad(deg)));
  ghost.updateMatrixWorld(true);ghost.position.z+=bottom-new THREE.Box3().setFromObject(ghost,true).min.z;ghost.updateMatrixWorld(true);this.refresh();
 }
 reason(){
  const pending=this.pending,env=this.environment;
  if(!pending)return '物体を選択してドラッグしてください';
  if(!env.items.includes(pending.item)||!visible(pending.item.group))return '元の物体が削除または非表示になりました';
  pending.item.group.updateWorldMatrix(true,true);
  if(pending.source.elements.some((v,i)=>Math.abs(v-pending.item.group.matrixWorld.elements[i])>1e-8))return '元の配置が変更されています。取消して再選択してください';
  if(window.simulator?.physics_panel?.socket)return '物理実行中です。物理を停止してから配置してください';
  if(!pending.has_surface)return '配置面がありません。床や水平な天板に合わせてください';
  const ghost=pending.ghost;
  if(!ghost.position.toArray().every(v=>Number.isFinite(v)&&Math.abs(v)<=100))return '配置範囲外です（±100 m）';
  if(this.mode!=='direct')return labels[this.mode]+'：計画器未接続。IK・衝突・把持／接触の可否は未判定';
  return '';
 }
 refresh(){
  this.apply_button.textContent=this.mode==='direct'?'確定':'未接続';
  if(!this.pending){this.apply_button.disabled=true;return;}
  const reason=this.reason();this.apply_button.disabled=!!reason;
  this.preview_box.box.setFromObject(this.pending.ghost,true);
  this.preview_box.material.color.set(reason?0xffa330:0x35d5bc);
  this.set_status(reason||'配置プレビュー：確定で直接移動（衝突・支持の安全保証なし）',reason?'blocked':'preview');
 }
 apply(){
  if(!this.pending)return;
  const reason=this.reason();if(reason){this.refresh();return;}
  const {item,ghost}=this.pending;
  ghost.updateWorldMatrix(true,true);item.group.parent.updateWorldMatrix(true,false);
  const local=item.group.parent.matrixWorld.clone().invert().multiply(ghost.matrixWorld);
  local.decompose(item.group.position,item.group.quaternion,item.group.scale);
  item.group.rotation.reorder('ZYX');this.environment.select(item.id);this.environment.syncObject();this.environment.changed();
  this.dispose_preview();this.apply_button.disabled=true;this.set_status('直接配置を完了（ロボット動作なし）','placed');
 }
 dispose_preview(){
  if(this.pending){this.pending.ghost.removeFromParent();for(const material of this.pending.materials)material.dispose();this.pending=null;}
  this.preview_box.visible=false;
 }
 cancel(has_message=true){this.release();this.dispose_preview();this.apply_button.disabled=true;this.set_status('','cancelled');}
}
