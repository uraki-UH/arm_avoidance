import * as THREE from 'three';
import {ObjectInteraction} from './object-interaction.js';
import {physics_modes,robot_physics_modes} from './physics-ui.js';
const $=id=>document.getElementById(id);

export function install_object_actions(environment){
 environment.object_interaction=new ObjectInteraction(environment);
 const panel=document.createElement('div');panel.innerHTML=`<div class="section-label">追加位置</div><label>座標系<select id="spawn-parent"><option value="table">テーブル基準</option><option value="world">world基準</option></select></label><div class="field-grid">${['x','y','z'].map(k=>`<label>${k.toUpperCase()} mm<input id="spawn-${k}" type="number" value="0" step="10"></label>`).join('')}</div><p class="sub-note">Ctrl＋クリック：複数選択　Delete：削除</p>`;
 $('object-add').parentElement.insertAdjacentElement('beforebegin',panel);
 const read_position=()=>{
  const position=['x','y','z'].map(k=>$('spawn-'+k).valueAsNumber/1000);
  if(position.some(v=>!Number.isFinite(v)||Math.abs(v)>100))throw Error('追加位置は±100000 mmの有限値を指定してください');
  return {position,parent:$('spawn-parent').value};
 };
 const place=(item,settings)=>{
  if(!item)return;
  (settings.parent==='world'?environment.scene:environment.objectRoot).add(item.group);
  item.parent=settings.parent;item.group.position.fromArray(settings.position);
  environment.syncObject();environment.changed();
 };
 $('object-add').onclick=()=>{try{const settings=read_position();place(environment.add($('object-type').value),settings);}catch(e){environment.toast(e.message);}};
 $('vehicle-add').onclick=async()=>{const button=$('vehicle-add');button.disabled=true;try{const settings=read_position();place(await environment.addVehicle($('vehicle-type').value),settings);}catch(e){environment.toast(e.message);}finally{button.disabled=false;}};
 const menu=document.createElement('div');menu.hidden=true;menu.className='object-context-menu';
 const robot_box=new THREE.Box3Helper(new THREE.Box3(),0x38bdf8);robot_box.visible=false;robot_box.material.depthTest=false;robot_box.renderOrder=1000;environment.overlay.add(robot_box);
 const hide_menu=()=>{menu.hidden=true;robot_box.visible=false;};
 environment.object_interaction.hide_context_menu=hide_menu;
 const label=document.createElement('div'),button=document.createElement('button');button.textContent='この物体を削除';button.dataset.action='delete';
 const physics=document.createElement('section');physics.className='object-context-physics';
 physics.innerHTML=`<label><input type="checkbox" data-action="toggle-physics"><span data-role="physics-label">物理を有効にする</span></label><details><summary>詳細設定</summary><label>物理モード<select aria-label="右クリック対象の物理モード"><option value="" disabled>選択物体で異なる設定</option>${Object.entries(physics_modes).map(([mode,name])=>`<option value="${mode}">${name}</option>`).join('')}</select></label><button data-action="apply-physics">選択物体に適用</button><button data-action="open-physics">物理設定を開く</button></details><small>変更後は「物理を開始」で反映</small>`;
 menu.append(label,physics,button);document.body.append(menu);
 const physics_select=physics.querySelector('select'),physics_apply=physics.querySelector('[data-action=apply-physics]');let physics_targets=[];
 const physics_toggle=physics.querySelector('[data-action=toggle-physics]'),saved_physics=new WeakMap();
 physics_toggle.onchange=()=>{
  const panel=environment.physics_panel;if(!panel)return;
  if(target_robot){panel.set_robot_mode(target_robot,physics_toggle.checked?'dynamic':'kinematic');hide_menu();return;}
  if(physics_targets.some(item=>!environment.items.includes(item)))return;
  for(const item of physics_targets){
   if(physics_toggle.checked){
    if((item.physics?.mode||'none')!=='none')continue;
    const previous=saved_physics.get(item);
    panel.set_object_mode([item],previous?.mode||'dynamic');
    if(previous?.constraint)item.physics.constraint=structuredClone(previous.constraint);
   }else if((item.physics?.mode||'none')!=='none'){
    saved_physics.set(item,structuredClone(item.physics));panel.set_object_mode([item],'none');
   }
  }
  hide_menu();
 };
 physics.querySelector('details').addEventListener('toggle',()=>{if(menu.hidden)return;const rect=menu.getBoundingClientRect();menu.style.top=Math.max(0,Math.min(rect.top,innerHeight-menu.offsetHeight))+'px';});
 physics_select.onchange=()=>{physics_apply.disabled=!physics_select.value||!environment.physics_panel;};
 physics_apply.onclick=()=>{
  if(target_robot){
   if(environment.physics_panel?.set_robot_mode(target_robot,physics_select.value)){environment.toast('ロボットの物理モードを設定。「物理」で開始してください');hide_menu();}
   else environment.toast('ロボットが変更されています。右クリックし直してください');
   return;
  }
  if(environment.physics_panel?.set_object_mode(physics_targets,physics_select.value)){
   environment.toast(`${physics_targets.length}個の物理モードを設定。「物理」で開始してください`);hide_menu();
  }else environment.toast('対象が変更されています。右クリックし直してください');
 };
 physics.querySelector('[data-action=open-physics]').onclick=()=>{
  if(target_robot&&environment.physics_panel?.robot()===target_robot){
   document.querySelector('[data-panel=physics]').click();$('physics-robot-mode').focus();
  }else if(target&&environment.items.includes(target)){environment.select(target.id);document.querySelector('[data-panel=physics]').click();}
  hide_menu();
 };
 const canvas=environment.renderer.domElement,ray=new THREE.Raycaster();let press=null,target=null,target_robot=null;
 robot_box.onBeforeRender=()=>{if(target_robot===environment.physics_panel?.robot()&&target_robot?.parent)robot_box.box.setFromObject(target_robot);else hide_menu();};
 const has_visible_ancestors=object=>{for(let node=object;node;node=node.parent)if(!node.visible)return false;return true;};
 canvas.addEventListener('contextmenu',event=>event.preventDefault());
 canvas.addEventListener('pointerdown',event=>{if(event.button===2)press={x:event.clientX,y:event.clientY};});
 canvas.addEventListener('pointerup',event=>{
  if(event.button!==2||!press)return;const start=press;press=null;
  if(Math.hypot(event.clientX-start.x,event.clientY-start.y)>5)return;
  const rect=canvas.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),environment.camera);
  environment.scene.updateMatrixWorld(true);
  const hit=ray.intersectObjects(environment.scene.children,true).find(h=>h.object.isMesh&&has_visible_ancestors(h.object));
  const robot=environment.physics_panel?.robot();
  target=null;target_robot=null;robot_box.visible=false;
  if(hit){for(let node=hit.object;node;node=node.parent){if(node===robot){target_robot=robot;break;}target=environment.items.find(item=>item.group===node);if(target)break;}}
  if(!target&&!target_robot){hide_menu();return;}
  if(target_robot){
   environment.object_interaction.cancel(false);environment.select(null);physics_targets=[];
   physics_select.innerHTML=Object.entries(robot_physics_modes).map(([mode,name])=>`<option value="${mode}">${name}</option>`).join('');
   physics_select.value=$('physics-robot-mode').value;physics_apply.textContent='ロボットに適用';
   label.textContent=`ロボット全体：${robot.name||robot.modelId}`;button.hidden=true;menu.dataset.targetKind='robot';
   robot_box.box.setFromObject(robot);robot_box.visible=true;
  }else{
   if(!environment.selected_ids.has(target.id))environment.select(target.id);
   physics_targets=environment.items.filter(item=>environment.selected_ids.has(item.id));
   const values=new Set(physics_targets.map(item=>item.physics?.mode||'none'));
   physics_select.innerHTML=`<option value="" disabled>選択物体で異なる設定</option>`+Object.entries(physics_modes).map(([mode,name])=>`<option value="${mode}">${name}</option>`).join('');
   physics_select.value=values.size===1?[...values][0]:'';physics_apply.textContent='選択物体に適用';
   const num=environment.selected_ids.size;label.textContent=num>1?`${num}個の物体を選択中`:target.group.name;button.textContent=num>1?`選択した${num}個を削除`:'この物体を削除';button.hidden=false;menu.dataset.targetKind='object';
  }
  physics.querySelector('details').open=false;
  physics.querySelector('[data-role=physics-label]').textContent=target_robot?'関節動力学を有効にする':'物理を有効にする';
  const enabled=target_robot?[$('physics-robot-mode').value==='dynamic']:physics_targets.map(item=>(item.physics?.mode||'none')!=='none');
  physics_toggle.checked=enabled.every(Boolean);physics_toggle.indeterminate=enabled.some(Boolean)&&!enabled.every(Boolean);physics_toggle.disabled=!environment.physics_panel;
  physics_apply.disabled=!physics_select.value||!environment.physics_panel;menu.hidden=false;
  menu.style.left=Math.max(0,Math.min(event.clientX,innerWidth-menu.offsetWidth))+'px';menu.style.top=Math.max(0,Math.min(event.clientY,innerHeight-menu.offsetHeight))+'px';
 });
 button.onclick=()=>{if(target&&environment.items.includes(target)){$('object-delete').click();}hide_menu();target=null;};
 document.addEventListener('pointerdown',event=>{if(!menu.contains(event.target))hide_menu();});
 window.addEventListener('blur',hide_menu);
 document.addEventListener('keydown',event=>{if(event.key==='Escape')hide_menu();if(event.key!=='Delete'||event.ctrlKey||event.metaKey||event.altKey||event.target.closest?.('input,select,textarea,[contenteditable]')||document.querySelector('dialog[open]'))return;if(environment.selected_ids.size){event.preventDefault();$('object-delete').click();hide_menu();target=null;}});
}
