import * as THREE from 'three';
const $=id=>document.getElementById(id);

export function install_object_actions(environment){
 const panel=document.createElement('div');panel.innerHTML=`<div class="section-label">追加位置</div><label>座標系<select id="spawn-parent"><option value="table">テーブル基準</option><option value="world">world基準</option></select></label><div class="field-grid">${['x','y','z'].map(k=>`<label>${k.toUpperCase()} mm<input id="spawn-${k}" type="number" value="0" step="10"></label>`).join('')}</div><p class="sub-note">物体・車両の追加位置。追加後は下の位置欄で移動可能。3D上の物体を右クリックすると削除メニューを表示。</p>`;
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
 const menu=document.createElement('div');menu.hidden=true;menu.style.cssText='position:fixed;z-index:10000;background:white;border:1px solid #bcbcbc;padding:8px;border-radius:6px;box-shadow:0 2px 12px #0003';
 const label=document.createElement('div'),button=document.createElement('button');button.textContent='この物体を削除';menu.append(label,button);document.body.append(menu);
 const canvas=environment.renderer.domElement,ray=new THREE.Raycaster();let press=null,target=null;
 const has_visible_ancestors=object=>{for(let node=object;node;node=node.parent)if(!node.visible)return false;return true;};
 canvas.addEventListener('contextmenu',event=>event.preventDefault());
 canvas.addEventListener('pointerdown',event=>{if(event.button===2)press={x:event.clientX,y:event.clientY};});
 canvas.addEventListener('pointerup',event=>{
  if(event.button!==2||!press)return;const start=press;press=null;
  if(Math.hypot(event.clientX-start.x,event.clientY-start.y)>5)return;
  const rect=canvas.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),environment.camera);
  const hit=ray.intersectObjects(environment.scene.children,true).find(h=>h.object.isMesh&&has_visible_ancestors(h.object));
  target=null;if(hit){for(let node=hit.object;node;node=node.parent){target=environment.items.find(item=>item.group===node);if(target)break;}}
  if(!target){menu.hidden=true;return;}
  environment.select(target.id);label.textContent=target.group.name;menu.hidden=false;
  menu.style.left=Math.max(0,Math.min(event.clientX,innerWidth-menu.offsetWidth))+'px';menu.style.top=Math.max(0,Math.min(event.clientY,innerHeight-menu.offsetHeight))+'px';
 });
 button.onclick=()=>{if(target&&environment.items.includes(target)){environment.select(target.id);$('object-delete').click();}menu.hidden=true;target=null;};
 document.addEventListener('pointerdown',event=>{if(!menu.contains(event.target))menu.hidden=true;});
 document.addEventListener('keydown',event=>{if(event.key==='Escape')menu.hidden=true;});
}
