import * as THREE from 'three';
import {TransformControls} from 'three/addons/controls/TransformControls.js';

const $=id=>document.getElementById(id),rad=Math.PI/180;
export const OBJECTS={cube:'立方体',box:'段ボール箱',sphere:'ボール',cylinder:'円柱',bottle:'ボトル',can:'缶',mug:'マグカップ',tray:'トレー',wall:'壁パネル',cabinet:'キャビネット'};
export const TABLES={wood:'木製ワークテーブル',steel:'ステンレス作業台',round:'丸テーブル',pallet:'低床パレット'};
const palette={cube:'#e69331',box:'#b68a56',sphere:'#4682ca',cylinder:'#8d59b8',bottle:'#257f83',can:'#b73b41',mug:'#d7dfeb',tray:'#c2ccd0',wall:'#b9c0c7',cabinet:'#788a98'};
const material=(color,metalness=0,roughness=.5)=>new THREE.MeshStandardMaterial({color,metalness,roughness});
function addMesh(parent,g,m,xyz=[0,0,0]){const mesh=new THREE.Mesh(g,m);mesh.position.fromArray(xyz);mesh.castShadow=mesh.receiveShadow=true;parent.add(mesh);return mesh;}
function cylinder(parent,r1,r2,h,m,z=0,segments=64){const mesh=addMesh(parent,new THREE.CylinderGeometry(r1,r2,h,segments),m,[0,0,z]);mesh.rotation.x=Math.PI/2;return mesh;}
function disposeGroup(group){group.traverse(x=>{if(x.isMesh){x.geometry.dispose();for(const m of Array.isArray(x.material)?x.material:[x.material])if(!m.userData.shared)m.dispose();}});group.clear();}
function grain(){const c=document.createElement('canvas');c.width=512;c.height=512;const ctx=c.getContext('2d');ctx.fillStyle='#b98b5e';ctx.fillRect(0,0,512,512);for(let y=0;y<512;y++){ctx.strokeStyle=`rgba(74,35,7,${.045+.035*Math.sin(y*1.9)})`;ctx.beginPath();for(let x=0;x<513;x+=8){const yy=y+2*Math.sin(x*.019+y*.07);x?ctx.lineTo(x,yy):ctx.moveTo(x,yy);}ctx.stroke();}const t=new THREE.CanvasTexture(c);t.colorSpace=THREE.SRGBColorSpace;t.wrapS=t.wrapT=THREE.RepeatWrapping;t.repeat.set(2,2);return t;}

export class WorkEnvironment {
  constructor({scene,overlay,camera,renderer,orbit,toast,onEdit,onChange}){
    Object.assign(this,{scene,overlay,camera,renderer,orbit,toast,onEdit,onChange});this.items=[];this.nextId=1;this.selected=null;this.editing=false;
    this.state={type:'wood',x:.50,y:0,z:.12,yaw:0,roll:0,pitch:0,width:.60,depth:.80,height:.26,color:'#ffffff',visible:true};
    this.root=new THREE.Group();this.root.name='Workspace table';scene.add(this.root);this.table=new THREE.Group();this.objectRoot=new THREE.Group();this.root.add(this.table,this.objectRoot);
    this.wood=material('#ffffff',.04,.48);this.wood.map=grain();this.wood.userData.shared=true;
    this.selection=new THREE.Box3Helper(new THREE.Box3(),0x8c1c8c);this.selection.visible=false;overlay.add(this.selection);
    this.gizmo=new TransformControls(camera,renderer.domElement);this.gizmo.setSize(.7);this.gizmo.setMode('translate');overlay.add(this.gizmo.getHelper());this.gizmo.getHelper().visible=false;this.gizmo.enabled=false;
    this.gizmo.addEventListener('dragging-changed',e=>{orbit.enabled=!e.value;});
    this.gizmo.addEventListener('objectChange',()=>{if(this.editTable){const s=this.state,p=this.root.position,r=this.root.rotation;Object.assign(s,{x:p.x,y:p.y,z:p.z,roll:r.x,pitch:r.y,yaw:r.z});this.syncTable();}else this.syncObject();this.changed();});
    const release=()=>{if(this.gizmo.dragging)this.gizmo.pointerUp({button:0});orbit.enabled=true;};window.addEventListener('pointerup',release);window.addEventListener('blur',release);window.addEventListener('pointercancel',release);
    this.ui();this.enhanceUI();this.buildTable();this.add('box',[-.10,-.16]);this.add('bottle',[.04,.02]);this.add('mug',[-.07,.21]);this.select(null);
  }
  enhanceUI(){}
  ui(){
    $('environment-panel').innerHTML=`<div class="panel-heading"><div><span class="eyebrow">WORKSPACE</span><h2>テーブルと物体</h2></div><span class="chip">m / mm</span></div>
    <label class="field-label">テーブルの種類<select id="table-type">${Object.entries(TABLES).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select></label>
    <div class="field-grid">${[['table-x','中心 X',500],['table-y','中心 Y',0],['table-z','天板 Z',120],['table-yaw','Yaw °',0],['table-width','奥行 X',600],['table-depth','幅 Y',800]].map(([id,t,v])=>`<label>${t}<input id="${id}" type="number" value="${v}" step="${id==='table-yaw'?5:10}" aria-label="テーブル ${t}"></label>`).join('')}</div>
    <p class="sub-note">位置は base_footprint 基準の mm。テーブルを動かすと、載せた物体も一緒に移動します。</p>
    <div class="section-label">物体を追加 <span>複数配置・最大30個</span></div><div class="row-actions"><select id="object-type" aria-label="追加する物体">${Object.entries(OBJECTS).map(([k,v])=>`<option value="${k}">${v}</option>`).join('')}</select><button id="object-add">＋ 追加</button></div>
    <label class="field-label">配置済みの物体<select id="object-list" size="4" aria-label="配置済みの物体"></select></label>
    <div id="object-editor" hidden><div class="row-actions"><button id="object-edit">配置マーカー</button><button id="object-snap">天板へ置く</button></div><div class="field-grid">${[['object-x','天板 X',0],['object-y','天板 Y',0],['object-z','天板上 Z',0],['object-yaw','Yaw °',0],['object-scale','倍率',1]].map(([id,t,v])=>`<label>${t}<input id="${id}" type="number" value="${v}" step="${id==='object-scale'?.1:5}" aria-label="物体 ${t}"></label>`).join('')}<label>色<input id="object-color" type="color" value="#e69331" aria-label="物体の色"></label></div><div class="row-actions"><button id="object-copy">複製</button><button id="object-delete">選択物体を削除</button></div></div>
    <div class="row-actions space-top"><button id="scene-save">↓ シーン保存</button><button id="scene-load">↑ シーン読込</button></div><input id="scene-file" type="file" accept=".json" hidden>
    <p class="sub-note">物体の位置・寸法を使って深度を描画します。接触・落下の物理計算は行いません。</p>`;
    $('table-type').onchange=()=>{this.state.type=$('table-type').value;this.buildTable();};
    for(const [key,id] of Object.entries({x:'table-x',y:'table-y',z:'table-z',yaw:'table-yaw',width:'table-width',depth:'table-depth'}))$(id).onchange=()=>{const n=+$(id).value;if(!Number.isFinite(n))return;this.state[key]=key==='yaw'?n*rad:key==='width'||key==='depth'?THREE.MathUtils.clamp(n/1000,.15,2):key==='z'?THREE.MathUtils.clamp(n/1000,-.08,1.5):THREE.MathUtils.clamp(n/1000,-3,3);this.buildTable();};
    $('object-add').onclick=()=>this.add($('object-type').value);$('object-list').onchange=()=>this.select(+$('object-list').value);
    $('object-edit').onclick=()=>this.setEditing(!this.editing);$('object-snap').onclick=()=>{if(this.selected){this.selected.group.position.z=0;this.syncObject();this.changed();}};
    for(const key of ['x','y','z','yaw','scale'])$('object-'+key).onchange=()=>{if(!this.selected)return;const n=+$('object-'+key).value;if(!Number.isFinite(n))return;const g=this.selected.group;if(key==='scale')g.scale.setScalar(THREE.MathUtils.clamp(n,.2,4));else if(key==='yaw')g.rotation.z=n*rad;else g.position[key]=THREE.MathUtils.clamp(n/1000,key==='z'?0:-2,2);this.syncObject();this.changed();};
    $('object-color').oninput=()=>{if(this.selected){this.selected.color=$('object-color').value;this.selected.group.userData.primary.color.set(this.selected.color);this.changed();}};
    $('object-copy').onclick=()=>{if(this.selected){const old=this.selected,n=this.add(old.type,[old.group.position.x+.055,old.group.position.y+.055]);if(n){n.group.scale.copy(old.group.scale);n.group.rotation.copy(old.group.rotation);n.group.position.z=old.group.position.z;n.color=old.color;n.group.userData.primary.color.set(old.color);this.syncObject();}}};
    for(const id of ['table-x','table-y','table-z','table-yaw','table-width','table-depth','object-x','object-y','object-z','object-yaw','object-scale'])$(id).oninput=()=>{if(Number.isFinite($(id).valueAsNumber))$(id).onchange();};
    $('object-delete').onclick=()=>{if(!this.selected)return;const old=this.selected;this.select(null);this.items=this.items.filter(x=>x!==old);this.objectRoot.remove(old.group);disposeGroup(old.group);this.updateList();this.changed();};
  }
  buildTable(){
    disposeGroup(this.table);const s=this.state;this.root.position.set(s.x,s.y,s.z);this.root.rotation.set(s.roll||0,s.pitch||0,s.yaw,'ZYX');this.table.visible=s.visible!==false;this.wood.color.set(s.color||'#ffffff');
    const top=s.type==='steel'?material('#abb6ba',.78,.31):this.wood,leg=material('#535f67',.72,.4);
    if(s.type==='round'){const mesh=cylinder(this.table,1,1,.032,top,-.016,96);mesh.scale.set(s.width/2,1,s.depth/2);}else if(s.type==='pallet'){for(let n=0;n<7;n++)addMesh(this.table,new THREE.BoxGeometry(s.width,s.depth/7*.82,.035),top,[0,-s.depth/2+(n+.5)*s.depth/7,-.0175]);}else addMesh(this.table,new THREE.BoxGeometry(s.width,s.depth,.032),top,[0,0,-.016]);
    const h=Math.max(.02,s.height-.032);for(const x of [-s.width*(s.type==='round'?.29:.39),s.width*(s.type==='round'?.29:.39)])for(const y of [-s.depth*(s.type==='round'?.29:.39),s.depth*(s.type==='round'?.29:.39)])addMesh(this.table,new THREE.BoxGeometry(.028,.028,h),leg,[x,y,-.032-h/2]);
    if(s.type==='steel')addMesh(this.table,new THREE.BoxGeometry(s.width*.85,s.depth*.85,.015),material('#879397',.65,.43),[0,0,-h*.73]);
    for(const [key,id] of Object.entries({x:'table-x',y:'table-y',z:'table-z',yaw:'table-yaw',width:'table-width',depth:'table-depth'}))if(document.activeElement!==$(id))$(id).value=(s[key]*(key==='yaw'?1/rad:1000)).toFixed(0);$('table-type').value=s.type;this.changed();
  }
  createObject(type,color){
    const g=new THREE.Group(),m=material(color,type==='can'?.65:.06,type==='bottle'?.28:.43);g.userData.primary=m;
    if(type==='wall')addMesh(g,new THREE.BoxGeometry(.08,2,1.8),m,[0,0,.9]);
    if(type==='cabinet'){addMesh(g,new THREE.BoxGeometry(.45,.8,1.1),m,[0,0,.55]);for(const y of [-.20,.20]){addMesh(g,new THREE.BoxGeometry(.012,.38,1.02),material('#d7dce0',.5,.3),[.231,y,.55]);addMesh(g,new THREE.BoxGeometry(.018,.012,.13),material('#30363b',.75,.28),[.25,y+Math.sign(-y)*.12,.57]);}}
    if(type==='cube')addMesh(g,new THREE.BoxGeometry(.07,.07,.07),m,[0,0,.035]);
    if(type==='box'){addMesh(g,new THREE.BoxGeometry(.12,.09,.08),m,[0,0,.04]);addMesh(g,new THREE.BoxGeometry(.023,.0902,.0004),material('#d8bb87'),[0,0,.0802]);}
    if(type==='sphere')addMesh(g,new THREE.SphereGeometry(.042,48,32),m,[0,0,.042]);
    if(type==='cylinder')cylinder(g,.032,.032,.10,m,.05);
    if(type==='can'){cylinder(g,.032,.032,.115,m,.0575);for(const z of [.001,.115])cylinder(g,.0325,.0325,.002,material('#b6c1c6',.85,.27),z);const ring=addMesh(g,new THREE.TorusGeometry(.008,.002,8,24),material('#69767d',.8,.3),[.007,0,.117]);}
    if(type==='bottle'){const profile=[[0,0],[.028,0],[.031,.01],[.031,.125],[.014,.155],[.014,.175],[0,.175]].map(p=>new THREE.Vector2(...p));const b=addMesh(g,new THREE.LatheGeometry(profile,64),m);b.rotation.x=Math.PI/2;cylinder(g,.0155,.0155,.018,material('#c3d4d4',.05,.3),.18);cylinder(g,.0313,.0313,.046,material('#e9ede5',.02,.6),.073);}
    if(type==='mug'){const pts=[[0,0],[.031,0],[.037,.012],[.039,.087],[.034,.087],[.031,.014],[0,.014]].map(p=>new THREE.Vector2(...p));const cup=addMesh(g,new THREE.LatheGeometry(pts,64),m);cup.rotation.x=Math.PI/2;const handle=addMesh(g,new THREE.TorusGeometry(.026,.006,16,48),m,[.049,0,.048]);handle.rotation.x=Math.PI/2;}
    if(type==='tray'){addMesh(g,new THREE.BoxGeometry(.14,.10,.006),m,[0,0,.003]);for(const x of [-.068,.068])addMesh(g,new THREE.BoxGeometry(.004,.10,.022),m,[x,0,.012]);for(const y of [-.048,.048])addMesh(g,new THREE.BoxGeometry(.14,.004,.022),m,[0,y,.012]);}
    return g;
  }
  add(type,xy){if(this.items.length>=30){this.toast('配置は30個までです');return null;}const id=this.nextId++,group=this.createObject(type,palette[type]);group.name=OBJECTS[type]+' '+id;const n=this.items.length;group.position.set(...(xy||[-.12+(n%3)*.12,-.22+Math.floor(n/3)%4*.14]),0);this.objectRoot.add(group);const item={id,type,color:palette[type],group};this.items.push(item);this.updateList();this.select(id);this.changed();return item;}
  updateList(){const list=$('object-list');list.replaceChildren();for(const item of this.items){const o=document.createElement('option');o.value=item.id;o.textContent=item.group.name;list.append(o);}if(this.selected)list.value=this.selected.id;}
  select(id){this.selected=this.items.find(x=>x.id===id)||null;$('object-editor').hidden=!this.selected;if(this.selected){$('object-list').value=id;this.syncObject();if(this.editing)this.gizmo.attach(this.selected.group);}else{this.setEditing(false);$('object-list').selectedIndex=-1;}this.selection.visible=!!this.selected;this.changed();}
  setEditing(value){this.editing=!!(value&&this.selected);this.gizmo.enabled=this.editing;this.gizmo.getHelper().visible=this.editing;if(this.editing)this.gizmo.attach(this.selected.group);else this.gizmo.detach();$('object-edit').textContent=this.editing?'配置を完了':'配置マーカー';this.onEdit(this.editing);}
  syncObject(){if(!this.selected)return;const g=this.selected.group;for(const k of ['x','y','z'])if(document.activeElement!==$('object-'+k))$('object-'+k).value=(g.position[k]*1000).toFixed(1);if(document.activeElement!==$('object-yaw'))$('object-yaw').value=(g.rotation.z/rad).toFixed(1);if(document.activeElement!==$('object-scale'))$('object-scale').value=g.scale.x.toFixed(2);$('object-color').value=this.selected.color;}
  changed(){this.root.updateMatrixWorld(true);if(this.selected)this.selection.box.setFromObject(this.selected.group);this.renderer.shadowMap.needsUpdate=true;this.onChange?.();}
  getState(){return {format:'topo-workspace/1',table:{...this.state},objects:this.items.map(x=>({type:x.type,color:x.color,position:x.group.position.toArray(),yaw:x.group.rotation.z,scale:x.group.scale.x}))};}
  load(value){
    const t=value?.table,objects=value?.objects;
    if(value?.format!=='topo-workspace/1'||!t||!TABLES[t.type]||!['x','y','z','yaw','width','depth'].every(k=>Number.isFinite(t[k]))||Math.abs(t.x)>3||Math.abs(t.y)>3||t.z<-.08||t.z>1.5||t.width<.15||t.width>2||t.depth<.15||t.depth>2||!Array.isArray(objects)||objects.length>30)throw Error('シーンの形式・テーブル寸法が不正です');
    for(const o of objects)if(!OBJECTS[o.type]||!/^#[0-9a-f]{6}$/i.test(o.color)||!Array.isArray(o.position)||o.position.length!==3||!o.position.every(x=>Number.isFinite(x)&&Math.abs(x)<=2)||o.position[2]<0||!Number.isFinite(o.yaw)||!Number.isFinite(o.scale)||o.scale<.2||o.scale>4)throw Error('物体の形式・寸法が不正です');
    this.select(null);this.items.forEach(x=>disposeGroup(x.group));this.objectRoot.clear();this.items=[];this.state={...t};this.buildTable();for(const o of objects){const x=this.add(o.type,o.position.slice(0,2));x.group.position.fromArray(o.position);x.group.rotation.z=o.yaw;x.group.scale.setScalar(o.scale);x.color=o.color;x.group.userData.primary.color.set(o.color);}this.updateList();this.select(null);this.changed();
  }
}
