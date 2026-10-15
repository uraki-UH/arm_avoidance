import * as three from 'three';
import {OrbitControls as orbit_controls} from '/vendor/three/examples/jsm/controls/OrbitControls.js';

const element = id => document.getElementById(id);
const state = {model:null, region_idx:0, neighbor_idx:null, pose_idx:0, timer:null, is_pending:false};
const max_num_display_cells = 50000;
const diagnostics = {is_ready:false, num_renders:0, num_voxel_updates:0, num_display_cells:0};
globalThis.region_viewer = {diagnostics};

function status(message, is_error=false) {
  element('status').textContent=message;
  element('status').dataset.is_error=String(is_error);
}

function validate_model(model) {
  const finite_row=(value,size)=>Array.isArray(value)&&value.length===size&&value.every(item=>Number.isFinite(item)&&Math.abs(item)<=1048576);
  if(model?.format!=='gng_region_viewer_v1'||model.has_hardware_output!==false)throw Error('領域Viewer用JSONの形式が一致しません');
  if(!['joint_box','convex_combination_of_joint_vertices','joint_ellipsoid_with_limits'].includes(model.region_type))throw Error('領域制約の形式が一致しません');
  const num_joints=model.joint_names?.length, num_spheres=model.sphere_radii?.length;
  if(!num_joints||num_joints>128||!model.joint_names.every(name=>typeof name==='string'&&name.length<=160))throw Error('関節名の形式が不正です');
  if(!Array.isArray(model.regions)||!model.regions.length||model.regions.length>2048)throw Error('領域数が範囲外です');
  if(!Number.isFinite(model.voxel_size)||model.voxel_size<=0||model.voxel_size>10)throw Error('格子幅が不正です');
  if(!Array.isArray(model.grid_cells)||model.grid_cells.length>2000000||!model.grid_cells.every(row=>finite_row(row,3)&&row.every(value=>Number.isSafeInteger(value)&&Math.abs(value)<=1048576)))throw Error('格子座標の形式が不正です');
  if(!num_spheres||num_spheres>4096||!model.sphere_radii.every(value=>Number.isFinite(value)&&value>0&&value<1000))throw Error('球形状が不正です');
  if(!Array.isArray(model.joint_types)||model.joint_types.length!==num_joints||!Array.isArray(model.joint_limits)||model.joint_limits.length!==num_joints||!model.joint_limits.every(row=>finite_row(row,2)&&row[0]<row[1]))throw Error('関節設定が不正です');
  if(!Array.isArray(model.active_indices)||!model.active_indices.length||!model.active_indices.every(idx=>Number.isInteger(idx)&&idx>=0&&idx<num_joints)||new Set(model.active_indices).size!==model.active_indices.length)throw Error('対象関節が不正です');
  let num_relations=0;
  for(const [idx,region] of model.regions.entries()) {
    if(model.region_type==='joint_box'&&typeof region.has_index!=='boolean')throw Error('箱領域の索引状態が不正です');
    if(region.region_idx!==idx||!finite_row(region.reference,num_joints)||!finite_row(region.min_position,num_joints)||!finite_row(region.max_position,num_joints)||region.min_position.some((value,joint_idx)=>value>region.max_position[joint_idx]))throw Error('領域の関節角度が不正です');
    if(!Number.isInteger(region.affine_rank)||region.affine_rank<0||region.affine_rank>model.active_indices.length)throw Error('領域の数値次元が不正です');
    if(!Array.isArray(region.vertices)||!region.vertices.length||region.vertices.length>512||!region.vertices.every(row=>finite_row(row,num_joints)))throw Error('領域の頂点が不正です');
    if(region.generators!==null&&(!Array.isArray(region.generators)||region.generators.length!==num_joints||!region.generators.every(row=>finite_row(row,num_joints))))throw Error('二次領域の生成行列が不正です');
    for(const name of ['occupied','exclusive','core','outer','inner']) {
      const cells=region[name];
      if(cells===null&&['outer','inner'].includes(name))continue;
      if(!Array.isArray(cells)||!cells.every(value=>Number.isSafeInteger(value)&&value>=0&&value<model.grid_cells.length))throw Error('占有セルの参照が不正です');
      num_relations+=cells.length;
    }
    if(!Array.isArray(region.neighbors)||!region.neighbors.every(value=>Number.isInteger(value)&&value>=0&&value<model.regions.length))throw Error('隣接領域が不正です');
    if(!Array.isArray(region.poses)||!region.poses.length||region.poses.length>32||!region.poses.every(pose=>finite_row(pose.positions,num_joints)&&Array.isArray(pose.centers)&&pose.centers.length===num_spheres&&pose.centers.every(row=>finite_row(row,3))))throw Error('姿勢例の形式が不正です');
  }
  if(num_relations>4000000)throw Error('セル関係数が表示上限を超えています');
  if(model.parents!==undefined&&model.parents!==null) {
    if(!Array.isArray(model.parents)||!model.parents.length||model.parents.length>model.regions.length)throw Error('親領域の件数が不正です');
    const leaves=[];
    for(const [idx,parent] of model.parents.entries()) {
      if(parent.parent_idx!==idx||!Array.isArray(parent.leaf_regions)||!parent.leaf_regions.length||parent.leaf_regions.length>16||!parent.leaf_regions.every(value=>Number.isInteger(value)&&value>=0&&value<model.regions.length))throw Error('親・葉の対応が不正です');
      leaves.push(...parent.leaf_regions);
    }
    if(leaves.length!==model.regions.length||new Set(leaves).size!==model.regions.length)throw Error('親・葉の対応に欠落または重複があります');
  }
  return model;
}

const renderer=new three.WebGLRenderer({antialias:true,alpha:false});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.setClearColor(0xfafcfe);
element('viewport').append(renderer.domElement);
const scene=new three.Scene();
const camera=new three.PerspectiveCamera(42,1,.001,1000);
camera.up.set(0,0,1);camera.position.set(2,-2,1.7);
const controls=new orbit_controls(camera,renderer.domElement);
controls.enableDamping=false;
scene.add(new three.AmbientLight(0xffffff,2));
const light=new three.DirectionalLight(0xffffff,2);light.position.set(3,-4,6);scene.add(light);
const grid=new three.GridHelper(4,40,0xa9bccb,0xe3ebf1);grid.rotation.x=Math.PI/2;scene.add(grid);
scene.add(new three.AxesHelper(.25));
const voxel_group=new three.Group();scene.add(voxel_group);
const box_geometry=new three.BoxGeometry(1,1,1), sphere_geometry=new three.SphereGeometry(1,12,8);
const materials=[0xe79732,0xaab7c3,0x209a94].map((color,idx)=>new three.MeshLambertMaterial({color,transparent:true,opacity:idx===1?.22:.46,depthWrite:false}));
const pose_material=new three.MeshStandardMaterial({color:0x24475f,roughness:.65});
const comparison_material=new three.MeshStandardMaterial({color:0x209a94,transparent:true,opacity:.55,roughness:.65});
const transform=new three.Object3D();
let pose_mesh=null, comparison_mesh=null;
const displayed_bounds=new three.Box3();

function render() {
  state.is_pending=false;
  renderer.render(scene,camera);
  diagnostics.num_renders++;
  diagnostics.num_triangles=renderer.info.render.triangles;
  diagnostics.num_geometries=renderer.info.memory.geometries;
}
function request_render() {
  if(!state.is_pending){state.is_pending=true;requestAnimationFrame(render);}
}
controls.addEventListener('change',request_render);
new ResizeObserver(()=>{
  const rect=element('viewport').getBoundingClientRect();
  renderer.setSize(rect.width,rect.height,false);camera.aspect=rect.width/Math.max(1,rect.height);camera.updateProjectionMatrix();
  request_render();draw_projection();
}).observe(element('viewport'));

function remove_mesh(mesh) {
  if(mesh){scene.remove(mesh);mesh.dispose();}
}
function stop_playback() {
  if(state.timer!==null)clearInterval(state.timer);
  state.timer=null;element('play').textContent='姿勢例を再生';element('play').setAttribute('aria-pressed','false');
}
function selected() {return state.model?.regions[state.region_idx];}
function comparison() {return state.neighbor_idx===null?null:state.model.regions[state.neighbor_idx];}
function unit_scale(idx) {return state.model.joint_types[idx]==='prismatic'?1:180/Math.PI;}
function unit_label(idx) {return state.model.joint_types[idx]==='prismatic'?'m':'deg';}

function update_pose() {
  if(!state.model)return;
  const has_pose=element('enable_pose').checked;
  for(const [mesh,pose] of [[pose_mesh,selected().poses[state.pose_idx]],[comparison_mesh,comparison()?.poses[0]]]) {
    mesh.visible=has_pose&&!!pose;
    if(!mesh.visible)continue;
    pose.centers.forEach((center,idx)=>{
      transform.position.fromArray(center);transform.scale.setScalar(state.model.sphere_radii[idx]);transform.updateMatrix();mesh.setMatrixAt(idx,transform.matrix);
    });
    mesh.instanceMatrix.needsUpdate=true;mesh.computeBoundingSphere();
  }
  element('pose').value=String(state.pose_idx);
  state.model.active_indices.forEach(idx=>{element('current_'+idx).textContent=(selected().poses[state.pose_idx].positions[idx]*unit_scale(idx)).toFixed(2);});
  draw_projection();request_render();
}

function layer_cells(region) {return region?.[element('layer').value]??null;}
function update_voxels() {
  if(!state.model)return;
  while(voxel_group.children.length){const mesh=voxel_group.children[0];voxel_group.remove(mesh);mesh.dispose();}
  displayed_bounds.makeEmpty();
  const own=layer_cells(selected()), other=comparison()?layer_cells(comparison()):[];
  const own_set=new Set(own??[]),other_set=new Set(other??[]);
  const sets=[[...(own??[])].filter(value=>!other_set.has(value)),[...(own??[])].filter(value=>other_set.has(value)),[...(other??[])].filter(value=>!own_set.has(value))];
  const num_cells=sets.reduce((count,values)=>count+values.length,0);
  diagnostics.num_display_cells=num_cells;diagnostics.num_voxel_updates++;
  if(num_cells>max_num_display_cells){status('表示上限50,000セル超過。別の層またはノードを選択してください。',true);request_render();return;}
  const size=state.model.voxel_size;
  sets.forEach((values,group_idx)=>{
    if(!values.length)return;
    const mesh=new three.InstancedMesh(box_geometry,materials[group_idx],values.length);
    mesh.name=['selected_only','common','comparison_only'][group_idx];
    values.forEach((palette_idx,idx)=>{
      const center=state.model.grid_cells[palette_idx].map(value=>(value+.5)*size);
      transform.position.fromArray(center);transform.scale.setScalar(size*.96);transform.updateMatrix();mesh.setMatrixAt(idx,transform.matrix);
      displayed_bounds.expandByPoint(new three.Vector3(...center));
    });
    mesh.instanceMatrix.needsUpdate=true;mesh.computeBoundingSphere();voxel_group.add(mesh);
  });
  const labels=[`N${state.region_idx}`,`${own?.length??'未登録'}セル`];
  if(comparison())labels.push(`比較 N${state.neighbor_idx}`,`共通 ${sets[1].length}セル`);
  if(own===null||other===null)labels.push('照会索引未登録');
  else if(num_cells===0)labels.push('この層のセルは0個');
  status(labels.join(' · '));request_render();
}

function fit_scene() {
  if(!state.model)return;
  const bounds=displayed_bounds.clone();
  if(element('enable_pose').checked)selected().poses[state.pose_idx].centers.forEach((center,idx)=>{
    const radius=state.model.sphere_radii[idx],point=new three.Vector3(...center);
    bounds.expandByPoint(point.clone().addScalar(radius));bounds.expandByPoint(point.clone().addScalar(-radius));
  });
  if(bounds.isEmpty()){bounds.min.set(-.5,-.5,0);bounds.max.set(.5,.5,1);}
  const center=bounds.getCenter(new three.Vector3()),extent=bounds.getSize(new three.Vector3());
  const dist=Math.max(extent.length(),.2)/Math.sin(camera.fov*Math.PI/360)*.7/Math.min(1,camera.aspect);
  const direction=camera.position.clone().sub(controls.target).normalize();
  controls.target.copy(center);camera.position.copy(center).addScaledVector(direction,dist);camera.far=Math.max(100,dist*10);camera.updateProjectionMatrix();controls.update();request_render();
}

function hull(points) {
  const sorted=[...new Map(points.map(point=>[point.join(','),point])).values()].sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
  if(sorted.length<3)return sorted;
  const cross=(o,a,b)=>(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);
  const lower=[],upper=[];
  for(const point of sorted){while(lower.length>=2&&cross(lower.at(-2),lower.at(-1),point)<=0)lower.pop();lower.push(point);}
  for(const point of sorted.toReversed()){while(upper.length>=2&&cross(upper.at(-2),upper.at(-1),point)<=0)upper.pop();upper.push(point);}
  return lower.slice(0,-1).concat(upper.slice(0,-1));
}
function projection(region,x,y) {
  if(state.model.region_type==='joint_box')return [[region.min_position[x],region.min_position[y]],[region.max_position[x],region.min_position[y]],[region.max_position[x],region.max_position[y]],[region.min_position[x],region.max_position[y]]].map(([a,b])=>[a*unit_scale(x),b*unit_scale(y)]);
  if(!region.generators)return hull(region.vertices.map(q=>[q[x]*unit_scale(x),q[y]*unit_scale(y)]));
  const first=region.generators[x],second=region.generators[y];
  const a=first.reduce((sum,v)=>sum+v*v,0),b=first.reduce((sum,v,idx)=>sum+v*second[idx],0),c=second.reduce((sum,v)=>sum+v*v,0);
  const delta=Math.hypot(a-c,2*b),angle=.5*Math.atan2(2*b,a-c),r1=Math.sqrt(Math.max(0,(a+c+delta)/2)),r2=Math.sqrt(Math.max(0,(a+c-delta)/2));
  return Array.from({length:80},(_,idx)=>{
    const t=idx*Math.PI/40,u=r1*Math.cos(t),v=r2*Math.sin(t);
    return [(region.reference[x]+u*Math.cos(angle)-v*Math.sin(angle))*unit_scale(x),(region.reference[y]+u*Math.sin(angle)+v*Math.cos(angle))*unit_scale(y)];
  });
}
function draw_projection() {
  if(!state.model)return;
  const canvas=element('joint_canvas'),rect=canvas.getBoundingClientRect(),ratio=Math.min(devicePixelRatio,2);
  canvas.width=Math.max(1,Math.floor(rect.width*ratio));canvas.height=Math.max(1,Math.floor(rect.height*ratio));
  const context=canvas.getContext('2d');context.scale(ratio,ratio);
  const x=Number(element('axis_x').value),y=Number(element('axis_y').value),first=projection(selected(),x,y),second=comparison()?projection(comparison(),x,y):[];
  const pose=selected().poses[state.pose_idx].positions,point=[pose[x]*unit_scale(x),pose[y]*unit_scale(y)],all=[...first,...second,point];
  const lower=[0,1].map(idx=>Math.min(...all.map(value=>value[idx]))),upper=[0,1].map(idx=>Math.max(...all.map(value=>value[idx])));
  const span=upper.map((value,idx)=>Math.max(value-lower[idx],.01));
  const to_screen=p=>[42+(p[0]-lower[0]+span[0]*.08)/(span[0]*1.16)*(rect.width-56),rect.height-27-(p[1]-lower[1]+span[1]*.08)/(span[1]*1.16)*(rect.height-44)];
  context.strokeStyle='#dce6ee';context.lineWidth=1;
  for(let idx=0;idx<5;idx++){const xx=42+(rect.width-56)*idx/4,yy=17+(rect.height-44)*idx/4;context.beginPath();context.moveTo(xx,17);context.lineTo(xx,rect.height-27);context.moveTo(42,yy);context.lineTo(rect.width-14,yy);context.stroke();}
  for(const [points,color,fill] of [[second,'#209a94','#209a9430'],[first,'#dc8a23','#e7973240']]) {
    if(!points.length)continue;
    context.beginPath();points.forEach((p,idx)=>{const v=to_screen(p);idx?context.lineTo(...v):context.moveTo(...v);});
    if(points.length>2){context.closePath();context.fillStyle=fill;context.fill();}context.strokeStyle=color;context.lineWidth=2;context.stroke();
  }
  const current=to_screen(point);context.beginPath();context.arc(...current,4,0,Math.PI*2);context.fillStyle='#24475f';context.fill();
  context.font='11px system-ui';context.fillStyle='#647b8c';context.fillText(lower[0].toFixed(2),42,rect.height-8);context.fillText(upper[0].toFixed(2),Math.max(70,rect.width-54),rect.height-8);context.fillText(upper[1].toFixed(2),1,24);context.fillText(lower[1].toFixed(2),1,rect.height-28);
  element('projection_note').textContent=state.model.region_type==='joint_box'?'角度箱の投影 · 他関節も各上下限内の全組合せ':selected().generators?'楕円体の投影外包 · 関節制限との共通部分は未反映':'全頂点の凸包投影 · 他関節は固定しない表示';
}

function select_node(idx) {
  stop_playback();state.region_idx=idx;state.pose_idx=0;state.neighbor_idx=null;
  const region=selected();element('node').value=String(idx);
  const parent=state.model.parents?.find(value=>value.leaf_regions.includes(idx));
  element('selection_title').textContent=parent?`親 P${parent.parent_idx} / 内部葉 L${idx}`:`領域ノード N${idx}`;
  element('neighbor').replaceChildren(new Option('なし',''));
  for(const [label,indices] of [['隣接',region.neighbors],['その他',state.model.regions.map((_,value)=>value).filter(value=>value!==idx&&!region.neighbors.includes(value))]]) {
    const group=document.createElement('optgroup');group.label=label;
    indices.forEach(value=>group.append(new Option('N'+value,String(value))));element('neighbor').append(group);
  }
  element('pose').replaceChildren(...region.poses.map((_,value)=>new Option(value===0?'基準姿勢':`姿勢例 ${value}`,String(value))));
  const metrics=[['関節領域の数値次元',`${region.affine_rank} / ${state.model.active_indices.length}`],['占有包絡',`${region.occupied.length} セル`],['セル体積和',`${(region.occupied.length*state.model.voxel_size**3).toFixed(6)} m³`],['他領域にない占有',`${region.exclusive.length} セル`],['格子幅',`${(state.model.voxel_size*100).toFixed(2)} cm`]];
  if(state.model.region_type==='joint_box')metrics.unshift(['索引状態',region.has_index?'登録済み':'未索引・要詳細検査']);
  element('metrics').replaceChildren(...metrics.flatMap(([name,value])=>{const term=document.createElement('dt'),description=document.createElement('dd');term.textContent=name;description.textContent=value;return [term,description];}));
  element('joint_values').replaceChildren(...state.model.active_indices.map(joint_idx=>{
    const row=document.createElement('tr');
    for(const [value,id] of [[`${state.model.joint_names[joint_idx]} [${unit_label(joint_idx)}]`,null],[(region.min_position[joint_idx]*unit_scale(joint_idx)).toFixed(2),null],['',`current_${joint_idx}`],[(region.max_position[joint_idx]*unit_scale(joint_idx)).toFixed(2),null]]){const cell=document.createElement('td');cell.textContent=value;if(id)cell.id=id;row.append(cell);}return row;
  }));
  element('constraint_label').textContent=state.model.region_type==='joint_box'?'q_min ≤ q ≤ q_max':region.generators?'q = c + Dz, ‖z‖ ≤ 1 と関節制限':'q = Σ λᵢvᵢ, λᵢ ≥ 0, Σλᵢ = 1';
  element('constraint_values').textContent=JSON.stringify(state.model.region_type==='joint_box'?{min_position:region.min_position,max_position:region.max_position}:region.generators?{reference:region.reference,generators:region.generators}:{vertices:region.vertices},null,2);
  update_pose();update_voxels();fit_scene();
}

function load_model(raw) {
  const model=validate_model(raw);stop_playback();state.model=model;
  remove_mesh(pose_mesh);remove_mesh(comparison_mesh);
  pose_mesh=new three.InstancedMesh(sphere_geometry,pose_material,model.sphere_radii.length);scene.add(pose_mesh);
  comparison_mesh=new three.InstancedMesh(sphere_geometry,comparison_material,model.sphere_radii.length);scene.add(comparison_mesh);
  if(model.parents) {
    element('node').replaceChildren(...model.parents.map(parent=>{
      const group=document.createElement('optgroup');group.label=`親 P${parent.parent_idx}`;
      group.append(...parent.leaf_regions.map(idx=>new Option('葉 L'+idx,String(idx))));return group;
    }));
  } else element('node').replaceChildren(...model.regions.map((_,idx)=>new Option('N'+idx,String(idx))));
  element('node_label').textContent=model.parents?'内部葉':'ノード';
  for(const name of ['axis_x','axis_y'])element(name).replaceChildren(...model.active_indices.map(idx=>new Option(`${model.joint_names[idx]} [${unit_label(idx)}]`,String(idx))));
  element('axis_y').value=String(model.active_indices[Math.min(1,model.active_indices.length-1)]);
  for(const id of ['node','neighbor','previous','next','play','pose'])element(id).disabled=false;
  element('model_label').textContent=`${model.parents?model.parents.length+'親 · '+model.regions.length+'葉':model.regions.length+'領域'} · ${model.active_indices.length}軸 · 読取り専用`;
  const default_idx=model.region_type==='joint_box'?Math.max(0,model.regions.findIndex(region=>region.has_index)):0;
  const requested=Number(new URL(location.href).searchParams.get('node')??default_idx);
  select_node(Number.isInteger(requested)&&requested>=0&&requested<model.regions.length?requested:default_idx);
  diagnostics.is_ready=true;
}

element('node').onchange=()=>select_node(Number(element('node').value));
element('previous').onclick=()=>select_node((state.region_idx+state.model.regions.length-1)%state.model.regions.length);
element('next').onclick=()=>select_node((state.region_idx+1)%state.model.regions.length);
element('neighbor').onchange=()=>{state.neighbor_idx=element('neighbor').value===''?null:Number(element('neighbor').value);update_pose();update_voxels();draw_projection();};
element('layer').onchange=update_voxels;
element('enable_pose').onchange=update_pose;
element('fit').onclick=fit_scene;
element('axis_x').onchange=draw_projection;element('axis_y').onchange=draw_projection;
element('pose').onchange=()=>{stop_playback();state.pose_idx=Number(element('pose').value);update_pose();};
element('play').onclick=()=>{
  if(state.timer!==null){stop_playback();return;}
  element('play').textContent='姿勢例を停止';element('play').setAttribute('aria-pressed','true');
  state.timer=setInterval(()=>{state.pose_idx=(state.pose_idx+1)%selected().poses.length;update_pose();},400);
};
element('load').onclick=()=>element('file').click();
element('file').onchange=async()=>{
  try{const file=element('file').files[0];if(!file)return;if(file.size>32*1024*1024)throw Error('JSONは32 MiBまでです');load_model(JSON.parse(await file.text()));}
  catch(error){status(error.message,true);}
  finally{element('file').value='';}
};
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop_playback();});
renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();stop_playback();status('描画コンテキストが失われました。ページの再読込みが必要です。',true);});
try {
  const response=await fetch('/api/model');if(!response.ok)throw Error('保存索引の読込みに失敗しました');
  const data=await response.text();if(data.length>32*1024*1024)throw Error('表示データが32 MiBを超えています');load_model(JSON.parse(data));
} catch(error) {status(error.message,true);}
