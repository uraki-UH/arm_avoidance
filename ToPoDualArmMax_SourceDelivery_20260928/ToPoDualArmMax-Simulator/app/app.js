import {create_ros_results} from './generated/ros-results.js';
import {PhysicsPanel} from './physics-ui.js';
import {RosPointsPanel} from './ros-points.js';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { SSAOPass } from 'three/addons/postprocessing/SSAOPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { Robot, solveIK, orientationError } from './robot.js';
import {SceneEnvironment as WorkEnvironment} from './environment-editor.js';
import {LidarWorkspace} from './lidar-ui.js';
import {RGBDWorkspace} from './rgbd-ui.js';
import {camera_workspace} from './camera-ui.js';
import {VMAIWorkspace} from './vm-ai.js';
import {ROBOT_MODELS,initialModel} from './models.js';
let physics_panel, ros_results;
let workspace, rgbd, lidar, ai, ros_points, demoMotion=null;
let color_camera_panel;
let model=initialModel(),switchingModel=false;
const modelCache=new Map(),modelStates=new Map();

const $ = id => document.getElementById(id), RAD = Math.PI / 180, DEG = 180 / Math.PI;
// Retain the requested fluorescent pigment chroma in highlights. Standard Neutral
// adds white near peak brightness; disable only that desaturation term.
THREE.ShaderChunk.tonemapping_pars_fragment=THREE.ShaderChunk.tonemapping_pars_fragment.replace('const float Desaturation = 0.15;', 'const float Desaturation = 0.0;');
const host=$('canvas-host'), viewport=$('viewport');
let robot, activeSide='L', mode='translate', ready=false, playing=null, photoMode=false, markersVisible=true;
let activeTarget=false, solveFrames=0, frameCount=0, lastStats=performance.now(), last=performance.now(), uiTick=0, toastTimer;
const targets={}, keyframes=[], trails={}, jointInputs={}, targetDirty={L:false,R:false}, ikGoals={L:null,R:null};
const diagnostics={fps:0,frames:0,ikMs:0,dragEvents:0,errors:[],ready:false};
window.addEventListener('error',e=>diagnostics.errors.push(e.message));
window.addEventListener('unhandledrejection',e=>diagnostics.errors.push(String(e.reason)));

function toast(text){$('toast').textContent=text;$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),3000);}

let renderer;
try {
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,preserveDrawingBuffer:false,powerPreference:'high-performance'});
} catch(e) {
  $('loading-text').textContent='WebGL 2を利用できません。ハードウェアアクセラレーションを有効にしたEdge / Chromeで開いてください。';
  throw e;
}
renderer.setPixelRatio(1);
renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.toneMapping=THREE.NeutralToneMapping;renderer.toneMappingExposure=.9;
renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
renderer.shadowMap.autoUpdate=false;host.appendChild(renderer.domElement);
renderer.domElement.setAttribute('aria-label','3Dロボットと手先マーカー');
const scene=new THREE.Scene(), overlay=new THREE.Scene();
scene.background=new THREE.Color('#edf0ed');scene.fog=new THREE.Fog('#edf0ed',18,65);
const camera=new THREE.PerspectiveCamera(34,1,.01,200);camera.up.set(0,0,1);
// 視点操作の慣性遅れを抑制し、ドラッグ量を即時反映
const orbit=new OrbitControls(camera,renderer.domElement);orbit.enableDamping=false;
orbit.minDistance=.25;orbit.maxDistance=100;orbit.maxPolarAngle=Math.PI*.52;orbit.target.set(0,0,.21);
camera.position.set(1.45,-1.15,1.0);orbit.target.set(.28,0,.22);orbit.update();

const environment=new RoomEnvironment();
const pmrem=new THREE.PMREMGenerator(renderer);scene.environment=pmrem.fromScene(environment,.035).texture;
scene.environmentRotation.set(Math.PI/2,0,.5);scene.environmentIntensity=.65;environment.dispose();pmrem.dispose();
const hemi=new THREE.HemisphereLight(0xffffff,0x777777,.9);hemi.position.set(0,0,3);scene.add(hemi);
function areaKey(color,power,xyz,shadow=false){const l=new THREE.DirectionalLight(color,power);l.position.set(...xyz);l.target.position.set(0,0,.25);scene.add(l,l.target);l.castShadow=shadow;if(shadow){l.shadow.mapSize.set(2048,2048);Object.assign(l.shadow.camera,{left:-.85,right:.85,top:.95,bottom:-.7,near:.1,far:6});l.shadow.bias=-.00004;l.shadow.normalBias=.001;l.shadow.radius=3;}return l;}
const vehicleLight=areaKey(0xffffff,.45,[6,-5,9],true);Object.assign(vehicleLight.shadow.camera,{left:-8,right:8,top:8,bottom:-8,near:.1,far:30});vehicleLight.shadow.camera.updateProjectionMatrix();vehicleLight.shadow.normalBias=.002;
const keyLight=areaKey(0xffffff,2.0,[1.4,1.4,2.4],true);
areaKey(0xffffff,.8,[-.6,-1.6,1.4]);areaKey(0xffffff,.9,[-1,1,.9]);

function material(color,metalness,roughness,extra={}) {return new THREE.MeshPhysicalMaterial({color,metalness,roughness,...extra});}
const materials={black:material('#393f3b',.45,.36),green:material('#00ff00',0,.31,{emissive:'#00ff00',emissiveIntensity:.075,specularIntensity:.25,clearcoat:.22,clearcoatRoughness:.22,polygonOffset:true,polygonOffsetFactor:-2,polygonOffsetUnits:-2}),silver:material('#a2abad',.92,.3),gray:material('#69716e',.65,.36),glass:material('#26383a',.65,.15,{transparent:true,opacity:.5}),blue:material('#235bdd',.15,.4),orange:material('#ed9b32',.1,.4)};
const floor=new THREE.Mesh(new THREE.PlaneGeometry(200,200),material('#e0e3e0',.08,.7));floor.position.z=-.14;floor.receiveShadow=true;scene.add(floor);
const pedestal=new THREE.Mesh(new RoundedBoxGeometry(.17,.17,.138,5,.01),material('#424c45',.72,.29));pedestal.position.set(0,0,-.07);pedestal.castShadow=true;pedestal.receiveShadow=true;scene.add(pedestal);
const mountPlate=new THREE.Mesh(new THREE.CylinderGeometry(.095,.097,.009,96),material('#626f64',.82,.25));mountPlate.rotation.x=Math.PI/2;mountPlate.position.z=-.0045;mountPlate.receiveShadow=true;mountPlate.castShadow=true;scene.add(mountPlate);
for(const x of [-.063,.063])for(const y of [-.063,.063]){const bolt=new THREE.Mesh(new THREE.CylinderGeometry(.0045,.0045,.003,6),materials.silver);bolt.rotation.x=Math.PI/2;bolt.position.set(x,y,.001);scene.add(bolt);}
const grid=new THREE.GridHelper(4,80,0x637d64,0x8a9e88);grid.rotation.x=Math.PI/2;grid.position.z=-.139;grid.material.transparent=true;grid.material.opacity=.18;grid.visible=false;scene.add(grid);

const composer=new EffectComposer(renderer);composer.addPass(new RenderPass(scene,camera));
const ao=new SSAOPass(scene,camera,512,512,12);ao.enabled=false;ao.kernelRadius=.035;ao.minDistance=.0006;ao.maxDistance=.018;composer.addPass(ao);composer.addPass(new OutputPass());

const gizmo=new TransformControls(camera,renderer.domElement);gizmo.setSize(.68);gizmo.setSpace('world');overlay.add(gizmo.getHelper());
gizmo.addEventListener('dragging-changed',event=>{orbit.enabled=!event.value;if(event.value){stopPlayback();activeTarget=true;solveFrames=0;}});
gizmo.addEventListener('objectChange',()=>{if(!ready)return;diagnostics.dragEvents++;targetDirty[activeSide]=true;solveFrames=0;activeTarget=true;syncTargetInputs();});
gizmo.addEventListener('mouseUp',()=>{solveFrames=0;});
// Release the marker even if the pointer ends outside the canvas or focus changes.
function releaseMarker(){if(gizmo.dragging)gizmo.pointerUp({button:0});orbit.enabled=true;}
window.addEventListener('pointerup',releaseMarker);
window.addEventListener('pointercancel',releaseMarker);
window.addEventListener('blur',releaseMarker);
document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseMarker();});
renderer.domElement.addEventListener('pointermove',e=>{if(gizmo.dragging&&e.buttons===0)releaseMarker();},true);

for(const side of ['L','R']) {
  const target=new THREE.Group();target.name=`${side} target`;
  const c=side==='L'?0xb8f174:0x71d3ed;
  const sphere=new THREE.Mesh(new THREE.SphereGeometry(.007,20,12),new THREE.MeshBasicMaterial({color:c,depthTest:false,transparent:true,opacity:.92}));
  const ring=new THREE.Mesh(new THREE.TorusGeometry(.011,.0008,8,40),new THREE.MeshBasicMaterial({color:c,depthTest:false,transparent:true,opacity:.75}));
  target.add(sphere,ring);overlay.add(target);targets[side]=target;
  const line=new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(),new THREE.Vector3()]),new THREE.LineDashedMaterial({color:c,dashSize:.006,gapSize:.004,transparent:true,opacity:.5,depthTest:false}));overlay.add(line);target.userData.errorLine=line;
  const trail=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:c,transparent:true,opacity:.55}));scene.add(trail);trails[side]={line:trail,points:[]};
}

function resize(){const w=host.clientWidth,h=host.clientHeight;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h);composer.setSize(w,h);}
new ResizeObserver(resize).observe(host);resize();
function setView(view){
  const views={perspective:[1.45,-1.15,1.0],front:[2.1,0,.42],side:[.25,-1.9,.50],top:[.30,.001,2.1]};
  camera.position.fromArray(views[view]||views.perspective);orbit.target.set(.28,0,.22);orbit.update();
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));
}
function setPhoto(value){photoMode=value;document.body.classList.toggle('photo-mode',value);updateMarkerVisibility();$('photo').classList.toggle('active',value);}
function updateMarkerVisibility(){const show=document.querySelector('[data-panel=robot]').classList.contains('active')&&markersVisible&&!photoMode&&!workspace?.editing&&!rgbd?.cloudOnly&&!lidar?.cloudOnly;gizmo.getHelper().visible=show;gizmo.enabled=show;for(const s of ['L','R']){targets[s].visible=show;targets[s].userData.errorLine.visible=show;$(`label-${s}`).style.display=show?'':'none';}}
function setSide(side){activeSide=side;gizmo.attach(targets[side]);document.querySelectorAll('[data-arm]').forEach(b=>b.classList.toggle('active',b.dataset.arm===side));syncTargetInputs();syncControls();updateMarkerVisibility();}
function setMode(next){mode=next;gizmo.setMode(next);$('translate-mode').classList.toggle('active',next==='translate');$('rotate-mode').classList.toggle('active',next==='rotate');if(next==='rotate')$('hold-orientation').checked=true;syncTargetInputs();}
function syncTargets(sides=['L','R']){for(const side of sides){const tcp=robot.tcp(side);targets[side].position.copy(tcp.position);targets[side].quaternion.copy(tcp.quaternion);targetDirty[side]=false;ikGoals[side]=null;}activeTarget=false;solveFrames=0;syncTargetInputs();}
function syncTargetInputs(){if(!robot)return;const t=targets[activeSide],e=new THREE.Euler().setFromQuaternion(t.quaternion,'ZYX');for(const [id,value] of Object.entries({'target-x':t.position.x*1000,'target-y':t.position.y*1000,'target-z':t.position.z*1000,'target-roll':e.x*DEG,'target-pitch':e.y*DEG,'target-yaw':e.z*DEG})){if(document.activeElement!==$(id))$(id).value=value.toFixed(1);}}
function syncControls(){if(!robot)return;$('grip').value=robot.joints[`${activeSide}_gripper_joint`].q/robot.joints[`${activeSide}_gripper_joint`].upper*100;$('grip-value').textContent=Math.round(+$('grip').value)+'%';for(const [name,entry] of Object.entries(jointInputs)){const q=robot.joints[name].q;entry.input.value=q*DEG;if(entry.output)entry.output.textContent=(q*DEG).toFixed(1)+'°';if(entry.number&&document.activeElement!==entry.number)entry.number.value=(q*DEG).toFixed(1);}}
function homePose(){const p=Object.fromEntries(robot.actuated.map(j=>[j.name,0]));p.L_joint2=-Math.PI/2;p.R_joint2=Math.PI/2;return p;}
function presetPose(name){const p=homePose();if(name==='ready'){Object.assign(p,{L_joint1:-.18,R_joint1:.18,L_joint2:-1.35,R_joint2:1.35,L_joint4:-1.15,R_joint4:1.15,L_joint6:.4,R_joint6:-.4,L_gripper_joint:.32,R_gripper_joint:.32});}if(name==='spread'){Object.assign(p,{L_joint2:-.65,R_joint2:.65,L_joint4:-.65,R_joint4:.65,L_joint6:.35,R_joint6:-.35});}return p;}
let motion_req=0,motion_requires_physics=false;
async function run_motion(begin){
 if(robot.pose_source!=='simulator'&&robot.pose_source){toast('ROS入力中です。シミュレータ操作へ切り替えてください');return;}
 stopPlayback();const req=motion_req;
 try{const has_physics=physics_panel.socket?await physics_panel.prepare_motion():false;if(req!==motion_req)return;motion_requires_physics=has_physics;begin();}
 catch(error){if(req===motion_req)toast('動作を開始できません：'+error.message);}
}
function motion_time(now){return motion_requires_physics?(scene.userData.physics_time_sec??0)*1000:now;}
function animatePose(pose,duration=.7){run_motion(()=>{playing={type:'single',from:robot.getPose(),to:pose,start:motion_time(performance.now()),duration:poseDuration(robot.getPose(),pose,duration)};activeTarget=false;targetDirty.L=targetDirty.R=false;});}
function poseDuration(a,b,requested){return Math.max(requested,...robot.actuated.map(j=>1.5*Math.abs((b[j.name]??a[j.name])-a[j.name])/Math.max(.01,j.velocity)));}
function stopPlayback(){motion_req++;motion_requires_physics=false;ros_points?.robot_panel&&(ros_points.robot_panel.active=null);playing=null;demoMotion=null;if($('ai-demo'))$('ai-demo').textContent='▶ ロボットを動かす';$('play').textContent='▶ 再生';}
function toggleDemoMotion(){if(demoMotion){stopPlayback();syncTargets();return;}run_motion(()=>{demoMotion={start:motion_time(performance.now()),from:robot.getPose()};$('ai-demo').textContent='■ 動きを止める';activeTarget=false;targetDirty.L=targetDirty.R=false;});}
function updateDemoMotion(now){
 if(!demoMotion)return false;if(motion_requires_physics&&!physics_panel.socket){stopPlayback();return false;}now=motion_time(now);
 const t=(now-demoMotion.start)/1000,blend=Math.min(1,t/2),u=blend*blend*(3-2*blend),p=presetPose('ready');
 Object.assign(p,{waist_joint:.25*Math.sin(t*.45),neck_pan_joint:.3*Math.sin(t*.35),neck_tilt_joint:-.55+.12*Math.sin(t*.6),L_joint4:-1.05+.25*Math.sin(t*.7),R_joint4:1.05+.25*Math.sin(t*.7+1.3),L_joint1:-.2+.16*Math.sin(t*.5),R_joint1:.2+.16*Math.sin(t*.5+1)});
 for(const j of robot.actuated)p[j.name]=demoMotion.from[j.name]+((p[j.name]??0)-demoMotion.from[j.name])*u;
 robot.setPose(p);syncTargets();return true;
}
function setManual(name,value){stopPlayback();robot.setJoint(name,value);robot.updateMatrixWorld(true);syncTargets();syncControls();renderer.shadowMap.needsUpdate=true;}
function buildJointControls(){
 const container=$('joint-controls');
 container.replaceChildren();$('body-controls').replaceChildren();for(const name of Object.keys(jointInputs))delete jointInputs[name];
 for(const [label,names] of [['左アーム',robot.chain('L').map(j=>j.name)],['右アーム',robot.chain('R').map(j=>j.name)]]){
  const heading=document.createElement('div');heading.className='joint-group';heading.textContent=label;container.append(heading);
  for(const name of names){const j=robot.joints[name],row=document.createElement('label');row.className='joint-row';const text=document.createElement('span');text.textContent=name.replace('_joint',' · ');const input=document.createElement('input');input.type='range';input.min=Number.isFinite(j.lower)?j.lower*DEG:-180;input.max=Number.isFinite(j.upper)?j.upper*DEG:180;input.step=.1;input.setAttribute('aria-label',name);const output=document.createElement('output');input.addEventListener('input',()=>setManual(name,+input.value*RAD));row.append(text,input,output);container.append(row);jointInputs[name]={input,output};}
 }
 for(const [name,id,label] of [['neck_pan_joint','neck-yaw','首 Yaw'],['neck_tilt_joint','neck-pitch','首 Pitch'],['waist_joint','waist-yaw','腰 Yaw']]){
  const j=robot.joints[name],row=document.createElement('div');row.className='body-joint-row';
  const title=document.createElement('label');title.textContent=label;title.htmlFor=id+'-range';
  const input=document.createElement('input');input.id=id+'-range';input.type='range';input.min=Number.isFinite(j.lower)?j.lower*DEG:-180;input.max=Number.isFinite(j.upper)?j.upper*DEG:180;input.step=.5;input.setAttribute('aria-label',label+' スライダー');
  const number=document.createElement('input');number.id=id+'-input';number.type='number';number.min=input.min;number.max=input.max;number.step=.5;number.setAttribute('aria-label',label+' 角度');
  const unit=document.createElement('span');unit.className='body-joint-number';unit.append(number,document.createTextNode('°'));
  input.addEventListener('input',()=>setManual(name,+input.value*RAD));
  number.addEventListener('input',()=>{const q=number.valueAsNumber;if(Number.isFinite(q))setManual(name,THREE.MathUtils.clamp(q,+input.min,+input.max)*RAD);});
  number.addEventListener('change',()=>{const q=Number(number.value);if(Number.isFinite(q)){const degrees=THREE.MathUtils.clamp(q,+input.min,+input.max);number.value=degrees.toFixed(1);setManual(name,degrees*RAD);}else syncControls();});
  row.append(title,input,unit);$('body-controls').append(row);jointInputs[name]={input,number};
 }
}

function renderKeyframes(){const container=$('keyframes');container.replaceChildren();if(!keyframes.length){const e=document.createElement('span');e.className='empty-timeline';e.textContent='手先を動かしてポーズを記録';container.append(e);}keyframes.forEach((frame,i)=>{const b=document.createElement('button');b.className='frame';b.title=`ポーズ ${i+1} に移動（右クリックで削除）`;b.textContent=String(i+1).padStart(2,'0');const s=document.createElement('small');s.textContent='POSE';b.append(s);b.addEventListener('click',()=>animatePose(frame));b.addEventListener('contextmenu',e=>{e.preventDefault();stopPlayback();keyframes.splice(i,1);renderKeyframes();});container.append(b);});}
function playSequence(){if(playing?.type==='sequence'){stopPlayback();return;}if(keyframes.length<2){toast('2つ以上のポーズを記録してください。');return;}run_motion(()=>{if(keyframes.length<2)return;activeTarget=false;targetDirty.L=targetDirty.R=false;const requested=Math.max(.5,Math.min(15,+$('duration').value||2));playing={type:'sequence',index:0,from:robot.getPose(),to:keyframes[0],start:motion_time(performance.now()),duration:poseDuration(robot.getPose(),keyframes[0],requested),requested};$('play').textContent='■ 停止';});}
function updatePlayback(now){if(!playing)return false;if(motion_requires_physics&&!physics_panel.socket){stopPlayback();return false;}now=motion_time(now);const p=playing,t=Math.min(1,(now-p.start)/1000/p.duration),u=t*t*(3-2*t);const pose={};for(const j of robot.actuated)pose[j.name]=p.from[j.name]+((p.to[j.name]??p.from[j.name])-p.from[j.name])*u;robot.setPose(pose);syncTargets();if(t===1){if(p.type==='sequence'&&p.index<keyframes.length-1){p.index++;p.from=robot.getPose();p.to=keyframes[p.index];p.start=now;p.duration=poseDuration(p.from,p.to,p.requested);}else{stopPlayback();toast('ポーズ指令の再生が完了しました');}}return true;}
async function download(name,data,type){
 const blob=new Blob([data],{type});
 try{
  const response=await fetch('/api/export?name='+encodeURIComponent(name),{method:'POST',headers:{'Content-Type':type,'X-ToPo-Export':'1'},body:blob});
  if(!response.ok)throw new Error('Static server');
  const result=await response.json();diagnostics.lastExport=result.url;
  const link=$('export-link');link.href=result.url;link.hidden=false;link.textContent='保存済みファイルを開く：'+(type==='image/png'?'PNG画像':type==='application/zip'?name:name==='ToPo-workspace.json'?'シーンJSON':'ポーズJSON');
  toast('シミュレータ内の exports フォルダーに保存しました');return;
 }catch{
  const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download=name;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),3000);
  toast('ブラウザのダウンロード先へ保存します');
 }
}
function exportPose(){const payload={format:'topo-motion-studio/1',robot:robot.name,model:model.id,units:'radian',joint_positions:robot.getPose(),keyframes,source:model.urdf};download(model.exportName+'-pose.json',JSON.stringify(payload,null,2),'application/json');}
function validatePose(pose){if(!pose||typeof pose!=='object')throw new Error('関節角度が見つかりません。');const result={};for(const j of robot.actuated){const q=pose[j.name];if(typeof q!=='number'||!Number.isFinite(q))throw new Error(`${j.name} の角度が不正です。`);if(q<j.lower-1e-7||q>j.upper+1e-7)throw new Error(`${j.name} が可動範囲外です。`);if(Math.abs(q)>Math.PI*100)throw new Error('角度が大きすぎます。');result[j.name]=q;}return result;}
async function importPose(file){try{if(file.size>1e6)throw new Error('ファイルが大きすぎます。');const p=JSON.parse(await file.text());if(p.format!=='topo-motion-studio/1'||p.robot!==robot.name||p.units!=='radian')throw new Error('このロボットのポーズファイルではありません。');if((p.model||'long')!==model.id)throw new Error('モデルが異なります。'+(ROBOT_MODELS[p.model||'long']?.label||p.model)+'タブに切り替えてください。');const pose=validatePose(p.joint_positions);if(p.keyframes!==undefined&&!Array.isArray(p.keyframes))throw new Error('シーケンスの形式が不正です。');const frames=(p.keyframes||[]).slice(0,50).map(validatePose);animatePose(pose);keyframes.splice(0,keyframes.length,...frames);renderKeyframes();toast('ポーズを読み込みました');}catch(e){toast('読み込み失敗：'+e.message);}}

function bindUI(){
 document.querySelectorAll('[data-model]').forEach(b=>{b.onclick=()=>switchModel(b.dataset.model);b.onkeydown=e=>{if(e.key==='ArrowLeft'||e.key==='ArrowRight'){e.preventDefault();const next=model.id==='long'?'standard':'long';document.querySelector(`[data-model=${next}]`).focus();switchModel(next);}};});
 document.querySelectorAll('[data-arm]').forEach(b=>b.onclick=()=>setSide(b.dataset.arm));for(const side of ['L','R'])$(`label-${side}`).onclick=()=>setSide(side);
 $('translate-mode').onclick=()=>setMode('translate');$('rotate-mode').onclick=()=>setMode('rotate');$('coordinate-space').onchange=e=>gizmo.setSpace(e.target.value);
 $('hold-orientation').onchange=()=>{if($('hold-orientation').checked){targets[activeSide].quaternion.copy(robot.tcp(activeSide).quaternion);syncTargetInputs();}targetDirty[activeSide]=true;activeTarget=true;solveFrames=0;};
 for(const id of ['target-x','target-y','target-z'])$(id).addEventListener('change',()=>{stopPlayback();const p=['target-x','target-y','target-z'].map(n=>+$(n).value/1000);if(p.every(Number.isFinite)&&p.every(v=>Math.abs(v)<=5)){targets[activeSide].position.fromArray(p);targetDirty[activeSide]=true;activeTarget=true;solveFrames=0;}else{toast('目標位置は ±5000 mm 内の数値で入力してください。');syncTargetInputs();}});
 for(const id of ['target-roll','target-pitch','target-yaw'])$(id).addEventListener('change',()=>{stopPlayback();const e=['target-roll','target-pitch','target-yaw'].map(n=>+$(n).value*RAD);if(e.every(Number.isFinite)){targets[activeSide].quaternion.setFromEuler(new THREE.Euler(...e,'ZYX'));$('hold-orientation').checked=true;targetDirty[activeSide]=true;activeTarget=true;solveFrames=0;}});
 for(const id of ['target-x','target-y','target-z','target-roll','target-pitch','target-yaw'])$(id).addEventListener('input',()=>{if(Number.isFinite($(id).valueAsNumber))$(id).dispatchEvent(new Event('change'));});
 $('grip').oninput=()=>setManual(`${activeSide}_gripper_joint`,+$('grip').value/100*robot.joints[`${activeSide}_gripper_joint`].upper);
 document.querySelectorAll('[data-pose]').forEach(b=>b.onclick=()=>{animatePose(presetPose(b.dataset.pose));toast(b.textContent+'へ移動');});
 document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));$('fit-view').onclick=()=>setView('perspective');
 $('photo').onclick=()=>setPhoto(!photoMode);$('screenshot').onclick=()=>{const prev=photoMode;setPhoto(true);draw();renderer.domElement.toBlob(b=>{if(b)download(model.exportName+'.png',b,'image/png');setPhoto(prev);},'image/png');};
 $('show-grid').onchange=e=>grid.visible=e.target.checked;$('show-trails').onchange=e=>{for(const t of Object.values(trails)){t.line.visible=e.target.checked;if(!e.target.checked){t.points=[];t.line.geometry.dispose();t.line.geometry=new THREE.BufferGeometry();}}};
 $('show-markers').onchange=e=>{markersVisible=e.target.checked;updateMarkerVisibility();};
 $('studio').onchange=e=>{const dark=e.target.value==='dark';document.body.classList.toggle('dark-studio',dark);scene.background.set(dark?'#1e1e1e':'#edf0ed');scene.fog.color.copy(scene.background);floor.material.color.set(dark?'#292929':'#e0e3e0');scene.environmentIntensity=dark?.6:.65;renderer.shadowMap.needsUpdate=true;};
 $('enable-shadows').onclick=()=>{const enable_shadows=!renderer.shadowMap.enabled;renderer.shadowMap.enabled=enable_shadows;renderer.shadowMap.needsUpdate=enable_shadows;
  // 影の有無に対応するシェーダーの再選択。更新は切り替え時のみ
  const shadow_materials=new Set();scene.traverse(object=>{if(object.material)for(const material of Array.isArray(object.material)?object.material:[object.material])shadow_materials.add(material);});for(const material of shadow_materials)material.needsUpdate=true;
  const button=$('enable-shadows');button.classList.toggle('active',enable_shadows);button.setAttribute('aria-pressed',String(enable_shadows));button.textContent=enable_shadows?'影 ON':'影 OFF';};
 $('quality').onchange=e=>{ao.enabled=e.target.value==='high';renderer.setPixelRatio(Math.min(devicePixelRatio,e.target.value==='high'?1.5:1));composer.setPixelRatio(renderer.getPixelRatio());resize();};
 $('exposure').oninput=e=>renderer.toneMappingExposure=+e.target.value;$('help-button').onclick=()=>$('help').showModal();$('close-help').onclick=()=>$('help').close();
 $('add-keyframe').onclick=()=>{if(keyframes.length>=50){toast('記録できるポーズは50個までです。');return;}keyframes.push(robot.getPose());renderKeyframes();toast(`ポーズ ${keyframes.length} を記録しました`);};$('play').onclick=playSequence;$('clear-frames').onclick=()=>{stopPlayback();keyframes.length=0;renderKeyframes();};
 $('save-pose').onclick=exportPose;$('load-pose').onclick=()=>$('pose-file').click();$('pose-file').onchange=e=>{if(e.target.files[0])importPose(e.target.files[0]);e.target.value='';};
 $('reset-body').onclick=()=>animatePose({...robot.getPose(),neck_pan_joint:0,neck_tilt_joint:0,waist_joint:0},.45);
 window.addEventListener('keydown',e=>{if(!ready)return;if(/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)||$('help').open||e.ctrlKey||e.metaKey||e.altKey)return;const k=e.key.toLowerCase();if(k==='w')setMode('translate');if(k==='e')setMode('rotate');if(k==='l')setSide('L');if(k==='r')setSide('R');if(k==='f')setView('perspective');if(k==='h')animatePose(homePose());if(k==='p')setPhoto(!photoMode);if(k==='escape'){stopPlayback();syncTargets();} });
}

function updateIK(dt){
 if(['ros','leader'].includes(robot.pose_source)||!activeTarget||playing)return false;
 const side=activeSide,hold=$('hold-orientation').checked,chain=robot.chain(side),before=chain.map(j=>j.q),start=performance.now();
 if(targetDirty[side]||!ikGoals[side]){
  // Solve a stable destination, then interpolate to it; avoid re-solving intermediate
  // velocity-limited poses, which can switch redundant elbow configurations.
  if(robot.tcp(side).position.distanceTo(targets[side].position)>.002&&Math.abs(chain[3].q)<.005)robot.setJoint(chain[3].name,side==='L'?-.035:.035);
  let result=solveIK(robot,side,targets[side],{orientation:hold,iterations:80});
  let best=chain.map(j=>j.q),cost=result.position**2+(hold ? .0144*result.angle**2:0);
  if(result.position>.002||(hold&&result.angle>.025)){
   for(const sign of [-1,1]){
    chain.forEach((j,i)=>robot.setJoint(j.name,before[i]));
    robot.setJoint(chain[2].name,before[2]+sign*.55);robot.setJoint(chain[4].name,before[4]-sign*.4);
    robot.setJoint(chain[3].name,before[3]+sign*.22);
    result=solveIK(robot,side,targets[side],{orientation:hold,iterations:65});
    const candidate=result.position**2+(hold ? .0144*result.angle**2:0);
    if(candidate<cost){cost=candidate;best=chain.map(j=>j.q);}
   }
  }
  ikGoals[side]=best;targetDirty[side]=false;
 }
 const goal=ikGoals[side];
 const fraction=Math.min(1,...chain.map((j,i)=>j.velocity*dt/Math.max(Math.abs(goal[i]-before[i]),1e-10)));
 chain.forEach((j,i)=>robot.setJoint(j.name,before[i]+(goal[i]-before[i])*fraction));
 robot.updateMatrixWorld(true);if(!hold){targets[side].quaternion.copy(robot.tcp(side).quaternion);}
 diagnostics.ikMs=performance.now()-start;solveFrames++;
 const tcp=robot.tcp(side),pos=tcp.position.distanceTo(targets[side].position),ang=orientationError(targets[side].quaternion,tcp.quaternion).length();
 if((pos<.0004&&(!hold||ang<.007))||fraction===1)activeTarget=false;
 return before.some((q,i)=>Math.abs(q-chain[i].q)>1e-8);
}
function updateLabels(){for(const side of ['L','R']){const tcp=robot.tcp(side),p=targets[side].position.clone().project(camera),label=$(`label-${side}`);label.style.left=((p.x*.5+.5)*host.clientWidth+(side==='L'?18:-96))+'px';label.style.top=((-p.y*.5+.5)*host.clientHeight+22)+'px';label.style.opacity=p.z>1||p.z< -1?'0':'1';const line=targets[side].userData.errorLine;line.geometry.setFromPoints([tcp.position,targets[side].position]);line.computeLineDistances();line.visible=markersVisible&&!photoMode&&tcp.position.distanceTo(targets[side].position)>.002;
 if($('show-trails').checked){const t=trails[side];if(!t.points.length||t.points.at(-1).distanceTo(tcp.position)>.002){t.points.push(tcp.position.clone());if(t.points.length>1500)t.points.shift();t.line.geometry.dispose();t.line.geometry=new THREE.BufferGeometry().setFromPoints(t.points);}}}
}
function updateUI(){const tcp=robot.tcp(activeSide),error=tcp.position.distanceTo(targets[activeSide].position)*1000,angle=orientationError(targets[activeSide].quaternion,tcp.quaternion).length()*DEG,hold=$('hold-orientation').checked,ok=error<1&&(!hold||angle<1);$('ik-state').textContent=playing?'ポーズを再生中':ok?'目標に到達':activeTarget&&solveFrames<40?'手先が追従中':'目標未到達';$('ik-error').textContent=error.toFixed(1)+' mm'+(hold?' / '+angle.toFixed(1)+'°':'');$('ik-dot').style.background=ok?'var(--green)':'var(--amber)';$('ik-state').style.color=ok?'#007700':'var(--amber)';$('actual-position').textContent='TCP '+tcp.position.toArray().map(v=>(v*1000).toFixed(1).padStart(6)).join(' / ')+' mm';syncControls();if(!gizmo.dragging)syncTargetInputs();if(window.simulator)document.documentElement.dataset.simState=JSON.stringify(window.simulator.getState());}
// 標準品質では中間バッファと画面全体の後処理を省略
let last_view_draw_ms=-Infinity,last_view_input_ms=-Infinity,is_view_input_active=false;
orbit.addEventListener('start',()=>{is_view_input_active=true;});
orbit.addEventListener('end',()=>{is_view_input_active=false;});
orbit.addEventListener('change',()=>{last_view_input_ms=performance.now();});
// RGB-D回収待ち中の重複描画を短時間抑制。視点操作・ロボット動作時は通常描画。
function draw(is_moving=false){
 const now=performance.now();
 if(rgbd?.enable_readback_priority&&!is_moving&&!gizmo.dragging&&!is_view_input_active&&now-last_view_input_ms>100&&rgbd.sensor.readback_pool.has_pending&&now-last_view_draw_ms<50)return false;
 last_view_draw_ms=now;if(ao.enabled)composer.render();else renderer.render(scene,camera);renderer.autoClear=false;renderer.clearDepth();renderer.render(overlay,camera);renderer.autoClear=true;return true;}
function loop(now){requestAnimationFrame(loop);const dt=Math.min(.035,Math.max(.001,(now-last)/1000));last=now;orbit.update();if(!ready){draw();return;}const moving=updateDemoMotion(now)||updatePlayback(now)||updateIK(dt);if(moving)renderer.shadowMap.needsUpdate=true;physics_panel?.tick(now);ros_results?.tick(now);rgbd?.tick(now);color_camera_panel?.tick(now);lidar?.tick(now);ai?.tick(now);ros_points?.tick(now);physics_panel?.sync_robot_pose();if(physics_panel?.socket&&physics_panel.enable_dynamics&&!activeTarget&&!gizmo.dragging)syncTargets();updateLabels();if(now-uiTick>90){updateUI();uiTick=now;}if(draw(moving)){diagnostics.frames++;frameCount++;}if(now-lastStats>1000){diagnostics.fps=frameCount*1000/(now-lastStats);$('render-stats').textContent=diagnostics.fps.toFixed(0)+' fps · '+(robot.triangles/1e6).toFixed(2)+'M tris';frameCount=0;lastStats=now;}}

function updateModelUI(){
 document.querySelector('.product-header strong').textContent=model.title;document.title=model.title+' · Motion Studio';
 document.querySelector('.loader-symbol span').textContent=model.id==='long'?'MAX LONG':'MAX STANDARD';
 for(const b of document.querySelectorAll('[data-model]')){const selected=b.dataset.model===model.id;b.setAttribute('aria-selected',String(selected));b.tabIndex=selected?0:-1;b.disabled=switchingModel;}
 $('model-status').textContent=switchingModel?'モデル読込中…':model.label+' · 19 DOF';
}
async function loadRobotModel(def){
 if(modelCache.has(def.id))return modelCache.get(def.id);
 const [xml,manifest]=await Promise.all([fetch(def.urdf).then(r=>{if(!r.ok)throw Error('URDFがありません');return r.text();}),fetch(def.assets).then(r=>{if(!r.ok)throw Error('メッシュ一覧がありません');return r.json();})]);
 const next=new Robot(xml);next.modelId=def.id;next.ros_frame_prefix=def.ros_frame_prefix;
 await next.loadVisuals(manifest,materials,p=>{$('load-progress').style.width=(p*90).toFixed(0)+'%';$('loading-text').textContent=`${def.label} CADメッシュを読み込み中 ${Math.round(p*100)}%`;});
 modelCache.set(def.id,next);return next;
}
async function switchModel(id){
 if(switchingModel||!ready||id===model.id||!Object.hasOwn(ROBOT_MODELS,id))return;
 const previous=model,previousRobot=robot,restoreTabFocus=!!document.activeElement?.closest('#model-tabs'),wasLive={lidar:lidar.live,rgbd:rgbd?.live,ai:ai?.running};
 modelStates.set(model.id,{pose:robot.getPose(),keyframes:structuredClone(keyframes)});
 stopPlayback();releaseMarker();ready=false;switchingModel=true;diagnostics.ready=false;
 lidar.resetForRobot(robot);rgbd?.resetForRobot(robot);ai?.resetForRobot(robot);color_camera_panel?.reset_for_robot(robot);
 document.querySelector('aside').inert=true;document.querySelector('footer').inert=true;
 $('loading').classList.remove('done');$('loading').setAttribute('aria-hidden','false');$('load-progress').style.width='0%';updateModelUI();
 try{
  const next=await loadRobotModel(ROBOT_MODELS[id]);scene.remove(robot);scene.add(next);robot=next;model=ROBOT_MODELS[id];
  robot.setPose(modelStates.get(id)?.pose||homePose());
  keyframes.splice(0,keyframes.length,...(modelStates.get(id)?.keyframes||[]));renderKeyframes();buildJointControls();
  lidar.resetForRobot(robot);rgbd?.resetForRobot(robot);ai?.resetForRobot(robot);color_camera_panel?.reset_for_robot(robot);
  for(const t of Object.values(trails)){t.points=[];t.line.geometry.dispose();t.line.geometry=new THREE.BufferGeometry();}
  syncTargets();syncControls();setSide(activeSide);updateCloudLayers();renderer.shadowMap.needsUpdate=true;
  await renderer.compileAsync(scene,camera);const url=new URL(location.href);url.searchParams.set('model',id);history.replaceState(null,'',url);
  toast(model.label+'版に切り替えました');
 }catch(e){
  scene.remove(robot);robot=previousRobot;model=previous;scene.add(robot);robot.setPose(modelStates.get(model.id).pose);
  keyframes.splice(0,keyframes.length,...modelStates.get(model.id).keyframes);renderKeyframes();buildJointControls();
  lidar.resetForRobot(robot);rgbd?.resetForRobot(robot);ai?.resetForRobot(robot);color_camera_panel?.reset_for_robot(robot);syncTargets();syncControls();
  toast('モデル切替失敗：'+e.message);diagnostics.errors.push(String(e));
 }finally{
  switchingModel=false;ready=true;diagnostics.ready=true;document.querySelector('aside').inert=false;document.querySelector('footer').inert=false;
  lidar.live=wasLive.lidar; if(rgbd)rgbd.live=wasLive.rgbd;if(ai)ai.running=wasLive.ai;
  if(rgbd)$('sensor-live').textContent=rgbd.live?'■ RGB-D停止':'▶ RGB-D開始';if(ai)$('ai-start').textContent=ai.running?'■ 入力を停止':'▶ VM処理を開始';
  $('loading').classList.add('done');$('loading').setAttribute('aria-hidden','true');updateModelUI();updateUI();lidar.publish();ai?.publish();
  if(restoreTabFocus)document.querySelector(`[data-model=${model.id}]`).focus({preventScroll:true});
 }
}
async function init(){
 try {
  updateModelUI();robot=await loadRobotModel(model);scene.add(robot);robot.setPose(homePose());
  buildJointControls();bindUI();syncTargets();setSide('L');syncControls();
  initWorkspace();
  $('loading-text').textContent='照明とマテリアルを準備中';$('load-progress').style.width='95%';
  renderer.shadowMap.needsUpdate=true;await renderer.compileAsync(scene,camera);draw();
  $('load-progress').style.width='100%';$('loading').classList.add('done');$('loading').setAttribute('aria-hidden','true');
  ready=true;diagnostics.ready=true;toast('手先の矢印をドラッグして操作できます');
  window.simulator={get robot(){return robot;},get model(){return model;},targets,camera,scene,renderer,gizmo,workspace,rgbd,lidar,ai,ros_points,ros_results,color_camera_panel,get physics_panel(){return physics_panel;},diagnostics,keyframes,
   cancel_robot_motion:()=>{playing=null;demoMotion=null;activeTarget=false;targetDirty.L=targetDirty.R=false;},
   apply_ros_pose:pose=>{playing=null;demoMotion=null;activeTarget=false;targetDirty.L=targetDirty.R=false;robot.set_received_pose(pose);syncTargets();renderer.shadowMap.needsUpdate=true;},
   set_robot_placement:values=>{stopPlayback();activeTarget=false;robot.position.fromArray(values);robot.quaternion.setFromEuler(new THREE.Euler(...values.slice(3).map(v=>v*RAD),'ZYX'));robot.updateMatrixWorld(true);syncTargets();renderer.shadowMap.needsUpdate=true;},setSide,setMode,solveIK,homePose,presetPose,animatePose,syncTargets,validatePose,switchModel,
   getState:()=>({ready,model:model.id,modelTitle:model.title,source:model.urdf,switchingModel,activeSide,mode,workspace:workspace?.getState(),rgbd:rgbd?.lastSummary,dragging:gizmo.dragging,playing:!!playing,quality:$('quality').value,holdOrientation:$('hold-orientation').checked,appearance:{greenBaseHex:'#'+materials.green.color.getHexString(),toneMapping:'Neutral, chroma preserved',logo:'branding/FuzzRoBo-logo.png'},joints:robot.getPose(),tcp:Object.fromEntries(['L','R'].map(s=>{const p=robot.tcp(s);return[s,{position:p.position.toArray(),quaternion:p.quaternion.toArray(),target:targets[s].position.toArray(),errorMm:p.position.distanceTo(targets[s].position)*1000,errorDeg:orientationError(targets[s].quaternion,p.quaternion).length()*DEG}]})),diagnostics:{...diagnostics},triangles:robot.triangles}),
   setTarget:(side,xyz,rpy)=>{setSide(side);targets[side].position.fromArray(xyz);if(rpy){targets[side].quaternion.setFromEuler(new THREE.Euler(...rpy,'ZYX'));$('hold-orientation').checked=true;}targetDirty[side]=true;activeTarget=true;solveFrames=0;syncTargetInputs();},
   reset:()=>{stopPlayback();robot.setPose(homePose());syncTargets();syncControls();renderer.shadowMap.needsUpdate=true;}
  };
  if(new URLSearchParams(location.search).has('qa')) {
    if(new URLSearchParams(location.search).get('qa')==='models'){
      const {runModelQA}=await import('./qa-models.js');await runModelQA(window.simulator);
    }else{const {runQA}=await import('./qa-tests.js');await runQA(window.simulator);}
  }
 }catch(e){console.error(e);$('loading-text').textContent='読み込み失敗：'+e.message;diagnostics.errors.push(String(e));}
}
function aimAtTable(){
 const saved=robot.getPose(),target=new THREE.Vector3(0,0,.06).applyMatrix4(workspace.root.matrixWorld),pan=robot.joints.neck_pan_joint;
 pan.frame.updateWorldMatrix(true,false);const local=target.clone().applyMatrix4(pan.frame.matrixWorld.clone().invert());
 const yaw=Math.atan2(local.y,local.x);robot.setJoint('neck_pan_joint',yaw);robot.updateMatrixWorld(true);
 const tilt=robot.joints.neck_tilt_joint;tilt.frame.updateWorldMatrix(true,false);const inTilt=target.clone().applyMatrix4(tilt.frame.matrixWorld.clone().invert());
 const pitch=THREE.MathUtils.clamp(Math.atan2(inTilt.z,inTilt.x),tilt.lower,tilt.upper);robot.setPose(saved);animatePose({...saved,neck_pan_joint:yaw,neck_tilt_joint:pitch},.6);
 toast('カメラをテーブルへ向けます');
}
function updateCloudLayers(){
 const only=rgbd?.cloudOnly||lidar?.cloudOnly;camera.layers.mask=0;if(!only)camera.layers.enable(0);if(rgbd?.cloud.visible)camera.layers.enable(1);if(lidar?.cloud.visible||lidar?.axes.visible)camera.layers.enable(2);if(ai)camera.layers.enable(3);updateMarkerVisibility();
}
function initWorkspace(){
 workspace=new WorkEnvironment({scene,overlay,camera,renderer,orbit,toast,onEdit:()=>updateMarkerVisibility(),has_priority_pick:(client_x,client_y)=>ros_results?.scene?.has_inspection_at(client_x,client_y)??false});
 workspace.register_static_surfaces({floor,pedestal});
 try{rgbd=new RGBDWorkspace({scene,overlay,renderer,camera,robot,environment:workspace,exclude:[grid,...Object.values(trails).map(x=>x.line),robot.links.camera_link],toast,download,aim:aimAtTable,onCloudOnly:()=>updateCloudLayers()});}
 catch(e){$('sensor-panel').textContent='RGB-Dを初期化できません：'+e.message;diagnostics.errors.push(String(e));}
 try{color_camera_panel=new camera_workspace({scene,renderer,robot,environment:workspace,exclude:[grid,...Object.values(trails).map(x=>x.line),robot.links.camera_link],toast,download,aim:aimAtTable});}
 catch(e){$('camera-panel').textContent='カメラを初期化できません：'+e.message;diagnostics.errors.push(String(e));}
 lidar=new LidarWorkspace({scene,overlay,robot,renderer,camera,environment:workspace,toast,download,onLayers:()=>updateCloudLayers()});
 ai=new VMAIWorkspace({scene,camera,robot,lidar,rgbd,environment:workspace,toast,motion:{toggle:toggleDemoMotion,active:()=>!!demoMotion}});
 ros_points=new RosPointsPanel({environment:workspace,rgbd,lidar,toast});
 ros_results=create_ros_results({scene,camera,renderer,controls:orbit,robot:()=>robot});
 physics_panel=new PhysicsPanel({environment:workspace,robot:()=>robot,scene});
 document.querySelectorAll('[data-panel]').forEach(b=>b.onclick=()=>{document.querySelectorAll('[data-panel]').forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});for(const name of ['robot','environment','sensors','physics','ai','ros'])$(name+'-panel').hidden=name!==b.dataset.panel;if(b.dataset.panel!=='environment')workspace.setEditing(false);workspace.selection.visible=['environment','physics'].includes(b.dataset.panel)&&!!workspace.selected;document.querySelector('aside').scrollTop=0;updateMarkerVisibility();});
 // センサUIの再生成なし。子タブ選択・設定値・取得状態の保持。
 document.querySelectorAll('[data-sensor-panel]').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('[data-sensor-panel]').forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});
  for(const name of ['sensor','camera','lidar'])$(name+'-panel').hidden=name!==b.dataset.sensorPanel;
  document.querySelector('aside').scrollTop=0;
 });
 $('scene-save').onclick=()=>download('ToPo-workspace.json',JSON.stringify(workspace.getState(),null,2),'application/json');
 $('scene-load').onclick=()=>$('scene-file').click();$('scene-file').onchange=async e=>{const f=e.target.files[0];e.target.value='';if(!f)return;try{if(f.size>1e6)throw Error('ファイルが大きすぎます');await workspace.load(JSON.parse(await f.text()));toast('シーンを読み込みました');}catch(error){toast('シーン読込エラー：'+error.message);}};
}
requestAnimationFrame(loop);init();
