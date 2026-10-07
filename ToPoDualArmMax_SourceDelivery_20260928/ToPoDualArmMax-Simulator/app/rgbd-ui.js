import {robot_snapshot} from './robot-ros-state.js';
import {CloudColorControls,displayPLY} from './pointcloud-colors.js';
import * as THREE from 'three';
import {RGBDSensor,nominalCalibration,validateCalibration,deproject,binaryPLY} from './rgbd-core.js';
import {zipFiles} from './capture-zip.js';
const $=id=>document.getElementById(id);
const colorLUT=Float32Array.from({length:256},(_,i)=>{const x=i/255;return x<=.04045?x/12.92:Math.pow((x+.055)/1.055,2.4);});
export class RGBDWorkspace {
 constructor({scene,overlay,renderer,camera,robot,environment,exclude,toast,download,aim,onCloudOnly}){
  Object.assign(this,{scene,overlay,renderer,camera,robot,environment,exclude,toast,download,aim,onCloudOnly});this.sensor=new RGBDSensor(renderer,scene);this.live=false;this.lastTime=0;this.rate=0;this.cloudOnly=false;this.last_capture_end_ms=0;this.last_capture_ms=0;this.is_capture_pending=false;this.capture_generation=0;
  this.cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:2,sizeAttenuation:false,vertexColors:true,toneMapped:false}));this.cloud.frustumCulled=false;this.cloud.layers.set(1);this.cloud.visible=false;scene.add(this.cloud);
  this.frustum=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xb553bb,transparent:true,opacity:.55}));overlay.add(this.frustum);this.frustum.visible=false;
  this.ui();this.lastSummary={ready:true,live:false,frame:null};
 }
 ui(){
  $('sensor-panel').innerHTML=`<div class="panel-heading"><div><span class="eyebrow">HEAD CAMERA</span><h2>RealSense D435i</h2></div><span class="chip">RGB-D</span></div>
  <div class="row-actions"><button id="sensor-aim">テーブルを見る</button><button id="sensor-live">▶ RGB-D開始</button><button id="sensor-once">1回取得</button></div>
  <label class="field-label">深度プロファイル<select id="sensor-profile"><option value="848">848 × 480 · 公称HD画角</option><option value="1280">1280 × 720 · 高密度</option><option value="424">424 × 240 · 軽量縮小</option><option value="custom" disabled>カスタム校正</option></select></label>
  <div class="row-actions"><select id="sensor-mode" aria-label="深度モード"><option value="ideal">幾何真値（float32 m）</option><option value="stereo" selected>ステレオ可視性 + Z16近似</option></select><select id="sensor-fps" aria-label="取得頻度"><option>5</option><option selected>10</option><option>15</option><option>30</option></select><span>Hz（上限）</span></div><p class="sub-note">設定Hzを上限に、前の取得完了後に次のフレームを取得。処理が間に合わない場合は実測Hzが低下。</p>
  <p class="sub-note">首2軸・腰Yawに追従。基線50 mmのステレオ可視性とZ16。各深度画素から1点、間引きなし。最短距離は848×480で195 mm、1280×720で280 mm、424×240で105 mm。実機個体の校正・IR照射・露光・欠損率は未再現。</p>
  <div class="sensor-images"><figure><figcaption>RGB <span>1280 × 720</span></figcaption><canvas id="rgb-preview" width="1280" height="720" aria-label="D435i RGB画像"></canvas></figure><figure><figcaption>Depth <span>Z [m] · クリックで計測</span></figcaption><canvas id="depth-preview" width="848" height="480" aria-label="D435i 深度画像"></canvas><div class="depth-scale"><span id="depth-near">0.195 m</span><span id="depth-far">3 m</span></div></figure></div>
  <div id="sensor-stats" class="sensor-stats" role="status">取得待ち</div>
  <div class="row-actions"><label><input id="cloud-show" type="checkbox">点群を重ねる</label><label><input id="cloud-only" type="checkbox">点群のみ</label><label><input id="frustum-show" type="checkbox">画角</label></div>
  <div id="rgbd-colors" class="cloud-color-controls"></div><div class="pixel-readout"><div class="row-actions"><label>u <input id="pixel-u" type="number" value="424" min="0" aria-label="深度画素 u"></label><label>v <input id="pixel-v" type="number" value="240" min="0" aria-label="深度画素 v"></label><button id="pixel-read">計測</button></div><output id="pixel-result">深度画像の点をクリックしてください。</output></div>
  <div class="row-actions"><select id="cloud-frame" aria-label="点群の保存座標系"><option value="world">base_footprint [m]</option><option value="optical">深度光学座標 [m]</option></select><button id="sensor-export" disabled>↓ RGB-D一式</button></div><p class="sub-note">PLY全点・RGB PNG・深度float32 / Z16・校正・撮影時の姿勢とシーンを、同じフレームのZIPで保存。</p>
  <details><summary>校正と精度検証 <span>fx / fy / cx / cy</span></summary><p class="sub-note">歪み補正済みの内部パラメータ、depth→color、URDF光学座標からの取付補正を設定できます。回転は行優先、移動はm、RPYはrad。</p><textarea id="calibration-json" spellcheck="false" aria-label="RGB-D校正JSON"></textarea><div class="row-actions"><button id="calibration-apply">校正を適用</button><button id="calibration-reset">公称値へ戻す</button></div><button id="sensor-verify" class="wide-button">幾何精度を検証</button><output id="sensor-qa"></output></details>`;
  this.colorControls=new CloudColorControls($('rgbd-colors'),'rgbd','rgb',()=>{if(this.sensor.lastFrame)this.updateCloud(this.sensor.lastFrame);});
  $('calibration-json').value=JSON.stringify(this.sensor.calibration,null,2);
  $('sensor-aim').onclick=()=>this.aim();$('sensor-live').onclick=()=>{this.live=!this.live;$('sensor-live').textContent=this.live?'■ RGB-D停止':'▶ RGB-D開始';if(this.live)this.capture();else{this.lastSummary.live=false;document.documentElement.dataset.rgbdState=JSON.stringify(this.lastSummary);}};$('sensor-once').onclick=()=>this.capture();
  $('sensor-profile').onchange=()=>{const w=+$('sensor-profile').value,h=w===1280?720:w===848?480:240;this.configure(nominalCalibration(w,h));};
  $('calibration-apply').onclick=()=>{try{this.configure(validateCalibration(JSON.parse($('calibration-json').value)));$('sensor-profile').value='custom';this.toast('校正を適用しました');}catch(e){this.toast('校正エラー：'+e.message);}};
  $('calibration-reset').onclick=()=>{$('sensor-profile').value='848';this.configure(nominalCalibration());};
  $('cloud-show').onchange=()=>this.setCloud();$('cloud-only').onchange=()=>this.setCloud();$('frustum-show').onchange=()=>this.frustum.visible=$('frustum-show').checked;
  $('pixel-read').onclick=()=>this.readPixel(+$('pixel-u').value,+$('pixel-v').value);$('depth-preview').onclick=e=>{const r=e.currentTarget.getBoundingClientRect(),k=this.sensor.lastFrame?.calibration.depth;if(k)this.readPixel(Math.floor((e.clientX-r.left)*k.width/r.width),Math.floor((e.clientY-r.top)*k.height/r.height));};
  $('sensor-export').onclick=()=>this.exportFrame();$('sensor-verify').onclick=()=>this.verify();
 }
 configure(c){this.capture_generation++;this.sensor.configure(c);$('calibration-json').value=JSON.stringify(c,null,2);this.lastTime=0;$('sensor-stats').textContent='校正更新済み。次の取得で反映されます。';}
 resetForRobot(robot){
  this.capture_generation++;
  this.exclude=this.exclude.map(o=>o===this.robot.links.camera_link?robot.links.camera_link:o);this.robot=robot;this.live=false;this.lastTime=0;this.sensor.lastFrame=null;this.last_scene_frame=null;this.lastSummary={ready:false,model:robot.modelId,frame:null};
  this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.cloud.visible=false;
  for(const id of ['rgb-preview','depth-preview']){const c=$(id);c.getContext('2d').clearRect(0,0,c.width,c.height);}
  $('sensor-export').disabled=true;$('sensor-stats').textContent='モデル変更済み · 取得待ち';$('pixel-result').textContent='新しいモデルで点群を取得してください。';
  document.documentElement.dataset.rgbdState=JSON.stringify(this.lastSummary);this.setCloud();if(this.frustum.visible)this.updateFrustum();
 }
 setCloud(){const only=$('cloud-only').checked,show=$('cloud-show').checked||only;this.cloud.visible=show;if(show&&this.sensor.lastFrame)this.updateCloud(this.sensor.lastFrame);if(only)this.camera.layers.set(1);else{this.camera.layers.set(0);if(show)this.camera.layers.enable(1);}this.cloudOnly=only;this.onCloudOnly(only);}
 opticalWorld(){const link=this.robot.links.camera_optical_frame;link.updateWorldMatrix(true,false);return link.matrixWorld.clone();}
 tick(now){
  if(this.frustum.visible)this.updateFrustum();
  // 設定周期と取得中ガードによる、追加休止なし・多重取得なしの制御
  if(this.live&&!this.is_capture_pending&&now-this.lastTime>=1000/+$('sensor-fps').value)this.capture();
 }
 async capture({target_group=null}={}){
  if(this.is_capture_pending)return null;
  this.is_capture_pending=true;const generation=this.capture_generation;
  try{
   const start=performance.now(),elapsed=start-this.lastTime;this.lastTime=start;
   const robot_state=robot_snapshot(this.robot),robot_pose=robot_state.robot_pose,robot_model=this.robot.modelId,workspace_state=this.environment.getState();
   const frame=await this.sensor.capture(this.opticalWorld(),{mode:$('sensor-mode').value,exclude:[...this.exclude,this.cloud],enable_async_read:true,target_group});
   if(generation!==this.capture_generation)return null;
   this.sensor.lastFrame=frame;frame.robot_state=robot_state;frame.robotPose=robot_pose;frame.robotModel=robot_model;frame.workspace=workspace_state;if(!target_group)this.last_scene_frame=frame;this.rate=elapsed>0?1000/elapsed:0;
   this.paint(frame);if(this.cloud.visible)this.updateCloud(frame);$('sensor-export').disabled=false;
   $('sensor-stats').textContent=`${frame.valid.toLocaleString()} 点 / ${(frame.calibration.depth.width*frame.calibration.depth.height).toLocaleString()}画素（1フレーム） · 着色 ${frame.colored.toLocaleString()}点\nステレオ除外 ${frame.stereoRejected.toLocaleString()}点 · Min-Z ${frame.calibration.min_depth_m} m\nZ ${frame.min.toFixed(3)}–${frame.max.toFixed(3)} m · ${frame.ms.toFixed(1)} ms / 取得${this.live?' · 実測 '+this.rate.toFixed(1)+' Hz':''}`;
   this.lastSummary={ready:true,model:this.robot.modelId,live:this.live,frame:{id:frame.id,timestamp:frame.timestamp,width:frame.calibration.depth.width,height:frame.calibration.depth.height,valid:frame.valid,colored:frame.colored,stereoRejected:frame.stereoRejected,min:frame.min,max:frame.max,ms:frame.ms,renderMs:frame.renderMs,hz:this.rate,mode:frame.mode,depthWorld:frame.depthWorld},workspace:frame.workspace};
   this.last_capture_end_ms=performance.now();this.last_capture_ms=this.last_capture_end_ms-start;
   document.documentElement.dataset.rgbdState=JSON.stringify(this.lastSummary);return frame;
  }catch(e){if(generation!==this.capture_generation)return null;this.live=false;$('sensor-live').textContent='▶ RGB-D開始';$('sensor-stats').textContent='RGB-D取得エラー：'+e.message;console.error(e);this.toast(e.message);return null;}
  finally{this.is_capture_pending=false;}
 }
 paint(f){
  const rgb=$('rgb-preview'),d=$('depth-preview'),kc=f.calibration.color,k=f.calibration.depth;rgb.width=kc.width;rgb.height=kc.height;rgb.previousElementSibling.querySelector('span').textContent=`${kc.width} × ${kc.height}`;rgb.getContext('2d').putImageData(new ImageData(f.rgba,kc.width,kc.height),0,0);d.width=k.width;d.height=k.height;
  const rgba=new Uint8ClampedArray(k.width*k.height*4),lo=f.calibration.min_depth_m,hi=f.calibration.max_depth_m;
  for(let i=0;i<f.depth.length;i++){const z=f.depth[i],a=i*4;if(z){const t=THREE.MathUtils.clamp((z-lo)/(hi-lo),0,1);rgba[a]=255*Math.max(0,1-Math.abs(t*3-2));rgba[a+1]=255*Math.max(0,1-Math.abs(t*3-1));rgba[a+2]=255*Math.max(0,1-Math.abs(t*3));}else{rgba[a]=18;rgba[a+1]=20;rgba[a+2]=29;}rgba[a+3]=255;}d.getContext('2d').putImageData(new ImageData(rgba,k.width,k.height),0,0);
  $('depth-near').textContent=lo+' m';$('depth-far').textContent=hi+' m';$('pixel-u').max=k.width-1;$('pixel-v').max=k.height-1;if(+$('pixel-u').value>=k.width)$('pixel-u').value=Math.floor(k.width/2);if(+$('pixel-v').value>=k.height)$('pixel-v').value=Math.floor(k.height/2);
 }
 updateCloud(f){
  const {colors}=this.colorControls.compute(f.xyz,f.depthWorld,f.colors);this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.cloud.geometry.setAttribute('position',new THREE.BufferAttribute(f.xyz,3));this.cloud.geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));this.cloud.matrixAutoUpdate=false;this.cloud.matrix.fromArray(f.depthWorld);this.cloud.updateMatrixWorld(true);
 }
 updateFrustum(){const k=this.sensor.calibration.depth,m=this.sensor.opticalToWorld(this.opticalWorld()),points=[],origin=new THREE.Vector3().applyMatrix4(m),corners=[[-.5,-.5],[k.width-.5,-.5],[k.width-.5,k.height-.5],[-.5,k.height-.5]].map(([u,v])=>deproject(u,v,.7,k).applyMatrix4(m));for(let i=0;i<4;i++)points.push(origin,corners[i],corners[i],corners[(i+1)%4]);this.frustum.geometry.dispose();this.frustum.geometry=new THREE.BufferGeometry().setFromPoints(points);}
 readPixel(u,v){const f=this.sensor.lastFrame;if(!f)return;const k=f.calibration.depth;u=Math.round(u);v=Math.round(v);$('pixel-u').value=u;$('pixel-v').value=v;if(u<0||v<0||u>=k.width||v>=k.height){$('pixel-result').textContent='画像の範囲外です';return;}const z=f.depth[v*k.width+u];if(!z){$('pixel-result').textContent=`(${u}, ${v}) 深度なし（範囲外・背景・遮蔽）`;return;}const p=deproject(u,v,z,k),w=p.clone().applyMatrix4(new THREE.Matrix4().fromArray(f.depthWorld));$('pixel-result').textContent=`(${u}, ${v}) Z = ${z.toFixed(6)} m\n光学 XYZ: ${p.toArray().map(x=>x.toFixed(6)).join(', ')} m\n世界 XYZ: ${w.toArray().map(x=>x.toFixed(6)).join(', ')} m`;}
 async exportFrame(){
  const f=this.sensor.lastFrame;if(!f)return;$('sensor-export').disabled=true;
  try{
   const canvas=document.createElement('canvas');canvas.width=f.calibration.color.width;canvas.height=f.calibration.color.height;canvas.getContext('2d').putImageData(new ImageData(f.rgba,canvas.width,canvas.height),0,0);const png=await new Promise(resolve=>canvas.toBlob(resolve,'image/png'));if(!png)throw Error('PNG生成に失敗しました');
   const world=$('cloud-frame').value==='world',view=this.colorControls.compute(f.xyz,f.depthWorld,f.colors);const metadata={format:'topo-rgbd/1',robot_model:f.robotModel,display_color:view.summary,frame_id:f.id,timestamp:f.timestamp,mode:f.mode,point_frame:world?'base_footprint':'camera_depth_optical_frame',units:'m',depth_definition:'optical Z, not Euclidean ray length',invalid_depth:0,pixel_convention:'integer coordinates are pixel centres; top-left row first',float32_depth_file:'depth.f32',z16_file:'depth.z16',endianness:'little',depth_scale:f.calibration.depth_scale,calibration:f.calibration,depth_optical_to_world_column_major:f.depthWorld,color_optical_to_world_column_major:f.colorWorld,valid_points:f.valid,colored_points:f.colored,color_invalid_value:'RGB 155,165,175; color_valid=0 in PLY',robot_joints_rad:f.robotPose};
   const decoder=`import json\nimport numpy as np\nfrom pathlib import Path\np=Path(__file__).parent\nm=json.loads((p/'frame.json').read_text(encoding='utf-8'))\nk=m['calibration']['depth']\nz=np.fromfile(p/'depth.f32',dtype='<f4').reshape(k['height'],k['width'])\nv,u=np.indices(z.shape)\nxyz=np.stack(((u-k['ppx'])*z/k['fx'],(v-k['ppy'])*z/k['fy'],z),axis=-1)[z>0]\nT=np.array(m['depth_optical_to_world_column_major']).reshape(4,4,order='F')\nworld=xyz@T[:3,:3].T+T[:3,3]\nprint('valid:',len(xyz),'world bounds:',(world.min(0),world.max(0)) if len(world) else 'empty')\nnp.save(p/'points_optical.npy',xyz)\nnp.save(p/'points_world.npy',world)\n`;
   const bundle=await zipFiles({'frame.json':JSON.stringify(metadata,null,2),'scene.json':JSON.stringify(f.workspace,null,2),'calibration.json':JSON.stringify(f.calibration,null,2),'rgb.png':png,'depth.f32':f.depth,'depth.z16':f.z16,'points.ply':binaryPLY(f,world),'points_display.ply':displayPLY(f.xyz,f.depthWorld,view.rgb,world),'read_capture.py':decoder});await this.download('D435i-capture.zip',bundle,'application/zip');
  }catch(e){this.toast('保存エラー：'+e.message);}finally{$('sensor-export').disabled=false;}
 }
 async verify(){const wasLive=this.live;this.live=false;$('sensor-verify').disabled=true;$('sensor-qa').textContent='検証中…';try{const {runSensorQA}=await import('./rgbd-qa.js');const report=await runSensorQA(this.renderer);this.qa=report;$('sensor-qa').textContent=report.passed?`${report.tests.length}項目 PASS · 最大幾何誤差 ${report.maxErrorMm.toFixed(4)} mm`:'FAIL: '+report.tests.filter(x=>!x.passed).map(x=>x.name+': '+x.error).join(' / ');document.documentElement.dataset.sensorQa=JSON.stringify(report);}catch(e){$('sensor-qa').textContent='検証エラー：'+e.message;console.error(e);}finally{this.live=wasLive;$('sensor-verify').disabled=false;}}
}
