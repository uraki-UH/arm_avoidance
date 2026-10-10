import {CloudColorControls,displayPLY,update_cloud_geometry} from './pointcloud-colors.js';
import {loadMeasuredScan,measuredMetadata} from './measured-scan.js';
import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {load_jt128_cad} from './jt128-model.js';
import {lidar_preset,lidar_ray_rate,defaultLidarConfig,validateLidarConfig,lidarPLY,waistLidarMount,scanDirection,PETAL_SCAN,packedPositions} from './lidar-core.js';
import {robot_snapshot} from './robot-ros-state.js';
import {zipFiles} from './capture-zip.js';
const $=id=>document.getElementById(id),rad=Math.PI/180;
export class LidarWorkspace{
 constructor({scene,overlay,robot,renderer,camera,environment,toast,download,onLayers}){
  Object.assign(this,{scene,overlay,robot,renderer,camera,environment,toast,download,onLayers});this.config=defaultLidarConfig();this.sequence=0;this.scanTime=0;this.known=new Set();this.live=false;this.busy=false;this.lastTime=0;this.id=0;
  Object.assign(this.config,this.mount_preset());
  this.mount=new THREE.Group();this.mount.name='MID-360 sensor origin';this.model=new THREE.Group();this.mount.add(this.model);scene.add(this.mount);this.mount.visible=false;
  this.cad_model=new THREE.Group();this.mount.add(this.cad_model);
  this.buildBracket();
  this.axes=new THREE.AxesHelper(.13);this.axes.visible=false;this.axes.layers.set(2);this.mount.add(this.axes);
  this.cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:2,sizeAttenuation:false,vertexColors:true,toneMapped:false}));this.cloud.layers.set(2);this.cloud.frustumCulled=false;this.cloud.visible=false;scene.add(this.cloud);
  this.generation=0;this.createWorker();
  this.ui();this.apply();loadMeasuredScan().then(()=>{this.drawPattern();this.publish();}).catch(e=>this.toast(e.message));environment.getSensorState=()=>({...structuredClone(this.config)});environment.validateSensorState=validateLidarConfig;environment.loadSensorState=c=>this.configure(c);this.publish();
 }
 ui(){
  $('lidar-panel').innerHTML=`<div class="panel-heading"><div><span class="eyebrow">360° LiDAR</span><h2>LiDAR</h2></div><span id="lidar-rate" class="chip">200k slots/s</span></div>
  <label class="field-label">センサの種類<select id="lidar-sensor-type" aria-label="LiDARの種類"><option value="mid360" selected>Livox MID-360</option><option value="jt128">Hesai JT128</option></select></label>
  <label class="sensor-enable"><input type="checkbox" id="lidar-enable"> センサを有効にする</label>
  <div class="row-actions"><select id="lidar-capture-mode" aria-label="LiDARの取得方式"><option value="continuous" selected>連続</option><option value="once">1回</option></select><button id="lidar-once" disabled hidden>1回取得</button></div>
  <label class="field-label">走査パターン<select id="lidar-pattern"><option value="measured">実測 · Livox公式サンプルの走査方向</option><option value="petal">旧・花びらの数式近似</option><option value="low-discrepancy">均等分布 · 旧方式</option></select></label>
  <figure class="lidar-pattern-preview"><figcaption>走査方向 <span>センサ上面から · 距離ではありません</span></figcaption><canvas id="lidar-pattern-preview" width="320" height="210" role="img" aria-label="MID-360の花びら状走査軌跡"></canvas><div id="lidar-pattern-time" class="sub-note"></div></figure>
  <label class="field-label">取得モード<select id="lidar-mode"><option value="ideal">幾何真値 · 最近傍の表面</option><option value="noise">距離ノイズ近似 · Gaussian</option></select></label>
  <div class="field-grid"><label>連続取得 Hz<input id="lidar-hz" type="number" min="0.1" max="40" step="0.1" value="10"></label><label>積分時間<select id="lidar-duration"><option value="0.025">0.025 s · 5,000スロット</option><option value="0.1" selected>0.1 s · 20,000スロット</option><option value="0.5">0.5 s · 100,000スロット</option><option value="1">1 s · 200,000スロット</option></select></label><label>最大距離 m<input id="lidar-range" type="number" min="0.2" max="100" step="1" value="40"></label><label>ノイズ σ mm<input id="lidar-noise" type="number" min="0" max="100" value="20"></label><label>乱数シード<input id="lidar-seed" type="number" value="12345" step="1"></label></div>
  <div id="lidar-stats" class="sensor-stats" role="status">無効 · 取得待ち</div><div class="row-actions"><label><input id="lidar-show" type="checkbox" checked>点群を表示</label><label><input id="lidar-only" type="checkbox">点群のみ</label><label><input id="lidar-axes" type="checkbox">座標軸</label></div>
  <div id="lidar-colors" class="cloud-color-controls"></div><div class="row-actions"><select id="lidar-frame"><option value="world">world [m]</option><option value="sensor">mid360 [m]</option></select><button id="lidar-export" disabled>↓ 点群一式 ZIP</button></div>
  <details open><summary>取付位置・姿勢</summary><button id="lidar-waist-preset" class="wide-button">ロボットの既定取付位置に戻す</button><label class="field-label">親座標<select id="lidar-parent"><option value="torso_link">腰上リンク · 腰Yawに追従</option><option value="neck_tilt_link">首 Pitchリンク</option><option value="neck_pan_link">首 Yawリンク</option><option value="base_footprint">ロボット基準</option><option value="world">ワールド固定</option></select></label><div class="field-grid">${['x','y','z','roll','pitch','yaw'].map(k=>`<label>${k.toUpperCase()} ${k.length===1?'mm':'°'}<input id="lidar-${k}" type="number" step="${k.length===1?'5':'1'}" value="0"></label>`).join('')}</div><p id="lidar-mount-note" class="sub-note"></p></details>
  <details><summary>モデル出典</summary><a id="lidar-model-source" href="assets/mid360/mid-360.stl" download>公式CAD変換 STL（mm）</a> · <a href="ASSET_SOURCES.md" target="_blank">モデル出典</a></details>
  <button id="lidar-verify" class="wide-button">点群の幾何精度を検証</button><output id="lidar-qa"></output>`;
  this.colorControls=new CloudColorControls($('lidar-colors'),'lidar','reflectance',()=>{this.cloud_frame=null;this.setCloud();this.publish();});
  $('lidar-enable').onchange=async()=>{
   this.config.enabled=$('lidar-enable').checked;this.apply();
   if(!this.config.enabled)return;
   this.live=$('lidar-capture-mode').value==='continuous';
   this.capture();this.publish();
   try{await this.loadModel();if(this.config.enabled)this.toast('センサを有効にしました');}catch(e){this.toast('STL読込エラー：'+e.message);}
  };
  $('lidar-sensor-type').onchange=()=>this.select_sensor($('lidar-sensor-type').value);
  $('lidar-once').onclick=()=>this.capture();
  $('lidar-capture-mode').onchange=()=>{
   const is_once=$('lidar-capture-mode').value==='once';
   this.live=this.config.enabled&&!is_once;
   $('lidar-once').hidden=!is_once;
   $('lidar-hz').disabled=is_once;this.publish();
  };
  $('lidar-pattern').onchange=()=>{this.config.scanPattern=$('lidar-pattern').value;this.drawPattern();this.publish();};
  $('lidar-hz').onchange=()=>{const hz=$('lidar-hz').valueAsNumber;if(!Number.isFinite(hz)||hz<.1||hz>40){$('lidar-hz').value=this.config.capture_hz;this.toast('連続取得は0.1〜40 Hzを指定してください');return;}this.config.capture_hz=hz;this.publish();};
  $('lidar-mode').onchange=()=>{this.config.mode=$('lidar-mode').value;};$('lidar-duration').onchange=()=>{this.config.duration=+$('lidar-duration').value;this.config.beams=Math.round(this.config.duration*lidar_ray_rate(this.config));this.drawPattern();this.publish();};
  $('lidar-waist-preset').onclick=()=>{Object.assign(this.config,this.mount_preset());this.apply();this.toast('LiDARを既定の取付位置に戻しました');};
  for(const [id,key,scale,min,max] of [['range','maxRange',1,.2,100],['noise','noiseSigma',.001,0,100],['seed','seed',1,0,4294967295]]){const el=$('lidar-'+id);el.oninput=()=>{if(Number.isFinite(el.valueAsNumber))this.config[key]=THREE.MathUtils.clamp(el.valueAsNumber,min,key==='maxRange'&&this.config.sensor_type==='jt128'?60:max)*scale;};}
  for(const [i,k]of['x','y','z','roll','pitch','yaw'].entries()){$('lidar-'+k).oninput=()=>{const v=$('lidar-'+k).valueAsNumber;if(!Number.isFinite(v))return;if(i<3)this.config.position[i]=THREE.MathUtils.clamp(v/1000,-100,100);else this.config.rpy[i-3]=v*rad;this.apply();};}
  $('lidar-parent').onchange=()=>{this.config.parent=$('lidar-parent').value;this.apply();};
  for(const id of ['lidar-show','lidar-only'])$(id).onchange=()=>this.setCloud();$('lidar-axes').onchange=()=>{this.axes.visible=$('lidar-axes').checked;this.onLayers();};$('lidar-export').onclick=()=>this.exportFrame();
  $('lidar-verify').onclick=async()=>{const b=$('lidar-verify');b.disabled=true;try{const {runLidarQA}=await import('./lidar-qa.js'),r=await runLidarQA();$('lidar-qa').textContent=`${r.passed?'PASS':'FAIL'} · ${r.tests.filter(x=>x.passed).length}/${r.tests.length}\n`+r.tests.map(x=>(x.passed?'✓ ':'✗ ')+x.name+(x.error?' '+x.error:'')).join('\n');document.documentElement.dataset.lidarQa=JSON.stringify(r);}catch(e){$('lidar-qa').textContent=e.message;}finally{b.disabled=false;}};
 }
 buildBracket(){
  const metal=new THREE.MeshStandardMaterial({color:'#424a4e',metalness:.75,roughness:.32}),silver=new THREE.MeshStandardMaterial({color:'#aab6bc',metalness:.85,roughness:.24});
  this.support=new THREE.Group();this.support.name='MID-360 waist mounting bracket';this.scene.add(this.support);
  const back=new THREE.Mesh(new THREE.BoxGeometry(.004,.068,.05),metal);this.supportBack=back;back.position.set(.0385,0,.080);back.castShadow=back.receiveShadow=true;this.support.add(back);
  this.supportBars=[-1,1].map(()=>{const mesh=new THREE.Mesh(new THREE.CylinderGeometry(1,1,1,12),metal);mesh.castShadow=mesh.receiveShadow=true;this.support.add(mesh);return mesh;});
  this.mountPlate=new THREE.Group();this.mountPlate.name='MID-360 tilted mounting plate';this.mount.add(this.mountPlate);
  const plate=new THREE.Mesh(new THREE.BoxGeometry(.064,.070,.003),metal);plate.position.z=-.028;plate.castShadow=plate.receiveShadow=true;this.mountPlate.add(plate);
  for(const x of [-.027,.027])for(const y of [-.027,.027]){const bolt=new THREE.Mesh(new THREE.CylinderGeometry(.0025,.0025,.002,6),silver);bolt.rotation.x=Math.PI/2;bolt.position.set(x,y,-.0255);this.mountPlate.add(bolt);}
 }
 updateBracket(parent){
  parent.add(this.support);this.support.visible=this.config.enabled&&this.config.parent==='torso_link';this.mountPlate.visible=this.config.parent==='torso_link';
  this.supportBack.position.z=this.config.position[2]-.025;const q=this.mount.quaternion,offset=this.mount.position,up=new THREE.Vector3(0,1,0);
  for(const [i,s]of[-1,1].entries()){const a=new THREE.Vector3(.0405,s*.026,this.config.position[2]-.025),b=new THREE.Vector3(0,s*.026,-.030).applyQuaternion(q).add(offset),d=b.clone().sub(a),bar=this.supportBars[i];bar.position.copy(a).add(b).multiplyScalar(.5);bar.quaternion.setFromUnitVectors(up,d.clone().normalize());bar.scale.set(.0035,Math.max(.001,d.length()),.0035);}
 }
 drawPattern(startTime=this.scanTime,duration=this.config.duration,beamStart=this.sequence,config=this.config){
  const canvas=$('lidar-pattern-preview');if(!canvas)return;
  // 非表示プレビューの描画保留。再表示時には最新の走査条件のみ反映
  if(canvas.offsetParent===null){this.pending_pattern={start_time:startTime,duration,beam_start:beamStart,config:{...config}};return;}
  this.pending_pattern=null;
  if(config.scanPattern==='measured'&&!measuredMetadata()){$('lidar-pattern-time').textContent='公式走査データ読込中…';return;}const ctx=canvas.getContext('2d'),cx=160,cy=105,R=86;
  ctx.clearRect(0,0,320,210);ctx.fillStyle='#fbf9fc';ctx.fillRect(0,0,320,210);ctx.strokeStyle='#e8deeb';ctx.lineWidth=1;
  for(const r of [R*.392,R*.65,R]){ctx.beginPath();ctx.arc(cx,cy,r,0,2*Math.PI);ctx.stroke();}
  ctx.beginPath();ctx.moveTo(cx-R,cy);ctx.lineTo(cx+R,cy);ctx.moveTo(cx,cy-R);ctx.lineTo(cx,cy+R);ctx.stroke();ctx.fillStyle='#80658a';ctx.font='10px sans-serif';ctx.textAlign='center';ctx.fillText('X 前',cx,12);ctx.fillText('Y 左',44,cy+3);ctx.fillText(config.sensor_type==='jt128'?'−4.43°':'−7°',cx+R+17,cy+15);ctx.fillText(config.sensor_type==='jt128'?'+88.99°':'+52°',cx+R*.392+19,cy+15);
  const n=Math.min(20000,config.beams),d=new THREE.Vector3();let prev=null;
  for(let i=0;i<n;i++){const u=i/(n-1),beam=Math.floor(u*(config.beams-1)),t=startTime+beam*duration/config.beams;scanDirection(beamStart+beam,d,config.scanPattern,t);if(d.lengthSq()===0){prev=null;continue;}const e=Math.asin(d.z)/rad,a=Math.atan2(d.y,d.x),r=R*(90-e)/97,x=cx-r*Math.sin(a),y=cy-r*Math.cos(a);ctx.strokeStyle=ctx.fillStyle=`hsla(${285-165*u},65%,42%,.8)`;if(config.scanPattern==='petal'&&prev){ctx.beginPath();ctx.moveTo(...prev);ctx.lineTo(x,y);ctx.stroke();}else ctx.fillRect(x,y,1,1);prev=[x,y];}
  canvas.dataset.scanStart=String(startTime);canvas.dataset.pattern=config.scanPattern;
  $('lidar-pattern-time').textContent=`${startTime.toFixed(3)}〜${(startTime+duration).toFixed(3)} s · ${config.scanPattern==='jt128'?'128チャンネル・設計角度の回転走査':config.scanPattern==='measured'?'実測方向・元記録5秒でループ':config.scanPattern==='petal'?'旧数式近似':'均等分布の比較用'}`;
 }
 mount_preset(){
  const sensor=this.robot.links.chest_lidar_link;
  if(!sensor){const preset=waistLidarMount();if(this.config.sensor_type==='jt128')preset.rpy=[0,0,0];return preset;}
  this.robot.updateWorldMatrix(true,true);
  const relative=this.robot.links.torso_link.matrixWorld.clone().invert().multiply(sensor.matrixWorld);
  const position=new THREE.Vector3(),rotation=new THREE.Quaternion(),scale=new THREE.Vector3();
  relative.decompose(position,rotation,scale);
  if(this.config.sensor_type==='jt128')return {parent:'torso_link',position:position.toArray(),rpy:[0,0,0]};
  return {parent:'torso_link',position:position.toArray(),rpy:new THREE.Euler().setFromQuaternion(rotation,'ZYX').toArray().slice(0,3)};
 }
 is_mount_preset(){
  const preset=this.mount_preset(),c=this.config;
  return c.parent===preset.parent&&['position','rpy'].every(key=>c[key].every((value,idx)=>Math.abs(value-preset[key][idx])<1e-9));
 }
 sync_robot_model(){
  const source=this.robot.links.chest_lidar_link;
  if(this.cad_source!==source){
   this.cad_model.clear();this.cad_source=source;
   // 描画キャッシュ共有。URDFリンク階層・関節TFの維持
   if(source)for(const child of source.children)this.cad_model.add(child.clone());
  }
  const is_jt128=this.config.sensor_type==='jt128';
  // 取得ON/OFFと筐体表示の分離。選択中のJT128は常時表示
  this.mount.visible=this.config.enabled||is_jt128;
  this.model.visible=!is_jt128&&!source;this.cad_model.visible=!is_jt128;if(this.jt128_model)this.jt128_model.visible=is_jt128;this.modelReady=is_jt128?!!this.has_jt128_model:!!source||!!this.has_legacy_model;
  if(is_jt128){this.support.visible=false;this.mountPlate.visible=false;}
  if(!source)return;
  // 無効時はロボット本体、有効時は設定位置の複製のみ表示
  source.visible=!this.config.enabled&&!is_jt128;
  this.robot.links.chest_lidar_mount_link.visible=!is_jt128&&(!this.config.enabled||this.is_mount_preset());
  this.support.visible=false;this.mountPlate.visible=false;
 }
 async loadModel(){if(this.config.sensor_type==='jt128')return this.load_jt128_model();if(this.robot.links.chest_lidar_link||this.has_legacy_model)return;if(this.modelPromise)return this.modelPromise;this.modelPromise=(async()=>{const geometry=await new STLLoader().loadAsync('./assets/mid360/mid-360.stl');const pos=geometry.attributes.position,colors=new Float32Array(pos.count*3),silver=new THREE.Color('#858f95'),dark=new THREE.Color('#151c20'),base=new THREE.Color('#454d53');for(let i=0;i<pos.count;i++)(pos.getY(i)>6?dark:pos.getY(i)<-17?base:silver).toArray(colors,i*3);geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));geometry.scale(.001,.001,.001);geometry.rotateX(Math.PI/2);geometry.computeVertexNormals();const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({vertexColors:true,metalness:.5,roughness:.28}));mesh.castShadow=mesh.receiveShadow=true;this.model.add(mesh);this.has_legacy_model=true;this.sync_robot_model();this.renderer.shadowMap.needsUpdate=true;this.publish();})().catch(e=>{this.modelPromise=null;throw e;});return this.modelPromise;}

 clear_capture(){
  this.finish_capture(null);this.generation++;this.worker.terminate();this.createWorker();this.busy=false;this.pending=null;this.last=null;this.known.clear();this.sequence=0;this.scanTime=0;this.pending_pattern=null;
  this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.cloud_frame=null;this.cloud.visible=false;$('lidar-export').disabled=true;
 }
 select_sensor(sensor_type){
  const is_live=this.live,enable_sensor=this.config.enabled;this.clear_capture();this.config={...lidar_preset(sensor_type),enabled:enable_sensor};Object.assign(this.config,this.mount_preset());this.apply();
  this.live=enable_sensor&&is_live;if(enable_sensor)this.capture();
 }
 load_jt128_model(){
  if(this.jt128_model_promise)return this.jt128_model_promise;
  this.jt128_model_promise=load_jt128_cad().then(model=>{
   this.jt128_model=model;this.mount.add(model);this.has_jt128_model=true;
   this.sync_robot_model();this.renderer.shadowMap.needsUpdate=true;this.publish();
  }).catch(error=>{this.jt128_model_promise=null;throw error;});
  return this.jt128_model_promise;
 }
 sync_sensor_ui(){
  const c=this.config,is_jt128=c.sensor_type==='jt128',sensor_name=is_jt128?'Hesai JT128':'Livox MID-360';$('lidar-sensor-type').value=c.sensor_type;
  $('lidar-rate').textContent=is_jt128?'1,152k slots/s':'200k slots/s';
  const patterns=is_jt128?[['jt128','JT128 · 128チャンネル回転走査']]:[['measured','実測 · Livox公式サンプルの走査方向'],['petal','旧・花びらの数式近似'],['low-discrepancy','均等分布 · 旧方式']];
  const pattern_select=$('lidar-pattern');if(pattern_select.dataset.sensor_type!==c.sensor_type){pattern_select.replaceChildren(...patterns.map(([value,text])=>new Option(text,value)));pattern_select.dataset.sensor_type=c.sensor_type;}
  const duration_select=$('lidar-duration');if(duration_select.dataset.sensor_type!==c.sensor_type){duration_select.replaceChildren(...[.025,.1,.5,1].map(value=>new Option(`${value} s · ${Math.round(value*lidar_ray_rate(c)).toLocaleString()}スロット`,String(value))));duration_select.dataset.sensor_type=c.sensor_type;}
  const model_source=$('lidar-model-source');model_source.href=is_jt128?'assets/jt128/jt128-side-connector.stp':'assets/mid360/mid-360.stl';model_source.textContent=is_jt128?'Hesai公式CAD（STEP・側面コネクタ型）':'公式CAD変換 STL（mm）';model_source.setAttribute('download','');
  $('lidar-range').max=is_jt128?'60':'100';$('lidar-frame').options[1].textContent=(is_jt128?'jt128':'mid360')+' [m]';$('lidar-pattern-preview').setAttribute('aria-label',sensor_name+'の走査方向');
 }
 configure(c){const config=validateLidarConfig(c);this.clear_capture();this.config=config;this.live=false;this.apply();if(this.config.enabled)return this.loadModel();}
 apply(){const c=this.config;this.sync_sensor_ui();if(c.sensor_type==='jt128')this.load_jt128_model().catch(error=>this.toast('CAD読込エラー：'+error.message));const parent=c.parent==='world'?this.scene:this.robot.links[c.parent];if(!parent)throw Error('親リンクがありません：'+c.parent);parent.add(this.mount);this.mount.position.fromArray(c.position);this.mount.rotation.set(...c.rpy,'ZYX');this.updateBracket(parent);$('lidar-pattern').value=c.scanPattern;this.drawPattern();$('lidar-enable').checked=c.enabled;$('lidar-once').disabled=!c.enabled;
  this.sync_robot_model();
  $('lidar-mount-note').textContent=c.sensor_type==='jt128'?'Hesai公式CAD・側面コネクタ型。既定の胸部位置・傾斜0°。':this.cad_source?'Long: STEP由来の45°取付。既定位置はURDFと一致。機械取付原点のため実機計測原点は未校正。任意配置では固定ブラケットを非表示。':'標準: 前77.69 mm・上105 mm、前下がり45°の簡易取付。';
  if(!c.enabled){this.live=false;$('lidar-only').checked=false;this.cloud.visible=false;$('lidar-stats').textContent='無効 · 点群取得停止';}else $('lidar-stats').textContent=this.last?'設定変更済み · 次の取得で反映':'有効 · 取得待ち';
  $('lidar-hz').value=c.capture_hz;$('lidar-parent').value=c.parent;$('lidar-mode').value=c.mode;$('lidar-duration').value=String(c.duration);for(const [i,k]of['x','y','z','roll','pitch','yaw'].entries())if(document.activeElement!==$('lidar-'+k))$('lidar-'+k).value=(i<3?c.position[i]*1000:c.rpy[i-3]/rad).toFixed(2);$('lidar-range').value=c.maxRange;$('lidar-noise').value=c.noiseSigma*1000;$('lidar-seed').value=c.seed;this.renderer.shadowMap.needsUpdate=true;this.setCloud();this.publish();
 }
 setCloud(){this.cloud.visible=!!(this.config.enabled&&this.last&&($('lidar-show').checked||$('lidar-only').checked));if(this.cloud.visible&&this.last!==this.cloud_frame)this.updateCloud(this.last);this.cloudOnly=$('lidar-only').checked;this.onLayers();}
 tick(now){
  if(this.pending_pattern&&$('lidar-pattern-preview').offsetParent!==null){const pending=this.pending_pattern;this.drawPattern(pending.start_time,pending.duration,pending.beam_start,pending.config);}
  if(this.live&&!this.busy&&now-this.lastTime>=1000/this.config.capture_hz)this.capture();
 }
 createWorker(){this.worker=new Worker(new URL('./lidar-worker.js',import.meta.url),{type:'module'});this.worker.onmessage=e=>this.receive(e.data);this.worker.onerror=e=>{this.finish_capture(null);this.busy=false;this.live=false;$('lidar-stats').textContent='取得エラー：'+e.message;this.publish({error:e.message});};}
 resetForRobot(robot){
  const is_preset=this.is_mount_preset();
  for(const name of ['chest_lidar_link','chest_lidar_mount_link'])if(this.robot.links[name])this.robot.links[name].visible=true;
  this.finish_capture(null);this.generation++;this.worker.terminate();this.createWorker();this.robot=robot;
  if(is_preset)Object.assign(this.config,this.mount_preset());
  this.live=false;this.busy=false;this.pending=null;this.last=null;this.known.clear();this.sequence=0;this.scanTime=0;this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.cloud_frame=null;this.cloud.visible=false;$('lidar-export').disabled=true;this.apply();
  if(this.config.enabled)this.loadModel().catch(e=>this.toast(e.message));
 }
 finish_capture(frame){this.resolve_capture?.(frame);this.resolve_capture=null;}
 async capture(){if(!this.config.enabled||this.busy)return null;const generation=this.generation;this.busy=true;this.lastTime=performance.now();if(!this.last)$('lidar-stats').textContent='取得中 · シーンの表面を計測しています';
  try{await this.loadModel();if(this.config.scanPattern==='measured')await loadMeasuredScan();if(generation!==this.generation)return;this.scene.updateMatrixWorld(true);const geometries=[],meshes=[],names={},used=new Set(),exclude=new Set();this.mount.traverse(x=>exclude.add(x));let mid=1;
   this.scene.traverseVisible(o=>{if(!o.isMesh||exclude.has(o)||!o.layers.isEnabled(0)||!o.geometry.attributes.position)return;const g=o.geometry,key=g.uuid;used.add(key);if(!this.known.has(key)){geometries.push({id:key,position:packedPositions(g.attributes.position),index:g.index?Uint32Array.from(g.index.array):null});this.known.add(key);}const m=Array.isArray(o.material)?o.material[0]:o.material,color=m.color;const reflectance=color?THREE.MathUtils.clamp(.2126*color.r+.7152*color.g+.0722*color.b,0,1):.5;names[mid]=o.name||o.parent?.name||'surface';meshes.push({id:mid++,geometry:key,matrix:o.matrixWorld.toArray(),reflectance});});
   this.known=used;const config=structuredClone(this.config),id=++this.id,pose=this.mount.matrixWorld.toArray();this.pending={id,pose,config,beamStart:this.sequence,scanStart:this.scanTime,timestamp:new Date().toISOString(),workspace:this.environment.getState(),robotPose:this.robot.getPose(),robotModel:this.robot.modelId,robot_state:robot_snapshot(this.robot),names};this.sequence+=config.beams;this.scanTime+=config.duration;
   const completed=new Promise(resolve=>{this.resolve_capture=resolve;});
   this.publish();this.worker.postMessage({id,config,pose,meshes,geometries,start:this.pending.beamStart,startTime:this.pending.scanStart},geometries.flatMap(g=>[g.position.buffer,...(g.index?[g.index.buffer]:[])]));
   return await completed;
  }catch(e){if(generation!==this.generation)return null;this.finish_capture(null);this.busy=false;this.live=false;$('lidar-stats').textContent='取得エラー：'+e.message;this.publish({error:e.message});}
 }
 receive(d){if(d.id!==this.pending?.id)return;this.busy=false;if(d.error){this.finish_capture(null);this.live=false;this.known.clear();$('lidar-stats').textContent='取得エラー：'+d.error;this.publish({error:d.error});return;}const f={...this.pending,...d.frame};f.elapsedMs=performance.now()-this.lastTime;this.last=f;this.finish_capture(f);this.drawPattern(f.scanStart,f.config.duration,f.beamStart,f.config);this.setCloud();
  $('lidar-export').disabled=false;$('lidar-stats').textContent=`有効 ${f.count.toLocaleString()} 点 / ${f.config.beams.toLocaleString()} スロット\n参照方向なし ${f.unknownDirections.toLocaleString()} · 命中なし ${f.noReturn.toLocaleString()} · 近距離除外 ${f.nearRejected.toLocaleString()}\n計算 ${f.ms.toFixed(1)} ms · 全体 ${f.elapsedMs.toFixed(0)} ms\nシミュレーション ${f.config.duration} s · 実測最大 ${(1000/f.elapsedMs).toFixed(1)} frame/s`;this.publish();
 }
 updateCloud(f){const view=this.colorControls.compute(f.xyz,f.pose,f.reflectance);update_cloud_geometry(this.cloud,f.xyz,view.colors);this.cloud.matrixAutoUpdate=false;this.cloud.matrix.fromArray(f.pose);this.cloud.updateMatrixWorld(true);this.cloud_frame=f;}
 publish(extra={}){this.mount.updateWorldMatrix(true,false);document.documentElement.dataset.lidarState=JSON.stringify({model:this.robot.modelId,mountPose:this.mount.matrixWorld.toArray(),scanTime:this.scanTime,enabled:this.config.enabled,modelReady:!!this.modelReady,busy:this.busy,live:this.live,config:this.config,colors:this.colorControls?.summary,reference:measuredMetadata(),frame:this.last?{model:this.last.robotModel,id:this.last.id,count:this.last.count,beams:this.last.config.beams,ms:this.last.ms,elapsedMs:this.last.elapsedMs,pose:this.last.pose,nearRejected:this.last.nearRejected,unknownDirections:this.last.unknownDirections,noReturn:this.last.noReturn,rangeRejected:this.last.rangeRejected,scanStart:this.last.scanStart,beamStart:this.last.beamStart,pattern:this.last.config.scanPattern}:null,...extra});}
 async exportFrame(){const f=this.last;if(!f)return;const world=$('lidar-frame').value==='world',view=this.colorControls.compute(f.xyz,f.pose,f.reflectance);const meta={format:f.config.sensor_type==='jt128'?'topo-lidar/1':'topo-mid360/3',robot_model:f.robotModel,display_color:view.summary,unknown_directions:f.unknownDirections,no_return:f.noReturn,range_rejected:f.rangeRejected,slot_status_codes:{valid:0,reference_direction_unknown:1,no_surface_in_range:2,below_minimum_range:3,noise_out_of_range:4},id:f.id,timestamp:f.timestamp,point_frame:world?'world':f.config.sensor_type||'mid360',units:'m',pose_column_major:f.pose,config:f.config,beam_start:f.beamStart,scan_start_s:f.scanStart,valid_points:f.count,near_rejected:f.nearRejected,compute_ms:f.ms,elapsed_ms:f.elapsedMs,scan_pattern:f.config.scanPattern==='jt128'?{model:'hesai-jt128-design-j01',num_channels:128,rotation_hz:10,nominal_rays_per_sec:1152000}:f.config.scanPattern==='measured'?measuredMetadata():f.config.scanPattern==='petal'?{...PETAL_SCAN,description:'continuous non-repetitive flower imitation; not the proprietary Livox trajectory or calibrated point density'}:{model:'low-discrepancy-legacy'},pose_model:'all rays use one frozen scene and pose',reflectance:'synthetic linear visible albedo; not calibrated infrared intensity',objects:f.names,robot_joints_rad:f.robotPose};const bundle=await zipFiles({'frame.json':JSON.stringify(meta,null,2),'scene.json':JSON.stringify(f.workspace,null,2),'points.ply':lidarPLY(f,world),'points_display.ply':displayPLY(f.xyz,f.pose,view.rgb,world),'slot_status.u8':f.slotStatus,'slot_range.f32':f.slotRange,'xyz_sensor.f32':f.xyz,'range.f32':f.range,'time_offset.f32':f.time,'beam_index.u32':f.beamIndex,'object_id.u32':f.objectId});await this.download(f.config.sensor_type==='jt128'?'JT128-capture.zip':'MID360-capture.zip',bundle,'application/zip');}
}
