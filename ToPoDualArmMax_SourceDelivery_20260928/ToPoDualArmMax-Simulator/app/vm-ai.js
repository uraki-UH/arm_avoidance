import * as THREE from 'three';
import {decodeFrame} from './vm-packet.js';
const $=id=>document.getElementById(id);

export function worldPoints(xyz,matrix){
 const out=new Float32Array(xyz.length),m=matrix;
 for(let i=0;i<xyz.length;i+=3){const x=xyz[i],y=xyz[i+1],z=xyz[i+2];out[i]=m[0]*x+m[4]*y+m[8]*z+m[12];out[i+1]=m[1]*x+m[5]*y+m[9]*z+m[13];out[i+2]=m[2]*x+m[6]*y+m[10]*z+m[14];}
 return out;
}
export function encodeInput(meta,points){
 const json=new TextEncoder().encode(JSON.stringify(meta)),offset=8+Math.ceil(json.length/4)*4;
 const bytes=new Uint8Array(offset+points.byteLength);bytes.set([84,80,67,49]);new DataView(bytes.buffer).setUint32(4,json.length,true);bytes.set(json,8);bytes.set(new Uint8Array(points.buffer,points.byteOffset,points.byteLength),offset);return bytes;
}
function geometry(points){return new THREE.BufferGeometry().setAttribute('position',new THREE.BufferAttribute(points,3));}
function dispose(group){while(group.children.length){const o=group.children[0];group.remove(o);o.geometry?.dispose();o.material?.dispose();}}
function cubes(rows){
 const n=rows.length/10,g=new THREE.BoxGeometry(1,1,1),rgba=new Float32Array(n*4);
 const material=new THREE.ShaderMaterial({transparent:true,depthWrite:false,uniforms:{},vertexShader:`attribute vec4 candidateColor; varying vec4 vColor; void main(){vColor=candidateColor; gl_Position=projectionMatrix*modelViewMatrix*instanceMatrix*vec4(position,1.);}`,fragmentShader:`varying vec4 vColor; void main(){gl_FragColor=vec4(vColor.rgb,vColor.a);}`});
 g.setAttribute('candidateColor',new THREE.InstancedBufferAttribute(rgba,4));
 const mesh=new THREE.InstancedMesh(g,material,n),matrix=new THREE.Matrix4();
 for(let i=0;i<n;i++){const k=i*10;matrix.makeScale(rows[k+3],rows[k+4],rows[k+5]);matrix.setPosition(rows[k],rows[k+1],rows[k+2]);mesh.setMatrixAt(i,matrix);rgba.set(rows.subarray(k+6,k+10),i*4);}
 mesh.frustumCulled=false;return mesh;
}

export class VMAIWorkspace{
 constructor({scene,camera,robot,lidar,rgbd,environment,toast,motion}){
  Object.assign(this,{scene,camera,robot,lidar,rgbd,environment,toast,motion});
  this.client=crypto.randomUUID();this.generation=0;this.after=-1;this.inputHistory=new Map();this.running=false;this.busy=false;this.pollBusy=false;this.lastSend=0;this.lastPoll=0;this.sent=0;this.received=0;this.lastStatus=0;this.source='mid360';
  this.group=new THREE.Group();this.group.name='VM AiS-GNG-FVG results';scene.add(this.group);camera.layers.enable(3);
  $('ai-panel').innerHTML=`<div class="panel-heading"><div><span class="eyebrow">VMWARE · ROS 2</span><h2>AiS-GNG-FVG</h2></div><span class="chip">VM · ROS 2</span></div>
  <p class="sub-note">Ubuntu VM上のAiS-GNGとFVGに、動くロボットのセンサ点群を入力します。計測・3D描画はこのブラウザ、学習・推論はVMで実行します。</p>
  <div class="row-actions"><button id="ai-start" class="primary">▶ VM処理を開始</button><button id="ai-demo">▶ ロボットを動かす</button></div>
  <button id="ai-demo-scene" class="body-reset">テーブル付きデモシーンを読み込む</button>
  <label class="field-label">入力センサ<select id="ai-source"><option value="mid360">MID-360 · 実測走査方向</option><option value="d435i">RealSense D435i · 全有効画素</option></select></label>
  <label class="field-label">入力上限 Hz<input id="ai-rate" type="number" value="5" min="1" max="10" step="1"></label>
  <div class="settings"><label>学習ノード・エッジ<input id="ai-graph" type="checkbox" checked></label><label>処理と同じフレームの点群<input id="ai-points" type="checkbox"></label><label>FVG追加候補（水色）<input id="ai-add" type="checkbox" checked></label><label>FVG削除候補（橙）<input id="ai-delete" type="checkbox" checked></label><label>FVG記憶候補（紫）<input id="ai-memory" type="checkbox" checked></label></div>
  <pre id="ai-status" class="sensor-stats">VM接続を確認中…</pre><p class="sub-note">FVGは既存タスクと同じ0.4 mセルの候補表示です。ロボット制御やノードの追加・削除指令ではありません。点群送信は同時に1タブが担当します。</p>`;
  this.badge=document.createElement('div');this.badge.id='ai-live-badge';this.badge.hidden=true;this.badge.setAttribute('aria-live','off');$('viewport').append(this.badge);
  $('ai-start').onclick=()=>this.toggle();$('ai-demo').onclick=()=>{motion.toggle();$('ai-demo').textContent=motion.active()?'■ 動きを止める':'▶ ロボットを動かす';};
  $('ai-demo-scene').onclick=()=>this.demoScene();$('ai-source').onchange=async()=>{this.source=$('ai-source').value;this.lastSensorId=this.source==='mid360'?this.lidar.last?.id:null;if(this.running&&this.source==='mid360')await this.lidar.configure({...this.lidar.config,enabled:true});};
  for(const id of ['graph','points','add','delete','memory'])$('ai-'+id).onchange=()=>{if(this.frame)this.draw(this.frame);};
  this.checkStatus();
 }
 async checkStatus(){try{const r=await fetch('/api/status');if(!r.ok)throw Error('VM版は http://127.0.0.1:8878/ で開いてください');const state=await r.json();if(state.service!=='topo-robot-vm')throw Error('VM接続先が違います');this.vm=state;this.connectionError=null;$('load-status').textContent='VM · CONNECTED';}catch(e){this.connectionError=e.message;this.vm=null;}this.publish();}
 async demoScene(){try{const r=await fetch('examples/MID360-measured-close-mount.json');await this.environment.load(await r.json());this.toast('テーブルと7個の物体を読み込みました');}catch(e){this.error=e.message;this.publish();}}
 async toggle(){
  if(this.running){this.running=false;$('ai-start').textContent='▶ VM処理を開始';this.publish();return;}
  await this.checkStatus();if(!this.vm){this.toast(this.connectionError);return;}
  if(this.source==='mid360'){await this.lidar.configure({...this.lidar.config,enabled:true});$('lidar-show').checked=true;this.lidar.setCloud();}
  this.running=true;this.error=null;this.lastSensorId=this.source==='mid360'?this.lidar.last?.id:null;$('ai-start').textContent='■ 入力を停止';this.publish();
 }
 tick(now){
  if(now-this.lastPoll>120&&!this.pollBusy){this.lastPoll=now;this.poll();}
  if(now-this.lastStatus>1500){this.lastStatus=now;this.checkStatus();}
  const rate=Math.max(1,Math.min(10,+$('ai-rate').value||5));
  // One outstanding algorithm frame: slow RGB-D learning must not accumulate
  // an increasingly old ROS input queue. A lost output may retry after 1.5 s.
  if(this.waiting&&now-this.waiting.start>1500)this.waiting=null;
  if(this.running&&!this.busy&&!this.waiting&&now-this.lastSend>=1000/rate){this.lastSend=now;this.send();}
  if(this.lastReceived&&now-this.lastReceived>2000){this.group.visible=false;this.publish();}
 }
 async send(){
  const generation=this.generation;
  this.busy=true;
  try{
   let f;
   if(this.source==='mid360'){
    f=this.lidar.last;
    if(!f||f.id===this.lastSensorId){if(!this.lidar.busy)await this.lidar.capture();return;}
    if(!this.lidar.busy)this.lidar.capture();
   }else f=await this.rgbd.capture();
   if(generation!==this.generation||!f||!f.xyz.length)return;
   this.lastSensorId=f.id;
   const points=worldPoints(f.xyz,this.source==='mid360'?f.pose:f.depthWorld);
   const meta={client:this.client,frame_id:'base_footprint',sensor:this.source,sensor_frame:f.id,count:points.length/3,robot_pose:f.robotPose,robot_model:this.robot.modelId};
   const start=performance.now(),r=await fetch('/api/input',{method:'POST',headers:{'Content-Type':'application/octet-stream','X-ToPo-VM':'1'},body:encodeInput(meta,points)});
   if(!r.ok)throw Error(r.status===409?'別タブが点群を送信中です。このタブでは結果を表示します。':await r.text());
   const result=await r.json();if(generation!==this.generation)return;this.sent++;this.waiting={stamp:result.stamp_ns,start};this.inputHistory.set(result.stamp_ns,{start,points:meta.count,robotPose:f.robotPose,sensor:meta.sensor,sensorFrame:f.id});
   while(this.inputHistory.size>128)this.inputHistory.delete(this.inputHistory.keys().next().value);
   this.error=null;
  }catch(e){if(generation!==this.generation)return;this.error=e.message;this.running=false;$('ai-start').textContent='▶ VM処理を開始';}
  finally{if(generation===this.generation){this.busy=false;this.publish();}}
 }
 async poll(){
  if(!this.vm||this.vm.stale)return;const generation=this.generation;this.pollBusy=true;
  try{const r=await fetch('/api/frame?after='+this.after);if(r.status===204)return;if(!r.ok)throw Error('VMフレーム取得失敗');const f=decodeFrame(await r.arrayBuffer());if(f.meta.frame_id!=='base_footprint')throw Error('異なる座標系の結果です');
   if(generation!==this.generation)return;this.after=f.meta.sequence;if(this.onlyOwnFrames&&!this.inputHistory.has(f.meta.stamp_ns))return;this.frame=f;this.received++;this.lastReceived=performance.now();this.matched=this.inputHistory.get(f.meta.stamp_ns)||null;this.latency=this.matched?performance.now()-this.matched.start:null;if(this.waiting?.stamp===f.meta.stamp_ns)this.waiting=null;this.draw(f);
  }catch(e){if(generation===this.generation)this.error=e.message;}finally{if(generation===this.generation){this.pollBusy=false;this.publish();}}
 }
 resetForRobot(robot){this.generation++;this.robot=robot;this.running=false;this.busy=false;this.pollBusy=false;this.onlyOwnFrames=true;this.waiting=null;this.lastSensorId=null;this.frame=null;this.matched=null;this.lastReceived=null;this.latency=null;this.inputHistory.clear();dispose(this.group);this.group.visible=false;this.publish();}
 draw(f){
  dispose(this.group);
  if($('ai-points').checked)this.group.add(new THREE.Points(geometry(f.points),new THREE.PointsMaterial({color:0x507caa,size:1.5,sizeAttenuation:false,toneMapped:false})));
  if($('ai-graph').checked){
   this.group.add(new THREE.Points(geometry(f.nodes),new THREE.PointsMaterial({color:0xff9700,size:5,sizeAttenuation:false,toneMapped:false})));
   const edges=new Float32Array(f.edges.length*3);for(let i=0;i<f.edges.length;i++){const n=f.edges[i];if(n*3+2>=f.nodes.length)throw Error('Invalid graph edge');edges.set(f.nodes.subarray(n*3,n*3+3),i*3);}
   this.group.add(new THREE.LineSegments(geometry(edges),new THREE.LineBasicMaterial({color:0x009dbd,transparent:true,opacity:.9,toneMapped:false})));
  }
  for(const [key,id] of [['fvgAdd','add'],['fvgDelete','delete'],['fvgMemory','memory']])if($('ai-'+id).checked&&f[key].length)this.group.add(cubes(f[key]));
  this.group.traverse(o=>o.layers.set(3));this.camera.layers.enable(3);this.group.visible=true;
 }
 publish(){
  const m=this.frame?.meta,age=this.lastReceived?performance.now()-this.lastReceived:null,stale=age===null||age>2000;
  const state={connected:!!this.vm,running:this.running,busy:this.busy,source:this.source,sent:this.sent,received:this.received,stale,age_ms:age,latency_ms:this.latency,matched:!!this.matched,matched_input_points:this.matched?.points,domain:this.vm?.domain,hostname:this.vm?.hostname,frame:m?{sequence:m.sequence,stamp_ns:m.stamp_ns,points:m.pointCount,nodes:m.nodeCount,edges:m.edgeCount,fvg_same_frame:m.fvg_same_frame,fvg_add:this.frame.fvgAdd.length/10,fvg_delete:this.frame.fvgDelete.length/10,fvg_memory:this.frame.fvgMemory.length/10,metrics:m.metrics}:null,error:this.error,robotMoving:this.motion.active()};
  document.documentElement.dataset.aiState=JSON.stringify(state);
  const fnum=n=>Number.isFinite(n)?n.toFixed(1):'—';
  $('ai-status').textContent=this.connectionError||this.error||`${this.vm?this.vm.hostname+' · ROS 2 domain '+this.vm.domain:'未接続'}\n入力 ${this.sent} / 描画 ${this.received} フレーム · ${stale?'更新待ち':this.running?'実行中':'入力停止'}\n${m?m.pointCount.toLocaleString()+' 点 → '+m.nodeCount.toLocaleString()+' ノード / '+m.edgeCount.toLocaleString()+' エッジ':'点群入力を待っています'}\nAiS ${fnum(m?.metrics.ais_ms)} ms / FVG ${fnum(m?.metrics.fvg_ms)} ms\n往復・待機・転送 ${fnum(this.latency)} ms\n入力実測 ${fnum(m?.metrics.input_hz)} Hz · FVG同一フレーム ${m?.fvg_same_frame?'一致':'—'}`;
  this.badge.hidden=!this.vm;this.badge.textContent=`VM · AiS-GNG-FVG ${stale?'更新待ち':m?.nodeCount+' nodes · '+m?.edgeCount+' edges'}${this.motion.active()?' · ROBOT MOVING':''}`;
 }
}
