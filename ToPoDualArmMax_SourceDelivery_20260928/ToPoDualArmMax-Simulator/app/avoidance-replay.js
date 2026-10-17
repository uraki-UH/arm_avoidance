import {format_fixed} from './display-format.js';
import {observe_balanced_layout} from './balanced-layout.js';
import * as THREE from 'three';
import {Line2} from 'three/addons/lines/Line2.js';
import {LineGeometry} from 'three/addons/lines/LineGeometry.js';
import {LineMaterial} from 'three/addons/lines/LineMaterial.js';

const policy_names = {vlut_graph:'GNG-VLUT',avoidance_first:'回避優先',vrm_predictive:'予測・不一致適応',rollout_reactive:'幾何候補・現在観測',rollout_cv:'幾何候補・等速予測',rollout_sequence:'時系列記憶',cbf_qp:'関節CBF-QP',oscbf:'OSCBF'};
const failure_names = {physical_contact:'接触',internal_interference:'内部干渉',abnormal_joint_speed:'異常速度',joint_limit:'試験可動域の超過（旧記録）',planning_domain_exit:'旧計画域の超過',physical_joint_limit:'関節可動域の超過',invalid_obstacle_trajectory:'障害物の禁止域侵入・評価対象外',qp_infeasible:'QP実行不能・試験停止',qp_no_solution:'QP解未成立・試験停止'};
const phase_names = {task:'往復',avoiding:'回避',holding:'保持',waiting:'待機',returning:'復帰',monitoring:'待機',waiting_for_clearance:'離隔待ち',qp_no_solution:'QP不成立・減速保持／再計算',constraint_recovery:'制約回復'};
const obstacle_phase_names = {pursuing:'追跡',sweep_approach:'通過点へ接近',sweeping:'横切り',random_walking:'ランダム移動',random_waiting:'ランダム移動・次の方向待ち',detouring:'迂回',route_blocked:'経路制約・移動困難',target_out_of_reach:'腕の到達域外'};
Object.assign(failure_names,{target_touch:'指定リンク接触',attacker_protected_contact:'近位部接触・評価対象外',attacker_off_target_contact:'指定外接触・評価対象外',attacker_shaft_contact:'先端以外の接触・評価対象外',attacker_environment_contact:'接触',robot_environment_contact:'接触'});
const startup_stage_names = {robot_geometry:'ロボット形状',drive_calibration:'速度基準',obstacle_setup:'障害物配置',cbf_solver:'QP初期化',gng_index:'GNG索引',ready:'計算開始'};
const max_file_size = 16*1024*1024;

export class live_playback_clock {
 constructor({buffer_ms=120,max_receive_age_ms=500,max_backlog_ms=600}={}){
  if(![buffer_ms,max_receive_age_ms,max_backlog_ms].every(value=>Number.isFinite(value)&&value>0)||max_backlog_ms<=buffer_ms)throw Error('表示バッファの時間設定不正');
  this.buffer_ms=buffer_ms;this.max_receive_age_ms=max_receive_age_ms;this.max_backlog_ms=max_backlog_ms;
  this.time_sec=null;this.latest_sec=null;this.last_receive_ms=null;this.last_tick_ms=null;
  this.rate=1;this.num_resync=0;this.last_resync_ms=-Infinity;
 }

 observe(first_sec,latest_sec,now_ms){
  if(![first_sec,latest_sec,now_ms].every(Number.isFinite)||first_sec>latest_sec||latest_sec<(this.latest_sec??first_sec)||now_ms<(this.last_receive_ms??now_ms))throw Error('表示時計の受信時刻不正');
  if(this.time_sec===null){
   this.time_sec=Math.max(first_sec,latest_sec-this.buffer_ms/1000);this.first_receive_ms=now_ms;this.last_tick_ms=now_ms;
  }else{
   const period_ms=now_ms-this.last_receive_ms;
   if(period_ms>0&&period_ms<this.max_receive_age_ms){
    // シミュレーションの進行率に追従する表示時計。物理時刻の書換えなし。
    const observed_rate=Math.max(.05,Math.min(4,(latest_sec-this.latest_sec)*1000/period_ms));
    this.rate+=(observed_rate-this.rate)*(1-Math.exp(-period_ms/500));
   }
   if(this.time_sec<first_sec||period_ms>=this.max_receive_age_ms||latest_sec-this.time_sec>this.max_backlog_ms*this.rate/1000){
    this.time_sec=Math.max(this.time_sec,first_sec,latest_sec-this.buffer_sec());this.last_tick_ms=now_ms;
    this.num_resync++;this.last_resync_ms=now_ms;
   }
  }
  this.latest_sec=latest_sec;this.last_receive_ms=now_ms;
 }

 buffer_sec(){return Math.max(.05,this.buffer_ms*this.rate/1000);}

 advance(now_ms){
  if(!Number.isFinite(now_ms)||now_ms<(this.last_tick_ms??now_ms))throw Error('表示時計の描画時刻不正');
  if(this.time_sec===null)return null;
  const period=Math.max(0,(now_ms-Math.max(this.last_tick_ms,this.first_receive_ms+this.buffer_ms))/1000);this.last_tick_ms=now_ms;
  if(now_ms-this.last_receive_ms>=this.max_receive_age_ms||now_ms-this.first_receive_ms<this.buffer_ms)return this.time_sec;
  if(period>.25){
   // 長い描画休止後の明示再同期。経過時間の一括積分による最新姿勢への飛越しの回避。
   this.time_sec=Math.max(this.time_sec,this.latest_sec-this.buffer_sec());this.num_resync++;this.last_resync_ms=now_ms;
   return this.time_sec;
  }
  // 受信済みの区間だけの連続補間。未受信の未来姿勢への外挿なし。
  const error=this.latest_sec-this.time_sec-this.buffer_sec();
  const play_rate=Math.max(0,Math.min(this.rate*1.5,this.rate+3*error));
  this.time_sec=Math.min(this.latest_sec,this.time_sec+period*play_rate);
  return this.time_sec;
 }

 status(now_ms){
  const receive_age_ms=this.last_receive_ms===null?Infinity:Math.max(0,now_ms-this.last_receive_ms);
  return {receive_age_ms,display_gap_ms:Math.max(0,(this.latest_sec-this.time_sec)*1000),
   is_stalled:receive_age_ms>=this.max_receive_age_ms,is_resync:now_ms-this.last_resync_ms<500};
 }
}

function local_recording_url(value) {
 const url = new URL(value,location.href);
 if(url.origin!==location.origin||!url.pathname.startsWith('/exports/avoidance-')||!url.pathname.endsWith('.json')||url.username||url.password)throw Error('表示専用の同一サーバー記録が必要です');
 return url.href;
}

async function load_json(value) {
 const response = await fetch(local_recording_url(value),{cache:'no-store',signal:AbortSignal.timeout(15000)});
 if(!response.ok)throw Error('記録がありません。先に --replay 付きの起動、または追跡球試験を実行してください');
 if(Number(response.headers.get('content-length'))>max_file_size)throw Error('記録の容量超過');
 const text = await response.text();
 if(new Blob([text]).size>max_file_size)throw Error('記録の容量超過');
 return JSON.parse(text);
}

export function validate_recording(value,joint_names,model='long',min_num_frames=value?.is_partial===true?1:2,allow_live_time=false) {
 if(value?.format!=='topo-avoidance-replay/1'||value.model!=='long'||value.model!==model||!Array.isArray(value.frames)||value.frames.length<min_num_frames||value.frames.length>20000)throw Error('LongモデルとLong用の回避実験記録が必要です');
 if(typeof value.failure_reason!=='string')throw Error('失敗状態の形式が不正です');
 if(value.task!==undefined){
  const task=value.task,is_point=point=>Array.isArray(point)&&point.length===3&&point.every(v=>Number.isFinite(v)&&Math.abs(v)<100);
  if(!task||!['cartesian','joint'].includes(task.mode)||task.eef_link!=='L_tcp'||task.frame!=='world'||task.reference_kind!=='goal_line'||!Array.isArray(task.reference_path)||task.reference_path.length!==2||!task.reference_path.every(is_point))throw Error('往復タスクの形式不正');
  for(const [idx,name] of ['a','b'].entries()){
   const goal=task.goals?.[name];
   if(!goal||!is_point(goal.position)||!Array.isArray(goal.quaternion)||goal.quaternion.length!==4||!goal.quaternion.every(Number.isFinite)||Math.abs(Math.hypot(...goal.quaternion)-1)>1e-5||goal.position.some((v,axis)=>Math.abs(v-task.reference_path[idx][axis])>1e-6))throw Error('手先目標の形式不正');
  }
  if(!Number.isFinite(task.max_position_error_th)||task.max_position_error_th<=0||task.max_position_error_th>.1||!Number.isFinite(task.max_orientation_error_rad_th)||task.max_orientation_error_rad_th<=0||task.max_orientation_error_rad_th>.5)throw Error('手先到達条件の形式不正');
 }
 if(value.pursuit_mode!==undefined&&!['link_point','distal_sweep','random_walk'].includes(value.pursuit_mode))throw Error('球の動きの形式が不正です');
 if(value.pursuit_mode==='random_walk'||value.random_walk!==undefined){
  const region=value.random_walk,is_point=point=>Array.isArray(point)&&point.length===3&&point.every(v=>Number.isFinite(v)&&Math.abs(v)<100);
  if(value.pursuit_mode!=='random_walk'||value.obstacle_shape!=='sphere'||!region||!is_point(region.min_position_m)||!is_point(region.max_position_m)||region.min_position_m.some((v,idx)=>v>=region.max_position_m[idx])||!Number.isFinite(region.change_sec)||region.change_sec<.1||region.change_sec>5)throw Error('ランダムウォーク範囲の形式不正');
 }
 if(value.failure_details!==undefined){
  const detail=value.failure_details;
  if(!detail||typeof detail!=='object'||Array.isArray(detail)||('contact_links' in detail&&(!Array.isArray(detail.contact_links)||!detail.contact_links.every(name=>typeof name==='string'&&name.length<=128)))||('joints' in detail&&(!Array.isArray(detail.joints)||!detail.joints.every(joint=>joint&&joint_names.includes(joint.name)))))throw Error('停止詳細の形式が不正です');
 }
 if(value.joint_position_limits!==undefined){
  const limits=value.joint_position_limits;
  if(!limits||typeof limits!=='object'||Array.isArray(limits)||Object.entries(limits).some(([name,bounds])=>!joint_names.includes(name)||!bounds||!Number.isFinite(bounds.min_position_rad)||!Number.isFinite(bounds.max_position_rad)||bounds.min_position_rad>=bounds.max_position_rad||typeof bounds.source!=='string'))throw Error('関節可動域の形式が不正です');
 }
 let last_sec = -Infinity;
 for(const frame of value.frames){
  const names=Object.keys(frame.joints||{});
  if(!Number.isFinite(frame.time_sec)||frame.time_sec<0||frame.time_sec>(allow_live_time?Number.MAX_SAFE_INTEGER:180)||frame.time_sec<=last_sec)throw Error('記録時刻の形式が不正です');
  if(names.length!==joint_names.length||joint_names.some(name=>!Object.hasOwn(frame.joints,name)||!Number.isFinite(frame.joints[name])||Math.abs(frame.joints[name])>100*Math.PI))throw Error('記録の関節構成が一致しません');
  for(const key of ['eef','sphere'])if(!Array.isArray(frame[key])||frame[key].length!==3||!frame[key].every(v=>Number.isFinite(v)&&Math.abs(v)<100))throw Error('記録位置の形式が不正です');
  if(value.random_walk&&frame.sphere.some((v,idx)=>v<value.random_walk.min_position_m[idx]-1e-6||v>value.random_walk.max_position_m[idx]+1e-6))throw Error('ランダムウォーク範囲外の記録');
  const is_point=point=>Array.isArray(point)&&point.length===3&&point.every(v=>Number.isFinite(v)&&Math.abs(v)<100);
  if(Object.hasOwn(frame,'target')!==Object.hasOwn(value.frames[0],'target')||(Object.hasOwn(frame,'target')&&!is_point(frame.target)))throw Error('追跡点の形式が不正です');
  if(Object.hasOwn(frame,'obstacle_segments')!==Object.hasOwn(value.frames[0],'obstacle_segments'))throw Error('障害物形状の不一致');
  if(Object.hasOwn(frame,'obstacle_segments')&&(!Array.isArray(frame.obstacle_segments)||frame.obstacle_segments.length!==2||!frame.obstacle_segments.every(segment=>Array.isArray(segment)&&segment.length===2&&segment.every(is_point))))throw Error('リンク障害物の形式が不正です');
  if(!Number.isFinite(frame.sphere_radius_m)||frame.sphere_radius_m<=0||frame.sphere_radius_m>2||typeof frame.has_contact!=='boolean'||typeof frame.phase!=='string'||!Number.isFinite(frame.proxy_gap_m))throw Error('記録状態の形式が不正です');
  if(frame.obstacle_phase!==undefined&&(!Object.hasOwn(obstacle_phase_names,frame.obstacle_phase)||!Number.isFinite(frame.obstacle_speed_mps)||frame.obstacle_speed_mps<0))throw Error('障害物状態の形式が不正です');
  if(frame.obstacle_trail_sec!==undefined&&(!Number.isFinite(frame.obstacle_trail_sec)||frame.obstacle_trail_sec<0||frame.obstacle_trail_sec>2))throw Error('障害物軌跡の時間幅が不正です');
  if(frame.task_goal!==undefined&&(!value.task||!['a','b'].includes(frame.task_goal)))throw Error('往復目標の形式不正');
  if(frame.num_arrivals!==undefined&&(!Number.isSafeInteger(frame.num_arrivals)||frame.num_arrivals<0))throw Error('到達回数の形式不正');
  for(const key of ['task_position_error_m','task_orientation_error_rad'])if(frame[key]!==undefined&&(!Number.isFinite(frame[key])||frame[key]<0))throw Error('手先誤差の形式不正');
  if(frame.motion!==undefined){
   if(!frame.motion||Array.isArray(frame.motion)||typeof frame.motion!=='object')throw Error('実験の関節運動状態の形式不正');
   for(const [field,values] of Object.entries(frame.motion)){
    if(!['velocity','effort','current_a'].includes(field)||!values||Array.isArray(values)||typeof values!=='object'||Object.entries(values).some(([name,value])=>!joint_names.includes(name)||!Number.isFinite(value)||typeof value!=='number'))throw Error('実験の関節運動値の不正');
   }
  }
  last_sec=frame.time_sec;
 }
 return value;
}

export function interpolate_joint_motion(first,second,fraction,previous=null){
 const motion={velocity:{},effort:{},current_a:{},estimated_velocity_joints:[]};
 for(const name of Object.keys(first.joints)){
  for(const field of ['velocity','effort','current_a']){
   const a=first.motion?.[field]?.[name],b=second.motion?.[field]?.[name];
   if(Number.isFinite(a)&&Number.isFinite(b))motion[field][name]=a+(b-a)*fraction;
  }
  if(Object.hasOwn(motion.velocity,name))continue;
  // 旧形式の角度標本からの差分推定。長い欠測・単一標本のゼロ補完なし。
  const start=first===second?previous:first,end=first===second?first:second;
  const period=end.time_sec-(start?.time_sec??end.time_sec);
  if(period>0&&period<=.3){motion.velocity[name]=(end.joints[name]-start.joints[name])/period;motion.estimated_velocity_joints.push(name);}
 }
 return motion;
}

export class avoidance_replay {
 constructor({robot,scene,renderer,camera,controls,on_pose}) {
  this.robot=robot;this.scene=scene;this.renderer=renderer;this.camera=camera;this.orbit=controls;this.on_pose=on_pose;
  this.recording=null;this.is_playing=false;this.last_ms=null;this.time_sec=0;this.load_generation=0;this.num_applied_frames=0;
  this.is_live=false;this.is_active=false;this.has_pose_control=false;this.is_closed=false;this.live_job_id=null;this.live_next_frame_idx=0;this.experiment_generation=0;
  sessionStorage.removeItem('avoidance-deleted-recording');
  this.root=new THREE.Group();this.root.name='avoidance-replay';scene.add(this.root);
  this.sphere=new THREE.Mesh(new THREE.SphereGeometry(1,32,20),new THREE.MeshStandardMaterial({color:0xf28b24,roughness:.45}));
  this.sphere.name='追跡球';this.root.add(this.sphere);this.root.visible=false;
  this.capsules=[0,1].map(()=>{
   const group=new THREE.Group(),material=this.sphere.material;
   const cylinder=new THREE.Mesh(new THREE.CylinderGeometry(1,1,1,16),material);
   const ends=[0,1].map(()=>new THREE.Mesh(new THREE.SphereGeometry(1,16,12),material));
   group.add(cylinder,...ends);this.root.add(group);return {group,cylinder,ends};
  });
  this.trails=['eef','sphere'].map((name,idx)=>{
   const line=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:idx?0x1984e8:0xe0ac12}));line.name=name+'-trajectory';this.root.add(line);return line;
  });
  this.make_controls();this.make_experiment_controls();
  this.load_catalog();
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.pause();});
  window.addEventListener('keydown',event=>{
   if(!document.querySelector('[data-panel=experiment]')?.classList.contains('active'))return;
   if(/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)||event.ctrlKey||event.metaKey||event.altKey)return;
   if(event.code==='Space'){event.preventDefault();event.stopImmediatePropagation();this.element('toggle').click();}
   if(event.key==='Escape'){event.stopImmediatePropagation();this.pause();}
  },true);
 }

 set_active(is_active) {
  this.is_active=is_active;
  const has_pose_control=!!this.is_live||(is_active&&!!this.recording);
  if(has_pose_control)this.robot.pose_source='ros';
  if(has_pose_control===this.has_pose_control)return;
  this.has_pose_control=has_pose_control;
  this.root.visible=has_pose_control&&!!this.recording;
  this.robot.pose_source=has_pose_control?'ros':'simulator';
  if(has_pose_control){
   window.simulator?.cancel_robot_motion();
   if(this.recording)this.seek(this.time_sec);
  }else this.pause();
  for(const button of document.querySelectorAll('[data-model]'))button.disabled=has_pose_control;
  const source=document.getElementById('robot-pose-source');if(source)source.disabled=has_pose_control;
 }

 make_controls() {
  const section=document.createElement('section');section.className='avoidance-replay';
  section.innerHTML=`<style>
   .avoidance-replay{font-size:14px;line-height:1.6}.avoidance-replay h2{font-size:20px}.avoidance-replay select{font-size:13px;width:100%;min-width:0;margin:10px 0}
   .avoidance-replay button{font-size:14px;background:#f4f5f7;color:#222;border-color:#bbc0c7}.avoidance-replay .replay-actions{display:grid;gap:6px;margin:12px 0}
   .avoidance-replay label{white-space:nowrap;gap:12px}.avoidance-replay label select{width:auto;flex:1}
   .avoidance-replay .replay-transport{gap:2px}.avoidance-replay .replay-transport button{min-width:0;font-size:12px;padding:7px 4px;white-space:nowrap}
   .avoidance-replay #replay-toggle:not(:disabled){background:#96e6ac;color:#153820;border-color:#53a86f}
   .avoidance-replay #replay-status{white-space:pre-wrap;font:13px/1.7 system-ui}.avoidance-replay #replay-status:empty{display:none}.avoidance-replay #replay-time{display:block;text-align:right;font-variant-numeric:tabular-nums}
   .avoidance-replay .replay-note{color:var(--muted);font-size:12px}.avoidance-replay a{color:var(--cyan)}
   </style><h2>回避実験</h2><div class="replay-note"></div>
   <select id="replay-run" aria-label="実験記録"></select>
   <div class="replay-actions replay-transport"><button id="replay-toggle" disabled>再生</button><button id="replay-reset" disabled>先頭</button><button id="replay-event" disabled>停止位置</button><button id="replay-delete" disabled>記録を削除</button></div>
   <input id="replay-seek" type="range" min="0" max="1" step="0.001" value="0" aria-label="再生時刻" disabled><output id="replay-time">0.00 sec</output>
   <label>速度<select id="replay-speed" aria-label="再生速度"><option value="0.25">0.25倍</option><option value="0.5">0.5倍</option><option value="1" selected>1倍</option><option value="2">2倍</option><option value="4">4倍</option></select></label>
   <div id="replay-task-controls" hidden><label><input id="replay-task-visible" type="checkbox" checked>基準線</label><div class="replay-note">破線: 基準線（軌道拘束なし） · 黄: 手先の実軌跡</div></div>
   <pre id="replay-status" role="status"></pre>`;
  document.getElementById('experiment-panel').append(section);this.controls=section;
  for(const container of section.querySelectorAll('.replay-actions'))observe_balanced_layout(container);
  this.element('toggle').onclick=()=>{
   if(this.is_live||this.is_experiment_running||this.is_archive_busy)return;
   const selected=this.element('run').value;
   if(selected&&new URL(selected,location.href).pathname!==this.current_recording_url)this.open(selected);
   else this.toggle();
  };
  this.element('reset').onclick=()=>{this.pause();this.seek(this.recording.frames[0].time_sec);};
  this.element('event').onclick=()=>{this.pause();this.seek(this.recording.frames.at(-1).time_sec);};
  this.element('seek').oninput=()=>{this.pause();this.seek(Number(this.element('seek').value));};
  this.element('run').onchange=()=>{this.pause();this.update_archive_controls();};
  this.element('delete').onclick=()=>this.delete_recording();
  this.element('task-visible').onchange=()=>{if(this.task_reference)this.task_reference.visible=this.element('task-visible').checked;};
 }

 element(name){return document.getElementById('replay-'+name);}

 set_task_reference(task){
  // 記録切替時だけの描画資源更新。フレームごとのメッシュ生成なし。
  if(this.task_reference){
   this.task_reference.traverse(object=>{
    object.geometry?.dispose();
    for(const material of Array.isArray(object.material)?object.material:[object.material]){material?.map?.dispose();material?.dispose();}
   });
   this.root.remove(this.task_reference);this.task_reference=null;
  }
  const controls=this.element('task-controls');if(controls)controls.hidden=!task;
  if(!task)return;
  const root=new THREE.Group();root.name='task-reference';
  // 表示倍率・GPUによらず実軌跡の細線より太い5px幅。照明・露出に依存しない濃青の破線
  const line=new Line2(new LineGeometry().setPositions(task.reference_path.flat()),new LineMaterial({color:0x1d4ed8,linewidth:5,dashed:true,dashSize:.025,gapSize:.015,toneMapped:false}));
  // 表示専用レイヤー。センサ点群・深度画像・物体選択の対象外
  line.layers.set(4);
  line.name='task-reference-line';line.computeLineDistances();root.add(line);
  root.visible=this.element('task-visible')?.checked!==false;this.root.add(root);this.task_reference=root;
 }

 async delete_recording(){
  if(this.is_live||this.is_experiment_running||this.is_archive_busy)return;
  const selected=this.element('run').value;
  if(!selected)return;
  const recording_url=new URL(local_recording_url(selected)).pathname;
  if(!window.confirm('選択した記録JSONと元の実験ログ・軌跡を完全削除しますか？\n取り消し不可。'))return;
  this.pause();this.is_archive_busy=true;this.update_archive_controls();
  try{
   const result=await this.request_experiment('/api/avoidance-experiments/delete',{recording_url});
   this.apply_catalog(result.catalog);
   if(this.current_recording_url===recording_url){
    this.current_recording_url=null;this.recording=null;this.motion_sample=null;this.root.visible=false;this.set_task_reference(null);
    this.set_active(this.is_active);
    for(const name of ['toggle','reset','event','seek'])this.element(name).disabled=true;
    this.element('time').textContent='0.00 sec';this.element('status').textContent=result.catalog.runs.length?'':'実験記録なし';
   }
  }catch(error){this.element('status').textContent='記録操作の失敗：'+error.message;}
  finally{this.is_archive_busy=false;this.update_archive_controls();}
 }

 update_archive_controls(){
  const can_edit=!this.is_live&&!this.is_experiment_running&&!this.is_archive_busy;
  const selected=this.element('run').value;
  const has_selected_recording=!!this.recording&&(!selected||new URL(selected,location.href).pathname===this.current_recording_url);
  this.element('delete').disabled=!can_edit||!selected;
  this.element('toggle').disabled=!can_edit||(!selected&&!this.recording)||(has_selected_recording&&!!this.recording.is_transient);
  for(const name of ['reset','seek'])this.element(name).disabled=!can_edit||!has_selected_recording||!!this.recording?.is_transient;
  this.element('event').disabled=!can_edit||!has_selected_recording||!this.recording?.failure_reason||!!this.recording?.is_transient;
  if(this.experiment_panel)this.experiment_panel.querySelector('#experiment-start').disabled=this.is_archive_busy||this.is_experiment_running||!this.is_experiment_available;
 }

 apply_catalog(data){
  if(data.format!=='topo-avoidance-catalog/1'||!Array.isArray(data.runs)||data.runs.length>256)throw Error('実験一覧が不正です');
  this.element('run').replaceChildren(...data.runs.map(run=>{
   const option=document.createElement('option');option.value=local_recording_url(run.url);
   option.textContent=`${run.has_unbounded_joint_limits?'[旧・角度制限未反映] ':run.has_legacy_planning_domain?'[旧試験] ':''}${policy_names[run.policy]||run.policy}${run.candidate_metric==='occupancy'?'・占有候補':''} · ${run.obstacle_shape==='linked_arm'?'腕型':'球'}${run.pursuit_mode==='random_walk'?'・ランダムウォーク':run.pursuit_mode==='distal_sweep'?'・横切り':''} → ${run.target_link||'L_tcp'}${['distal_sweep','random_walk'].includes(run.pursuit_mode)?'以降':''} · 条件${run.seed} · ${Number(run.duration_sec).toFixed(2)} sec`;return option;
  }));
  this.element('run').disabled=!data.runs.length;
  if(data.selected)this.element('run').value=local_recording_url(data.selected);
 }

 async load_catalog(){
  // 一覧取得と記録再生の分離。画面起動・タブ選択による姿勢変更なし。
  const generation=this.load_generation;
  try{
   const data=await load_json('/exports/avoidance-replays.json');
   if(generation!==this.load_generation||this.is_closed||this.is_archive_busy)return;
   this.apply_catalog(data);this.update_archive_controls();
  }catch(error){
   if(generation===this.load_generation&&!this.is_closed)this.element('status').textContent='記録一覧なし：'+error.message;
  }
 }

 make_experiment_controls(){
  const panel=document.createElement('div');
  panel.innerHTML=`<style>
   #experiment-form .experiment-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;margin:12px 0;width:100%;min-width:0}
   #experiment-form label{display:flex;flex-direction:column;align-items:stretch;min-width:0;gap:4px;white-space:normal;font-size:13px}
   #experiment-form input,#experiment-form select{box-sizing:border-box;width:100%;min-width:0;max-width:100%;flex:none;font-size:14px;margin:0;padding:7px}
   #experiment-form .experiment-offset{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:4px}
   #experiment-start:not(:disabled){background:#96e6ac;color:#153820;border-color:#53a86f}#experiment-stop:not(:disabled){background:#ffe1df;color:#7e2420;border-color:#d58984}#experiment-status{white-space:pre-wrap;font-size:13px;margin:8px 0}
   </style><form id="experiment-form"><div class="experiment-grid">
   <label>実行方式<select name="execution_mode"><option value="continuous" selected>連続・記録なし</option><option value="recorded">時間指定・記録あり</option></select></label>
   <label>回避方式<select name="policy"><option value="vlut_graph">GNG-VLUT</option><option value="rollout_cv">幾何候補・等速予測</option><option value="rollout_reactive">幾何候補・現在観測</option><option value="cbf_qp">関節CBF-QP</option><option value="oscbf">OSCBF</option></select></label>
   <label>障害物<select name="obstacle_shape"><option value="sphere">球</option><option value="linked_arm">二節腕・球関節×2</option></select></label>
   <label>球の動き<select name="pursuit_mode"><option value="distal_sweep">末端側を横切る</option><option value="link_point">リンク点を追跡</option><option value="random_walk">ランダムウォーク</option></select></label>
   <label>移動経路<select name="enable_proximal_guard"><option value="true">近位部を迂回</option><option value="false">迂回制約なし</option></select></label>
   <label style="grid-column:1/-1"><span id="experiment-link-label">通過を許す開始リンク</span><select name="target_link"></select></label>
   <label id="experiment-offset-label" style="grid-column:1/-1">リンク内の追跡点 [m]<span class="experiment-offset"><input name="offset_x" aria-label="追跡点X" type="number" min="-.5" max=".5" step=".01" value="0" required><input name="offset_y" aria-label="追跡点Y" type="number" min="-.5" max=".5" step=".01" value="0" required><input name="offset_z" aria-label="追跡点Z" type="number" min="-.5" max=".5" step=".01" value="0" required></span></label>
   ${[['duration_sec','速度上昇時間 [sec]',18,1,60,1],['seed','条件seed',401,0,2147483647,1],['radius_m','半径 [m]',.06,.015,.15,.005],['speed_start','開始速度比',.4,.05,3,.05],['speed_end','最終速度比',1.3,.05,3,.05],['max_ball_acceleration','加速度 [m/s²]',3,.1,30,.1],['max_ball_deceleration','減速力 [m/s²]',8,.1,30,.1],['max_ball_turn_acceleration','旋回加速度 [m/s²]',10,.1,30,.1],['random_walk_extent_m','移動範囲 ±[m]',.6,.3,1.5,.05],['random_walk_change_sec','方向更新間隔 [sec]',.6,.1,5,.1]].map(([name,label,value,min,max,step])=>`<label><span${name==='duration_sec'?' id="experiment-time-label"':''}>${label}</span><input type="number" name="${name}" value="${value}" min="${min}" max="${max}" step="${step}" required></label>`).join('')}
   </div>
   <div class="replay-actions"><button id="experiment-start" type="submit" disabled>実験を実行</button><button id="experiment-view" type="button" hidden>実行中を表示</button><button id="experiment-stop" type="button" disabled>中止</button></div></form><div id="experiment-status" role="status">実行環境を確認中</div>`;
  this.controls.prepend(panel);this.experiment_panel=panel;
  for(const container of panel.querySelectorAll('.replay-actions'))observe_balanced_layout(container);
  const form=panel.querySelector('form'),links=form.elements.namedItem('target_link');
  form.elements.namedItem('execution_mode').onchange=()=>{
   const is_continuous=form.elements.namedItem('execution_mode').value==='continuous';
   panel.querySelector('#experiment-time-label').textContent=is_continuous?'速度上昇時間 [sec]':'時間 [sec]';
  };
  for(const name of Object.keys(this.robot.links)){const option=document.createElement('option');option.value=name;option.textContent=name;links.append(option);}links.value='L_link4';
  const update_pursuit_mode=()=>{
   const mode=form.elements.namedItem('pursuit_mode'),is_linked=form.elements.namedItem('obstacle_shape').value==='linked_arm';
   // 旧サーバーの対応範囲はリンク点追跡のみ。未確認時の既定選択の維持。
   const supported=this.supported_pursuit_modes??['link_point'];
   for(const option of mode.options){
    option.disabled=this.is_experiment_available!==undefined&&!supported.includes(option.value);
    if(option.value==='distal_sweep')option.textContent=option.disabled?'末端側を横切る（サーバー未対応）':'末端側を横切る';
    if(option.value==='random_walk')option.textContent=option.disabled?'ランダムウォーク（サーバー未対応）':'ランダムウォーク';
   }
   if(mode.selectedOptions[0]?.disabled)mode.value='link_point';
   if(is_linked)mode.value='link_point';mode.disabled=is_linked;
   const is_sweep=mode.value==='distal_sweep',is_random=mode.value==='random_walk',has_distal_range=is_sweep||is_random;
   panel.querySelector('#experiment-link-label').textContent=is_random?'範囲中心・対象開始リンク（開始時）':is_sweep?'通過を許す開始リンク':'追跡リンク';
   panel.querySelector('#experiment-offset-label').style.display=has_distal_range?'none':'';
   for(const axis of ['x','y','z'])form.elements.namedItem('offset_'+axis).disabled=has_distal_range;
   for(const name of ['random_walk_extent_m','random_walk_change_sec']){const field=form.elements.namedItem(name);field.disabled=!is_random;field.closest('label').style.display=is_random?'':'none';}
   for(const option of links.options)option.disabled=has_distal_range&&!/^L_(link[1-7]|tcp|gripper_base|finger_(left|right))$/.test(option.value);
   if(links.selectedOptions[0]?.disabled)links.value='L_link4';
  };
  this.update_pursuit_mode=update_pursuit_mode;
  form.elements.namedItem('pursuit_mode').onchange=update_pursuit_mode;
  form.elements.namedItem('obstacle_shape').onchange=()=>{form.elements.namedItem('radius_m').value=form.elements.namedItem('obstacle_shape').value==='linked_arm'?.035:.06;update_pursuit_mode();};
  update_pursuit_mode();
  this.owned_job_id=sessionStorage.getItem('avoidance-experiment-job');
  const request=async(pathname,config)=>{
   const response=await fetch(pathname,{method:config===undefined?'GET':'POST',headers:config===undefined?{}:{'Content-Type':'application/json','X-Topo-Experiment':'1'},body:config===undefined?undefined:JSON.stringify(config),signal:AbortSignal.timeout(15000),cache:'no-store'});
   if(response.status===404)throw Error('画面サーバーの更新が必要です');
   const result=await response.json();if(!response.ok)throw Error(result.message||'実験操作の失敗');return result;
  };
  this.request_experiment=request;
  form.onsubmit=async event=>{
   event.preventDefault();if(this.is_archive_busy||!form.reportValidity())return;
   const fields=new FormData(form),number=name=>Number(fields.get(name));
   const pursuit_mode=form.elements.namedItem('pursuit_mode').value;
   const config={execution_mode:fields.get('execution_mode'),policy:fields.get('policy'),obstacle_shape:fields.get('obstacle_shape'),pursuit_mode,enable_proximal_guard:fields.get('enable_proximal_guard')==='true',target_link:fields.get('target_link'),target_offset_m:pursuit_mode!=='link_point'?[0,0,0]:['x','y','z'].map(axis=>number('offset_'+axis)),speed_ratios:[number('speed_start'),number('speed_end')]};
   if(pursuit_mode==='random_walk')for(const name of ['random_walk_extent_m','random_walk_change_sec'])config[name]=number(name);
   for(const name of ['duration_sec','seed','radius_m','max_ball_acceleration','max_ball_deceleration','max_ball_turn_acceleration'])config[name]=number(name);
   if(config.execution_mode==='continuous'){config.speed_ramp_sec=config.duration_sec;config.duration_sec=0;}
   document.getElementById('experiment-start').disabled=true;this.pause();
   ++this.experiment_generation;clearTimeout(this.experiment_timer);
   try{
    if(pursuit_mode==='link_point'&&!this.supported_pursuit_modes)delete config.pursuit_mode;
    else if(!this.supported_pursuit_modes?.includes(pursuit_mode))throw Error('選択した球の動きは画面サーバー未対応です');
    const result=await request('/api/avoidance-experiments',config);this.owned_job_id=result.job.job_id;sessionStorage.setItem('avoidance-experiment-job',this.owned_job_id);this.begin_live(result.job);await this.refresh_experiment();
   }
   catch(error){document.getElementById('experiment-status').textContent=error.message;document.getElementById('experiment-start').disabled=false;}
  };
  document.getElementById('experiment-stop').onclick=async()=>{try{await request('/api/avoidance-experiments/stop',{job_id:this.owned_job_id});await this.refresh_experiment();}catch(error){document.getElementById('experiment-status').textContent=error.message;}};
  this.refresh_experiment();
  window.addEventListener('pagehide',()=>{this.is_closed=true;++this.experiment_generation;clearTimeout(this.experiment_timer);});
  window.addEventListener('pageshow',event=>{if(event.persisted){this.is_closed=false;this.refresh_experiment();}});
 }

 async refresh_experiment(){
  if(this.is_closed)return;
  clearTimeout(this.experiment_timer);
  const status=document.getElementById('experiment-status'),generation=++this.experiment_generation;
  try{
   const query=new URLSearchParams({after:String(this.live_next_frame_idx)});
   if(this.live_job_id)query.set('job_id',this.live_job_id);
   const result=await this.request_experiment('/api/avoidance-experiments?'+query),job=result.job;
   if(generation!==this.experiment_generation||this.is_closed)return;
   if(result.live_protocol_version!==1)throw Error('実行中表示には画面サーバーの更新が必要です');
   this.supported_pursuit_modes=result.pursuit_modes;
   const policies=result.policies??['rollout_cv','rollout_reactive','cbf_qp','oscbf'];
   const policy=this.experiment_panel.querySelector('form').elements.namedItem('policy');
   for(const option of policy.options)option.disabled=!policies.includes(option.value);
   if(policy.selectedOptions[0]?.disabled)policy.value='rollout_cv';
   const is_running=job&&['running','stopping'].includes(job.state);
   if(is_running&&job.config.execution_mode==='continuous'){
    this.owned_job_id=job.job_id;sessionStorage.setItem('avoidance-experiment-job',this.owned_job_id);
   }
   this.is_experiment_running=!!is_running;this.is_experiment_available=result.is_available;this.update_pursuit_mode();this.update_archive_controls();
   document.getElementById('experiment-stop').disabled=!is_running||job.state==='stopping'||job.job_id!==this.owned_job_id;
   const view=document.getElementById('experiment-view');
   view.hidden=!is_running||this.is_live;
   view.disabled=!!this.is_archive_busy;
   view.onclick=()=>{this.begin_live(job);this.refresh_experiment();};
   // 起動・再読込み時の自動表示なし。表示開始済みセッションだけの再接続。
   if(is_running&&this.is_live&&job.job_id!==this.live_job_id)this.begin_live(job);
   if(job&&job.job_id===this.live_job_id){this.receive_live(job.live);if(!is_running&&this.is_live)this.finish_live(job);}
   status.textContent=!result.is_available?'実行にはROSコンテナ内の画面サーバーが必要です':!job?'実行待ち':is_running?
    job.state==='stopping'?'実験を中止処理中':this.live_next_frame_idx?`実行中 · ${this.recording.frames.at(-1).time_sec.toFixed(2)}${this.is_continuous?' sec · 連続・記録なし':` / ${job.config.duration_sec} sec`}`:`準備 · ${startup_stage_names[job.progress?.stage]||'プロセス起動'} · ${Math.max(0,(Date.now()-job.started_ms)/1000).toFixed(1)} sec`:
    job.state==='completed'?`実験終了 · ${job.failure_reason?failure_names[job.failure_reason]||job.failure_reason:'打切りなし'} · 到達 ${job.metrics.num_arrivals} 回`:job.state==='stopped'?job.config.execution_mode==='continuous'?'実験を中止 · 記録なし':this.recording?'実験を中止 · 取得済み軌跡を表示':'実験を中止 · 軌跡なし':`実行エラー\n${job.output||''}`;
   if(is_running)this.experiment_timer=setTimeout(()=>this.refresh_experiment(),this.live_next_frame_idx?100:250);
  }catch(error){
   if(generation!==this.experiment_generation||this.is_closed)return;
   status.textContent=this.is_live?'表示通信を再接続中 · 計算の停止とは別の状態\n'+error.message:error.message;
   document.getElementById('experiment-start').disabled=true;
   // 新しいジョブ・受信位置の再同期。計算側への停止・再実行要求なし。
   if(this.is_live){this.live_job_id=null;this.live_next_frame_idx=0;this.experiment_timer=setTimeout(()=>this.refresh_experiment(),500);}
  }
 }

 begin_live(job){
  ++this.load_generation;this.pause();this.is_live=true;this.live_job_id=job.job_id;this.live_next_frame_idx=0;
  this.is_archive_busy=false;
  this.live_metadata=null;this.live_duration_sec=job.config.duration_sec;this.recording=null;this.motion_sample=null;this.root.visible=false;
  this.set_task_reference(null);
  this.is_continuous=job.config.execution_mode==='continuous';
  this.live_clock=new live_playback_clock();this.last_status_ms=-Infinity;this.has_pending_status=true;
  this.current_recording_url=null;this.update_archive_controls();
  this.time_sec=0;this.element('time').textContent=this.is_continuous?'0.00 sec · 連続':`0.00 / ${Number(this.live_duration_sec).toFixed(2)} sec`;
  this.max_num_live_frames=this.is_continuous?120:Math.min(20000,Math.ceil(this.live_duration_sec*20)+2);
  for(const name of ['run','toggle','reset','event','seek','speed'])this.element(name).disabled=true;
  this.element('toggle').textContent='ライブ表示';this.element('status').textContent='実験の準備中';
 }

 receive_live(value,now_ms=performance.now()){
  if(!value||!this.is_live)return;
  const has_gap=this.is_continuous&&value.has_gap===true&&value.first_frame_idx>this.live_next_frame_idx;
  if(!Number.isSafeInteger(value.first_frame_idx)||!Number.isSafeInteger(value.num_frames)||(!has_gap&&value.first_frame_idx!==this.live_next_frame_idx)||!Array.isArray(value.frames)||value.num_frames!==value.first_frame_idx+value.frames.length||value.frames.length>this.max_num_live_frames||(!this.is_continuous&&value.num_frames>this.max_num_live_frames))throw Error('実行中フレームの連続性不正');
  if(value.metadata)this.live_metadata=value.metadata;
  if(!this.live_metadata||!value.frames.length)return;
  const previous=this.recording?.frames.at(-1),frames=previous&&!has_gap?[previous,...value.frames]:value.frames;
  validate_recording({...this.live_metadata,frames},this.robot.actuated.map(joint=>joint.name),this.robot.modelId,1,this.is_continuous);
  if(!this.recording){
   this.recording={...this.live_metadata,is_transient:this.is_continuous,frames:[]};this.root.visible=true;
   this.set_task_reference(this.recording.task);
   for(const line of this.trails){
    line.geometry.dispose();line.geometry=new THREE.BufferGeometry();line.geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(this.max_num_live_frames*3),3).setUsage(THREE.DynamicDrawUsage));line.frustumCulled=false;
   }
  }
  if(has_gap)this.recording.frames=[];
  const first_idx=this.recording.frames.length;
  this.recording.frames.push(...value.frames);this.live_next_frame_idx=value.num_frames;
  if(this.is_continuous&&this.recording.frames.length>this.max_num_live_frames){
   const num_removed=this.recording.frames.length-this.max_num_live_frames;
   this.recording.frames.splice(0,num_removed);this.display_frame_idx=Math.max(0,(this.display_frame_idx??0)-num_removed);
  }
  for(const [idx,key] of ['eef','sphere'].entries()){
   const attribute=this.trails[idx].geometry.getAttribute('position');
   const appended=this.is_continuous?this.recording.frames:value.frames,start_idx=this.is_continuous?0:first_idx;
   appended.forEach((frame,offset)=>attribute.setXYZ(start_idx+offset,...frame[key]));
   if(this.is_continuous)attribute.clearUpdateRanges();
   attribute.addUpdateRange(start_idx*3,appended.length*3);attribute.needsUpdate=true;
  }
  this.element('seek').min=this.recording.frames[0].time_sec;this.element('seek').max=this.recording.frames.at(-1).time_sec;
  this.live_clock.observe(this.recording.frames[0].time_sec,this.recording.frames.at(-1).time_sec,now_ms);
  this.has_pending_status=true;
  if(first_idx===0){this.seek(this.live_clock.time_sec,now_ms);this.fit_view();}
 }

 finish_live(job){
  this.is_live=false;this.loaded_job_id=job.job_id;this.pause();
  for(const name of ['run','speed'])this.element(name).disabled=false;
  if(!this.recording){this.element('status').textContent=job.state==='stopped'?'実験を中止 · 軌跡なし':'表示可能な軌跡なし';return;}
  const data={...this.recording,...this.live_metadata};
  if(this.is_continuous){
   this.recording={...data,is_transient:true};this.seek(data.frames.at(-1).time_sec);
   for(const name of ['toggle','reset','event','seek','speed'])this.element(name).disabled=true;
   this.element('toggle').textContent='記録なし';this.element('run').selectedIndex=-1;this.update_archive_controls();
   return;
  }
  if(job.state!=='completed')data.is_partial=true;
  this.set_recording(data,1);this.seek(data.frames.at(-1).time_sec);this.pause();
  if(job.recording_url){
   const option=document.createElement('option');option.value=local_recording_url(job.recording_url);option.textContent=`今回の結果 · 条件${job.config.seed}`;
   this.element('run').prepend(option);this.element('run').value=option.value;
   this.current_recording_url=new URL(option.value,location.href).pathname;
  }else this.element('run').selectedIndex=-1;
  this.update_archive_controls();
 }

 async open(value) {
  if(this.is_live)return;
  const generation=++this.load_generation;this.pause();
  this.is_archive_busy=true;this.update_archive_controls();
  try{
   let recording_url=value==='latest'?'/exports/avoidance-replays.json':value;
   let data=await load_json(recording_url);
   if(generation!==this.load_generation)return;
   if(data.format==='topo-avoidance-catalog/1'){
    this.apply_catalog(data);
    if(!data.selected){this.element('status').textContent='実験記録なし';return;}
    recording_url=data.selected;data=await load_json(recording_url);
   }
   if(generation!==this.load_generation)return;
   this.set_recording(data);this.current_recording_url=new URL(local_recording_url(recording_url)).pathname;this.update_archive_controls();this.toggle();
  }catch(error){if(generation===this.load_generation)this.show_error(error);}
  finally{if(generation===this.load_generation){this.is_archive_busy=false;this.update_archive_controls();}}
 }

 set_recording(value,min_num_frames=value?.is_partial===true?1:2) {
  const data=validate_recording(value,this.robot.actuated.map(joint=>joint.name),this.robot.modelId,min_num_frames);
  this.is_continuous=false;
  this.current_recording_url=null;this.update_archive_controls();
  this.recording=data;this.motion_sample=null;this.root.visible=!!this.is_active;
  this.set_task_reference(data.task);
  for(const [idx,key] of ['eef','sphere'].entries()){
   const geometry=new THREE.BufferGeometry().setFromPoints(data.frames.map(frame=>new THREE.Vector3(...frame[key])));
   this.trails[idx].geometry.dispose();this.trails[idx].geometry=geometry;
   this.trails[idx].frustumCulled=true;
  }
  this.element('seek').min=data.frames[0].time_sec;this.element('seek').max=data.frames.at(-1).time_sec;
  for(const name of ['toggle','reset','seek'])this.element(name).disabled=false;
  this.element('event').disabled=!data.failure_reason;
  this.element('event').textContent=data.frames.some(frame=>frame.has_contact)?'接触位置':'停止位置';
  this.fit_view();
  if(this.is_active)this.seek(data.frames[0].time_sec);
 }

 fit_view() {
  if((!this.is_active&&!this.is_live)||!this.recording||!this.camera||!this.orbit)return;
  const bounds=new THREE.Box3().setFromObject(this.robot);
  let radius=0;
  for(const frame of this.recording.frames){bounds.expandByPoint(new THREE.Vector3(...frame.eef));bounds.expandByPoint(new THREE.Vector3(...frame.sphere));for(const point of frame.obstacle_segments?.flat()||[])bounds.expandByPoint(new THREE.Vector3(...point));radius=Math.max(radius,frame.sphere_radius_m);}
  for(const point of this.recording.task?.reference_path||[])bounds.expandByPoint(new THREE.Vector3(...point));
  bounds.expandByScalar(radius);
  const ball=bounds.getBoundingSphere(new THREE.Sphere()),direction=this.camera.position.clone().sub(this.orbit.target).normalize();
  const vertical=THREE.MathUtils.degToRad(this.camera.fov/2),horizontal=Math.atan(Math.tan(vertical)*this.camera.aspect);
  const dist=1.12*ball.radius/Math.sin(Math.min(vertical,horizontal));
  this.orbit.target.copy(ball.center);this.camera.position.copy(ball.center).addScaledVector(direction,dist);this.orbit.update();
 }

 show_error(error) {
  this.pause();this.element('status').textContent='読込み失敗：'+error.message;
 }

 pause(){this.is_playing=false;this.last_ms=null;this.element('toggle').textContent='再生';}

 toggle() {
  if(this.is_live)return;
  if(this.recording?.is_transient)return;
  if(!this.recording)return;
  if(this.is_playing){this.pause();return;}
  if(this.time_sec>=this.recording.frames.at(-1).time_sec)this.seek(this.recording.frames[0].time_sec);
  this.is_playing=true;this.last_ms=null;this.element('toggle').textContent='一時停止';
 }

 seek(time_sec,now_ms=performance.now()) {
  if(!this.recording||(!this.is_active&&!this.is_live&&!this.has_pose_control))return;
  if(!Number.isFinite(time_sec))throw Error('再生時刻は有限値が必要です');
  const frames=this.recording.frames;
  this.time_sec=Math.max(frames[0].time_sec,Math.min(frames.at(-1).time_sec,time_sec));
  let lower=0,upper=frames.length-1;
  while(lower<upper){const middle=Math.ceil((lower+upper)/2);if(frames[middle].time_sec<=this.time_sec)lower=middle;else upper=middle-1;}
  const first=frames[lower],second=frames[Math.min(lower+1,frames.length-1)];
  this.display_frame_idx=lower;
  const fraction=second.time_sec===first.time_sec?0:(this.time_sec-first.time_sec)/(second.time_sec-first.time_sec);
  const pose=Object.fromEntries(Object.entries(first.joints).map(([name,q])=>[name,q+(second.joints[name]-q)*fraction]));
  // 記録の非ラップ角の補間。IK・関節指令・物理積分なし。
  this.robot.set_received_pose(pose);
  this.motion_sample={model:this.recording.model,pose,motion:interpolate_joint_motion(first,second,fraction,frames[lower-1]),time_ms:this.time_sec*1000,is_simulation_input:true};
  this.sphere.position.fromArray(first.sphere).lerp(new THREE.Vector3(...second.sphere),fraction);
  this.sphere.scale.setScalar(first.sphere_radius_m+(second.sphere_radius_m-first.sphere_radius_m)*fraction);
  this.sphere.visible=!first.obstacle_segments;
  for(const [idx,capsule] of this.capsules.entries()){
   capsule.group.visible=!!first.obstacle_segments;if(!capsule.group.visible)continue;
   const points=first.obstacle_segments[idx].map((point,end)=>new THREE.Vector3(...point).lerp(new THREE.Vector3(...second.obstacle_segments[idx][end]),fraction));
   const delta=points[1].clone().sub(points[0]);
   capsule.cylinder.position.copy(points[0]).add(points[1]).multiplyScalar(.5);
   capsule.cylinder.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),delta.clone().normalize());
   capsule.cylinder.scale.set(this.sphere.scale.x,delta.length(),this.sphere.scale.x);
   capsule.ends.forEach((end,idx)=>{end.position.copy(points[idx]);end.scale.setScalar(this.sphere.scale.x);});
  }
  const is_end=this.time_sec>=frames.at(-1).time_sec;
  this.trails[0].geometry.setDrawRange(0,lower+1);
  // 計算側の予測時間に対応する直近実軌跡。未来の予測線・観測履歴とは別。
  // 時間幅のない旧記録は非表示。固定秒数による推定なし。
  const trail_sec=first.obstacle_trail_sec??0;
  let start_idx=0,end_idx=lower+1;
  while(start_idx<end_idx){const middle=Math.floor((start_idx+end_idx)/2);if(frames[middle].time_sec<this.time_sec-trail_sec)start_idx=middle+1;else end_idx=middle;}
  this.trails[1].geometry.setDrawRange(start_idx,trail_sec>0?lower+1-start_idx:0);
  this.update_status(first,is_end,now_ms);
  this.renderer.shadowMap.needsUpdate=true;this.num_applied_frames++;
 }

 get_motion_sample(now_ms=performance.now()){
  if(!this.recording||(!this.is_live&&!this.has_pose_control)||this.recording.model!==this.robot.modelId)return null;
  const received_ms=this.is_live?this.live_clock.last_receive_ms:now_ms;
  if(received_ms===null||now_ms-received_ms<0||now_ms-received_ms>1000)return null;
  if(this.is_live){
   // 最新の実測標本によるモータ監視。描画の補間・休止からの独立。
   const frames=this.recording.frames,latest=frames.at(-1);
   if(!latest)return null;
   return {model:this.recording.model,pose:latest.joints,motion:interpolate_joint_motion(latest,latest,0,frames.at(-2)),time_ms:latest.time_sec*1000,received_ms,is_simulation_input:true};
  }
  return this.motion_sample?.model===this.robot.modelId?{...this.motion_sample,received_ms}:null;
 }

 update_status(first,is_end,now_ms){
  const frames=this.recording.frames,latest=frames.at(-1);
  const has_latest_contact=this.is_live&&latest.has_contact;
  // 最新の接触警告は補間待ちと独立。画面上の非接触を安全判定に使用不可。
  this.sphere.material.color.setHex(first.has_contact||has_latest_contact||(is_end&&this.recording.failure_reason==='physical_contact')?0xe33d37:0xf28b24);
  if(this.is_live&&!this.has_pending_status&&now_ms-this.last_status_ms<100)return;
  this.has_pending_status=false;this.last_status_ms=now_ms;
  this.element('seek').value=this.time_sec;this.element('time').textContent=this.is_continuous?`${this.time_sec.toFixed(2)} sec · 連続`:`${this.time_sec.toFixed(2)} / ${this.is_live?Number(this.live_duration_sec).toFixed(2):frames.at(-1).time_sec.toFixed(2)} sec`;
  const failure=this.recording.failure_reason;
  const detail=this.recording.failure_details||{},links=(detail.contact_links||[]).filter(name=>!['world','ground','experiment_pedestal'].includes(name)),joints=detail.joints||[];
  const extra=links.length?'\n接触リンク: '+links.join(', '):joints.length?'\n該当関節: '+joints.map(joint=>joint.name).join(', '):'';
  const has_fallback=Object.values(this.recording.joint_position_limits||{}).some(bounds=>bounds.source==='simulation_fallback');
  const limit_note=has_fallback?'\n可動域: 暫定値あり・実機未照合':this.recording.has_unbounded_joint_limits?'\n旧記録: 一部の角度制限が未反映':'';
  const live=this.is_live?this.live_clock.status(now_ms):null;
  const live_note=live?`\n表示: ${live.is_stalled?'受信停滞・保持':live.is_resync?'再同期':this.recording.frames.length<2?'補間待ち':'補間'} · 受信差 ${live.display_gap_ms.toFixed(0)} ms（試験時刻） · 更新 ${live.receive_age_ms.toFixed(0)} ms前`:'';
  const contact=has_latest_contact?'あり（最新受信）':first.has_contact?'あり':'なし';
  const gap=this.is_live?latest.proxy_gap_m:first.proxy_gap_m;
  const phase=this.is_live?latest.phase:first.phase;
  const obstacle_frame=this.is_live?latest:first;
  const obstacle_note=obstacle_frame.obstacle_phase?`\n障害物: ${obstacle_phase_names[obstacle_frame.obstacle_phase]} · ${obstacle_frame.obstacle_speed_mps.toFixed(2)} m/s`:'';
  const rule_note=this.recording.contact_rule==='target_tip'?`\n迂回制約: ${this.recording.enable_proximal_guard?'あり':'なし'}`:'';
  const task_note=first.task_goal?`\n到達: ${first.num_arrivals??0} 回${Number.isFinite(first.task_position_error_m)&&Number.isFinite(first.task_orientation_error_rad)?` · 誤差 ${format_fixed(first.task_position_error_m*1000,1)} mm / ${format_fixed(first.task_orientation_error_rad*180/Math.PI,1)}°`:''}`:'';
  this.element('status').textContent=`${this.recording.pursuit_mode==='random_walk'?'対象範囲':this.recording.pursuit_mode==='distal_sweep'?'通過範囲':'追跡先'}: ${this.recording.target_link||'L_tcp'}${['distal_sweep','random_walk'].includes(this.recording.pursuit_mode)?' 以降':''}\n状態: ${this.is_live?'実行中':is_end?(failure?'実験終了':this.is_continuous?'連続実験終了・記録なし':this.recording.is_partial?'中断記録':'記録終了'):phase_names[phase]||phase}\n制御: ${phase_names[phase]||phase}${task_note}${obstacle_note}${rule_note}\n接触: ${contact}\n${this.is_live?'最新離隔':'形状離隔'}: ${format_fixed(gap*1000,1)} mm\n終了理由: ${failure?`${failure_names[failure]||failure} · ${frames.at(-1).time_sec.toFixed(2)} sec`:'なし'}${extra}${limit_note}${live_note}`;
  this.on_pose?.();
 }

 tick(now) {
  if(!this.is_active&&!this.is_live)return false;
  if(this.is_live&&this.recording){
   const next_sec=this.live_clock.advance(now);
   if(next_sec!==this.time_sec){this.seek(next_sec,now);return true;}
   this.update_status(this.recording.frames[this.display_frame_idx??0],this.time_sec>=this.recording.frames.at(-1).time_sec,now);
   return false;
  }
  if(!this.is_playing||!this.recording)return false;
  if(this.last_ms===null){this.last_ms=now;return false;}
  const period=Math.max(0,(now-this.last_ms)/1000)*Number(this.element('speed').value);this.last_ms=now;
  this.seek(this.time_sec+period);
  if(this.time_sec>=this.recording.frames.at(-1).time_sec)this.pause();
  return true;
 }
}
