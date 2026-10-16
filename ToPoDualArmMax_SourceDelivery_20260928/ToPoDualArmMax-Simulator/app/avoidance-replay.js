import * as THREE from 'three';

const policy_names = {avoidance_first:'回避優先',vrm_predictive:'予測・不一致適応',rollout_reactive:'現在観測',rollout_cv:'等速予測',rollout_sequence:'時系列記憶',cbf_qp:'関節CBF-QP',oscbf:'OSCBF'};
const failure_names = {physical_contact:'接触',internal_interference:'内部干渉',abnormal_joint_speed:'異常速度',joint_limit:'試験可動域の超過（旧記録）',planning_domain_exit:'旧計画域の超過',physical_joint_limit:'関節可動域の超過',invalid_obstacle_trajectory:'障害物の禁止域侵入・評価対象外',qp_infeasible:'QP実行不能・試験停止',qp_no_solution:'QP解未成立・試験停止'};
const phase_names = {task:'往復',avoiding:'回避',holding:'保持',waiting:'待機',returning:'復帰',monitoring:'待機',waiting_for_clearance:'離隔待ち',qp_no_solution:'QP不成立・減速保持／再計算'};
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
  const is_point=point=>Array.isArray(point)&&point.length===3&&point.every(v=>Number.isFinite(v)&&Math.abs(v)<100);
  if(Object.hasOwn(frame,'target')!==Object.hasOwn(value.frames[0],'target')||(Object.hasOwn(frame,'target')&&!is_point(frame.target)))throw Error('追跡点の形式が不正です');
  if(Object.hasOwn(frame,'obstacle_segments')!==Object.hasOwn(value.frames[0],'obstacle_segments'))throw Error('障害物形状の不一致');
  if(Object.hasOwn(frame,'obstacle_segments')&&(!Array.isArray(frame.obstacle_segments)||frame.obstacle_segments.length!==2||!frame.obstacle_segments.every(segment=>Array.isArray(segment)&&segment.length===2&&segment.every(is_point))))throw Error('リンク障害物の形式が不正です');
  if(!Number.isFinite(frame.sphere_radius_m)||frame.sphere_radius_m<=0||frame.sphere_radius_m>2||typeof frame.has_contact!=='boolean'||typeof frame.phase!=='string'||!Number.isFinite(frame.proxy_gap_m))throw Error('記録状態の形式が不正です');
  last_sec=frame.time_sec;
 }
 return value;
}

export class avoidance_replay {
 constructor({robot,scene,renderer,camera,controls,on_pose}) {
  this.robot=robot;this.scene=scene;this.renderer=renderer;this.camera=camera;this.orbit=controls;this.on_pose=on_pose;
  this.recording=null;this.is_playing=false;this.last_ms=null;this.time_sec=0;this.load_generation=0;this.num_applied_frames=0;
  this.is_live=false;this.is_closed=false;this.live_job_id=null;this.live_next_frame_idx=0;this.experiment_generation=0;
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
  this.target_marker=new THREE.Mesh(new THREE.SphereGeometry(.018,16,12),new THREE.MeshBasicMaterial({color:0x26ba77,wireframe:true}));this.root.add(this.target_marker);
  this.trails=['eef','sphere'].map((name,idx)=>{
   const line=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:idx?0x1984e8:0xe0ac12}));line.name=name+'-trajectory';this.root.add(line);return line;
  });
  this.make_controls();this.make_experiment_controls();this.make_read_only();
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.pause();});
  window.addEventListener('keydown',event=>{
   if(/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)||event.ctrlKey||event.metaKey||event.altKey)return;
   if(event.code==='Space'){event.preventDefault();event.stopImmediatePropagation();this.toggle();}
   if(event.key==='Escape'){event.stopImmediatePropagation();this.pause();}
  },true);
 }

 make_read_only() {
  this.robot.pose_source='ros';
  document.body.classList.add('avoidance-replay-view');
  document.querySelector('footer').hidden=true;
  document.querySelector('.view-top h1').textContent='回避実験';
  document.querySelector('.view-top p').textContent='橙：追跡障害物　緑：追跡点　黄：手先軌跡　青：障害物軌跡　赤：接触';
  document.querySelector('.hint').textContent='Space：再生／一時停止　Esc：一時停止';
  document.title='回避実験 · '+document.title;
  for(const button of document.querySelectorAll('[data-panel]')){
   const is_replay_tab=button.dataset.panel==='physics';button.disabled=!is_replay_tab;button.classList.toggle('active',is_replay_tab);
   button.setAttribute('aria-pressed',String(is_replay_tab));
   if(is_replay_tab){button.textContent='実験';button.onclick=()=>{};}
  }
  for(const name of ['robot','environment','sensors','physics','ai','ros'])document.getElementById(name+'-panel').hidden=name!=='physics';
  for(const element of document.querySelectorAll('[data-model],#help-button'))element.disabled=true;
 }

 make_controls() {
  const section=document.createElement('section');section.className='avoidance-replay';
  section.innerHTML=`<style>
   body.avoidance-replay-view main{height:calc(100dvh - 66px)}body.avoidance-replay-view footer{display:none}
   .avoidance-replay{font-size:14px;line-height:1.6}.avoidance-replay h2{font-size:20px}.avoidance-replay select{font-size:13px;width:100%;min-width:0;margin:10px 0}
   .avoidance-replay button{font-size:14px;background:#f4f5f7;color:#222;border-color:#bbc0c7}.avoidance-replay .replay-actions{display:flex;gap:6px;margin:12px 0;flex-wrap:wrap}
   .avoidance-replay label{white-space:nowrap;gap:12px}.avoidance-replay label select{width:auto;flex:1}
   .avoidance-replay #replay-toggle{background:#258c49;color:white}.avoidance-replay #replay-file-button{background:#246eb7;color:white}
   .avoidance-replay #replay-status{white-space:pre-wrap;font:13px/1.7 system-ui}.avoidance-replay #replay-time{display:block;text-align:right;font-variant-numeric:tabular-nums}
   .avoidance-replay .replay-note{color:var(--muted);font-size:12px}.avoidance-replay a{color:var(--cyan)}
   </style><h2>回避実験</h2><div class="replay-note">実行中表示・記録再生／実機出力なし</div>
   <select id="replay-run" aria-label="実験記録"></select>
   <div class="replay-actions"><button id="replay-delete" disabled>記録を削除</button></div>
   <div class="replay-actions"><button id="replay-toggle" disabled>再生</button><button id="replay-reset" disabled>先頭</button><button id="replay-event" disabled>停止位置</button></div>
   <input id="replay-seek" type="range" min="0" max="1" step="0.001" value="0" aria-label="再生時刻" disabled><output id="replay-time">0.00 sec</output>
   <label>速度<select id="replay-speed" aria-label="再生速度"><option value="0.25">0.25倍</option><option value="0.5">0.5倍</option><option value="1" selected>1倍</option><option value="2">2倍</option><option value="4">4倍</option></select></label>
   <pre id="replay-status" role="status">記録を読込中</pre><div class="replay-actions"><button id="replay-file-button">記録JSONを開く</button><input id="replay-file" type="file" accept=".json" hidden></div><a href="/?model=long">通常のシミュレータへ</a>`;
  document.getElementById('physics-panel').append(section);this.controls=section;
  this.element('toggle').onclick=()=>this.toggle();
  this.element('reset').onclick=()=>{this.pause();this.seek(this.recording.frames[0].time_sec);};
  this.element('event').onclick=()=>{this.pause();this.seek(this.recording.frames.at(-1).time_sec);};
  this.element('seek').oninput=()=>{this.pause();this.seek(Number(this.element('seek').value));};
  this.element('run').onchange=()=>this.open(this.element('run').value);
  this.element('delete').onclick=()=>this.delete_recording();
  this.element('file-button').onclick=()=>this.element('file').click();
  this.element('file').onchange=async event=>{
   const file=event.target.files[0];event.target.value='';if(!file)return;
   const generation=++this.load_generation;this.pause();
   try{if(file.size>max_file_size)throw Error('記録の容量超過');const data=JSON.parse(await file.text());if(generation!==this.load_generation)return;this.set_recording(data);this.element('run').selectedIndex=-1;this.toggle();}catch(error){if(generation===this.load_generation)this.show_error(error);}
  };
 }

 element(name){return document.getElementById('replay-'+name);}

 async delete_recording(){
  if(this.is_live||this.is_experiment_running||this.is_archive_busy)return;
  const recording_url=this.current_recording_url;
  if(!recording_url)return;
  if(!window.confirm('選択した記録JSONと元の実験ログ・軌跡を完全削除しますか？\n取り消し不可。'))return;
  this.pause();this.is_archive_busy=true;this.update_archive_controls();
  try{
   const result=await this.request_experiment('/api/avoidance-experiments/delete',{recording_url});
   this.current_recording_url=null;
   this.apply_catalog(result.catalog);
   if(result.catalog.selected){await this.open(result.catalog.selected);this.pause();}
   else{
    this.recording=null;this.root.visible=false;
    for(const name of ['toggle','reset','event','seek'])this.element(name).disabled=true;
    this.element('time').textContent='0.00 sec';this.element('status').textContent='実験記録なし';
   }
  }catch(error){this.element('status').textContent='記録操作の失敗：'+error.message;}
  finally{this.is_archive_busy=false;this.update_archive_controls();}
 }

 update_archive_controls(){
  const can_edit=!this.is_live&&!this.is_experiment_running&&!this.is_archive_busy;
  this.element('delete').disabled=!can_edit||!this.current_recording_url;
  if(this.experiment_panel)this.experiment_panel.querySelector('#experiment-start').disabled=this.is_archive_busy||this.is_experiment_running||!this.is_experiment_available;
 }

 apply_catalog(data){
  if(data.format!=='topo-avoidance-catalog/1'||!Array.isArray(data.runs)||data.runs.length>256)throw Error('実験一覧が不正です');
  this.element('run').replaceChildren(...data.runs.map(run=>{
   const option=document.createElement('option');option.value=local_recording_url(run.url);
   option.textContent=`${run.has_unbounded_joint_limits?'[旧・角度制限未反映] ':run.has_legacy_planning_domain?'[旧試験] ':''}${policy_names[run.policy]||run.policy}${run.candidate_metric==='occupancy'?'・占有候補':''} · ${run.obstacle_shape==='linked_arm'?'腕型':'球'} → ${run.target_link||'L_tcp'} · 条件${run.seed} · ${Number(run.duration_sec).toFixed(2)} sec`;return option;
  }));
  this.element('run').disabled=!data.runs.length;
  if(data.selected)this.element('run').value=local_recording_url(data.selected);
 }

 make_experiment_controls(){
  const panel=document.createElement('details');panel.open=true;
  panel.innerHTML=`<summary>実験設定</summary><style>
   #experiment-form .experiment-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;margin:12px 0;width:100%;min-width:0}
   #experiment-form label{display:flex;flex-direction:column;align-items:stretch;min-width:0;gap:4px;white-space:normal;font-size:13px}
   #experiment-form input,#experiment-form select{box-sizing:border-box;width:100%;min-width:0;max-width:100%;flex:none;font-size:14px;margin:0;padding:7px}
   #experiment-form .experiment-offset{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:4px}
   .avoidance-replay details summary{justify-content:flex-start;font-size:16px}
   #experiment-start{background:#258c49;color:white}#experiment-stop{background:#a6403d;color:white}#experiment-status{white-space:pre-wrap;font-size:13px;margin:8px 0}
   </style><form id="experiment-form"><div class="experiment-grid">
   <label>実行方式<select name="execution_mode"><option value="continuous" selected>連続・記録なし</option><option value="recorded">時間指定・記録あり</option></select></label>
   <label>回避方式<select name="policy"><option value="rollout_cv">GNG候補・等速予測</option><option value="rollout_reactive">GNG候補・現在観測</option><option value="cbf_qp">関節CBF-QP</option><option value="oscbf">OSCBF</option></select></label>
   <label>障害物<select name="obstacle_shape"><option value="sphere">球</option><option value="linked_arm">二節腕・球関節×2</option></select></label>
   <label style="grid-column:1/-1">追跡リンク<select name="target_link"></select></label>
   <label style="grid-column:1/-1">リンク内の追跡点 [m]<span class="experiment-offset"><input name="offset_x" aria-label="追跡点X" type="number" min="-.5" max=".5" step=".01" value="0" required><input name="offset_y" aria-label="追跡点Y" type="number" min="-.5" max=".5" step=".01" value="0" required><input name="offset_z" aria-label="追跡点Z" type="number" min="-.5" max=".5" step=".01" value="0" required></span></label>
   ${[['duration_sec','速度上昇時間 [sec]',18,1,60,1],['seed','条件seed',401,0,2147483647,1],['radius_m','半径 [m]',.06,.015,.15,.005],['speed_start','開始速度比',.4,.05,3,.05],['speed_end','最終速度比',1.3,.05,3,.05],['max_ball_acceleration','加速度 [m/s²]',3,.1,30,.1],['max_ball_deceleration','減速力 [m/s²]',8,.1,30,.1],['max_ball_turn_acceleration','旋回加速度 [m/s²]',10,.1,30,.1]].map(([name,label,value,min,max,step])=>`<label><span${name==='duration_sec'?' id="experiment-time-label"':''}>${label}</span><input type="number" name="${name}" value="${value}" min="${min}" max="${max}" step="${step}" required></label>`).join('')}
   </div><div class="replay-note" id="experiment-mode-note">終了：中止／接触・異常。タブを閉じても継続。記録なし。</div><div class="replay-note">速度比：無障害往復の手先最高速度基準。実機出力なし。</div>
   <div class="replay-actions"><button id="experiment-start" type="submit">実験を実行</button><button id="experiment-stop" type="button" disabled>中止</button></div></form><div id="experiment-status" role="status">実行環境を確認中</div>`;
  this.controls.prepend(panel);this.experiment_panel=panel;
  const form=panel.querySelector('form'),links=form.elements.namedItem('target_link');
  form.elements.namedItem('execution_mode').onchange=()=>{
   const is_continuous=form.elements.namedItem('execution_mode').value==='continuous';
   panel.querySelector('#experiment-time-label').textContent=is_continuous?'速度上昇時間 [sec]':'時間 [sec]';
   panel.querySelector('#experiment-mode-note').textContent=is_continuous?'終了：中止／接触・異常。タブを閉じても継続。記録なし。':'時間指定：終了後の記録再生。';
  };
  for(const name of Object.keys(this.robot.links)){const option=document.createElement('option');option.value=name;option.textContent=name;links.append(option);}links.value='L_tcp';
  form.elements.namedItem('obstacle_shape').onchange=()=>{form.elements.namedItem('radius_m').value=form.elements.namedItem('obstacle_shape').value==='linked_arm'?.035:.06;};
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
   const config={execution_mode:fields.get('execution_mode'),policy:fields.get('policy'),obstacle_shape:fields.get('obstacle_shape'),target_link:fields.get('target_link'),target_offset_m:['x','y','z'].map(axis=>number('offset_'+axis)),speed_ratios:[number('speed_start'),number('speed_end')]};
   for(const name of ['duration_sec','seed','radius_m','max_ball_acceleration','max_ball_deceleration','max_ball_turn_acceleration'])config[name]=number(name);
   if(config.execution_mode==='continuous'){config.speed_ramp_sec=config.duration_sec;config.duration_sec=0;}
   document.getElementById('experiment-start').disabled=true;this.pause();
   ++this.experiment_generation;clearTimeout(this.experiment_timer);
   try{const result=await request('/api/avoidance-experiments',config);this.owned_job_id=result.job.job_id;sessionStorage.setItem('avoidance-experiment-job',this.owned_job_id);this.begin_live(result.job);await this.refresh_experiment();}
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
   const is_running=job&&['running','stopping'].includes(job.state);
   if(is_running&&job.config.execution_mode==='continuous'){
    this.owned_job_id=job.job_id;sessionStorage.setItem('avoidance-experiment-job',this.owned_job_id);
   }
   this.is_experiment_running=!!is_running;this.is_experiment_available=result.is_available;this.update_archive_controls();
   document.getElementById('experiment-stop').disabled=!is_running||job.state==='stopping'||job.job_id!==this.owned_job_id;
   if(is_running&&job.job_id!==this.live_job_id)this.begin_live(job);
   if(job&&job.job_id===this.live_job_id){this.receive_live(job.live);if(!is_running&&this.is_live)this.finish_live(job);}
   status.textContent=!result.is_available?'実行にはROSコンテナ内の画面サーバーが必要です':!job?'実行待ち':is_running?
    job.state==='stopping'?'実験を中止処理中':this.live_next_frame_idx?`実行中 · ${this.recording.frames.at(-1).time_sec.toFixed(2)}${this.is_continuous?' sec · 連続・記録なし':` / ${job.config.duration_sec} sec`}`:'準備中 · モデル・駆動の初期化':
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
  this.live_metadata=null;this.live_duration_sec=job.config.duration_sec;this.recording=null;this.root.visible=false;
  this.is_continuous=job.config.execution_mode==='continuous';
  this.live_clock=new live_playback_clock();this.last_status_ms=-Infinity;this.has_pending_status=true;
  this.current_recording_url=null;this.update_archive_controls();
  this.time_sec=0;this.element('time').textContent=this.is_continuous?'0.00 sec · 連続':`0.00 / ${Number(this.live_duration_sec).toFixed(2)} sec`;
  this.max_num_live_frames=this.is_continuous?120:Math.min(20000,Math.ceil(this.live_duration_sec*20)+2);
  for(const name of ['run','toggle','reset','event','seek','speed','file-button'])this.element(name).disabled=true;
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
  for(const name of ['run','speed','file-button'])this.element(name).disabled=false;
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
 }

 set_recording(value,min_num_frames=value?.is_partial===true?1:2) {
  const data=validate_recording(value,this.robot.actuated.map(joint=>joint.name),this.robot.modelId,min_num_frames);
  this.is_continuous=false;
  this.current_recording_url=null;this.update_archive_controls();
  this.recording=data;this.root.visible=true;
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
  this.seek(data.frames[0].time_sec);
 }

 fit_view() {
  if(!this.recording||!this.camera||!this.orbit)return;
  const bounds=new THREE.Box3().setFromObject(this.robot);
  let radius=0;
  for(const frame of this.recording.frames){bounds.expandByPoint(new THREE.Vector3(...frame.eef));bounds.expandByPoint(new THREE.Vector3(...frame.sphere));for(const point of frame.obstacle_segments?.flat()||[])bounds.expandByPoint(new THREE.Vector3(...point));radius=Math.max(radius,frame.sphere_radius_m);}
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
  if(!this.recording)return;
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
  this.target_marker.visible=!!first.target;
  if(first.target)this.target_marker.position.fromArray(first.target).lerp(new THREE.Vector3(...second.target),fraction);
  const is_end=this.time_sec>=frames.at(-1).time_sec;
  for(const line of this.trails)line.geometry.setDrawRange(0,lower+1);
  this.update_status(first,is_end,now_ms);
  this.renderer.shadowMap.needsUpdate=true;this.num_applied_frames++;
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
  const detail=this.recording.failure_details||{},links=detail.contact_links||[],joints=detail.joints||[];
  const extra=links.length?'\n接触リンク: '+links.join(', '):joints.length?'\n該当関節: '+joints.map(joint=>joint.name).join(', '):'';
  const has_fallback=Object.values(this.recording.joint_position_limits||{}).some(bounds=>bounds.source==='simulation_fallback');
  const limit_note=has_fallback?'\n可動域: 暫定値あり・実機未照合':this.recording.has_unbounded_joint_limits?'\n旧記録: 一部の角度制限が未反映':'';
  const live=this.is_live?this.live_clock.status(now_ms):null;
  const live_note=live?`\n表示: ${live.is_stalled?'受信停滞・保持':live.is_resync?'再同期':this.recording.frames.length<2?'補間待ち':'補間'} · 受信差 ${live.display_gap_ms.toFixed(0)} ms（試験時刻） · 更新 ${live.receive_age_ms.toFixed(0)} ms前`:'';
  const contact=has_latest_contact?'あり（最新受信）':first.has_contact?'あり':'なし';
  const gap=this.is_live?latest.proxy_gap_m:first.proxy_gap_m;
  const phase=this.is_live?latest.phase:first.phase;
  this.element('status').textContent=`追跡先: ${this.recording.target_link||'L_tcp'}\n状態: ${this.is_live?'実行中':is_end?(failure?'試験停止':this.is_continuous?'連続実験終了・記録なし':this.recording.is_partial?'中断記録':'記録終了'):phase_names[phase]||phase}\n制御: ${phase_names[phase]||phase}\n接触: ${contact}\n${this.is_live?'最新離隔':'形状離隔'}: ${(gap*1000).toFixed(1)} mm\n停止理由: ${failure?`${failure_names[failure]||failure} · ${frames.at(-1).time_sec.toFixed(2)} sec`:'なし'}${extra}${limit_note}${live_note}`;
  this.on_pose?.();
 }

 tick(now) {
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
