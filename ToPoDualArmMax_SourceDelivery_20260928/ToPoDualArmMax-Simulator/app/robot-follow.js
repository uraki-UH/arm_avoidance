const $=id=>document.getElementById(id);

// 共通launchの接続構成と実測状態。構成選択と実機の出力許可・追従開始の分離
export class robot_follow_panel {
 constructor(panel){
  this.panel=panel;this.session_id='';this.config=null;this.is_busy=false;this.is_polling=false;this.enable_heartbeat=false;
  const section=document.createElement('section');section.innerHTML=`<h3>s・r・fの接続構成</h3><p>s: Simulator / r: 実機リーダー / f: 実機フォロワー</p><label>構成<select id="robot-follow-profile" disabled></select></label><div class="row-actions"><button id="robot-follow-apply" disabled>構成を適用</button><button id="robot-follow-start-s" disabled>sの力学を開始</button></div><pre id="robot-follow-paths">共通launch未接続</pre><pre id="robot-follow-status"></pre><details><summary>fの操作</summary><div class="row-actions"><button id="robot-follow-enable" disabled>fの出力を準備</button><button id="robot-follow-follow" disabled>fの追従を開始</button><button id="robot-follow-stop" disabled>fを停止</button><button id="robot-follow-reset" disabled>停止解除・出力OFF</button><button id="robot-follow-torque_off" disabled>fのトルクOFF</button></div></details>`;
  $('ros-receive-panel').before(section);
  $('robot-follow-apply').onclick=()=>this.perform('profile',{profile:$('robot-follow-profile').value});
  $('robot-follow-start-s').onclick=()=>window.simulator?.physics_panel?.start();
  for(const action of ['enable','follow','stop','reset','torque_off'])$('robot-follow-'+action).onclick=()=>this.perform(action);
  $('ros-endpoint').addEventListener('change',()=>this.disconnect());
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.enable_heartbeat=false;});
  this.timer=setInterval(()=>this.refresh(),1000);
  this.heartbeat_timer=setInterval(()=>{if(this.enable_heartbeat&&!this.is_heartbeat_busy){this.is_heartbeat_busy=true;this.request('heartbeat').catch(()=>{this.enable_heartbeat=false;}).finally(()=>{this.is_heartbeat_busy=false;});}},100);
  window.addEventListener('pagehide',()=>{clearInterval(this.timer);clearInterval(this.heartbeat_timer);this.disconnect();});
 }
 endpoint(){return new URL('/api/follow',$('ros-endpoint').value);}
 async request(action,extra={}){
  const response=await fetch(this.endpoint(),{method:'POST',headers:{'Content-Type':'application/json','X-ToPo-Follow':'1'},body:JSON.stringify({action,session_id:this.session_id,...extra}),signal:AbortSignal.timeout(4000)});
  const result=await response.json();if(!response.ok)throw Error(result.error||'構成操作に失敗しました');return result;
 }
 async perform(action,extra={}){
  if(this.is_busy)return;this.is_busy=true;
  try{
   if(['profile','stop','reset','torque_off'].includes(action))this.enable_heartbeat=false;
   if(action==='enable'){this.enable_heartbeat=true;await this.request('heartbeat');}
   await this.request(action,extra);$('robot-follow-status').textContent='操作を受付';
   await this.refresh();
  }catch(error){if(action==='enable')this.enable_heartbeat=false;$('robot-follow-status').textContent=error.message;}
  finally{this.is_busy=false;}
 }
 disconnect(){
  this.enable_heartbeat=false;
  if(this.session_id){window.simulator?.physics_panel?.hold_leader('追従管理の接続失効');this.panel.joint_stream.stop();}
  this.session_id='';this.config=null;this.panel.joint_stream.enable_duplex=false;this.panel.joint_stream.follow_config=null;
  for(const id of ['robot-pose-source','ros-joints-topic','ros-joints-receive','ros-state-all'])$(id).disabled=false;
  for(const id of ['profile','apply','start-s','enable','follow','stop','reset','torque_off'])$('robot-follow-'+id).disabled=true;
  $('robot-follow-paths').textContent='共通launch未接続';
 }
 async refresh(){
  if(this.is_polling)return;this.is_polling=true;
  try{
   const response=await fetch(this.endpoint(),{signal:AbortSignal.timeout(2500)});if(!response.ok)throw Error('共通launch未接続');
   const value=await response.json();if(!value.has_manager){this.disconnect();return;}
   const config=value.config,profile=config.profiles[config.profile];
   if(config.robot_model!==this.panel.rgbd.robot.modelId){this.disconnect();$('robot-follow-paths').textContent=`モデル不一致: 共通launchは ${config.robot_model}`;return;}
   if(config.session_id!==this.session_id)this.apply_config(config);
   const status=value.status,hardware=status.hardware??{},fresh=status.has_fresh_input??{};
   $('robot-follow-status').textContent=`入力: r=${fresh.r?'受信':'未受信/失効'} f=${fresh.f?'受信':'未受信/失効'} s=${fresh.s?'受信':'未受信/失効'}\nf: ${status.has_fresh_hardware_status?hardware.mode:'未受信'} / ${hardware.detail??''}`;
   $('robot-follow-start-s').disabled=profile.simulator_mode!=='dynamics';
   const can_output=config.allow_hardware_output&&profile.follower_source!=='none'&&status.has_fresh_hardware_status;
   $('robot-follow-enable').disabled=!can_output||hardware.mode!=='off'||hardware.is_stop_latched||!hardware.has_fresh_state;
   $('robot-follow-follow').disabled=!can_output||hardware.mode!=='hold'||!hardware.is_stationary||!hardware.has_fresh_target;
   $('robot-follow-reset').disabled=!status.has_fresh_hardware_status||!['stopped','torque_off'].includes(hardware.mode)||!hardware.is_stationary;
   $('robot-follow-torque_off').disabled=!config.allow_hardware_output;
  }catch{this.disconnect();}
  finally{this.is_polling=false;}
 }
 apply_config(config){
  this.enable_heartbeat=false;this.panel.stop();window.simulator?.physics_panel?.stop('接続構成の変更。力学の再開始が必要');
  this.session_id=config.session_id;this.config=config;
  const profile=config.profiles[config.profile],select=$('robot-follow-profile');
  select.replaceChildren(...Object.entries(config.profiles).map(([name,entry])=>{const option=document.createElement('option');option.value=name;option.textContent=entry.label;return option;}));select.value=config.profile;
  for(const id of ['profile','apply','stop','reset','torque_off'])$('robot-follow-'+id).disabled=false;
  const stream=this.panel.joint_stream;stream.follow_config=config;stream.enable_duplex=true;
  const source=profile.simulator_mode==='manual'?'simulator':profile.simulator_mode==='display'?'ros':'leader';
  $('robot-pose-source').value=source;$('ros-joints-topic').value=config.simulator_target_topic;
  $('ros-joints-receive').checked=source!=='simulator';$('ros-state-all').checked=true;
  for(const id of ['robot-pose-source','ros-joints-topic','ros-joints-receive','ros-state-all'])$(id).disabled=true;
  if(profile.simulator_mode==='dynamics'){const physics=window.simulator?.physics_panel;if(physics){physics.enable_robot_physics=true;physics.max_leader_age_ms=config.max_state_age_sec*1000;}}
  stream.connect();
  const paths=[];
  if(profile.simulator_source!=='none')paths.push(`${profile.simulator_source} → s（${profile.simulator_mode==='display'?'描画':'力学'}）`);else paths.push('s: 手動操作');
  if(profile.follower_source!=='none')paths.push(`${profile.follower_source} → f（${profile.follow_mode==='relative'?'開始時からの角度差':'絶対角'}）`);else paths.push('f: 出力経路なし');
  $('robot-follow-paths').textContent=paths.join('\n')+`\n実機出力許可: ${config.allow_hardware_output?'ON（操作待ち）':'OFF'}`;
 }
}
