import {pose_sources,pose_source_labels} from './robot-ros-state.js';
const $=id=>document.getElementById(id);
const role_labels=Object.freeze({none:'指定なし',s:'シミュレータ',r:'実機リーダー',f:'実機フォロワー',s_and_f:'シミュレータ＋実機フォロワー'});
const mode_labels=Object.freeze({manual:pose_source_labels[pose_sources.simulator],display:pose_source_labels[pose_sources.ros],dynamics:pose_source_labels[pose_sources.leader]});

function connection_roles(profile){
 const has_simulator=profile.simulator_source!=='none',has_follower=profile.follower_source!=='none';
 return {leader:has_simulator?profile.simulator_source:profile.follower_source,
  follower:has_simulator?(has_follower?'s_and_f':'s'):(has_follower?'f':'none')};
}

function connection_choice(profile){
 return {...connection_roles(profile),mode:[profile.simulator_mode,profile.follow_mode,profile.follower_source].join(':')};
}

function offline_choices(){
 // 未接続時の接続先と追従方式。ROSのprofile名・トピック定義への非依存
 const followers={none:['none'],r:['s','f','s_and_f'],f:['s'],s:['f']};
 return Object.entries(followers).flatMap(([leader,targets])=>targets.flatMap(follower=>{
  const modes=['s','s_and_f'].includes(follower)?['display','dynamics']:['manual'];
  return modes.map(mode=>{
   const source=follower==='f'?leader:follower==='s_and_f'?(mode==='dynamics'?'s':leader):'none';
   return {leader,follower,mode:[mode,mode==='display'&&follower==='s'?'absolute':'relative',source].join(':')};
  });
 }));
}

function set_options(select,entries,preferred){
 select.replaceChildren(...entries.map(([value,label])=>{const option=document.createElement('option');option.value=value;option.textContent=label;return option;}));
 if(entries.some(([value])=>value===preferred))select.value=preferred;
}

// 共通launchの接続構成と実測状態。構成選択と実機の出力許可・追従開始の分離
export class robot_follow_panel {
 constructor(panel){
  this.panel=panel;this.session_id='';this.config=null;this.is_busy=false;this.is_polling=false;this.enable_heartbeat=false;this.has_pending_selection=false;
  const section=document.createElement('section');section.id='robot-follow-panel';section.innerHTML=`<h3>ロボットの追従</h3><div class="robot-follow-roles"><label>リーダー<select id="robot-follow-leader"></select></label><label>フォロワー<select id="robot-follow-follower"></select></label></div><label id="robot-follow-mode-label" hidden>シミュレータの追従<select id="robot-follow-mode" disabled></select></label><div class="row-actions"><button id="robot-follow-apply" disabled>接続を適用</button><button id="robot-follow-start-s" disabled>シミュレータの物理を開始</button></div><pre id="robot-follow-paths">共通launch未接続</pre><pre id="robot-follow-status"></pre><details><summary>実機フォロワーの操作</summary><div class="row-actions"><button id="robot-follow-enable" disabled>出力を準備</button><button id="robot-follow-follow" disabled>追従を開始</button><button id="robot-follow-stop" disabled>停止</button><button id="robot-follow-reset" disabled>停止解除・出力OFF</button><button id="robot-follow-torque_off" disabled>トルクOFF</button></div></details>`;
  $('ros-receive-panel').before(section);
  $('robot-follow-mode-label').after($('ros-joints-panel'));
  for(const id of ['leader','follower','mode']){
   $('robot-follow-'+id).disabled=false;
   $('robot-follow-'+id).onchange=()=>{this.has_pending_selection=true;this.update_choices();};
  }
  this.update_choices({leader:'none',follower:'none',mode:'manual:relative:none'});
  $('robot-follow-apply').onclick=()=>{
   const profile=this.selected_profile();
   if(profile)this.perform('profile',{profile});
   else $('robot-follow-status').textContent='選択した構成は、この接続先では未登録です';
  };
  $('robot-follow-start-s').onclick=()=>window.simulator?.physics_panel?.start();
  for(const action of ['enable','follow','stop','reset','torque_off'])$('robot-follow-'+action).onclick=()=>this.perform(action);
  $('ros-endpoint').addEventListener('change',()=>this.disconnect());
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.enable_heartbeat=false;});
  this.timer=setInterval(()=>this.refresh(),1000);
  this.heartbeat_timer=setInterval(()=>{if(this.enable_heartbeat&&!this.is_heartbeat_busy){this.is_heartbeat_busy=true;this.request('heartbeat').catch(()=>{this.enable_heartbeat=false;}).finally(()=>{this.is_heartbeat_busy=false;});}},100);
  window.addEventListener('pagehide',()=>{clearInterval(this.timer);clearInterval(this.heartbeat_timer);this.disconnect();});
 }
 selection(){
  return {leader:$('robot-follow-leader').value,follower:$('robot-follow-follower').value,mode:$('robot-follow-mode').value};
 }
 selected_profile(){
  const choice=this.selection();
  return Object.entries(this.config?.profiles??{}).find(([,profile])=>{
   const current=connection_choice(profile);
   return current.leader===choice.leader&&current.follower===choice.follower&&current.mode===choice.mode;
  })?.[0];
 }
 update_choices(preferred=this.selection(),allow_unregistered_selection=false){
  // 選択中の構成と現在の接続状態の分離。接続先が非対応でも編集中の選択を保持。
  const choices=this.config?Object.values(this.config.profiles).map(connection_choice):offline_choices();
  if(allow_unregistered_selection&&!choices.some(choice=>choice.leader===preferred.leader&&choice.follower===preferred.follower&&choice.mode===preferred.mode))choices.push(preferred);
  const leader=$('robot-follow-leader'),follower=$('robot-follow-follower'),mode=$('robot-follow-mode');
  const leaders=[...new Set(choices.map(choice=>choice.leader))];
  set_options(leader,leaders.map(value=>[value,role_labels[value]]),preferred.leader);
  const by_leader=choices.filter(choice=>choice.leader===leader.value);
  const followers=[...new Set(by_leader.map(choice=>choice.follower))];
  set_options(follower,followers.map(value=>[value,role_labels[value]]),preferred.follower);
  const matches=[...new Set(by_leader.filter(choice=>choice.follower===follower.value).map(choice=>choice.mode))];
  const modes=matches.map(value=>{
   const [simulator_mode,follow_mode,follower_source]=value.split(':');
   const has_duplicate_mode=matches.some(other=>other!==value&&other.split(':')[0]===simulator_mode);
   const route=follower.value==='s_and_f'?`・${role_labels[follower_source]}から実機へ`:'';
   const label=mode_labels[simulator_mode]+(has_duplicate_mode?`（${follow_mode==='relative'?'相対角':'絶対角'}${route}）`:'');
   return [value,label];
  });
  set_options(mode,modes,preferred.mode);
  // 単独のROS入力と共通接続構成の排他的な編集。同じ追従方法の二重表示なし。
  const has_profile_selection=!!this.config||leader.value!=='none'||follower.value!=='none';
  $('robot-pose-source-label').hidden=has_profile_selection||!!window.simulator?.robot_control;
  $('robot-follow-mode-label').hidden=!has_profile_selection||matches.length<=1;
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
   if(action==='profile'&&this.selected_profile()===extra.profile)this.has_pending_selection=false;
   await this.refresh();
  }catch(error){if(action==='enable')this.enable_heartbeat=false;$('robot-follow-status').textContent=error.message;}
  finally{this.is_busy=false;}
 }
 disconnect(){
  this.enable_heartbeat=false;
  const choice=this.selection(),has_connection=!!this.session_id;
  if(this.session_id){window.simulator?.physics_panel?.hold_leader('追従管理の接続失効');this.panel.joint_stream.stop();}
  this.session_id='';this.config=null;this.panel.joint_stream.enable_duplex=false;this.panel.joint_stream.follow_config=null;
  for(const id of ['robot-pose-source','ros-joints-topic','ros-target-topic','ros-joints-receive','ros-state-all'])$(id).disabled=false;
  this.panel.joint_stream.update_input_controls();
  window.simulator?.robot_control?.render();
  if(has_connection){this.has_pending_selection=true;this.update_choices(choice,true);}
  for(const id of ['apply','start-s','enable','follow','stop','reset','torque_off'])$('robot-follow-'+id).disabled=true;
  $('robot-follow-paths').textContent='未接続 · 接続後に適用できます';
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
   const inputs=[...new Set([profile.simulator_source,profile.follower_source].filter(role=>role!=='none'))];
   $('robot-follow-status').textContent=inputs.map(role=>`${role_labels[role]}: ${fresh[role]?'受信中':'未受信/失効'}`).join('\n')
    +(profile.follower_source!=='none'?`\n実機フォロワー: ${status.has_fresh_hardware_status?hardware.mode:'未受信'} / ${hardware.detail??''}`:'');
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
  const pending=this.has_pending_selection?this.selection():null;
  this.session_id=config.session_id;this.config=config;
  const profile=config.profiles[config.profile];
  window.simulator?.robot_control?.follow_config(config);
  this.update_choices(pending??connection_choice(profile),!!pending);
  for(const id of ['leader','follower','mode','apply','stop','reset','torque_off'])$('robot-follow-'+id).disabled=false;
  const stream=this.panel.joint_stream;stream.follow_config=config;stream.enable_duplex=true;
  const source=profile.simulator_mode==='manual'?'simulator':profile.simulator_mode==='display'?'ros':'leader';
  $('robot-pose-source').value=source;
  if(source!==pose_sources.simulator)stream.set_input_topic(source,config.simulator_target_topic);
  $('ros-joints-receive').checked=source!=='simulator';$('ros-state-all').checked=true;
  for(const id of ['robot-pose-source','ros-joints-topic','ros-target-topic','ros-joints-receive','ros-state-all'])$(id).disabled=true;
  if(profile.simulator_mode==='dynamics'){const physics=window.simulator?.physics_panel;if(physics){physics.enable_robot_physics=true;physics.max_leader_age_ms=config.max_state_age_sec*1000;}}
  stream.connect();
  const paths=[];
  if(profile.simulator_source!=='none')paths.push(`${role_labels[profile.simulator_source]} → シミュレータ（${mode_labels[profile.simulator_mode]}）`);else paths.push('シミュレータ: 手動操作');
  if(profile.follower_source!=='none')paths.push(`${role_labels[profile.follower_source]} → 実機フォロワー（${profile.follow_mode==='relative'?'開始時からの角度差':'絶対角'}）`);
  $('robot-follow-paths').textContent=paths.join('\n')+`\n実機出力許可: ${config.allow_hardware_output?'ON（操作待ち）':'OFF'}`;
 }
}
