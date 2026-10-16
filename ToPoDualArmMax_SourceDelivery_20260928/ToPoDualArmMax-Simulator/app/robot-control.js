import {pose_sources} from './robot-ros-state.js';

const $=id=>document.getElementById(id);
export const robot_usages=Object.freeze({simulation:'MuJoCoシミュレーション',observation:'ROS実測の表示',preview:'姿勢プレビュー（力学なし）'});
export const command_sources=Object.freeze({browser:'ブラウザ操作',ros:'ROS関節指令',leader:'ROSリーダー姿勢',avoidance:'回避モジュール',api:'CLI / API'});
export const control_modes=Object.freeze({position:'位置制御',position_velocity:'位置・速度追従',velocity:'速度制御',torque:'トルク制御'});
const command_fields=Object.freeze({position:['position'],position_velocity:['position','velocity','effort'],velocity:['velocity'],torque:['effort']});
const options=values=>Object.entries(values).map(([value,label])=>`<option value="${value}">${label}</option>`).join('');

// 用途・指令元・制御方式の単一管理。GUIと外部APIで共用する操作入口。
export class robot_control_panel {
 #config;
 get config(){return this.#config;}
 constructor(simulator){
  this.simulator=simulator;this.panel=simulator.ros_points.robot_panel;this.physics=simulator.physics_panel;
  this.#config=Object.freeze({usage:'simulation',command_source:'browser',control_mode:'position',enable_gravity_compensation:true,command_topic:'/sim/joint_command'});
  this.session_id=crypto.randomUUID();this.results=[];this.is_closed=false;
  const section=this.section=document.createElement('section');section.id='robot-control-panel';
  section.innerHTML=`<h3>ロボットの用途と制御</h3><label class="robot-control-choice">用途<select id="robot-usage">${options(robot_usages)}</select></label><div id="robot-simulation-controls"><label class="robot-control-choice">指令元<select id="robot-command-source">${options(command_sources)}</select></label><label class="robot-control-choice">制御方式<select id="robot-control-mode">${options(control_modes)}</select></label><label id="robot-command-topic-field">ROS指令トピック（JointTrajectory・単一点）<input id="robot-command-topic" value="/sim/joint_command"></label><label><input id="robot-gravity" type="checkbox" checked>重力・コリオリ補償</label><div id="robot-control-avoidance"></div><details id="robot-direct-command"><summary>関節指令</summary><label>関節<select id="robot-command-joint"></select></label><label id="robot-command-position-field">目標位置 rad<input id="robot-command-position" type="number" step="0.01" value="0"></label><label id="robot-command-velocity-field">目標速度 rad/s<input id="robot-command-velocity" type="number" step="0.01" value="0"></label><label id="robot-command-effort-field">トルク N m<input id="robot-command-effort" type="number" step="0.1" value="0"></label><button id="robot-command-apply" class="wide-button">指令を適用</button></details><div class="row-actions"><button id="robot-control-start">シミュレーション開始</button><button id="robot-control-stop">停止</button></div></div><pre id="robot-control-status">停止中</pre>`;
  const follow=$('robot-follow-panel');$('ros-send-panel').before(section);section.append($('ros-joints-panel'));
  const managed=document.createElement('details');managed.innerHTML='<summary>リーダー・フォロワー接続</summary>';section.after(managed);managed.append(follow);follow.querySelector('h3').remove();
  $('robot-control-avoidance').append($('physics-avoidance').parentElement,$('physics-plugin-settings'));
  $('physics-robot-settings').querySelector('summary').textContent='関節固定・自己接触';
  for(const id of ['robot-usage','robot-command-source','robot-control-mode','robot-gravity','robot-command-topic'])$(id).onchange=()=>this.run_ui(()=>{
   const mode=$('robot-control-mode').value;
   return this.configure({usage:$('robot-usage').value,command_source:$('robot-command-source').value,control_mode:mode,
    enable_gravity_compensation:id==='robot-control-mode'?mode!=='torque':$('robot-gravity').checked,command_topic:$('robot-command-topic').value});
  });
  $('robot-control-start').onclick=()=>this.run_ui(()=>this.start());$('robot-control-stop').onclick=()=>this.stop();
  $('robot-command-joint').onchange=()=>this.sync_joint_inputs();
  $('robot-command-apply').onclick=()=>this.run_ui(()=>{
   const joint=$('robot-command-joint').value,command={mode:this.config.control_mode};
   for(const field of command_fields[command.mode])command[field]={[joint]:$( 'robot-command-'+field).valueAsNumber};
   return this.command(command,'browser');
  });
  $('physics-avoidance').addEventListener('change',()=>this.run_ui(()=>this.configure({command_source:$('physics-avoidance').value==='none'?'browser':'avoidance',control_mode:'position'})));
  window.addEventListener('pagehide',()=>{this.is_closed=true;clearTimeout(this.timer);},{once:true});
  this.render();this.exchange();
 }
 async run_ui(action){try{await action();$('robot-control-status').textContent='設定済み';}catch(error){$('robot-control-status').textContent=error.message;this.render();}}
 configure(update){
  if(this.panel.follow_panel.config)throw Error('共通接続構成の管理中です。接続構成側で変更してください');
  if(this.simulator.avoidance_replay?.has_pose_control)throw Error('回避実験の表示を終了してから変更してください');
  if(!update||typeof update!=='object'||Array.isArray(update)||Object.keys(update).some(key=>!Object.hasOwn(this.config,key)))throw Error('未対応のロボット設定です');
  const next={...this.config,...update};
  if(update.control_mode!==undefined&&!Object.hasOwn(update,'enable_gravity_compensation'))next.enable_gravity_compensation=next.control_mode!=='torque';
  if(!Object.hasOwn(robot_usages,next.usage)||!Object.hasOwn(command_sources,next.command_source)||!Object.hasOwn(control_modes,next.control_mode)||typeof next.enable_gravity_compensation!=='boolean'||typeof next.command_topic!=='string'||!/^\/[A-Za-z_][A-Za-z0-9_/]*$/.test(next.command_topic)||next.command_topic.includes('//')||next.command_topic.endsWith('/'))throw Error('用途・指令元・制御方式の設定が不正です');
  if(['leader','avoidance'].includes(next.command_source)&&next.control_mode!=='position')throw Error('現在のリーダー・回避モジュールは位置制御専用です');
  this.physics.stop('制御設定を変更');this.panel.joint_stream.stop();this.simulator.cancel_robot_motion();
  this.#config=Object.freeze(next);
  const source=next.usage==='observation'?pose_sources.ros:next.usage==='preview'?pose_sources.simulator:next.command_source==='leader'?pose_sources.leader:next.command_source==='browser'&&['position','position_velocity'].includes(next.control_mode)||next.command_source==='avoidance'?pose_sources.simulator:pose_sources.command;
  this.panel.instance_panel.set_source(source);
  if(next.usage==='observation'||next.usage==='simulation'&&next.command_source==='leader'){
   $('ros-joints-receive').checked=true;this.panel.joint_stream.connect();
  }
  this.render();return this.state();
 }
 observe_source(source){
  if(source===pose_sources.ros)this.#config=Object.freeze({...this.config,usage:'observation'});
  else if(source===pose_sources.leader)this.#config=Object.freeze({...this.config,usage:'simulation',command_source:'leader',control_mode:'position'});
  this.render();
 }
 follow_config(config){
  const profile=config.profiles[config.profile];
  this.#config=Object.freeze({...this.config,usage:profile.simulator_mode==='display'?'observation':'simulation',command_source:profile.simulator_mode==='dynamics'?'leader':'browser',control_mode:'position',enable_gravity_compensation:true});this.render();
 }
 get can_edit_pose(){return this.config.usage==='preview'||this.config.usage==='simulation'&&['browser','avoidance'].includes(this.config.command_source)&&['position','position_velocity'].includes(this.config.control_mode);}
 physics_config(){
  if(this.config.usage!=='simulation')throw Error('実測表示中はロボットの物理計算を開始できません');
  if(this.config.command_source==='avoidance'&&$('physics-avoidance').value==='none')throw Error('回避モジュールを選択してください');
  return {control_mode:this.config.control_mode,command_source:this.config.command_source,enable_gravity_compensation:this.config.enable_gravity_compensation,command_topic:this.config.command_topic};
 }
 async start(){
  this.physics.enable_robot_physics=true;this.physics.start();
  if(!this.physics.socket)throw Error($('physics-status').textContent);
  await this.physics.prepare_motion();return this.state();
 }
 stop(){this.physics.stop();return this.state();}
 async command(value,source='api'){
  if(this.config.usage!=='simulation'||this.config.command_source!==source)throw Error('指令元が一致しません');
  if(!this.physics.socket||!this.physics.enable_dynamics)throw Error('シミュレーションを開始してください');
  await this.physics.send_command(value,source);return this.state();
 }
 goal(position){
  if(this.config.usage!=='simulation'||this.config.command_source!=='avoidance')throw Error('回避モジュールの実行中に名目目標を指定してください');
  if(!this.physics.plugins.can_set_goal())throw Error('この回避モジュールは名目目標の変更に未対応です');
  if(!position||typeof position!=='object'||Array.isArray(position))throw Error('名目目標は関節名と角度の組が必要です');
  if(!this.physics.set_goal_pose({...this.physics.targets,...position}))throw Error('名目目標の変更に対応する回避モジュールを開始してください');
  return {...this.state(),goal:{...this.physics.targets}};
 }
 sync_joint_inputs(){
  const name=$('robot-command-joint').value,joint=this.simulator.robot.joints[name];
  $('robot-command-position').value=this.simulator.robot.getPose()[name]??0;$('robot-command-velocity').value=0;$('robot-command-effort').value=0;
  for(const [field,label,unit,limit] of [['position','目標位置',joint?.type==='prismatic'?'m':'rad',null],['velocity','目標速度',joint?.type==='prismatic'?'m/s':'rad/s',joint?.velocity],['effort','トルク・力',joint?.type==='prismatic'?'N':'N m',joint?.effort]]){
   $('robot-command-'+field+'-field').firstChild.textContent=label+' '+unit;
   const input=$('robot-command-'+field);input.removeAttribute('min');input.removeAttribute('max');
   if(field==='position'){if(Number.isFinite(joint?.lower))input.min=joint.lower;if(Number.isFinite(joint?.upper))input.max=joint.upper;}
   else if(Number.isFinite(limit)){input.min=-limit;input.max=limit;}
  }
 }
 render(){
  if(!this.config)return;
  if(this.bound_robot&&this.bound_robot!==this.simulator.robot)this.#config=Object.freeze({...this.config,usage:'simulation',command_source:'browser',control_mode:'position',enable_gravity_compensation:true});
  this.bound_robot=this.simulator.robot;
  const c=this.config,has_manager=!!this.panel.follow_panel.config;
  $('robot-usage').value=c.usage;$('robot-command-source').value=c.command_source;$('robot-control-mode').value=c.control_mode;$('robot-gravity').checked=c.enable_gravity_compensation;$('robot-command-topic').value=c.command_topic;
  for(const id of ['robot-usage','robot-command-source','robot-control-mode','robot-gravity','robot-command-topic'])$(id).disabled=has_manager;
  $('robot-simulation-controls').hidden=c.usage!=='simulation';$('robot-command-topic-field').hidden=c.command_source!=='ros';$('robot-control-avoidance').hidden=c.command_source!=='avoidance';
  $('robot-direct-command').hidden=c.command_source!=='browser';
  for(const field of ['position','velocity','effort'])$('robot-command-'+field+'-field').hidden=!command_fields[c.control_mode].includes(field);
  $('robot-pose-source-label').hidden=true;
  this.panel.joint_stream.update_input_controls();
  const names=this.simulator.robot.actuated.filter(joint=>!document.querySelector(`#physics-locks input[value="${joint.name}"]:checked`)).map(joint=>joint.name),select=$('robot-command-joint');
  if([...select.options].map(option=>option.value).join()!==names.join()){select.replaceChildren(...names.map(name=>{const option=document.createElement('option');option.value=option.textContent=name;return option;}));this.sync_joint_inputs();}
 }
 state(){
  const physics_sample=this.physics.get_motion_sample(),sample=physics_sample??this.panel.joint_stream.get_motion_sample(),robot=this.simulator.robot;
  return {model:robot.modelId,config:{...this.config},state_source:physics_sample||this.physics.socket&&this.physics.enable_dynamics?'mujoco':this.config.usage==='observation'?'ros':'preview',is_running:!!this.physics.socket,has_physics_state:!!physics_sample,has_state:!!sample,is_command_stopped:!!this.physics.is_command_stopped,time_sec:sample?sample.time_ms/1000:null,
   joints:sample?.pose??robot.getPose(),motion:sample?.motion??null,status:$('physics-status').textContent};
 }
 async execute(action){
  if(action?.model!==undefined&&action.model!==this.simulator.robot.modelId)throw Error('操作対象のロボットモデルが変わっています');
  if(!action||!['configure','start','stop','command','goal'].includes(action.type))throw Error('未対応のロボット操作です');
  if(action.type==='configure')return this.configure(action.config);
  if(action.type==='start')return this.start();
  if(action.type==='stop')return this.stop();
  if(action.type==='goal')return this.goal(action.position);
  return this.command(action.command,'api');
 }
 async exchange(){
  if(this.is_closed)return;
  try{
   const response=await fetch('/api/robot',{method:'POST',headers:{'Content-Type':'application/json','X-ToPo-Robot':'1'},body:JSON.stringify({session_id:this.session_id,state:this.state(),results:this.results}),signal:AbortSignal.timeout(5000)});
   if(!response.ok)throw Error('ロボットAPI未接続');
   const result=await response.json();this.results=[];
   for(const action of result.actions){
    if(Date.now()>action.expires_ms){this.results.push({id:action.id,error:'操作の有効期限切れ'});continue;}
    try{this.results.push({id:action.id,result:await this.execute(action.value)});}catch(error){this.results.push({id:action.id,error:error.message});}
   }
  }catch{}
  finally{if(!this.is_closed)this.timer=setTimeout(()=>this.exchange(),250);}
 }
}
