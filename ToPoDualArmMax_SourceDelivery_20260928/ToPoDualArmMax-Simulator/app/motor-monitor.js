import {bridge_socket_url} from './robot-ros-state.js';

const metrics={position_rad:{label:'角度',unit:'deg',scale:180/Math.PI},velocity_rad_sec:{label:'角速度',unit:'deg/s',scale:180/Math.PI},torque_nm:{label:'トルク',unit:'N·m',scale:1},current_a:{label:'電流',unit:'A',scale:1},jerk_rad_sec3:{label:'ジャーク',unit:'deg/s³',scale:180/Math.PI}};
const metric_names=Object.keys(metrics),input_units=metric_names.filter(name=>name!=='jerk_rad_sec3'),max_sample_age_ms=1000;
const finite_or_null=value=>typeof value==='number'&&Number.isFinite(value)?value:null;

// 表示専用の出力軸換算。電気回路・電圧制限の計算や駆動設定の変更なし
export function motor_values(torque,current,coefficient){
 torque=finite_or_null(torque);current=finite_or_null(current);
 const has_coefficient=typeof coefficient==='number'&&Number.isFinite(coefficient)&&coefficient!==0;
 return {torque_nm:torque??(has_coefficient&&current!==null?finite_or_null(current*coefficient):null),
  current_a:current??(has_coefficient&&torque!==null?finite_or_null(torque/coefficient):null),
  is_torque_estimated:torque===null&&has_coefficient&&current!==null,
  is_current_estimated:current===null&&has_coefficient&&torque!==null};
}

export function validate_motor_profiles(config,robot){
 if(!config||config.robot_model!==robot.modelId||!config.joints||typeof config.joints!=='object'||Array.isArray(config.joints))throw Error('設定のモデル・関節辞書が不正です');
 const result={},aliases=new Map(robot.actuated.map(joint=>[joint.name,joint.name]));
 for(const [name,profile] of Object.entries(config.joints)){
  if(!robot.actuated.some(joint=>joint.name===name)||!profile||typeof profile!=='object')throw Error('設定の関節名が不正です');
  const ros_name=profile.ros_name??name,coefficient=profile.output_torque_nm_per_a??null;
  if(typeof ros_name!=='string'||!ros_name||ros_name.length>128||(aliases.has(ros_name)&&aliases.get(ros_name)!==name))throw Error('実機名・IDが不正または重複です');
  if(coefficient!==null&&(typeof coefficient!=='number'||!Number.isFinite(coefficient)||coefficient===0))throw Error('換算係数は有限の非ゼロ値が必要です');
  if(profile.has_verified_calibration!==undefined&&typeof profile.has_verified_calibration!=='boolean')throw Error('校正状態は真偽値が必要です');
  aliases.set(ros_name,name);result[name]={ros_name,output_torque_nm_per_a:coefficient,has_verified_calibration:profile.has_verified_calibration===true};
 }
 return result;
}

// 不等間隔の角速度差分によるジャーク推定。重複時刻・欠測・受信間隔断絶の除外
export class jerk_estimator{
 constructor(){this.clear();}
 clear(){this.previous=null;this.acceleration=null;this.value=null;}
 update(velocity,time_ms){
  velocity=finite_or_null(velocity);
  if(velocity===null||!Number.isFinite(time_ms)){this.clear();return null;}
  if(this.previous?.time_ms===time_ms&&this.previous.velocity===velocity)return this.value;
  const previous=this.previous;this.previous={velocity,time_ms};this.value=null;
  if(!previous||time_ms-previous.time_ms<5||time_ms-previous.time_ms>300){this.acceleration=null;return null;}
  const acceleration={value:(velocity-previous.velocity)/((time_ms-previous.time_ms)/1000),time_ms:(time_ms+previous.time_ms)/2};
  if(this.acceleration)this.value=finite_or_null((acceleration.value-this.acceleration.value)/((acceleration.time_ms-this.acceleration.time_ms)/1000));
  this.acceleration=acceleration;return this.value;
 }
}

// 表示関節ごとの固定長履歴。5指標×2系列、欠測はNaN、ゼロ補完なし
export class motor_history{
 constructor(max_points=600){this.max_points=max_points;this.num_columns=1+metric_names.length*2;this.data=new Float64Array(max_points*this.num_columns);this.flags=new Uint16Array(max_points);this.num_points=0;this.idx=0;}
 clear(){this.num_points=0;this.idx=0;}
 push(time_ms,sim,hardware){
  this.data[this.idx*this.num_columns]=time_ms;let flags=0;
  for(const [source_idx,values] of [sim,hardware].entries())for(const [metric_idx,metric] of metric_names.entries()){
   const idx=source_idx*metric_names.length+metric_idx;
   this.data[this.idx*this.num_columns+1+idx]=finite_or_null(values[metric])??NaN;
   if(is_estimated(values,metric))flags|=1<<idx;
  }
  this.flags[this.idx]=flags;
  this.idx=(this.idx+1)%this.max_points;this.num_points=Math.min(this.num_points+1,this.max_points);
 }
 each(callback){for(let num=0;num<this.num_points;num++){const idx=(this.idx-this.num_points+num+this.max_points)%this.max_points;callback(this.data,idx*this.num_columns,this.flags[idx]);}}
}

const is_estimated=(values,metric)=>metric==='jerk_rad_sec3'?finite_or_null(values[metric])!==null:!!values[metric==='torque_nm'?'is_torque_estimated':metric==='current_a'?'is_current_estimated':'is_motion_estimated'];
const set_text=(element,value)=>{if(element.textContent!==value)element.textContent=value;};

export class motor_monitor{
 constructor(simulator){
  this.simulator=simulator;this.model=null;this.socket=null;this.samples={};this.profiles={};this.rows=new Map();this.metric='position_rad';this.visibility={};
  this.section=document.createElement('section');this.section.className='motor-monitor';this.section.id='motor-monitor';
  this.section.innerHTML=`<div class="motor-heading"><h3>モータ</h3><span data-field="metric_note" class="motor-hint" hidden tabindex="0"></span><div class="motor-legend"><span class="sim">シミュレーション</span><span class="hardware">実機</span></div></div><div class="motor-tabs" role="tablist" aria-label="表示指標"></div><details class="motor-visibility"><summary>関節表示</summary><button data-field="show_all">すべて表示</button><div class="motor-joint-checks"></div></details><div class="motor-joint-list" role="tabpanel" id="motor-joint-list"></div><p data-field="empty" class="motor-note" hidden>「関節表示」から選択</p><details><summary>設定</summary><label><input data-field="enable_hardware" type="checkbox">実機値を受信</label><label>角度・角速度用JointState<input data-field="state_topic" type="text" placeholder="未設定：DynamixelPresentの姿勢を使用"></label><label>トルク用JointState［N·m］<input data-field="torque_topic" type="text" placeholder="未設定"></label><label>電流形式<select data-field="current_type"><option value="dynamixel_present">DynamixelPresent</option><option value="joint_state">JointState.effort［A］</option></select></label><label>電流トピック<input data-field="current_topic" type="text" value="/dynamixel/state/present"></label><label>換算設定対象<select data-field="joint"></select></label><label>実機名／ID<input data-field="ros_name" type="text"></label><label>出力軸換算［N·m/A］<input data-field="coefficient" type="number" step="0.01" placeholder="未設定"></label><p class="motor-note">係数未設定は換算なし。ID対応と係数は関節別。</p><button data-field="load" class="motor-load">設定JSON読込</button><input data-field="file" type="file" accept="application/json,.json" hidden><button data-field="clear">履歴クリア</button><p data-field="status" class="motor-note">実機監視OFF</p><p class="motor-note">角度以外は表示専用。グラフ：60秒・最大10 Hz、破線：推定値。</p></details>`;
  document.getElementById('joint-controls').append(this.section);
  this.elements=Object.fromEntries([...this.section.querySelectorAll('[data-field]')].map(element=>[element.dataset.field,element]));
  this.tabs=new Map();
  for(const [metric,description] of Object.entries(metrics)){
   const button=document.createElement('button');button.textContent=description.label;button.dataset.metric=metric;button.id='motor-tab-'+metric;button.setAttribute('role','tab');button.setAttribute('aria-controls','motor-joint-list');button.onclick=()=>this.select_metric(metric);this.tabs.set(metric,button);this.section.querySelector('.motor-tabs').append(button);
   button.onkeydown=event=>{let idx=metric_names.indexOf(metric);if(event.key==='ArrowRight')idx=(idx+1)%metric_names.length;else if(event.key==='ArrowLeft')idx=(idx+metric_names.length-1)%metric_names.length;else if(event.key==='Home')idx=0;else if(event.key==='End')idx=metric_names.length-1;else return;event.preventDefault();this.select_metric(metric_names[idx]);this.tabs.get(metric_names[idx]).focus();};
  }
  this.elements.joint.onchange=()=>this.select_joint();
  this.elements.clear.onclick=()=>{for(const row of this.rows.values())row.history?.clear();this.tick();};
  this.elements.show_all.onclick=()=>{for(const row of this.rows.values())this.set_visible(row,true);};
  this.elements.enable_hardware.onchange=()=>this.elements.enable_hardware.checked?this.connect():this.disconnect();
  for(const field of ['state_topic','torque_topic','current_topic','current_type'])this.elements[field].onchange=()=>this.disconnect('入力設定変更。受信ONで再接続');
  this.elements.ros_name.onchange=()=>this.update_profile();this.elements.coefficient.onchange=()=>this.update_profile();
  this.elements.load.onclick=()=>this.elements.file.click();
  this.elements.file.onchange=async()=>{
   const file=this.elements.file.files[0];this.elements.file.value='';if(!file)return;
   try{if(file.size>100000)throw Error('設定ファイルは100 kB以内です');const profiles=validate_motor_profiles(JSON.parse(await file.text()),this.simulator.robot);this.disconnect();this.profiles=profiles;this.clear_history();this.select_joint();this.elements.status.textContent='設定読込済み。実機受信はOFF';}
   catch(error){this.elements.status.textContent='設定エラー：'+error.message;}
  };
  document.getElementById('ros-endpoint').addEventListener('change',()=>this.disconnect('接続先変更。受信ONで再接続'));
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.disconnect('ページ非表示のため受信OFF');});
  const release_angle_inputs=()=>{for(const row of this.rows.values())row.is_angle_dragging=false;};
  for(const event of ['pointerup','pointercancel','blur'])window.addEventListener(event,release_angle_inputs);
  this.timer=setInterval(()=>this.tick(),100);window.addEventListener('pagehide',()=>{this.disconnect();clearInterval(this.timer);},{once:true});this.tick();
 }
 profile(name=this.elements.joint.value){return this.profiles[name]??{ros_name:name,output_torque_nm_per_a:null,has_verified_calibration:false};}
 select_joint(){const profile=this.profile();this.elements.ros_name.value=profile.ros_name;this.elements.coefficient.value=profile.output_torque_nm_per_a??'';}
 clear_history(){for(const row of this.rows.values())row.history?.clear();}
 update_profile(){
  try{
   const name=this.elements.joint.value,raw=this.elements.coefficient.value;
   const profiles={...this.profiles,[name]:{ros_name:this.elements.ros_name.value.trim(),output_torque_nm_per_a:raw===''?null:Number(raw),has_verified_calibration:false}};
   this.profiles=validate_motor_profiles({robot_model:this.model,joints:profiles},this.simulator.robot);
   this.disconnect('換算・ID設定変更。受信ONで再接続');this.clear_history();this.select_joint();
  }catch(error){this.elements.status.textContent='設定エラー：'+error.message;this.select_joint();}
 }
 disconnect(message='実機監視OFF'){
  const socket=this.socket;this.socket=null;if(socket)socket.close();this.samples={};this.elements.enable_hardware.checked=false;this.elements.status.textContent=message;
  for(const row of this.rows.values())row.jerk.hardware.clear();
  this.simulator.ros_points?.robot_panel?.set_connection_state('motors',false);
 }
 connect(){
  this.disconnect();
  try{
   const inputs=[],state_topic=this.elements.state_topic.value.trim(),torque_topic=this.elements.torque_topic.value.trim(),current_topic=this.elements.current_topic.value.trim();
   if(state_topic)inputs.push({unit:'motion',topic:state_topic,message_type:'joint_state'});
   if(torque_topic)inputs.push({unit:'torque_nm',topic:torque_topic,message_type:'joint_state'});
   if(current_topic)inputs.push({unit:'current_a',topic:current_topic,message_type:this.elements.current_type.value});
   if(!inputs.length||inputs.some(input=>!input.topic.startsWith('/'))||(torque_topic&&torque_topic===current_topic))throw Error('絶対トピック名が必要です。トルクと電流は別トピックです');
   const name_map=Object.fromEntries(this.simulator.robot.actuated.map(joint=>[joint.name,joint.name]));
   for(const joint of this.simulator.robot.actuated)name_map[this.profile(joint.name).ros_name]=joint.name;
   const socket=this.socket=new WebSocket(bridge_socket_url(document.getElementById('ros-endpoint').value,'/motors'));
   this.elements.enable_hardware.checked=true;this.elements.status.textContent='実機監視へ接続中';
   socket.onopen=()=>{if(this.socket===socket)socket.send(JSON.stringify({type:'config',model:this.model,inputs,name_map}));};
   socket.onmessage=event=>{
    if(this.socket!==socket)return;
    try{const data=JSON.parse(event.data);
     if(data.type==='ready'){this.elements.status.textContent='実機監視接続済み';this.simulator.ros_points?.robot_panel?.set_connection_state('motors',true);}
     else if(data.type==='error')throw Error(data.error);
     else if(data.type==='motor_state')this.receive(data.samples);
    }catch(error){this.disconnect('実機監視エラー：'+error.message);}
   };
   socket.onerror=()=>{if(this.socket===socket)this.disconnect('監視接続失敗。ブリッジの更新・再起動を確認');};
   socket.onclose=()=>{if(this.socket===socket)this.disconnect('実機監視切断');};
  }catch(error){this.disconnect('実機監視エラー：'+error.message);}
 }
 receive(samples,now_ms=performance.now()){
  if(!Array.isArray(samples)||samples.length>4)throw Error('モータ受信形式が不正です');
  const next={...this.samples};
  const seen=new Set();
  for(const sample of samples){
   if(!sample||!input_units.includes(sample.unit)||seen.has(sample.unit)||!sample.values||typeof sample.values!=='object'||Array.isArray(sample.values))throw Error('受信単位・値が不正です');
   seen.add(sample.unit);
   if(sample.stamp_sec!==null&&(!Number.isFinite(sample.stamp_sec)||Date.now()/1000-sample.stamp_sec<0||Date.now()/1000-sample.stamp_sec>1))continue;
   for(const [name,value] of Object.entries(sample.values))if(!this.simulator.robot.actuated.some(joint=>joint.name===name)||finite_or_null(value)===null)throw Error('受信関節名・値が不正です');
   next[sample.unit]={values:sample.values,received_ms:now_ms,time_ms:sample.stamp_sec===null?now_ms:sample.stamp_sec*1000,has_source_stamp:sample.stamp_sec!==null};
  }
  this.samples=next;
 }
 tick(now_ms=performance.now()){
  const robot=this.simulator.robot;
  if(this.model!==robot.modelId){this.disconnect('モデル切替。実機監視OFF');this.model=robot.modelId;this.profiles={};this.last_motion=null;this.sim_received_ms=-Infinity;this.build_joints(robot);this.select_joint();}
  const physics=this.simulator.physics_panel,motion=physics.motion_state;
  if(physics.socket&&physics.enable_dynamics&&physics.robot_model===robot&&motion&&motion!==this.last_motion){this.last_motion=motion;this.sim_received_ms=now_ms;}
  const has_sim=!!(physics.socket&&physics.enable_dynamics&&physics.robot_model===robot&&motion&&now_ms-this.sim_received_ms<=max_sample_age_ms);
  const sample=unit=>{const value=this.samples[unit];return value&&now_ms-value.received_ms<=max_sample_age_ms?value:null;};
  const inputs=Object.fromEntries(input_units.map(unit=>[unit,sample(unit)])),can_render=!document.hidden&&!document.getElementById('robot-panel').hidden;
  for(const row of this.rows.values()){
   if(!row.is_visible)continue;
   const name=row.joint.name,coefficient=this.profile(name).output_torque_nm_per_a;
   const sim={...motor_values(has_sim?motion.effort?.[name]:null,has_sim?motion.current_a?.[name]:null,coefficient),position_rad:finite_or_null(has_sim?physics.actual_joints?.[name]??physics.actual?.[name]:physics.socket&&physics.enable_dynamics?null:row.joint.q),velocity_rad_sec:finite_or_null(has_sim?motion.velocity?.[name]:null)};
   const hardware={...motor_values(inputs.torque_nm?.values[name],inputs.current_a?.values[name],coefficient),position_rad:finite_or_null(inputs.position_rad?.values[name]),velocity_rad_sec:finite_or_null(inputs.velocity_rad_sec?.values[name])};
   sim.jerk_rad_sec3=row.jerk.sim.update(sim.velocity_rad_sec,has_sim?this.sim_received_ms:NaN);
   hardware.jerk_rad_sec3=row.jerk.hardware.update(hardware.velocity_rad_sec,inputs.velocity_rad_sec?.time_ms);
   if(now_ms-(row.last_history_ms??-Infinity)>=100){row.history.push(now_ms,sim,hardware);row.last_history_ms=now_ms;}row.values={sim,hardware};
   for(const metric of metric_names)row.display_limits[metric]=Math.max(row.display_limits[metric],Math.abs((sim[metric]??0)*metrics[metric].scale),Math.abs((hardware[metric]??0)*metrics[metric].scale));
   if(can_render)this.render_row(row,has_sim,inputs);
  }
  if(can_render)for(const row of this.rows.values())if(row.is_visible&&row.views[this.metric]==='graph')this.draw_plot(row,now_ms);
 }
 build_joints(robot){
  this.rows.clear();this.section.querySelector('.motor-joint-list').replaceChildren();this.section.querySelector('.motor-joint-checks').replaceChildren();
  this.elements.joint.replaceChildren(...robot.actuated.map(joint=>{const option=document.createElement('option');option.value=joint.name;option.textContent=joint.name;return option;}));
  if(robot.joints.L_joint1)this.elements.joint.value='L_joint1';
  for(const joint of robot.actuated){
   const card=document.createElement('article');card.className='motor-joint';card.dataset.joint=joint.name;
   card.innerHTML=`<div class="motor-joint-heading"><strong></strong><div class="motor-view-tabs" role="tablist" aria-label="関節の表示形式"><button role="tab" data-view="slider">スライダー</button><button role="tab" data-view="graph">グラフ</button></div><div class="motor-values"><output class="sim"></output><output class="hardware" hidden></output></div></div><div class="motor-gauges"><div class="sim"><input type="range" min="-1" max="1" step="any" value="0" disabled></div><div class="hardware" hidden><input type="range" min="-1" max="1" step="any" value="0" disabled></div></div>`;
   card.querySelector('strong').textContent=joint.name;card.querySelector('.motor-view-tabs').setAttribute('aria-label',joint.name+'の表示形式');
   const checkbox=document.createElement('input');checkbox.type='checkbox';const label=document.createElement('label');label.append(checkbox,document.createTextNode(joint.name));this.section.querySelector('.motor-joint-checks').append(label);
   const max_effort=Number(robot.urdf.querySelector(`joint[name="${joint.name}"] > limit`)?.getAttribute('effort'));
   const row={joint,card,checkbox,is_visible:true,is_angle_dragging:false,views:{},jerk:{sim:new jerk_estimator(),hardware:new jerk_estimator()},display_limits:{position_rad:Math.max(180,Number.isFinite(joint.lower)?Math.abs(joint.lower*180/Math.PI):0,Number.isFinite(joint.upper)?Math.abs(joint.upper*180/Math.PI):0),velocity_rad_sec:Math.max(1,Math.abs(joint.velocity*180/Math.PI)),torque_nm:Math.max(1,max_effort||0),current_a:1,jerk_rad_sec3:1},fields:{}};
   for(const source of ['sim','hardware']){const gauge_box=card.querySelector('.motor-gauges .'+source);row.fields[source]={output:card.querySelector('output.'+source),gauge_box,gauge:gauge_box.querySelector('input')};}
   const angle_input=row.fields.sim.gauge;
   angle_input.oninput=()=>{if(this.can_edit_angle(row))this.simulator.set_manual_joint(joint.name,angle_input.valueAsNumber*Math.PI/180);};
   angle_input.onpointerdown=()=>{row.is_angle_dragging=this.can_edit_angle(row);};
   for(const event of ['pointerup','pointercancel','blur'])angle_input.addEventListener(event,()=>{row.is_angle_dragging=false;});
   checkbox.onchange=()=>{this.set_visible(row,checkbox.checked);this.tick();};
   for(const button of card.querySelectorAll('[data-view]'))button.onclick=()=>{row.views[this.metric]=button.dataset.view;this.apply_view(row);this.tick();};
   this.rows.set(joint.name,row);this.section.querySelector('.motor-joint-list').append(card);this.set_visible(row,this.visibility[this.model]?.[joint.name]!==false);
  }
  this.select_metric(this.metric,false);
 }
 set_visible(row,is_visible){
  row.is_visible=is_visible;row.card.hidden=!is_visible;row.checkbox.checked=is_visible;
  this.visibility[this.model]??={};this.visibility[this.model][row.joint.name]=is_visible;
  if(is_visible)row.history??=new motor_history();else{row.history=null;row.jerk.sim.clear();row.jerk.hardware.clear();}
  this.elements.empty.hidden=[...this.rows.values()].some(item=>item.is_visible);
 }
 select_metric(metric,can_tick=true){
  if(!metrics[metric])return;this.metric=metric;
  for(const [name,button] of this.tabs){button.setAttribute('aria-selected',String(name===metric));button.tabIndex=name===metric?0:-1;}
  this.section.querySelector('.motor-joint-list').setAttribute('aria-labelledby',this.tabs.get(metric).id);
  const is_jerk=metric==='jerk_rad_sec3';this.elements.metric_note.hidden=!is_jerk;this.elements.metric_note.textContent=is_jerk?'推定':'';
  this.elements.metric_note.title=is_jerk?'角速度の2段差分によるジャーク推定［deg/s³］。ノイズの影響あり。':'';this.elements.metric_note.setAttribute('aria-label',this.elements.metric_note.title);
  for(const row of this.rows.values())this.apply_view(row);if(can_tick)this.tick();
 }
 apply_view(row){
  const is_graph=row.views[this.metric]==='graph';
  for(const button of row.card.querySelectorAll('[data-view]'))button.setAttribute('aria-selected',String((button.dataset.view==='graph')===is_graph));
  row.card.querySelector('.motor-gauges').hidden=is_graph;
  if(is_graph&&!row.canvas){row.canvas=document.createElement('canvas');row.canvas.className='motor-plot';row.card.append(row.canvas);}
  if(row.canvas){row.canvas.hidden=!is_graph;row.canvas.setAttribute('aria-label',row.joint.name+' '+metrics[this.metric].label+'の時系列グラフ');}
 }
 can_edit_angle(row){return this.metric==='position_rad'&&this.simulator.diagnostics.ready&&this.simulator.robot.pose_source==='simulator'&&this.simulator.robot.joints[row.joint.name]===row.joint&&typeof this.simulator.set_manual_joint==='function';}
 render_row(row,has_sim,inputs){
  const metric=this.metric,description=metrics[metric],max_value=row.display_limits[metric],profile=this.profile(row.joint.name);
  for(const source of ['sim','hardware']){
   const fields=row.fields[source],values=row.values[source],value=values[metric]===null?null:values[metric]*description.scale,has_hardware=!!this.socket;
   fields.output.hidden=source==='hardware'&&!has_hardware;fields.gauge_box.hidden=source==='hardware'&&!has_hardware;
   const is_estimated_value=is_estimated(values,metric),label=source==='sim'?'シミュレーション':'実機';
   set_text(fields.output,`${value===null?'—':value.toFixed(3)} ${description.unit}${is_estimated_value?'（推定）':''}`);
   fields.output.setAttribute('aria-label',row.joint.name+' '+label+' '+description.label+' '+fields.output.textContent);
   const can_edit=source==='sim'&&this.can_edit_angle(row),is_missing=value===null&&!can_edit;fields.gauge.disabled=!can_edit;fields.gauge.hidden=false;fields.gauge.classList.toggle('is-missing',is_missing);
   fields.gauge.min=can_edit&&Number.isFinite(row.joint.lower)?row.joint.lower*description.scale:-max_value;fields.gauge.max=can_edit&&Number.isFinite(row.joint.upper)?row.joint.upper*description.scale:max_value;fields.gauge.step=can_edit?'0.1':'any';
   const physics=this.simulator.physics_panel,goal=can_edit&&physics.socket&&physics.enable_dynamics&&physics.robot_model===this.simulator.robot?finite_or_null(physics.targets?.[row.joint.name]):null;
   if(!can_edit||!row.is_angle_dragging)fields.gauge.value=can_edit?(goal??row.joint.q)*description.scale:value??0;
   fields.gauge.setAttribute('aria-valuetext',is_missing?'未受信・未計算':fields.gauge.value+' '+description.unit);
   fields.gauge.setAttribute('aria-label',row.joint.name+' '+label+' '+description.label+(can_edit?'の調整':'の読取り専用ゲージ'));
   let note=value===null?'未受信・未計算・失効':metric==='jerk_rad_sec3'?'角速度の差分推定':is_estimated_value?'換算・推定'+(profile.has_verified_calibration?'':'（未校正）'):source==='sim'?(has_sim?'MuJoCo計算値':'ブラウザ表示角'):'ROS配信値';
   const source_sample=inputs[metric==='jerk_rad_sec3'?'velocity_rad_sec':metric]??(is_estimated_value?inputs[metric==='torque_nm'?'current_a':'torque_nm']:null);
   if(source==='hardware'&&source_sample&&!source_sample.has_source_stamp)note+='・取得時刻なし';
   fields.output.title=label+'・'+note;fields.gauge.title=can_edit?'シミュレータ目標角［deg］。現在角は右上の数値。':fields.output.title;
  }
 }
 draw_plot(row,now_ms){
  const canvas=row.canvas;if(!canvas||!canvas.clientWidth)return;
  const rect=canvas.getBoundingClientRect();if(rect.bottom<0||rect.top>innerHeight)return;
  const width=Math.round(canvas.clientWidth),height=150,ratio=Math.min(devicePixelRatio||1,2);
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==height*ratio){canvas.width=Math.round(width*ratio);canvas.height=height*ratio;}
  const context=canvas.getContext('2d');context.setTransform(ratio,0,0,ratio,0,0);context.clearRect(0,0,width,height);
  const metric=this.metric,max_value=row.display_limits[metric],left=42,right=width-8,top=20,bottom=height-24;
  context.font='11px sans-serif';context.fillStyle='#a6b4aa';context.fillText(max_value.toFixed(1)+' '+metrics[metric].unit,2,13);context.fillText('-'+max_value.toFixed(1),2,bottom);context.fillText('−60秒',left,height-5);context.fillText('現在',right-24,height-5);
  context.strokeStyle='#3b4940';context.lineWidth=1;context.setLineDash([]);context.beginPath();context.moveTo(left,(top+bottom)/2);context.lineTo(right,(top+bottom)/2);context.stroke();
  const idx=metric_names.indexOf(metric);
  for(const [offset,color,flag] of [[idx+1,'#79bb35',1<<idx],[idx+1+metric_names.length,'#39aeca',1<<(idx+metric_names.length)]]){
   context.strokeStyle=color;context.lineWidth=2;let has_previous=false,last_time_ms=0,last_flag=null;context.beginPath();
   row.history.each((values,start,flags)=>{
    const time_ms=values[start],value=values[start+offset]*metrics[metric].scale,is_estimated=!!(flags&flag);
    if(time_ms<now_ms-60000||!Number.isFinite(value)){has_previous=false;return;}
    if(is_estimated!==last_flag){context.stroke();context.beginPath();context.setLineDash(is_estimated?[5,3]:[]);has_previous=false;last_flag=is_estimated;}
    const x=left+(time_ms-(now_ms-60000))/60000*(right-left),y=top+(1-value/max_value)*(bottom-top)/2;
    if(!has_previous||time_ms-last_time_ms>300)context.moveTo(x,y);else context.lineTo(x,y);
    has_previous=true;last_time_ms=time_ms;
   });context.stroke();
  }context.setLineDash([]);
 }
}

// 関節操作とモータ表示の共通スタイル
if(typeof document!=='undefined'){
 const stylesheet=document.createElement('link');stylesheet.rel='stylesheet';stylesheet.href=new URL('./motor-monitor.css',import.meta.url);document.head.append(stylesheet);
}
