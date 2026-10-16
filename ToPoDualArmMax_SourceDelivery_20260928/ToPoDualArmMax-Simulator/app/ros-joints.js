import {robot_snapshot,attach_camera,bridge_socket_url,pose_sources,pose_source_labels} from './robot-ros-state.js';
const $=id=>document.getElementById(id);
const state_outputs=Object.freeze(['joints','base','tf']);

// 点群取得・描画周期から独立した関節通信。受信姿勢は最新一件のみ保持
export class RosJointStream {
 constructor(panel){
  this.panel=panel;this.socket=null;this.latest=null;this.is_ready=false;this.state_hz=100;this.next_snapshot_ms=0;this.motion_sample=null;this.is_simulation_input=false;
  const controls=document.createElement('div');controls.id='ros-joints-panel';
  controls.innerHTML=`<label id="robot-pose-source-label">シミュレータの追従<select id="robot-pose-source">${Object.entries(pose_source_labels).map(([value,label])=>`<option value="${value}">${label}</option>`).join('')}</select></label><label id="ros-joints-topic-field" class="field-label">現在姿勢トピック（JointState）<input id="ros-joints-topic" value="/joint_states"></label><label id="ros-target-topic-field" class="field-label"><span id="ros-target-topic-label"></span><input id="ros-target-topic" value="/leader/joint_states"></label><label>Hz <input id="ros-joints-hz" type="number" min="1" max="200" value="100"></label><label><input id="ros-state-all" type="checkbox"> ロボット状態をROSへ送信（関節角・配置・TF）</label><label><input id="ros-joints-receive" type="checkbox"> <span id="ros-joints-receive-label"></span></label><pre id="ros-joints-status">通信OFF</pre>`;
  $('ros-receive-panel').before(controls);
  $('robot-pose-source').onchange=()=>{
   const source=$('robot-pose-source').value,checkbox=$('ros-joints-receive');
   checkbox.checked=source===pose_sources.ros||source===pose_sources.leader;
   checkbox.dispatchEvent(new Event('change'));
  };
  for(const id of ['ros-state-all','ros-joints-hz','ros-joints-topic','ros-target-topic','ros-joints-receive'])$(id).addEventListener('change',()=>{
   if(id==='ros-joints-topic'||id==='ros-target-topic')this.is_simulation_input=false;
   if(id==='ros-state-all'&&$(id).checked&&!this.enable_duplex)$('ros-joints-receive').checked=false;
   if(id==='ros-joints-receive'&&$(id).checked){if(!this.enable_duplex)$('ros-state-all').checked=false;panel.active=null;$('ros-command-enable').checked=false;}
   this.connect();
  });
  $('ros-endpoint').addEventListener('change',()=>{this.is_simulation_input=false;});
  this.update_input_controls();
 }
 // 描画姿勢と力学入力で独立したトピック設定。切替時の値の流用なし。
 topic_input(source=$('robot-pose-source').value){return $(source===pose_sources.leader?'ros-target-topic':'ros-joints-topic');}
 set_input_topic(source,topic){this.topic_input(source).value=topic;}
 update_input_controls(){
  const source=$('robot-pose-source').value;
  const control=window.simulator?.robot_control;
  const can_receive=!control||control.config.usage==='observation'||control.config.command_source==='leader'||!!this.follow_config;
  const is_absolute=this.follow_config?.profiles[this.follow_config.profile]?.follow_mode==='absolute';
  $('ros-joints-topic-field').hidden=source!==pose_sources.ros;
  $('ros-target-topic-field').hidden=source!==pose_sources.leader;
  $('ros-target-topic-label').textContent=is_absolute?'目標角トピック（JointState・絶対追従）':'リーダー姿勢トピック（JointState・相対追従）';
  $('ros-joints-receive-label').textContent=source===pose_sources.ros?'現在姿勢を受信':source===pose_sources.leader?(is_absolute?'目標角を受信':'リーダー姿勢を受信'):'ROS入力を受信';
  $('ros-joints-receive').parentElement.hidden=!can_receive;
 }
 close(){
  const socket=this.socket;this.socket=null;if(socket)socket.terminate();this.latest=null;this.motion_sample=null;this.is_ready=false;this.next_snapshot_ms=0;
  this.panel.set_connection_state('joints',false);
 }
 // 関節角・配置・TFの一括送信。同一スナップショットと共通ROS時刻
 outputs(){return $('ros-state-all').checked?state_outputs:[];}
 stop(){if(this.panel.rgbd.robot.pose_source===pose_sources.leader)window.simulator?.physics_panel?.hold_leader();this.close();$('ros-state-all').checked=false;$('ros-joints-receive').checked=false;$('ros-joints-status').textContent='通信OFF';}
 // CLIなどの外部シミュレーション実測への表示接続。送信・ブラウザ内の力学はOFF。
 receive_topic(topic,endpoint,is_simulation_input=false){
  if(this.follow_config)throw Error('共通接続構成を解除してから、外部シミュレーションへ接続してください');
  window.simulator?.robot_control?.observe_source(pose_sources.ros);
  this.panel.stop();window.simulator?.physics_panel?.stop();
  if(endpoint){$('ros-endpoint').value=endpoint;$('ros-endpoint').dispatchEvent(new Event('change'));}
  this.is_simulation_input=is_simulation_input;
  $('robot-pose-source').value=pose_sources.ros;$('ros-joints-topic').value=topic;
  $('ros-state-all').checked=false;$('ros-joints-receive').checked=true;this.connect();
 }
 // モデル・入力元・受信鮮度を一致させた、表示ロボットの実測サンプル。
 get_motion_sample(now_ms=performance.now()){
  const sample=this.motion_sample;
  return this.is_ready&&$('ros-joints-receive').checked&&this.panel.rgbd.robot.pose_source===pose_sources.ros&&sample?.model===this.panel.rgbd.robot.modelId&&now_ms-sample.received_ms<=1000?sample:null;
 }
 connect(){
  this.close();
  const control=window.simulator?.robot_control;
  if(control&&control.config.usage==='simulation'&&!['leader'].includes(control.config.command_source))$('ros-joints-receive').checked=false;
  this.panel.instance_panel.set_source(control?.config.usage==='observation'?pose_sources.ros:$('ros-joints-receive').checked?($('robot-pose-source').value===pose_sources.leader?pose_sources.leader:pose_sources.ros):control&&control.config.usage==='simulation'&&!control.can_edit_pose?pose_sources.command:pose_sources.simulator);
  if(!this.outputs().length&&!$('ros-joints-receive').checked){$('ros-joints-status').textContent='通信OFF';return;}
  const hz=Number($('ros-joints-hz').value);
  if(!Number.isFinite(hz)||hz<1||hz>200){this.stop();$('ros-joints-status').textContent='通信上限は1～200 Hzです';return;}
  let url;try{url=bridge_socket_url($('ros-endpoint').value,'/joints');}catch{this.stop();$('ros-joints-status').textContent='接続先HTTP URLとポートを確認してください';return;}
  this.state_hz=hz;
  const socket=this.socket=new Worker(new URL('./ros-joints-worker.js',import.meta.url),{type:'module'}),model=this.panel.rgbd.robot.modelId;
  const fail=message=>{if(this.socket!==socket)return;this.stop();$('ros-joints-status').textContent=message;};
  socket.onmessage=event=>{
   if(this.socket!==socket||model!==this.panel.rgbd.robot.modelId)return;
   try{
    const data=event.data;
    if(data.type==='ready'){this.is_ready=true;this.panel.set_connection_state('joints',true);$('ros-joints-status').textContent=`接続済み · 上限 ${hz} Hz`;}
    else if(data.type==='error')fail(data.error);
    else if(data.type==='joints'&&$('ros-joints-receive').checked){
     const robot=this.panel.rgbd.robot;
     const is_leader=robot.pose_source===pose_sources.leader;
     const max_age_sec=window.simulator?.physics_panel?.max_leader_age_ms/1000;
     if(is_leader&&(!Number.isFinite(data.stamp_sec)||Date.now()/1000-data.stamp_sec<0||Date.now()/1000-data.stamp_sec>=max_age_sec))return;
     const pose={},invalid_names=[];
     for(const [name,value] of Object.entries(data.pose)){
      const joint=robot.joints[name];if(!joint||!Number.isFinite(value))throw Error('受信関節角がモデルと一致しません');
      const is_gripper=/^(?:L|R)_gripper_(?:joint|mimic)$/.test(name);
      if(!is_leader&&!is_gripper&&(value<joint.lower||value>joint.upper)){invalid_names.push(name);continue;}
      pose[name]=!is_leader&&is_gripper?Math.max(joint.lower,Math.min(joint.upper,value)):value;
     }
     // 直接表示のグリッパーだけの開閉端飽和。その他の可動域外関節は前回表示の保持
     if(!is_leader)$('ros-joints-status').textContent=invalid_names.length?`可動域外・表示更新なし: ${invalid_names.join(', ')} / 他の関節は受信継続`:`接続済み · 上限 ${hz} Hz`;
     this.latest=Object.keys(pose).length?pose:null;
     if(this.latest){
      const motion=Object.fromEntries(['velocity','effort'].map(field=>[field,Object.fromEntries(Object.keys(pose).filter(name=>typeof data.motion?.[field]?.[name]==='number'&&Number.isFinite(data.motion[field][name])).map(name=>[name,data.motion[field][name]]))]));
      const received_ms=performance.now();
      this.motion_sample={model,pose,motion,is_simulation_input:this.is_simulation_input,received_ms,time_ms:Number.isFinite(data.stamp_sec)?data.stamp_sec*1000:received_ms,stamp_sec:data.stamp_sec};
     }
     this.latest_stamp_sec=data.stamp_sec;
     if(is_leader){const physics=window.simulator?.physics_panel;if(physics?.set_leader_target(data.pose))physics.leader_stamp_sec=data.stamp_sec;this.latest=null;}
    }
   }catch(error){fail(error.message);}
  };
  socket.onerror=()=>fail('関節WebSocketへ接続できません。ブリッジの更新と起動を確認してください');
  socket.postMessage({type:'connect',url:url.href,send:this.outputs().length>0,config:{type:'config',model,hz,receive:$('ros-joints-receive').checked,topic:this.topic_input().value,enable_fresh_input:this.panel.rgbd.robot.pose_source===pose_sources.leader,max_state_age_sec:window.simulator?.physics_panel?.max_leader_age_ms/1000}});
  this.tick();
 }
 // 描画側の状態取得。通信上限より低頻度の設定では不要なTF計算とWorker転送を省略
 publish_state(){
  const physics=window.simulator?.physics_panel;
  if(physics?.socket&&physics.enable_dynamics)physics.sync_robot_pose();
  const state=robot_snapshot(this.panel.rgbd.robot);
  if(physics?.socket&&physics.enable_dynamics&&physics.motion_state){
   for(const field of ['velocity','effort'])state['joint_'+field]=Object.fromEntries(Object.keys(state.robot_pose).map(name=>[name,physics.motion_state[field][name]]));
  }
  if(this.follow_config){state.follow_session_id=this.follow_config.session_id;state.follow_simulation={is_running:!!(physics?.socket&&physics?.actual_joints),is_input_stopped:physics?.is_leader_stopped!==false};}
  attach_camera(state,this.panel.rgbd.sensor.opticalToWorld(this.panel.rgbd.opticalWorld()).toArray());
  this.socket.postMessage({type:'pose',state,outputs:this.outputs()});
 }
 tick(now=performance.now()){
  if(this.socket&&$('ros-joints-receive').checked)this.socket.postMessage({type:'poll'});
  if(this.socket&&this.outputs().length&&now>=this.next_snapshot_ms){
   const period_ms=1000/this.state_hz;
   this.next_snapshot_ms=this.next_snapshot_ms?this.next_snapshot_ms+(Math.floor((now-this.next_snapshot_ms)/period_ms)+1)*period_ms:now+period_ms;
   this.publish_state();
  }
  if(this.latest&&$('ros-joints-receive').checked&&!window.simulator?.avoidance_replay?.has_pose_control){
  try{if(this.panel.rgbd.robot.pose_source===pose_sources.leader){
   const age_sec=Date.now()/1000-this.latest_stamp_sec;
   if(Number.isFinite(age_sec)&&age_sec>=0&&age_sec<window.simulator?.physics_panel?.max_leader_age_ms/1000)window.simulator?.physics_panel?.set_leader_target(this.latest);
  }else window.simulator.apply_ros_pose(this.latest);}
  catch(error){this.stop();$('ros-joints-status').textContent=error.message;}
  this.latest=null;
 }}
}
