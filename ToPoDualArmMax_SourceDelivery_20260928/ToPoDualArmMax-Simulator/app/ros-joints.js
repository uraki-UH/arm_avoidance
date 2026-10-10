import {robot_snapshot,attach_camera,bridge_socket_url,pose_sources} from './robot-ros-state.js';
const $=id=>document.getElementById(id);
const state_outputs=Object.freeze(['joints','base','tf']);

// 点群取得・描画周期から独立した関節通信。受信姿勢は最新一件のみ保持
export class RosJointStream {
 constructor(panel){
  this.panel=panel;this.socket=null;this.latest=null;this.is_ready=false;this.state_hz=100;this.next_snapshot_ms=0;
  const controls=document.createElement('section');
  controls.innerHTML=`<h3>ロボット状態の送受信</h3><label>Hz <input id="ros-joints-hz" type="number" min="1" max="200" value="100"></label><label class="field-label">受信 JointState トピック<input id="ros-joints-topic" value="/joint_states"></label><label><input id="ros-state-all" type="checkbox"> ロボット状態をROSへ送信（関節角・配置・TF）</label><label><input id="ros-joints-receive" type="checkbox"> ROSの現在関節角を追従表示</label><pre id="ros-joints-status">通信OFF</pre>`;
  $('ros-receive-panel').before(controls);
  for(const id of ['ros-state-all','ros-joints-hz','ros-joints-topic','ros-joints-receive'])$(id).addEventListener('change',()=>{
   if(id==='ros-state-all'&&$(id).checked&&!this.enable_duplex)$('ros-joints-receive').checked=false;
   if(id==='ros-joints-receive'&&$(id).checked){if(!this.enable_duplex)$('ros-state-all').checked=false;panel.active=null;$('ros-command-enable').checked=false;}
   this.connect();
  });
 }
 close(){
  const socket=this.socket;this.socket=null;if(socket)socket.terminate();this.latest=null;this.is_ready=false;this.next_snapshot_ms=0;
  this.panel.set_connection_state('joints',false);
 }
 // 関節角・配置・TFの一括送信。同一スナップショットと共通ROS時刻
 outputs(){return $('ros-state-all').checked?state_outputs:[];}
 stop(){if(this.panel.rgbd.robot.pose_source===pose_sources.leader)window.simulator?.physics_panel?.hold_leader();this.close();$('ros-state-all').checked=false;$('ros-joints-receive').checked=false;$('ros-joints-status').textContent='通信OFF';}
 connect(){
  this.close();
  this.panel.instance_panel.set_source($('ros-joints-receive').checked?($('robot-pose-source').value===pose_sources.leader?pose_sources.leader:pose_sources.ros):pose_sources.simulator);
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
     this.latest_stamp_sec=data.stamp_sec;
     if(is_leader){const physics=window.simulator?.physics_panel;if(physics?.set_leader_target(data.pose))physics.leader_stamp_sec=data.stamp_sec;this.latest=null;}
    }
   }catch(error){fail(error.message);}
  };
  socket.onerror=()=>fail('関節WebSocketへ接続できません。ブリッジの更新と起動を確認してください');
  socket.postMessage({type:'connect',url:url.href,send:this.outputs().length>0,config:{type:'config',model,hz,receive:$('ros-joints-receive').checked,topic:$('ros-joints-topic').value,enable_fresh_input:this.panel.rgbd.robot.pose_source===pose_sources.leader,max_state_age_sec:window.simulator?.physics_panel?.max_leader_age_ms/1000}});
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
  if(this.latest&&$('ros-joints-receive').checked){
  try{if(this.panel.rgbd.robot.pose_source===pose_sources.leader){
   const age_sec=Date.now()/1000-this.latest_stamp_sec;
   if(Number.isFinite(age_sec)&&age_sec>=0&&age_sec<window.simulator?.physics_panel?.max_leader_age_ms/1000)window.simulator?.physics_panel?.set_leader_target(this.latest);
  }else window.simulator.apply_ros_pose(this.latest);}
  catch(error){this.stop();$('ros-joints-status').textContent=error.message;}
  this.latest=null;
 }}
}
