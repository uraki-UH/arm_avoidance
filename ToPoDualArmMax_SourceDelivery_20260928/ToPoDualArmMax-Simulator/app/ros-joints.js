import {robot_snapshot,attach_camera} from './robot-ros-state.js';
const $=id=>document.getElementById(id);

// 点群取得・描画周期から独立した関節通信。受信姿勢は最新一件のみ保持
export class RosJointStream {
 constructor(panel){
  this.panel=panel;this.socket=null;this.timer=null;this.latest=null;this.is_ready=false;this.is_pending=false;
  const controls=document.createElement('section');
  controls.innerHTML=`<h3>姿勢・TFの送受信</h3><label>Hz <input id="ros-joints-hz" type="number" min="1" max="200" value="100"></label><label class="field-label">受信 JointState トピック<input id="ros-joints-topic" value="/joint_states"></label><label><input id="ros-state-all" type="checkbox"> シミュレータ状態をROSへ送信（関節・TF・配置）</label><label><input id="ros-base-send" type="checkbox"> 配置 /sim/base_pose</label><label><input id="ros-tf-send" type="checkbox"> TF /tf・/sim/tf</label><label><input id="ros-joints-send" type="checkbox"> 関節角を /sim/joint_states へ送信</label><label><input id="ros-joints-receive" type="checkbox"> ROSの現在関節角を追従表示</label><pre id="ros-joints-status">通信OFF</pre><p class="sub-note">最大200 Hz。描画は最新姿勢のみ反映。点群に付随する取得時の姿勢・TFは別送信。関節角の送信と受信追従は択一。通信・描画の実周期は負荷に依存し、実機の制御周期を保証しません。</p>`;
  $('ros-receive-panel').before(controls);
  $('ros-state-all').onchange=()=>{for(const id of ['ros-joints-send','ros-base-send','ros-tf-send'])$(id).checked=$('ros-state-all').checked;if($('ros-joints-send').checked)$('ros-joints-receive').checked=false;this.connect();};
  for(const id of ['ros-base-send','ros-tf-send','ros-joints-hz','ros-joints-topic','ros-joints-send','ros-joints-receive'])$(id).addEventListener('change',()=>{
   if(id==='ros-joints-send'&&$(id).checked)$('ros-joints-receive').checked=false;
   if(id==='ros-joints-receive'&&$(id).checked){$('ros-joints-send').checked=false;panel.active=null;$('ros-command-enable').checked=false;}
   this.connect();
  });
 }
 close(){
  clearInterval(this.timer);this.timer=null;const socket=this.socket;this.socket=null;if(socket)socket.terminate();this.latest=null;this.is_ready=false;this.is_pending=false;
 }
 outputs(){return [['ros-joints-send','joints'],['ros-base-send','base'],['ros-tf-send','tf']].filter(([id])=>$(id).checked).map(([,name])=>name);}
 sync_selection(){const num=this.outputs().length;$('ros-state-all').checked=num===3;$('ros-state-all').indeterminate=num>0&&num<3;}
 stop(){this.close();for(const id of ['ros-joints-send','ros-base-send','ros-tf-send'])$(id).checked=false;this.sync_selection();$('ros-joints-receive').checked=false;$('ros-joints-status').textContent='通信OFF';}
 connect(){
  this.close();this.sync_selection();
  this.panel.instance_panel.set_source($('ros-joints-receive').checked?'ros':'simulator');
  if(!this.outputs().length&&!$('ros-joints-receive').checked){$('ros-joints-status').textContent='通信OFF';return;}
  const hz=Number($('ros-joints-hz').value);
  if(!Number.isFinite(hz)||hz<1||hz>200){this.stop();$('ros-joints-status').textContent='通信上限は1～200 Hzです';return;}
  let url;try{url=new URL($('ros-endpoint').value);if(!['http:','https:'].includes(url.protocol)||Number(url.port)>65534)throw Error();}catch{this.stop();$('ros-joints-status').textContent='接続先HTTP URLとポートを確認してください';return;}url.protocol=url.protocol==='https:'?'wss:':'ws:';url.port=String(Number(url.port||(url.protocol==='wss:'?443:80))+1);url.pathname='/joints';url.search='';url.hash='';
  const socket=this.socket=new Worker(new URL('./ros-joints-worker.js',import.meta.url),{type:'module'}),model=this.panel.rgbd.robot.modelId;
  const fail=message=>{if(this.socket!==socket)return;this.stop();$('ros-joints-status').textContent=message;};
  socket.onmessage=event=>{
   if(this.socket!==socket||model!==this.panel.rgbd.robot.modelId)return;
   try{
    const data=event.data;
    if(data.type==='ready'){this.is_ready=true;$('ros-joints-status').textContent=`接続済み · 上限 ${hz} Hz`;}
    else if(data.type==='ack')this.is_pending=false;
    else if(data.type==='error')fail(data.error);
    else if(data.type==='joints'&&$('ros-joints-receive').checked){
     const robot=this.panel.rgbd.robot;
     for(const [name,value] of Object.entries(data.pose)){const joint=robot.joints[name];if(!joint||!Number.isFinite(value)||value<joint.lower||value>joint.upper)throw Error('受信関節角がモデルの可動域と一致しません');}
     this.latest=data.pose;
    }
   }catch(error){fail(error.message);}
  };
  socket.onerror=()=>fail('関節WebSocketへ接続できません。ブリッジの更新と起動を確認してください');
  socket.postMessage({type:'connect',url:url.href,send:this.outputs().length>0,config:{type:'config',model,hz,receive:$('ros-joints-receive').checked,topic:$('ros-joints-topic').value}});
  this.tick();
 }
 tick(){if(this.socket&&$('ros-joints-receive').checked)this.socket.postMessage({type:'poll'});if(this.socket&&this.outputs().length){const physics=window.simulator?.physics_panel;if(physics?.socket&&physics.enable_dynamics)physics.sync_robot_pose();const state=robot_snapshot(this.panel.rgbd.robot);if(physics?.socket&&physics.enable_dynamics&&physics.motion_state){for(const field of ['velocity','effort'])state['joint_'+field]=Object.fromEntries(Object.keys(state.robot_pose).map(name=>[name,physics.motion_state[field][name]]));}attach_camera(state,this.panel.rgbd.sensor.opticalToWorld(this.panel.rgbd.opticalWorld()).toArray());this.socket.postMessage({type:'pose',state,outputs:this.outputs()});}if(this.latest&&$('ros-joints-receive').checked){window.simulator.apply_ros_pose(this.latest);this.latest=null;}}
}
