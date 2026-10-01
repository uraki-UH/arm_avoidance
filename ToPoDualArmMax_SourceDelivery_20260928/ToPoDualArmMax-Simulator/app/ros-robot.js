import * as THREE from 'three';
import {robot_snapshot,attach_camera} from './robot-ros-state.js';
const $=id=>document.getElementById(id);

export function validate_trajectory(command,robot){
 const names=command.joint_names,points=command.points;
 if(!Array.isArray(names)||!names.length||new Set(names).size!==names.length||names.some(name=>!robot.actuated.some(j=>j.name===name)))throw Error('関節名が対象モデルと一致しません');
 if(!Array.isArray(points)||!points.length||points.length>1000)throw Error('軌道点数が不正です');
 let previous=names.map(name=>robot.joints[name].q),last_sec=0;
 for(const point of points){
  if(!Number.isFinite(point.time_sec)||point.time_sec<=last_sec||point.time_sec>120||!Array.isArray(point.positions)||point.positions.length!==names.length)throw Error('時刻または関節角の数が不正です');
  for(let idx=0;idx<names.length;idx++){
   const joint=robot.joints[names[idx]],value=point.positions[idx];
   if(!Number.isFinite(value)||value<joint.lower||value>joint.upper)throw Error('関節可動域を超えています');
   if(Math.abs(value-previous[idx])/(point.time_sec-last_sec)>joint.velocity+1e-6)throw Error('URDFの関節速度上限を超えています');
  }
  previous=point.positions;last_sec=point.time_sec;
 }
 return {joint_names:names,points:[{time_sec:0,positions:names.map(name=>robot.joints[name].q)},...points]};
}

export class RosRobotPanel{
 constructor({rgbd,toast}){
  this.rgbd=rgbd;this.toast=toast;this.is_busy=false;this.last_ms=0;this.sequence=null;this.generation=0;this.active=null;this.model=null;
  const panel=document.createElement('div');panel.innerHTML=`<h2>ロボットとROS</h2><p>world基準の配置。XYZはm、角度はdeg。</p><div class="field-grid">${['x','y','z','roll','pitch','yaw'].map(name=>`<label>${name}<input id="ros-base-${name}" type="number" value="0" step="0.1"></label>`).join('')}</div><button id="ros-base-apply">配置を適用</button><label><input id="ros-state-enable" type="checkbox"> 姿勢・TFを定期送信（上限10 Hz）</label><label><input id="ros-command-enable" type="checkbox"> ROSの関節軌道をブラウザで再生</label><button id="ros-command-stop">軌道停止・受信OFF</button><pre id="ros-robot-status">接続待ち</pre><p>点群送信時は取得時の姿勢・TFも送信。軌道は位置のみの線形補間、受信後の相対時刻で再生。実機への指令なし。</p>`;
  $('ros-panel').append(panel);
  $('ros-base-apply').onclick=()=>{try{const values=['x','y','z','roll','pitch','yaw'].map(name=>Number($('ros-base-'+name).value));if(values.some(v=>!Number.isFinite(v))||values.slice(0,3).some(v=>Math.abs(v)>100))throw Error('配置は有限値、XYZは±100 mです');this.stop();window.simulator.set_robot_placement(values);$('ros-robot-status').textContent='配置を更新';}catch(error){toast(error.message);}};
  $('ros-command-stop').onclick=()=>this.stop();
  $('ros-command-enable').onchange=()=>{this.active=null;this.sequence=null;this.generation++;};
  $('ros-endpoint').addEventListener('change',()=>this.stop());
  // 手動操作・モデル切替・タブ非表示での受信解除と再生停止
  document.addEventListener('pointerdown',event=>{if(this.active&&!$('ros-panel').contains(event.target)&&event.target!==this.rgbd.renderer.domElement)this.stop();},true);
  document.addEventListener('keydown',event=>{if(event.key==='Escape')this.stop();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.stop();});
 }
 stop(){this.active=null;this.sequence=null;this.generation++;$('ros-command-enable').checked=false;}
 tick(now){
  const robot=this.rgbd.robot;
  if(this.model!==robot.modelId){this.stop();this.model=robot.modelId;const e=new THREE.Euler().setFromQuaternion(robot.quaternion,'ZYX');[...robot.position.toArray(),...['x','y','z'].map(k=>THREE.MathUtils.radToDeg(e[k]))].forEach((v,idx)=>$('ros-base-'+['x','y','z','roll','pitch','yaw'][idx]).value=v);}
  if(this.active){
   const {command,start}=this.active,t=(now-start)/1000,points=command.points;let idx=1;while(idx<points.length-1&&points[idx].time_sec<t)idx++;
   const a=points[idx-1],b=points[idx],u=Math.min(1,Math.max(0,(t-a.time_sec)/(b.time_sec-a.time_sec))),pose={};
   command.joint_names.forEach((name,j)=>pose[name]=a.positions[j]+u*(b.positions[j]-a.positions[j]));window.simulator.apply_ros_pose(pose);
   if(t>=points.at(-1).time_sec){this.active=null;$('ros-robot-status').textContent='軌道再生完了';}
  }
  if(!this.is_busy&&now-this.last_ms>=100&&($('ros-state-enable').checked||$('ros-command-enable').checked)){this.last_ms=now;this.exchange();}
 }
 async exchange(){
  this.is_busy=true;const generation=this.generation,model=this.rgbd.robot.modelId;
  try{
   const endpoint=new URL($('ros-endpoint').value);
   if($('ros-state-enable').checked){const state=robot_snapshot(this.rgbd.robot);attach_camera(state,this.rgbd.sensor.opticalToWorld(this.rgbd.opticalWorld()).toArray());const response=await fetch(new URL('/api/state',endpoint),{method:'POST',headers:{'Content-Type':'application/json','X-ToPo-Points':'1'},body:JSON.stringify(state),signal:AbortSignal.timeout(10000)});if(!response.ok)throw Error(await response.text());}
   if($('ros-command-enable').checked){
    const response=await fetch(new URL('/api/trajectory?model='+model,endpoint),{signal:AbortSignal.timeout(10000)});if(!response.ok)throw Error(await response.text());const result=await response.json();
    if(generation!==this.generation||this.rgbd.robot.modelId!==model)return;
    if(this.sequence===null){this.sequence=result.sequence;return;}
    if(result.sequence!==this.sequence){this.sequence=result.sequence;if(result.command){const command=validate_trajectory(result.command,this.rgbd.robot);this.active={command,start:performance.now()};$('ros-robot-status').textContent='ROS軌道を再生中';}}
   }
  }catch(error){if(generation===this.generation){this.stop();$('ros-state-enable').checked=false;$('ros-robot-status').textContent='ROS接続／軌道エラー：'+error.message;}}
  finally{this.is_busy=false;}
 }
}
