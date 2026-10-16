import {format_fixed} from './display-format.js';
import {robot_follow_panel} from './robot-follow.js';
import * as THREE from 'three';
import {RobotInstancePanel} from './robot-instance.js';
import {RosJointStream} from './ros-joints.js';
const $=id=>document.getElementById(id);
const placement_axes=['x','y','z','roll','pitch','yaw'];

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
  this.rgbd=rgbd;this.toast=toast;this.is_busy=false;this.last_ms=0;this.sequence=null;this.generation=0;this.active=null;this.model=null;this.online_sources=new Set();
  this.placement_controls=new Map();this.placement_rotation=new THREE.Euler(0,0,0,'ZYX');this.placement_input_quaternion=new THREE.Quaternion();
  const placement_panel=document.createElement('section');
  placement_panel.innerHTML='<div class="section-label">ロボット配置 <span>world</span></div>';
  for(const [idx,name] of placement_axes.entries()){
   const is_position=idx<3,label=is_position?name.toUpperCase():{roll:'Roll',pitch:'Pitch',yaw:'Yaw'}[name],unit=is_position?'m':'°',max_range=is_position?1:180,step=is_position?.001:.1;
   const row=document.createElement('div');row.className='body-joint-row robot-placement-row';
   row.innerHTML=`<label for="ros-base-${name}-range">${label}</label><input id="ros-base-${name}-range" type="range" min="${-max_range}" max="${max_range}" step="${step}" value="0" aria-label="ロボット配置 ${label} スライダー"><span class="body-joint-number"><input id="ros-base-${name}" type="number" step="${step}" value="0" ${is_position?'min="-100" max="100"':''} aria-label="ロボット配置 ${label} ${unit}"><span class="robot-placement-unit" aria-hidden="true">${unit}</span></span>`;
   const input=row.querySelector('input[type=range]'),number=row.querySelector('input[type=number]');
   this.placement_controls.set(name,{input,number,max_range});
   input.oninput=()=>{number.value=input.value;this.apply_placement();};
   number.oninput=()=>{if(Number.isFinite(number.valueAsNumber))this.apply_placement();};
   number.onchange=()=>this.apply_placement(true);
   placement_panel.append(row);
  }
  $('robot-panel').append(placement_panel);
  const panel=document.createElement('section');panel.id='ros-receive-panel';panel.innerHTML=`<h3>ROS → ブラウザ：受信</h3><label><input id="ros-command-enable" type="checkbox"> 関節軌道を受信して再生</label><button id="ros-command-stop" class="wide-button">軌道停止・受信OFF</button><pre id="ros-robot-status">受信OFF</pre>`;
  $('ros-panel').append(panel);
  this.joint_stream=new RosJointStream(this);
  this.instance_panel=new RobotInstancePanel(this);
  this.follow_panel=new robot_follow_panel(this);
  const preview=document.createElement('details');preview.innerHTML='<summary>軌道の表示プレビュー（力学なし）</summary>';panel.before(preview);preview.append(panel);panel.querySelector('h3').remove();
  this.sync_placement_controls();
  $('ros-command-stop').onclick=()=>this.stop();
  $('ros-command-enable').onchange=()=>{
   const enable_preview=$('ros-command-enable').checked;
   if(enable_preview){
    try{if(window.simulator?.robot_control)window.simulator.robot_control.configure({usage:'preview'});else window.simulator?.physics_panel?.stop();}
    catch(error){$('ros-command-enable').checked=false;$('ros-robot-status').textContent=error.message;return;}
    this.joint_stream.stop();this.instance_panel.set_source('simulator');$('ros-command-enable').checked=true;
   }
   this.active=null;this.sequence=null;this.generation++;this.set_connection_state('trajectory',false);
  };
  $('ros-endpoint').addEventListener('change',()=>this.stop());
  // 手動操作・モデル切替・タブ非表示での受信解除と再生停止
  document.addEventListener('pointerdown',event=>{if(this.active&&!$('ros-panel').contains(event.target)&&!placement_panel.contains(event.target)&&event.target!==this.rgbd.renderer.domElement)this.stop();},true);
  document.addEventListener('keydown',event=>{if(event.key==='Escape')this.stop();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&this.active)this.stop();});
 }
 // スライダー・数値入力で共用する配置検証と適用。途中入力のゼロ補完なし。
 apply_placement(enable_error_message=false){
  if(!window.simulator?.diagnostics.ready)return false;
  const values=placement_axes.map(name=>this.placement_controls.get(name).number.valueAsNumber);
  if(values.some(value=>!Number.isFinite(value))||values.slice(0,3).some(value=>Math.abs(value)>100)){
   if(enable_error_message)this.toast('配置は有限値、XYZは±100 mです');
   return false;
  }
  this.stop();window.simulator.set_robot_placement(values);this.sync_placement_controls();$('ros-robot-status').textContent='配置を更新';return true;
 }
 // 実際の配置との同期。数値欄の編集中は文字列を維持、スライダーの幅は値に追従。
 sync_placement_controls(){
  const robot=this.rgbd.robot,rotation=this.placement_rotation;
  let angles=placement_axes.slice(3).map(name=>this.placement_controls.get(name).number.valueAsNumber);
  const has_input_angles=angles.every(Number.isFinite);
  if(has_input_angles)this.placement_input_quaternion.setFromEuler(rotation.set(...angles.map(THREE.MathUtils.degToRad),'ZYX'));
  // 同じ姿勢を表すEuler入力の維持。Pitch境界でのRoll・Yawの反転防止。
  if(!has_input_angles||Math.abs(this.placement_input_quaternion.dot(robot.quaternion))<1-1e-12){
   rotation.setFromQuaternion(robot.quaternion,'ZYX');angles=['x','y','z'].map(axis=>THREE.MathUtils.radToDeg(rotation[axis]));
  }
  const values=[...robot.position.toArray(),...angles];
  placement_axes.forEach((name,idx)=>{
   const {input,number,max_range}=this.placement_controls.get(name),value=values[idx],range=Math.max(max_range,Math.ceil(Math.abs(value)/max_range)*max_range);
   if(Number(input.max)!==range){input.min=-range;input.max=range;}
   if(Number(input.value)!==value)input.value=value;
   const text=format_fixed(value,idx<3?3:1);
   if(document.activeElement!==number&&number.value!==text)number.value=text;
  });
 }
 // 有効なROS通信経路の集約。状態変化時だけのヘッダー更新
 set_connection_state(source,is_online){
  if(is_online)this.online_sources.add(source);else this.online_sources.delete(source);
  const has_connection=this.online_sources.size>0,status=$('load-status'),label=has_connection?'Online':'Offline';
  if(status.textContent===label)return;
  status.textContent=label;status.parentElement.classList.toggle('is-online',has_connection);
 }
 stop(){this.joint_stream?.stop();this.active=null;this.sequence=null;this.generation++;$('ros-command-enable').checked=false;this.set_connection_state('trajectory',false);}
 tick(now){
  const robot=this.rgbd.robot;
  if(this.model!==robot.modelId){this.follow_panel.disconnect();this.stop();this.model=robot.modelId;}
  this.sync_placement_controls();
  this.instance_panel.bind();
  this.joint_stream.tick(now);
  if(this.active){
   const {command,start}=this.active,t=(now-start)/1000,points=command.points;let idx=1;while(idx<points.length-1&&points[idx].time_sec<t)idx++;
   const a=points[idx-1],b=points[idx],u=Math.min(1,Math.max(0,(t-a.time_sec)/(b.time_sec-a.time_sec))),pose={};
   command.joint_names.forEach((name,j)=>pose[name]=a.positions[j]+u*(b.positions[j]-a.positions[j]));window.simulator.apply_ros_pose(pose);
   if(t>=points.at(-1).time_sec){this.active=null;$('ros-robot-status').textContent='軌道再生完了';}
  }
  if(!this.is_busy&&now-this.last_ms>=100&&$('ros-command-enable').checked){this.last_ms=now;this.exchange();}
 }
 async exchange(){
  this.is_busy=true;const generation=this.generation,model=this.rgbd.robot.modelId;
  try{
   const endpoint=new URL($('ros-endpoint').value);
   if($('ros-command-enable').checked){
    const response=await fetch(new URL('/api/trajectory?model='+model,endpoint),{signal:AbortSignal.timeout(10000)});if(!response.ok)throw Error(await response.text());const result=await response.json();
    if(generation!==this.generation||this.rgbd.robot.modelId!==model)return;
    this.set_connection_state('trajectory',true);
    if(this.sequence===null){this.sequence=result.sequence;return;}
    if(result.sequence!==this.sequence){this.sequence=result.sequence;if(result.command){const command=validate_trajectory(result.command,this.rgbd.robot);this.active={command,start:performance.now()};$('ros-robot-status').textContent='ROS軌道を再生中';}}
   }
  }catch(error){if(generation===this.generation){this.stop();$('ros-robot-status').textContent='ROS接続／軌道エラー：'+error.message;}}
  finally{this.is_busy=false;}
 }
}
