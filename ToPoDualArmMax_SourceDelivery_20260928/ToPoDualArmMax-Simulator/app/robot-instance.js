const $=id=>document.getElementById(id);

// 操作対象の姿勢入力と外観。Viewer内のROSロボットとは独立した個体
export class RobotInstancePanel {
 constructor(panel){
  this.panel=panel;this.robot=null;
  const section=document.createElement('section');
  section.innerHTML=`<h3>操作対象ロボット</h3><label>姿勢の入力元<select id="robot-pose-source"><option value="simulator">シミュレータ操作</option><option value="ros">ROS実測の直接表示</option><option value="leader">リーダー → 物理フォロワー</option></select></label><label><input id="robot-visible" type="checkbox" checked>ロボットを表示</label><label>不透明度 <input id="robot-opacity" type="range" min="0" max="1" step="0.05" value="1"><output id="robot-opacity-value">100%</output></label>`;
  $('robot-panel').prepend(section);
  $('robot-pose-source').onchange=()=>{const source=$('robot-pose-source').value,checkbox=$('ros-joints-receive');checkbox.checked=source!=='simulator';if(source==='leader')$('ros-joints-topic').value='/leader/joint_states';checkbox.dispatchEvent(new Event('change'));};
  $('robot-visible').onchange=() =>this.appearance();$('robot-opacity').oninput=()=>this.appearance();
 }
 bind(){
  const robot=this.panel.rgbd.robot;if(this.robot===robot)return;
  this.robot=robot;robot.pose_source='simulator';$('robot-pose-source').value='simulator';
  // 共用パレットからの分離。床・他個体の材質への影響防止
  this.materials=[];
  for(const mesh of robot.renderMeshes){
   const originals=Array.isArray(mesh.material)?mesh.material:[mesh.material];
   const copies=originals.map(original=>{const material=original.clone();this.materials.push({material,opacity:original.opacity,transparent:original.transparent,depth_write:original.depthWrite});return material;});
   mesh.material=Array.isArray(mesh.material)?copies:copies[0];
  }
  this.appearance();
 }
 set_source(source){
  const previous=this.robot?.pose_source;
  this.bind();this.robot.pose_source=source;$('robot-pose-source').value=source;
  if(source==='ros'){window.simulator?.physics_panel?.stop('ROS追従のため物理停止');this.panel.active=null;window.simulator?.apply_ros_pose(this.robot.getPose());}
  if(source==='leader'||previous==='leader'){
   window.simulator?.physics_panel?.hold_leader('入力元の切替。物理の再開始が必要');
   this.panel.active=null;
   window.simulator?.cancel_robot_motion();
  }
 }
 appearance(){
  if(!this.robot)return;
  const opacity=Number($('robot-opacity').value);$('robot-opacity-value').textContent=Math.round(opacity*100)+'%';
  for(const mesh of this.robot.renderMeshes)mesh.visible=$('robot-visible').checked;
  for(const entry of this.materials){entry.material.opacity=entry.opacity*opacity;entry.material.transparent=entry.transparent||opacity<1;entry.material.depthWrite=opacity<1?false:entry.depth_write;entry.material.needsUpdate=true;}
  this.panel.rgbd.renderer.shadowMap.needsUpdate=true;
 }
}
