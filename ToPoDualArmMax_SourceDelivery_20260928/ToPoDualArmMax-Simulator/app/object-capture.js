import {sample_object_surface} from './object-points.js';
import {robot_snapshot} from './robot-ros-state.js';
const $=id=>document.getElementById(id);

export class ObjectCapturePanel{
 constructor({environment,rgbd}){
  Object.assign(this,{environment,rgbd});this.is_running=false;this.is_busy=false;this.last_ms=0;this.generation=0;this.frames={};
  const panel=document.createElement('section');panel.innerHTML=`<h2>物体点群の取得</h2>
  <label class="field-label">対象物体<select id="object-capture-target"></select></label>
  <label class="field-label">取得方式<select id="object-capture-source"><option value="object_full">完全表面</option><option value="object_visible">遮蔽付きRGB-D</option></select></label>
  <div class="field-grid"><label>完全表面の点数<input id="object-capture-count" type="number" min="1" max="200000" value="10000"></label><label>連続取得 Hz<input id="object-capture-hz" type="number" min="0.1" max="10" step="0.1" value="2"></label></div>
  <div class="row-actions"><button id="object-capture-once">1回取得</button><button id="object-capture-live">連続取得</button></div><pre id="object-capture-status" class="sensor-stats">取得待ち</pre>`;
  $('environment-panel').append(panel);
  $('object-capture-once').onclick=()=>this.capture();$('object-capture-live').onclick=()=>{this.is_running=!this.is_running;this.update_button();};
  $('object-capture-target').onchange=()=>{this.generation++;this.frames={};};
  $('object-capture-source').onchange=()=>{this.generation++;};
  $('object-capture-count').onchange=()=>{this.generation++;delete this.frames.object_full;};
  this.refresh();
 }
 update_button(){$('object-capture-live').textContent=this.is_running?'取得を停止':'連続取得';}
 refresh(){
  const items=this.environment.items,list=$('object-capture-target'),signature=items.map(x=>x.id+':'+x.group.name).join('|');
  if(signature===this.signature)return;this.signature=signature;const old=list.value;list.replaceChildren();
  for(const item of items){const option=document.createElement('option');option.value=item.id;option.textContent=item.group.name;list.append(option);}
  if(items.some(x=>String(x.id)===old))list.value=old;
  else if(old){this.frames={};this.generation++;this.is_running=false;this.update_button();}
 }
 latest(source){const f=this.frames[source];return f&&f.robot===this.rgbd.robot&&this.environment.items.includes(f.item)?f:null;}
 tick(now){
  this.refresh();const hz=Number($('object-capture-hz').value);
  if(this.is_running&&!this.is_busy){
   if(!Number.isFinite(hz)||hz<.1||hz>10){this.is_running=false;this.update_button();$('object-capture-status').textContent='取得Hzは0.1〜10を指定してください';return;}
   if(now-this.last_ms>=1000/hz)this.capture();
  }
 }
 async capture(){
  if(this.is_busy)return null;this.is_busy=true;this.last_ms=performance.now();const generation=this.generation,robot=this.rgbd.robot;
  try{
   const item=this.environment.items.find(x=>String(x.id)===$('object-capture-target').value);if(!item)throw Error('対象物体を選択してください');
   const source=$('object-capture-source').value;item.group.updateWorldMatrix(true,true);const object_pose=item.group.matrixWorld.toArray();let frame;
   if(source==='object_full'){
    const state=robot_snapshot(robot),points=(await sample_object_surface(item.group,Number($('object-capture-count').value))).slice();
    frame={points,robot_state:state,robotPose:state.robot_pose,robotModel:state.robot_model};
   }else frame=await this.rgbd.capture({target_group:item.group});
   if(!frame||generation!==this.generation||robot!==this.rgbd.robot||!this.environment.items.includes(item))return null;
   item.group.updateWorldMatrix(true,true);if(object_pose.some((v,i)=>v!==item.group.matrixWorld.elements[i]))throw Error('取得中に物体が移動しました。再取得してください');
   const result={frame,item,robot,object_pose};this.frames[source]=result;
   $('object-capture-status').textContent=`${(frame.points??frame.xyz).length/3} 点取得済み`;
   return result;
  }catch(error){this.is_running=false;this.update_button();$('object-capture-status').textContent=error.message;return null;}
  finally{this.is_busy=false;}
 }
}
