import {robot_snapshot,attach_camera,points_to_base} from './robot-ros-state.js';
import {RosRobotPanel} from './ros-robot.js';
import {sample_object_surface} from './object-points.js';
import {encodeInput,worldPoints} from './vm-ai.js';
const $=id=>document.getElementById(id);
export class RosPointsPanel{
 constructor({environment,rgbd,toast}){
  Object.assign(this,{environment,rgbd,toast});this.is_running=false;this.is_busy=false;this.last_send_ms=0;this.last_list_ms=0;this.generation=0;
  $('ros-panel').innerHTML=`<h2>ROS 2へ点群を送信</h2>
  <label class="field-label">送信先ブリッジ<input id="ros-endpoint" value="http://127.0.0.1:8879" type="url"></label>
  <label class="field-label">点群の種類<select id="ros-source"><option value="rgbd">RGB-D：シーン全体</option><option value="object_full">対象物体：完全表面</option><option value="object_visible">対象物体：遮蔽付きRGB-D</option></select></label>
  <label><input id="ros-depth" type="checkbox" checked> RGB-D全体の深度画像・CameraInfo・画素対応点群も送信</label>
  <label class="field-label">対象物体<select id="ros-object"></select></label><button id="ros-use-selected">環境で選択中の物体を使用</button>
  <div class="field-grid"><label>完全表面の点数<input id="ros-count" type="number" min="1" max="200000" value="10000"></label><label>送信上限 Hz<input id="ros-hz" type="number" min="0.1" max="10" step="0.1" value="2"></label></div>
  <div class="row-actions"><button id="ros-once">1回送信</button><button id="ros-start">連続送信</button></div><pre id="ros-status">取得待ち</pre>
  <p class="sub-note">XYZ・メートル・base_footprint座標。RGB-Dは現在のカメラ校正・姿勢・深度モードを使用。完全表面は裏面を含むメッシュ面のサンプル。遮蔽付きはロボットや他の物体を含むシーン全体との深度照合。追加の画素対応出力はカメラ光学座標系・深度32FC1。FVGやGNGの結果待ちは不要。</p>`;
  if(location.port==='8879')$('ros-endpoint').value=location.origin;
  $('ros-once').onclick=()=>this.send();$('ros-start').onclick=()=>{this.is_running=!this.is_running;this.generation++;this.update_button();};
  $('ros-use-selected').onclick=()=>{this.refresh_objects();if(environment.selected)$('ros-object').value=environment.selected.id;};
  for(const id of ['ros-source','ros-object','ros-endpoint'])$(id).onchange=()=>{this.generation++;};
  this.refresh_objects();this.robot_panel=new RosRobotPanel({rgbd,toast});
 }
 update_button(){$('ros-start').textContent=this.is_running?'送信を停止':'連続送信';}
 refresh_objects(){
  const list=$('ros-object'),old=list.value,items=this.environment.items;
  const signature=items.map(x=>x.id+':'+x.group.name).join('|');if(signature===this.list_signature)return;
  this.list_signature=signature;list.replaceChildren();for(const item of items){const option=document.createElement('option');option.value=item.id;option.textContent=item.group.name;list.append(option);}
  if(items.some(x=>String(x.id)===old))list.value=old;else if(old){list.selectedIndex=-1;this.is_running=false;this.generation++;this.update_button();$('ros-status').textContent='対象物体が削除されました。対象を選び直してください';}
 }
 tick(now){
  this.robot_panel.tick(now);
  if(now-this.last_list_ms>300){this.last_list_ms=now;this.refresh_objects();}
  const hz=Number($('ros-hz').value);
  if(this.is_running&&!this.is_busy&&now-this.last_send_ms>=1000/hz)this.send();
 }
 async send(){
  if(this.is_busy)return null;this.is_busy=true;this.last_send_ms=performance.now();const generation=this.generation;
  try{
   const hz=Number($('ros-hz').value);if(!Number.isFinite(hz)||hz<.1||hz>10)throw Error('送信上限は0.1〜10 Hzを指定してください');
   const endpoint=new URL($('ros-endpoint').value);if(!['http:','https:'].includes(endpoint.protocol))throw Error('送信先はHTTPのURLを指定してください');
   const source=$('ros-source').value,item=this.environment.items.find(x=>String(x.id)===$('ros-object').value);
   if(source!=='rgbd'&&!item)throw Error('対象物体を選択してください');
   let points,robot_pose,robot_model,object_pose,depth_frame,robot_state;
   const captured_at_ms=Date.now();
   if(item){item.group.updateWorldMatrix(true,true);object_pose=item.group.matrixWorld.toArray();}
   if(source==='object_full'){
    robot_state=robot_snapshot(this.rgbd.robot);robot_pose=robot_state.robot_pose;robot_model=this.rgbd.robot.modelId;
    points=(await sample_object_surface(item.group,Number($('ros-count').value))).slice();
    item.group.updateWorldMatrix(true,true);if(object_pose.some((v,i)=>v!==item.group.matrixWorld.elements[i]))throw Error('取得中に対象が移動しました。再送してください');
   }else{
    const frame=await this.rgbd.capture({target_group:source==='object_visible'?item.group:null});
    if(!frame){$('ros-status').textContent='RGB-D取得中またはモデル変更中。次の送信で再試行';return null;}
    if(source==='rgbd'&&$('ros-depth').checked)depth_frame=frame;
    robot_state=frame.robot_state;attach_camera(robot_state,frame.depthWorld);
    points=worldPoints(frame.xyz,frame.depthWorld);robot_pose=frame.robotPose;robot_model=frame.robotModel;
   }
   if(generation!==this.generation||(source!=='rgbd'&&!this.environment.items.includes(item)))return null;
   points_to_base(points,robot_state);
   const meta={robot_state,source,frame_id:'base_footprint',count:points.length/3,captured_at_ms,robot_pose,robot_model,object_id:source==='rgbd'?null:item.id,object_to_world:source==='rgbd'?null:object_pose};
   let body;
   if(depth_frame){
    meta.depth_image={...depth_frame.calibration.depth,optical_to_world:depth_frame.depthWorld};
    const packed=encodeInput(meta,points);body=new Blob([packed,depth_frame.depth]);
   }else body=encodeInput(meta,points);
   const response=await fetch(new URL('/api/points',endpoint),{method:'POST',headers:{'Content-Type':'application/octet-stream','X-ToPo-Points':'1'},body,signal:AbortSignal.timeout(10000)});
   if(!response.ok)throw Error(await response.text());const result=await response.json();
   if(generation===this.generation)$('ros-status').textContent=`${result.topic}${result.depth_topics?'\n'+result.depth_topics.join('\n'):''}\n${meta.count.toLocaleString()} 点送信済み\nframe: ${meta.frame_id}`;
   return result;
  }catch(error){this.is_running=false;this.update_button();$('ros-status').textContent='送信エラー：'+error.message+'\npointcloud_bridge.py の起動と送信先を確認してください';return null;}
  finally{this.is_busy=false;}
 }
}
