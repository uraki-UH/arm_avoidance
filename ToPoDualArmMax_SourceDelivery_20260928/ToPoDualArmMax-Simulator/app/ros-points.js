import {attach_camera,attach_lidar,points_to_base} from './robot-ros-state.js';
import {RosRobotPanel} from './ros-robot.js';
import {ObjectCapturePanel} from './object-capture.js';
import {encodeInput,worldPoints} from './vm-ai.js';
const $=id=>document.getElementById(id);
const sources=['rgbd','mid360','object_full','object_visible'];
export class RosPointsPanel{
 constructor({environment,rgbd,lidar,toast}){
  Object.assign(this,{environment,rgbd,lidar,toast});this.is_running=false;this.is_busy=false;this.generation=0;this.last_sent_frames={};this.next_source_idx=0;
  $('ros-panel').innerHTML=`<h2>ROS2連携</h2>
  <label class="ros-connection"><span>接続先</span><input id="ros-endpoint" value="http://127.0.0.1:8879" type="url" aria-label="ROSブリッジ接続先（送受信共通）" spellcheck="false"></label>
  <section id="ros-send-panel"><h3>ブラウザ → ROS：送信</h3>
  <fieldset><legend>点群トピック（複数選択可）</legend>
  ${[['rgbd','/sim/rgbd/points（RGB-D）'],['mid360','/sim/lidar/points（LiDAR）'],['object_full','/sim/object/full_points（完全表面）'],['object_visible','/sim/object/visible_points（遮蔽付き）']].map(([source,label])=>`<label style="display:block;margin:8px 0"><input id="ros-send-${source}" type="checkbox"> ${label}</label>`).join('')}</fieldset>
  <label><input id="ros-depth" type="checkbox" checked> 深度画像・CameraInfo・画素対応点群も送信</label>
  <div class="row-actions"><button id="ros-start">連続送信</button></div><pre id="ros-status">取得待ち</pre>

</section>`;
  if(location.port==='8879')$('ros-endpoint').value=location.origin;
  this.object_capture=new ObjectCapturePanel({environment,rgbd});
  $('ros-start').onclick=()=>{this.is_running=!this.is_running;this.generation++;this.update_button();if(this.is_running)$('ros-status').textContent='取得待ち';};
  for(const id of ['ros-endpoint','ros-depth',...sources.map(source=>'ros-send-'+source)])$(id).onchange=()=>{this.generation++;};
  $('ros-endpoint').addEventListener('change',()=>{this.last_sent_frames={};this.robot_panel.set_connection_state('points',false);});
  $('ros-depth').addEventListener('change',()=>{delete this.last_sent_frames.rgbd;});
  // 取得開始時だけ送信候補を選択。送信開始と手動の選択解除は独立
  const select_source=source=>{const checkbox=$('ros-send-'+source);if(checkbox&&!checkbox.checked){checkbox.checked=true;this.generation++;}};
  $('sensor-live').addEventListener('click',()=>{if(rgbd.live)select_source('rgbd');});
  $('sensor-once').addEventListener('click',()=>select_source('rgbd'));
  $('lidar-enable').addEventListener('change',()=>{if(lidar.config.enabled)select_source('mid360');});
  $('lidar-once').addEventListener('click',()=>select_source('mid360'));
  $('object-capture-live').addEventListener('click',()=>{if(this.object_capture.is_running)select_source($('object-capture-source').value);});
  $('object-capture-once').addEventListener('click',()=>select_source($('object-capture-source').value));
  $('object-capture-source').addEventListener('change',()=>{if(this.object_capture.is_running)select_source($('object-capture-source').value);});
  this.robot_panel=new RosRobotPanel({rgbd,toast});
 }
 update_button(){$('ros-start').textContent=this.is_running?'送信を停止':'連続送信';if(!this.is_running)this.robot_panel.set_connection_state('points',false);}
 latest_frame(source){
  if(source==='mid360')return this.lidar.config.enabled?this.lidar.last:null;
  if(source==='rgbd')return this.rgbd.last_scene_frame??null;
  return this.object_capture.latest(source);
 }
 tick(now){
  this.robot_panel.tick(now);this.object_capture.tick(now);
  if(!this.is_running||this.is_busy)return;
  for(let offset=0;offset<sources.length;offset++){
   const idx=(this.next_source_idx+offset)%sources.length,source=sources[idx],frame=this.latest_frame(source);
   if($('ros-send-'+source).checked&&frame&&frame!==this.last_sent_frames[source]){this.next_source_idx=(idx+1)%sources.length;this.send(source);return;}
  }
 }
 async send(source){
  if(!sources.includes(source)||!$('ros-send-'+source).checked||this.is_busy)return null;
  this.is_busy=true;const generation=this.generation;
  try{
   const endpoint=new URL($('ros-endpoint').value);if(!['http:','https:'].includes(endpoint.protocol))throw Error('送信先はHTTPのURLを指定してください');
   const captured=this.latest_frame(source);
   if(!captured){$('ros-status').textContent='取得済み点群がありません。センサまたは環境タブで取得してください';return null;}
   const is_object=source==='object_full'||source==='object_visible',frame=is_object?captured.frame:captured;
   const item=is_object?captured.item:null,object_pose=is_object?captured.object_pose:null;
   const robot_state=structuredClone(frame.robot_state),robot_pose=frame.robotPose,robot_model=frame.robotModel,captured_at_ms=robot_state.captured_at_ms;
   let points,depth_frame,lidar_frame,color_frame;
   if(source==='object_full')points=frame.points.slice();
   else if(source==='mid360'){
    lidar_frame=frame;attach_lidar(robot_state,frame.pose);points=worldPoints(frame.xyz,frame.pose);
   }else{
    if(source==='rgbd'&&$('ros-depth').checked)depth_frame=frame;
    color_frame=frame;attach_camera(robot_state,frame.depthWorld);points=worldPoints(frame.xyz,frame.depthWorld);
   }
   points_to_base(points,robot_state);
   const meta={robot_state,source,frame_id:'base_footprint',count:points.length/3,captured_at_ms,robot_pose,robot_model,object_id:is_object?item.id:null,object_to_world:is_object?object_pose:null};
   if(lidar_frame){meta.captured_at_ms=robot_state.captured_at_ms;meta.lidar={sensor_type:lidar_frame.config.sensor_type||'mid360',frame_id:'sim_mid360_frame',scan_start_sec:lidar_frame.scanStart,duration_sec:lidar_frame.config.duration,num_slots:lidar_frame.config.beams,scan_pattern:lidar_frame.config.scanPattern};}
   let color_data;
   if(color_frame){
    meta.color_format='rgb8_valid8';color_data=new Uint8Array(meta.count*4);
    for(let idx=0;idx<meta.count;idx++){color_data.set(color_frame.colors.subarray(idx*3,idx*3+3),idx*4);color_data[idx*4+3]=color_frame.colorValid[idx];}
   }
   if(color_frame){
    const status_response=await fetch(new URL('/api/points/status',endpoint),{signal:AbortSignal.timeout(5000)});
    const status=await status_response.json();
    if(!status_response.ok||(status.protocol_version??1)<2)throw Error('ROSブリッジが旧版です。bash start_ros.sh --restartで再起動してください');
   }
   let body;
   if(depth_frame){
    meta.depth_image={...depth_frame.calibration.depth,optical_to_world:depth_frame.depthWorld};
    const packed=encodeInput(meta,points);body=new Blob([packed,depth_frame.depth,...(color_data?[color_data]:[])]);
   }else body=color_data?new Blob([encodeInput(meta,points),color_data]):encodeInput(meta,points);
   if(generation!==this.generation||!$('ros-send-'+source).checked)return null;
   const response=await fetch(new URL('/api/points',endpoint),{method:'POST',headers:{'Content-Type':'application/octet-stream','X-ToPo-Points':'1'},body,signal:AbortSignal.timeout(10000)});
   if(!response.ok)throw Error(await response.text());const result=await response.json();
   if(generation===this.generation){this.last_sent_frames[source]=captured;this.robot_panel.set_connection_state('points',this.is_running);}
   if(generation===this.generation)$('ros-status').textContent=`${result.topic}${result.depth_topics?'\n'+result.depth_topics.join('\n'):''}\n${meta.count.toLocaleString()} 点送信済み\nframe: ${meta.frame_id}`;
   return result;
  }catch(error){this.is_running=false;this.update_button();$('ros-status').textContent='送信エラー：'+error.message+'\npointcloud_bridge.py の起動と送信先を確認してください';return null;}
  finally{this.is_busy=false;}
 }
}
