import * as THREE from 'three';

export const robot_frames=Object.freeze({world:'world',base:'base_footprint',camera:'sim_camera_depth_optical_frame',lidar:'sim_mid360_frame'});
export const pose_sources=Object.freeze({simulator:'simulator',ros:'ros',leader:'leader'});
const joint_topologies=new WeakMap();

// HTTPブリッジに対応するWebSocket接続先。既定HTTPポートと明示ポートの共通規則
export function bridge_socket_url(endpoint,path){
 const url=new URL(endpoint);
 if(!['http:','https:'].includes(url.protocol))throw Error('接続先はHTTPまたはHTTPSのURLを指定してください');
 const is_secure=url.protocol==='https:',port=Number(url.port||(is_secure?443:80));
 if(port>=65535)throw Error('WebSocket用ポートを確保できません');
 url.protocol=is_secure?'wss:':'ws:';url.port=String(port+1);url.pathname=path;url.search='';url.hash='';return url;
}

// 座標行列の一回適用。入力点群の保持と、明示出力配列への書込み
export function transform_points(points,matrix,output=new Float32Array(points.length)){
 const values=matrix.elements??matrix;
 for(let idx=0;idx<points.length;idx+=3){
  const x=points[idx],y=points[idx+1],z=points[idx+2];
  output[idx]=values[0]*x+values[4]*y+values[8]*z+values[12];
  output[idx+1]=values[1]*x+values[5]*y+values[9]*z+values[13];
  output[idx+2]=values[2]*x+values[6]*y+values[10]*z+values[14];
 }
 return output;
}

// 取得時のセンサ座標からロボット基準への変換。中間world点群の生成なし
export function points_in_base(points,sensor_to_world,state){
 const matrix=new THREE.Matrix4().fromArray(state.base_to_world).invert().multiply(new THREE.Matrix4().fromArray(sensor_to_world));
 return transform_points(points,matrix);
}

export function transform_record(parent,child,matrix){
 const position=new THREE.Vector3(),rotation=new THREE.Quaternion(),scale=new THREE.Vector3();matrix.decompose(position,rotation,scale);
 return {parent,child,translation:position.toArray(),rotation:rotation.toArray()};
}
export function robot_snapshot(robot){
 robot.updateWorldMatrix(true,true);
 const base=robot.links[robot_frames.base].matrixWorld.clone(),transforms=[transform_record(robot_frames.world,robot_frames.base,base)];
 if(!joint_topologies.has(robot)){
  const topology=[];
  for(const element of robot.urdf.children){
   if(element.tagName!=='joint')continue;
   const parent=element.querySelector('parent').getAttribute('link'),child=element.querySelector('child').getAttribute('link');
   topology.push({parent,child,parent_link:robot.links[parent],child_link:robot.links[child]});
  }
  joint_topologies.set(robot,topology);
 }
 for(const {parent,child,parent_link,child_link} of joint_topologies.get(robot)){
  transforms.push(transform_record(parent,child,parent_link.matrixWorld.clone().invert().multiply(child_link.matrixWorld)));
 }
 return {robot_model:robot.modelId,robot_pose:robot.getPose(),captured_at_ms:Date.now(),base_to_world:base.toArray(),transforms};
}
export function attach_camera(state,depth_world){
 const matrix=new THREE.Matrix4().fromArray(state.base_to_world).invert().multiply(new THREE.Matrix4().fromArray(depth_world));
 state.transforms.push(transform_record(robot_frames.base,robot_frames.camera,matrix));
}
export function attach_lidar(state,lidar_world){
 const matrix=new THREE.Matrix4().fromArray(state.base_to_world).invert().multiply(new THREE.Matrix4().fromArray(lidar_world));
 state.transforms.push(transform_record(robot_frames.base,robot_frames.lidar,matrix));
}
export function points_to_base(points,state){
 return transform_points(points,new THREE.Matrix4().fromArray(state.base_to_world).invert(),points);
}
