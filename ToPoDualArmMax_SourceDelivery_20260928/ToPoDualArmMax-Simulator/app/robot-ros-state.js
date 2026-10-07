import * as THREE from 'three';

export function transform_record(parent,child,matrix){
 const position=new THREE.Vector3(),rotation=new THREE.Quaternion(),scale=new THREE.Vector3();matrix.decompose(position,rotation,scale);
 return {parent,child,translation:position.toArray(),rotation:rotation.toArray()};
}
export function robot_snapshot(robot){
 robot.updateWorldMatrix(true,true);
 const base=robot.links.base_footprint.matrixWorld.clone(),transforms=[transform_record('world','base_footprint',base)];
 for(const element of robot.urdf.children){
  if(element.tagName!=='joint')continue;
  const parent=element.querySelector('parent').getAttribute('link'),child=element.querySelector('child').getAttribute('link');
  transforms.push(transform_record(parent,child,robot.links[parent].matrixWorld.clone().invert().multiply(robot.links[child].matrixWorld)));
 }
 return {robot_model:robot.modelId,robot_pose:robot.getPose(),captured_at_ms:Date.now(),base_to_world:base.toArray(),transforms};
}
export function attach_camera(state,depth_world){
 const matrix=new THREE.Matrix4().fromArray(state.base_to_world).invert().multiply(new THREE.Matrix4().fromArray(depth_world));
 state.transforms.push(transform_record('base_footprint','sim_camera_depth_optical_frame',matrix));
}
export function attach_lidar(state,lidar_world){
 const matrix=new THREE.Matrix4().fromArray(state.base_to_world).invert().multiply(new THREE.Matrix4().fromArray(lidar_world));
 state.transforms.push(transform_record('base_footprint','sim_mid360_frame',matrix));
}
export function points_to_base(points,state){
 const matrix=new THREE.Matrix4().fromArray(state.base_to_world).invert(),point=new THREE.Vector3();
 for(let idx=0;idx<points.length;idx+=3){point.fromArray(points,idx).applyMatrix4(matrix).toArray(points,idx);}return points;
}
