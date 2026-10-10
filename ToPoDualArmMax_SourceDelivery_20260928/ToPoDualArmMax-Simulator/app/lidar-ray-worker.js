import * as THREE from './vendor/three/build/three.module.js';
import {loadMeasuredScan} from './measured-scan.js';
import {MeshBVH} from './vendor/three-mesh-bvh/index.module.js';
import {prepareMeshes,firstSurface,scanDirection} from './lidar-core.js';

// 親Worker構築済みBVHの複製と、割り当てられた走査方向だけの交点計算
const geometries=new Map();
self.onmessage=async event=>{
 const d=event.data;
 try{
  if(d.geometries){
   for(const geometry of geometries.values())geometry.dispose();geometries.clear();
   for(const value of d.geometries){
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(value.position,3));
    geometry.boundsTree=MeshBVH.deserialize(value.bvh,geometry);geometry.computeBoundingBox();geometries.set(value.id,geometry);
   }
  }
  if(d.config.scanPattern==='measured')await loadMeasuredScan();
  const matrix=new THREE.Matrix4().fromArray(d.pose),rotation=new THREE.Quaternion().setFromRotationMatrix(matrix),meshes=prepareMeshes(d.meshes,geometries),origin=new THREE.Vector3().setFromMatrixPosition(matrix),direction=new THREE.Vector3(),num_rays=d.num_rays,distances=new Float64Array(num_rays),object_ids=new Uint32Array(num_rays),beam_directions=new Float64Array(num_rays*3),reflectances=new Uint8Array(num_rays);distances.fill(Infinity);
  for(let idx=0;idx<num_rays;idx++){
   const beam_idx=d.offset+idx*d.step;scanDirection(d.start+beam_idx,direction,d.config.scanPattern,d.start_time+beam_idx*d.config.duration/d.config.beams);beam_directions[idx*3]=direction.x;beam_directions[idx*3+1]=direction.y;beam_directions[idx*3+2]=direction.z;if(direction.lengthSq()===0)continue;direction.applyQuaternion(rotation);
   const hit=firstSurface(origin,direction,meshes,d.config.maxRange);if(hit){distances[idx]=hit.distance;object_ids[idx]=hit.id;reflectances[idx]=Math.round(255*hit.reflectance);}
  }
  self.postMessage({id:d.id,distances,object_ids,beam_directions,reflectances},[distances.buffer,object_ids.buffer,beam_directions.buffer,reflectances.buffer]);
 }catch(error){self.postMessage({id:d.id,error:error.message});}
};
