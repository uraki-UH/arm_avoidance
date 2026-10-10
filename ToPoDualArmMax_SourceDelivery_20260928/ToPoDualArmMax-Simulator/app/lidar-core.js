import {raycast_surface} from './lidar-raycast.js';
import {jt128_direction,jt128_channels,jt128_scan} from './jt128-scan.js';
import {measuredDirection} from './measured-scan.js';
import * as THREE from './vendor/three/build/three.module.js';
import {MeshBVH,SAH} from './vendor/three-mesh-bvh/index.module.js';
const rad=Math.PI/180;
export const PETAL_SCAN=Object.freeze({model:'quasiperiodic-rosette-v1',azimuthHz:10,petalHz:10*Math.sqrt(50),azimuthWobbleRad:.42,elevationCenterDeg:22.5,elevationAmplitudeDeg:29.5,nominalRaysPerSecond:200000});
export const waistLidarMount=()=>({parent:'torso_link',position:[.07769,0,.105],rpy:[0,45*rad,0]});
export const defaultLidarConfig=()=>({sensor_type:'mid360',enabled:false,...waistLidarMount(),scanPattern:'measured',capture_hz:10,duration:.1,beams:20000,minRange:.1,maxRange:40,mode:'ideal',noiseSigma:.02,seed:12345});
export function lidar_preset(sensor_type='mid360'){if(sensor_type==='mid360')return defaultLidarConfig();if(sensor_type!=='jt128')throw Error('未対応のLiDARです');return {...defaultLidarConfig(),sensor_type,scanPattern:'jt128',minRange:0,maxRange:60,beams:115200,rpy:[0,0,0]};}
export function lidar_ray_rate(c){return c.sensor_type==='jt128'?jt128_scan.nominal_rays_per_sec:200000;}
export function validateLidarConfig(v){const c={...lidar_preset(v?.sensor_type||'mid360'),...structuredClone(v)};const f3=a=>Array.isArray(a)&&a.length===3&&a.every(Number.isFinite);if(!['torso_link','base_footprint','neck_pan_link','neck_tilt_link','world'].includes(c.parent)||!f3(c.position)||c.position.some(x=>Math.abs(x)>100)||!f3(c.rpy)||![c.capture_hz,c.duration,c.beams,c.minRange,c.maxRange,c.noiseSigma,c.seed].every(Number.isFinite)||c.capture_hz<.1||c.capture_hz>40||c.duration<.01||c.duration>1||!Number.isInteger(c.beams)||c.beams<100||c.beams>(c.sensor_type==='jt128'?1152000:200000)||c.minRange<(c.sensor_type==='jt128'?0:.1)||c.maxRange>(c.sensor_type==='jt128'?60:100)||c.maxRange<=c.minRange||c.noiseSigma<0||c.noiseSigma>.1||!['ideal','noise'].includes(c.mode)||!(c.sensor_type==='jt128'?['jt128']:['measured','petal','low-discrepancy']).includes(c.scanPattern))throw Error('LiDARの設定値が範囲外です');return c;}
export function scanDirection(i,out=new THREE.Vector3(),pattern='petal',timeSeconds=i/PETAL_SCAN.nominalRaysPerSecond){
 if(pattern==='jt128')return jt128_direction(i,out);
 if(pattern==='measured')return measuredDirection(i,out);
 // 非周期の花びら軌跡の数式近似。Livoxの個体校正とは独立
 let a,e;
 if(pattern==='low-discrepancy'){a=2*Math.PI*((i*.6180339887498949)%1);e=(-7+59*((i*.4142135623730951)%1))*rad;}
 else {const phase=2*Math.PI*PETAL_SCAN.petalHz*timeSeconds;a=2*Math.PI*PETAL_SCAN.azimuthHz*timeSeconds+PETAL_SCAN.azimuthWobbleRad*Math.sin(phase);e=(PETAL_SCAN.elevationCenterDeg+PETAL_SCAN.elevationAmplitudeDeg*Math.cos(phase))*rad;}
 const ce=Math.cos(e);
 return out.set(ce*Math.cos(a),ce*Math.sin(a),Math.sin(e));
}
export function buildGeometry(d){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(new Float32Array(d.position),3));if(d.index)g.setIndex(new THREE.BufferAttribute(new Uint32Array(d.index),1));g.computeBoundingBox();g.boundsTree=new MeshBVH(g,{maxLeafSize:8,strategy:SAH});return g;}
export function packedPositions(a){const result=new Float32Array(a.count*3);for(let i=0;i<a.count;i++){result[3*i]=a.getX(i);result[3*i+1]=a.getY(i);result[3*i+2]=a.getZ(i);}return result;}
export function prepareMeshes(descriptors,geometries){return descriptors.map(d=>{const matrix=new THREE.Matrix4().fromArray(d.matrix),geometry=geometries.get(d.geometry),inverse=matrix.clone().invert();return{...d,matrix,inverse,geometry,box:geometry.boundingBox.clone().applyMatrix4(matrix)};});}
// フレーム内のメッシュ外枠索引。三角形BVHの前段で交差候補のみ抽出
const mesh_queries=new WeakMap();
function mesh_query(meshes){
 let query=mesh_queries.get(meshes);if(query)return query;
 const build=indices=>{
  const box=new THREE.Box3();for(const idx of indices)box.union(meshes[idx].box);
  if(indices.length<=4)return {box,indices};
  const size=box.getSize(new THREE.Vector3()),axis=size.x>=size.y&&size.x>=size.z?'x':size.y>=size.z?'y':'z';
  indices.sort((a,b)=>(meshes[a].box.min[axis]+meshes[a].box.max[axis])-(meshes[b].box.min[axis]+meshes[b].box.max[axis]));
  const middle=Math.floor(indices.length/2);return {box,left:build(indices.slice(0,middle)),right:build(indices.slice(middle))};
 };
 query={root:meshes.length?build(meshes.map((_,idx)=>idx)):null,ray:new THREE.Ray(),local:new THREE.Ray(),point:new THREE.Vector3(),stack:[]};
 mesh_queries.set(meshes,query);return query;
}
export function firstSurface(origin,direction,meshes,maxRange=100){
 const query=mesh_query(meshes);if(!query.root)return null;
 const {ray,local,point,stack}=query;ray.set(origin,direction);stack.length=0;stack.push(query.root);
 let best=null,dist=maxRange,best_idx=Infinity;
 while(stack.length){
  const branch=stack.pop();if(!ray.intersectsBox(branch.box))continue;
  if(!branch.indices){stack.push(branch.right,branch.left);continue;}
  for(const idx of branch.indices){
   const mesh=meshes[idx];if(!ray.intersectsBox(mesh.box))continue;
   local.copy(ray).applyMatrix4(mesh.inverse);
   // レイ方向に沿った距離換算。非一様スケール・せん断への対応
   const e=mesh.inverse.elements,dx=direction.x,dy=direction.y,dz=direction.z;
   const local_scale=Math.hypot(e[0]*dx+e[4]*dy+e[8]*dz,e[1]*dx+e[5]*dy+e[9]*dz,e[2]*dx+e[6]*dy+e[10]*dz);
   if(!raycast_surface(mesh.geometry,local,dist*local_scale+1e-7,point))continue;
   point.applyMatrix4(mesh.matrix);const value=point.distanceTo(origin);
   // 同距離の表面は従来のメッシュ順を優先。最大距離境界も従来どおり
   if(value<dist||(best&&value===dist&&idx<best_idx)){dist=value;best_idx=idx;best={distance:value,id:mesh.id,reflectance:mesh.reflectance??.5};}
  }
 }
 return best;
}
export function scan(c,pose,meshes,start=0,startTime=start/PETAL_SCAN.nominalRaysPerSecond){
 const begin=performance.now(),matrix=new THREE.Matrix4().fromArray(pose),origin=new THREE.Vector3().setFromMatrixPosition(matrix),q=new THREE.Quaternion().setFromRotationMatrix(matrix),direction=new THREE.Vector3(),worldDirection=new THREE.Vector3();
 const xyz=new Float32Array(c.beams*3),range=new Float32Array(c.beams),time=new Float32Array(c.beams),beamIndex=new Uint32Array(c.beams),objectId=new Uint32Array(c.beams),reflectance=new Uint8Array(c.beams);const slotStatus=new Uint8Array(c.beams),slotRange=new Float32Array(c.beams);let count=0,nearRejected=0,unknownDirections=0,noReturn=0,rangeRejected=0,seed=c.seed>>>0;
 const random=()=>{seed=(1664525*seed+1013904223)>>>0;return(seed+.5)/4294967296;};
 for(let i=0;i<c.beams;i++){scanDirection(start+i,direction,c.scanPattern,startTime+i*c.duration/c.beams);if(direction.lengthSq()===0){unknownDirections++;slotStatus[i]=1;continue;}worldDirection.copy(direction).applyQuaternion(q);const hit=firstSurface(origin,worldDirection,meshes,c.maxRange);if(!hit){noReturn++;slotStatus[i]=2;continue;}const min_range=c.sensor_type==='jt128'?Math.max(c.minRange,jt128_channels[(start+i)%128][2]):c.minRange;if(hit.distance<min_range){nearRejected++;slotStatus[i]=3;continue;}let r=hit.distance;if(c.mode==='noise')r+=c.noiseSigma*Math.sqrt(-2*Math.log(random()))*Math.cos(2*Math.PI*random());if(r<min_range||r>c.maxRange){rangeRejected++;slotStatus[i]=4;continue;}slotRange[i]=r;
  xyz[count*3]=direction.x*r;xyz[count*3+1]=direction.y*r;xyz[count*3+2]=direction.z*r;range[count]=r;time[count]=i*c.duration/c.beams;beamIndex[count]=i;objectId[count]=hit.id;reflectance[count]=Math.round(255*hit.reflectance);count++;
 }
 return{xyz:xyz.slice(0,count*3),range:range.slice(0,count),time:time.slice(0,count),beamIndex:beamIndex.slice(0,count),objectId:objectId.slice(0,count),reflectance:reflectance.slice(0,count),count,nearRejected,unknownDirections,noReturn,rangeRejected,slotStatus,slotRange,ms:performance.now()-begin};
}
export function lidarPLY(f,world=true){const header=new TextEncoder().encode(`ply\nformat binary_little_endian 1.0\ncomment Units m; reflectance is synthetic display albedo, not calibrated IR reflectivity\nelement vertex ${f.count}\nproperty float x\nproperty float y\nproperty float z\nproperty float range\nproperty float time_offset\nproperty uint beam_index\nproperty uint object_id\nproperty uchar reflectance\nend_header\n`),buffer=new ArrayBuffer(header.length+f.count*29);new Uint8Array(buffer).set(header);const d=new DataView(buffer),p=new THREE.Vector3(),m=new THREE.Matrix4().fromArray(f.pose);for(let i=0;i<f.count;i++){const o=header.length+i*29;p.fromArray(f.xyz,i*3);if(world)p.applyMatrix4(m);d.setFloat32(o,p.x,true);d.setFloat32(o+4,p.y,true);d.setFloat32(o+8,p.z,true);d.setFloat32(o+12,f.range[i],true);d.setFloat32(o+16,f.time[i],true);d.setUint32(o+20,f.beamIndex[i],true);d.setUint32(o+24,f.objectId[i],true);d.setUint8(o+28,f.reflectance[i]);}return buffer;}
