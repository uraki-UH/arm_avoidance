import {measuredDirection} from './measured-scan.js';
import * as THREE from './vendor/three/build/three.module.js';
import {MeshBVH} from './vendor/three-mesh-bvh/index.module.js';
const rad=Math.PI/180;
export const PETAL_SCAN=Object.freeze({model:'quasiperiodic-rosette-v1',azimuthHz:10,petalHz:10*Math.sqrt(50),azimuthWobbleRad:.42,elevationCenterDeg:22.5,elevationAmplitudeDeg:29.5,nominalRaysPerSecond:200000});
export const waistLidarMount=()=>({parent:'torso_link',position:[.07769,0,.105],rpy:[0,45*rad,0]});
export const defaultLidarConfig=()=>({enabled:false,...waistLidarMount(),scanPattern:'measured',capture_hz:10,duration:.1,beams:20000,minRange:.1,maxRange:40,mode:'ideal',noiseSigma:.02,seed:12345});
export function validateLidarConfig(v){const c={...defaultLidarConfig(),...structuredClone(v)};const f3=a=>Array.isArray(a)&&a.length===3&&a.every(Number.isFinite);if(!['torso_link','base_footprint','neck_pan_link','neck_tilt_link','world'].includes(c.parent)||!f3(c.position)||c.position.some(x=>Math.abs(x)>100)||!f3(c.rpy)||![c.capture_hz,c.duration,c.beams,c.minRange,c.maxRange,c.noiseSigma,c.seed].every(Number.isFinite)||c.capture_hz<.1||c.capture_hz>40||c.duration<.01||c.duration>1||!Number.isInteger(c.beams)||c.beams<100||c.beams>200000||c.minRange<.1||c.maxRange>100||c.maxRange<=c.minRange||c.noiseSigma<0||c.noiseSigma>.1||!['ideal','noise'].includes(c.mode)||!['measured','petal','low-discrepancy'].includes(c.scanPattern))throw Error('Mid-360の設定値が範囲外です');return c;}
export function scanDirection(i,out=new THREE.Vector3(),pattern='petal',timeSeconds=i/PETAL_SCAN.nominalRaysPerSecond){
 if(pattern==='measured')return measuredDirection(i,out);
 // Continuous quasi-periodic angular rosette, an imitation, not a Livox calibration.
 let a,e;
 if(pattern==='low-discrepancy'){a=2*Math.PI*((i*.6180339887498949)%1);e=(-7+59*((i*.4142135623730951)%1))*rad;}
 else {const phase=2*Math.PI*PETAL_SCAN.petalHz*timeSeconds;a=2*Math.PI*PETAL_SCAN.azimuthHz*timeSeconds+PETAL_SCAN.azimuthWobbleRad*Math.sin(phase);e=(PETAL_SCAN.elevationCenterDeg+PETAL_SCAN.elevationAmplitudeDeg*Math.cos(phase))*rad;}
 const ce=Math.cos(e);
 return out.set(ce*Math.cos(a),ce*Math.sin(a),Math.sin(e));
}
export function buildGeometry(d){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(new Float32Array(d.position),3));if(d.index)g.setIndex(new THREE.BufferAttribute(new Uint32Array(d.index),1));g.computeBoundingBox();g.boundsTree=new MeshBVH(g,{maxLeafSize:8});return g;}
export function packedPositions(a){const result=new Float32Array(a.count*3);for(let i=0;i<a.count;i++){result[3*i]=a.getX(i);result[3*i+1]=a.getY(i);result[3*i+2]=a.getZ(i);}return result;}
export function prepareMeshes(descriptors,geometries){return descriptors.map(d=>{const matrix=new THREE.Matrix4().fromArray(d.matrix),geometry=geometries.get(d.geometry),inverse=matrix.clone().invert();return{...d,matrix,inverse,geometry,box:geometry.boundingBox.clone().applyMatrix4(matrix)};});}
export function firstSurface(origin,direction,meshes,maxRange=100){const ray=new THREE.Ray(origin,direction),local=new THREE.Ray(),point=new THREE.Vector3();let best=null,distance=maxRange;
 for(const m of meshes){if(!ray.intersectsBox(m.box))continue;local.copy(ray).applyMatrix4(m.inverse);const hit=m.geometry.boundsTree.raycastFirst(local,THREE.DoubleSide,0,Infinity);if(!hit)continue;point.copy(hit.point).applyMatrix4(m.matrix);const d=point.distanceTo(origin);if(d<distance){distance=d;best={distance:d,id:m.id,reflectance:m.reflectance??.5};}}
 return best;
}
export function scan(c,pose,meshes,start=0,startTime=start/PETAL_SCAN.nominalRaysPerSecond){
 const begin=performance.now(),matrix=new THREE.Matrix4().fromArray(pose),origin=new THREE.Vector3().setFromMatrixPosition(matrix),q=new THREE.Quaternion().setFromRotationMatrix(matrix),direction=new THREE.Vector3(),worldDirection=new THREE.Vector3();
 const xyz=new Float32Array(c.beams*3),range=new Float32Array(c.beams),time=new Float32Array(c.beams),beamIndex=new Uint32Array(c.beams),objectId=new Uint32Array(c.beams),reflectance=new Uint8Array(c.beams);const slotStatus=new Uint8Array(c.beams),slotRange=new Float32Array(c.beams);let count=0,nearRejected=0,unknownDirections=0,noReturn=0,rangeRejected=0,seed=c.seed>>>0;
 const random=()=>{seed=(1664525*seed+1013904223)>>>0;return(seed+.5)/4294967296;};
 for(let i=0;i<c.beams;i++){scanDirection(start+i,direction,c.scanPattern,startTime+i*c.duration/c.beams);if(direction.lengthSq()===0){unknownDirections++;slotStatus[i]=1;continue;}worldDirection.copy(direction).applyQuaternion(q);const hit=firstSurface(origin,worldDirection,meshes,c.maxRange);if(!hit){noReturn++;slotStatus[i]=2;continue;}if(hit.distance<c.minRange){nearRejected++;slotStatus[i]=3;continue;}let r=hit.distance;if(c.mode==='noise')r+=c.noiseSigma*Math.sqrt(-2*Math.log(random()))*Math.cos(2*Math.PI*random());if(r<c.minRange||r>c.maxRange){rangeRejected++;slotStatus[i]=4;continue;}slotRange[i]=r;
  xyz[count*3]=direction.x*r;xyz[count*3+1]=direction.y*r;xyz[count*3+2]=direction.z*r;range[count]=r;time[count]=i*c.duration/c.beams;beamIndex[count]=i;objectId[count]=hit.id;reflectance[count]=Math.round(255*hit.reflectance);count++;
 }
 return{xyz:xyz.slice(0,count*3),range:range.slice(0,count),time:time.slice(0,count),beamIndex:beamIndex.slice(0,count),objectId:objectId.slice(0,count),reflectance:reflectance.slice(0,count),count,nearRejected,unknownDirections,noReturn,rangeRejected,slotStatus,slotRange,ms:performance.now()-begin};
}
export function lidarPLY(f,world=true){const header=new TextEncoder().encode(`ply\nformat binary_little_endian 1.0\ncomment Units m; reflectance is synthetic display albedo, not calibrated IR reflectivity\nelement vertex ${f.count}\nproperty float x\nproperty float y\nproperty float z\nproperty float range\nproperty float time_offset\nproperty uint beam_index\nproperty uint object_id\nproperty uchar reflectance\nend_header\n`),buffer=new ArrayBuffer(header.length+f.count*29);new Uint8Array(buffer).set(header);const d=new DataView(buffer),p=new THREE.Vector3(),m=new THREE.Matrix4().fromArray(f.pose);for(let i=0;i<f.count;i++){const o=header.length+i*29;p.fromArray(f.xyz,i*3);if(world)p.applyMatrix4(m);d.setFloat32(o,p.x,true);d.setFloat32(o+4,p.y,true);d.setFloat32(o+8,p.z,true);d.setFloat32(o+12,f.range[i],true);d.setFloat32(o+16,f.time[i],true);d.setUint32(o+20,f.beamIndex[i],true);d.setUint32(o+24,f.objectId[i],true);d.setUint8(o+28,f.reflectance[i]);}return buffer;}
