import {MeshBVH} from './vendor/three-mesh-bvh/index.module.js';
import {frame_from_hits} from './lidar-frame.js';
import {PETAL_SCAN} from './lidar-core.js';

// 走査方向の分配と複数WorkerでのBVH複製利用。乱数・出力順は親側で統一
export class lidar_cpu_pool {
 constructor(num_workers=Math.min(16,Math.max(1,(navigator.hardwareConcurrency||2)-2))){
  if(!Number.isInteger(num_workers)||num_workers<1||num_workers>16)throw Error('LiDAR並列数は1〜16を指定してください');
  this.num_workers=num_workers;this.geometry_keys=[];this.sequence=0;
  this.entries=Array.from({length:num_workers},()=>{
   const worker=new Worker(new URL('./lidar-ray-worker.js',import.meta.url),{type:'module'}),entry={worker,pending:new Map()};
   worker.onmessage=event=>{const d=event.data,request=entry.pending.get(d.id);if(!request)return;entry.pending.delete(d.id);clearTimeout(request.timer);d.error?request.reject(Error(d.error)):request.resolve(d);};
   worker.onerror=event=>{for(const request of entry.pending.values()){clearTimeout(request.timer);request.reject(Error(event.message));}entry.pending.clear();};
   return entry;
  });
 }
 request(entry,data){return new Promise((resolve,reject)=>{const id=++this.sequence,timer=setTimeout(()=>{entry.pending.delete(id);reject(Error('LiDAR並列計算の応答がありません'));},30000);entry.pending.set(id,{resolve,reject,timer});entry.worker.postMessage({...data,id});});}
 async capture(c,pose,descriptors,meshes,geometries,start=0,start_time=start/PETAL_SCAN.nominalRaysPerSecond){
  const begin=performance.now(),keys=[...geometries.values()],has_geometry_change=keys.length!==this.geometry_keys.length||keys.some((geometry,idx)=>geometry!==this.geometry_keys[idx]);
  const geometry_data=has_geometry_change?[...geometries].map(([id,geometry])=>({id,position:geometry.attributes.position.array,bvh:MeshBVH.serialize(geometry.boundsTree,{cloneBuffers:false})})):undefined;
  const query_start=performance.now();
  const responses_pending=Promise.all(this.entries.map((entry,idx)=>this.request(entry,{geometries:geometry_data,meshes:descriptors,pose,config:c,start,start_time,offset:idx,step:this.num_workers,num_rays:Math.ceil((c.beams-idx)/this.num_workers)})));
  const prepare_ms=performance.now()-begin,responses=await responses_pending;
  const query_ms=performance.now()-query_start;this.geometry_keys=keys;
  const frame=frame_from_hits(c,responses,start);
  return {...frame,ms:performance.now()-begin,compute_backend:'cpu_parallel',num_workers:this.num_workers,prepare_ms,query_ms,output_ms:frame.ms};
 }
 dispose(){for(const entry of this.entries){entry.worker.terminate();for(const request of entry.pending.values()){clearTimeout(request.timer);request.reject(Error('LiDAR並列計算の停止'));}entry.pending.clear();}this.entries=[];}
}
