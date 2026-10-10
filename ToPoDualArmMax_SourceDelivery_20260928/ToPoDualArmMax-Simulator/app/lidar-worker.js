import {loadMeasuredScan} from './measured-scan.js';
import {buildGeometry,prepareMeshes,scan} from './lidar-core.js';
import {lidar_cpu_pool} from './lidar-cpu-pool.js';
const geometries=new Map();let pool=null,has_failed_pool=false;
self.onmessage=async e=>{
 try{
  const d=e.data;if(d.config.scanPattern==='measured')await loadMeasuredScan();
  for(const g of d.geometries)geometries.set(g.id,buildGeometry(g));
  const used=new Set(d.meshes.map(m=>m.geometry));for(const [id,g]of geometries)if(!used.has(id)){g.dispose();geometries.delete(id);}
  const meshes=prepareMeshes(d.meshes,geometries);let frame;
  if(['auto','cpu_parallel',undefined].includes(d.config.compute_backend)&&!has_failed_pool){
   try{const num_workers=d.config.num_workers??Math.min(d.config.sensor_type==='jt128'?16:4,Math.max(1,(navigator.hardwareConcurrency||2)-2));pool??=new lidar_cpu_pool(num_workers);frame=await pool.capture(d.config,d.pose,d.meshes,meshes,geometries,d.start,d.startTime);}
   catch(error){pool?.dispose();pool=null;has_failed_pool=true;frame={...scan(d.config,d.pose,meshes,d.start,d.startTime),compute_backend:'cpu',pool_error:error.message};}
  }else frame={...scan(d.config,d.pose,meshes,d.start,d.startTime),compute_backend:'cpu'};
  self.postMessage({id:d.id,frame},Object.values(frame).filter(ArrayBuffer.isView).map(v=>v.buffer));
 }catch(error){self.postMessage({id:e.data.id,error:error.message});}
};
