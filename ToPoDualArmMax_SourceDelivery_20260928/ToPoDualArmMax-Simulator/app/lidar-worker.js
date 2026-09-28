import {loadMeasuredScan} from './measured-scan.js';
import {buildGeometry,prepareMeshes,scan} from './lidar-core.js';
const geometries=new Map();
self.onmessage=async e=>{try{const d=e.data;if(d.config.scanPattern==='measured')await loadMeasuredScan();for(const g of d.geometries)geometries.set(g.id,buildGeometry(g));const used=new Set(d.meshes.map(m=>m.geometry));for(const [id,g]of geometries)if(!used.has(id)){g.dispose();geometries.delete(id);}const frame=scan(d.config,d.pose,prepareMeshes(d.meshes,geometries),d.start,d.startTime);self.postMessage({id:d.id,frame},Object.values(frame).filter(ArrayBuffer.isView).map(v=>v.buffer));}catch(e){self.postMessage({id:e.data.id,error:e.message});}};
