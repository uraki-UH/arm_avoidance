import test from 'node:test';
import assert from 'node:assert/strict';
import net from 'node:net';
import path from 'node:path';
import fs from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
test('Relocatable server, both models, exports, origin and path restrictions',async()=>{
 const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');const port=probe.address().port;await new Promise(r=>probe.close(r));
 const child=spawn(process.execPath,[path.join(root,'app/server.mjs')],{cwd:root,env:{...process.env,PORT:String(port)},stdio:'pipe',windowsHide:true});
 let logs='';child.stdout.on('data',d=>logs+=d);child.stderr.on('data',d=>logs+=d);
 const base='http://127.0.0.1:'+port;const created=[];
 try{
  let health;for(let i=0;i<100;i++){try{health=await fetch(base+'/api/health').then(r=>r.json());break;}catch{await new Promise(r=>setTimeout(r,50));}}
  assert.equal(health?.app,'topo-motion-studio',logs);assert.match(health.instance,/^[0-9a-f]{16}$/);
  for(const url of ['/','/models.js','/source.urdf','/models/standard/source.urdf','/assets/mid360/measured-directions.f32','/assets/vehicles/CarConcept.glb','/vendor/three/examples/jsm/libs/draco/gltf/draco_decoder.wasm']){
   const r=await fetch(base+url,{method:'HEAD'});assert.equal(r.status,200,url);assert.ok(Number(r.headers.get('content-length'))>0,url);
  }
  for(const file of ['assets.json','models/standard/assets.json']){
   const data=await fetch(base+'/'+file).then(r=>r.json());for(const mesh of Object.values(data.meshes))assert.equal((await fetch(base+'/'+mesh.file,{method:'HEAD'})).status,200,mesh.file);
  }
  const endpoint=base+'/api/export?name=ToPoDualArmMax-pose.json';
  const body=JSON.stringify({model:'standard',test:true});const headers={'Origin':base,'X-ToPo-Export':'1','Content-Type':'application/json'};
  for(let i=0;i<2;i++){const r=await fetch(endpoint,{method:'POST',headers,body});assert.equal(r.status,200);const out=await r.json();created.push(out.url);assert.deepEqual(await fetch(base+out.url).then(x=>x.json()),JSON.parse(body));}
  assert.notEqual(created[0],created[1]);
  assert.equal((await fetch(endpoint,{method:'POST',headers:{...headers,Origin:'https://example.invalid'},body})).status,403);
  assert.equal((await fetch(base+'/api/export?name=unknown.txt',{method:'POST',headers,body})).status,400);
  assert.equal((await fetch(base+'/%2e%2e%2fpackage.json')).status,403);
 }finally{
  for(const url of created)await fs.unlink(path.join(root,'app',url));
  const exited=once(child,'exit');child.kill();await exited;
 }
});
