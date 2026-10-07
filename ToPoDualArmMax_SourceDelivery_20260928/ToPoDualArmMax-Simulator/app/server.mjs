import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { randomUUID, createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
const root=path.dirname(fileURLToPath(import.meta.url));
const port=Number(process.env.PORT||8877);
if(!Number.isInteger(port)||port<1024||port>65535)throw Error('PORT must be 1024..65535');
const startedAt=Math.round((Date.now()-process.uptime()*1000)/1000);
const instance=createHash('sha256').update(process.platform==='win32'?root.toLowerCase():root).digest('hex').slice(0,16);
const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.mjs':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.json':'application/json; charset=utf-8','.urdf':'application/xml; charset=utf-8','.wasm':'application/wasm','.glb':'model/gltf-binary','.bin':'application/octet-stream','.stl':'application/octet-stream','.png':'image/png','.md':'text/plain; charset=utf-8','.zip':'application/zip'};
const server=http.createServer((req,res)=>{
 let pathname;try{pathname=decodeURIComponent(new URL(req.url,'http://localhost').pathname);}catch{res.writeHead(400);res.end();return;}
 if(pathname==='/api/health'){
  res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'});
  res.end(JSON.stringify({app:'topo-motion-studio',pid:process.pid,port,instance,startedAt,delivery:'source-1.0.0'}));return;
 }
 if(pathname==='/api/export'){
  const origin=req.headers.origin;
  if(req.method!=='POST'||req.headers['x-topo-export']!=='1'||![`http://127.0.0.1:${port}`,`http://localhost:${port}`].includes(origin)){res.writeHead(403);res.end();return;}
  const requested=new URL(req.url,'http://localhost').searchParams.get('name');
  if(!['ToPoDualArmMax-Long.png','ToPoDualArmMax-Long-pose.json','ToPoDualArmMax.png','ToPoDualArmMax-pose.json','ToPo-workspace.json','D435i-capture.zip','MID360-capture.zip','Camera-capture.zip'].includes(requested)){res.writeHead(400);res.end();return;}
  const chunks=[];let size=0,tooLarge=false;
  req.on('data',chunk=>{size+=chunk.length;if(size>80*1024*1024){tooLarge=true;req.destroy();return;}chunks.push(chunk);});
  req.on('end',async()=>{if(tooLarge)return;const data=Buffer.concat(chunks);
   try{
    if(requested.endsWith('.png')&&data.subarray(0,8).toString('hex')!=='89504e470d0a1a0a')throw new Error('Invalid PNG');
    if(requested.endsWith('.json'))JSON.parse(data.toString('utf8'));
    if(requested.endsWith('.zip')&&data.subarray(0,4).toString('hex')!=='504b0304')throw new Error('Invalid ZIP');
    const dir=path.join(root,'exports');await fs.promises.mkdir(dir,{recursive:true});
    const filename=Date.now()+'-'+randomUUID()+'-'+requested;await fs.promises.writeFile(path.join(dir,filename),data,{flag:'wx'});
    res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify({url:'/exports/'+filename,filename}));
   }catch{res.writeHead(400);res.end('Invalid export');}
  });return;
 }
 const file=path.resolve(root,'.'+(pathname==='/'?'/index.html':pathname));
 const relative=path.relative(root,file);
 if(relative.startsWith('..')||path.isAbsolute(relative)){res.writeHead(403);res.end();return;}
 fs.stat(file,(err,stat)=>{if(err||!stat.isFile()){res.writeHead(404);res.end('Not found');return;}
  res.writeHead(200,{'Content-Type':types[path.extname(file)]||'application/octet-stream','Content-Length':stat.size,'Cache-Control':'no-cache','X-Content-Type-Options':'nosniff'});
  if(req.method==='HEAD'){res.end();return;}fs.createReadStream(file).pipe(res);
 });
});
server.listen(port,'127.0.0.1',()=>console.log(`ToPo Motion Studio: http://127.0.0.1:${port}\nClose this process to stop.`));
server.on('error',e=>{console.error(e.code==='EADDRINUSE'?`Port ${port} is already in use. Open http://127.0.0.1:${port} or set PORT to another number.`:e);process.exitCode=1;});
