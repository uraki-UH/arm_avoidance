import {randomUUID} from 'node:crypto';

// ブラウザ単位の操作窓口。期限付きの単一要求と応答、再読込み後の操作継承なし。
export function create_robot_control_api(port){
 const sessions=new Map(),pending=new Map();
 const respond=(res,status,value)=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'});res.end(JSON.stringify(value));};
 const read=async req=>{let size=0,parts=[];for await(const chunk of req){size+=chunk.length;if(size>128000)throw Error('要求サイズの超過');parts.push(chunk);}return JSON.parse(Buffer.concat(parts).toString());};
 const origins=[`http://127.0.0.1:${port}`,`http://localhost:${port}`];
 return {async handle(req,res){
  try{
   if(req.headers.origin&&!origins.includes(req.headers.origin)){respond(res,403,{error:'同一画面サーバーからの操作が必要です'});return;}
   for(const [id,session] of sessions)if(Date.now()-session.updated_ms>60000&&!pending.has(id))sessions.delete(id);
   if(req.method==='GET'){
    respond(res,200,{sessions:[...sessions].map(([session_id,s])=>({session_id,is_online:Date.now()-s.updated_ms<30000,updated_ms:s.updated_ms,state:s.state}))});return;
   }
   if(req.method!=='POST'||req.headers['x-topo-robot']!=='1'){respond(res,403,{error:'X-ToPo-Robot: 1が必要です'});return;}
   const value=await read(req);
   if(!value||typeof value!=='object'||typeof value.session_id!=='string'||!/^[a-f0-9-]{36}$/.test(value.session_id))throw Error('セッションIDが不正です');
   const id=value.session_id;
   if(value.state){
    if(!sessions.has(id)&&sessions.size>=32)throw Error('接続画面数の上限です');
    sessions.set(id,{state:value.state,updated_ms:Date.now()});
    const request=pending.get(id);
    for(const result of value.results??[])if(request&&result.id===request.action.id){clearTimeout(request.timer);pending.delete(id);request.resolve(result);}
    const next=pending.get(id),actions=next&&!next.is_delivered&&Date.now()<=next.action.expires_ms?[next.action]:[];
    if(actions.length)next.is_delivered=true;
    respond(res,200,{actions});return;
   }
   if(!sessions.has(id)||Date.now()-sessions.get(id).updated_ms>30000)throw Error('操作対象のブラウザが未接続です');
   if(pending.has(id)){respond(res,409,{error:'前の操作の完了待ちです'});return;}
   if(!['configure','start','stop','command','goal'].includes(value.action?.type))throw Error('未対応のロボット操作です');
   const result=await new Promise(resolve=>{
    const action={id:randomUUID(),expires_ms:Date.now()+20000,value:value.action};
    const timer=setTimeout(()=>{pending.delete(id);resolve({error:'ブラウザの応答時間超過'});},20000);
    pending.set(id,{action,resolve,timer,is_delivered:false});
   });
   respond(res,result.error?400:200,result);
  }catch(error){if(!res.headersSent)respond(res,400,{error:error.message});}
 },close(){for(const request of pending.values()){clearTimeout(request.timer);request.resolve({error:'画面サーバー終了'});}pending.clear();sessions.clear();}};
}
