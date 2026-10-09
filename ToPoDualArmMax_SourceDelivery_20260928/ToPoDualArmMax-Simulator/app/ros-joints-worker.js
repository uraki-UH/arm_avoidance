// 描画スレッドから独立した通信周期と、一件ずつの送信確認
let socket=null,timer=null,pose=null,is_ready=false,is_pending=false,latest=null;
self.onmessage=event=>{
 const data=event.data;
 if(data.type==='poll'){if(latest){self.postMessage(latest);latest=null;}return;}
 if(data.type==='pose'){pose={type:'state',state:data.state,outputs:data.outputs};return;}
 if(data.type!=='connect')return;
 socket=new WebSocket(data.url);
 socket.onopen=()=>socket.send(JSON.stringify(data.config));
 socket.onmessage=event=>{
  try{
   const message=JSON.parse(event.data);
   if(message.type==='ack'){is_pending=false;return;}
   if(message.type==='ready')is_ready=true;
   if(message.type==='joints'){latest=message;return;}
   self.postMessage(message);
  }catch(error){self.postMessage({type:'error',error:error.message});}
 };
 socket.onerror=()=>self.postMessage({type:'error',error:'関節WebSocketへ接続できません。ブリッジの更新と起動を確認してください'});
 socket.onclose=()=>{clearInterval(timer);self.postMessage({type:'error',error:'関節WebSocketの接続が切れました'});};
 if(data.send)timer=setInterval(()=>{
  if(!pose||!is_ready||is_pending||socket.readyState!==WebSocket.OPEN||socket.bufferedAmount>0)return;
  is_pending=true;socket.send(JSON.stringify(pose));
 },1000/data.config.hz);
};
