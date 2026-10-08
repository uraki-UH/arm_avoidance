// 実WS受信後と同じ復号・状態更新・描画経路の負荷。通信・ROSノードの時間は対象外。
export function install_graph_fixture(num_nodes, stream_hz) {
 const tag='/performance/graph', encoder=new TextEncoder(), tag_bytes=encoder.encode(tag), frame_bytes=encoder.encode('world');
 const num_edge_indices=num_nodes*6, offset=36+tag_bytes.length+frame_bytes.length;
 const packet=new ArrayBuffer(offset+num_nodes*96+num_edge_indices*2), view=new DataView(packet);
 view.setUint32(0,0x31474d54,true);view.setUint16(4,2,true);
 for(const [position,value] of [[8,tag_bytes.length],[12,frame_bytes.length],[20,num_nodes],[24,num_edge_indices],[32,packet.byteLength-36]])view.setUint32(position,value,true);
 new Uint8Array(packet,36,tag_bytes.length).set(tag_bytes);new Uint8Array(packet,36+tag_bytes.length,frame_bytes.length).set(frame_bytes);
 for(let idx=0;idx<num_nodes;idx++){
  const position=offset+idx*96;view.setUint16(position,idx,true);view.setUint8(position+2,1);
  view.setFloat32(position+24,(idx%100)*.012-.3,true);view.setFloat32(position+28,Math.floor(idx/100)*.012-.6,true);view.setFloat32(position+32,.65,true);view.setFloat32(position+44,1,true);
  for(let edge_idx=0;edge_idx<3;edge_idx++){const edge_position=offset+num_nodes*96+(idx*6+edge_idx*2)*2;view.setUint16(edge_position,idx,true);view.setUint16(edge_position+2,(idx+[1,100,101][edge_idx])%num_nodes,true);}
 }
 const sockets=new Set();let sequence=0, timer;
 class fixture_socket extends EventTarget {
  static OPEN=1;static CONNECTING=0;static CLOSED=3;
  constructor(){super();this.readyState=0;sockets.add(this);queueMicrotask(()=>{this.readyState=1;this.emit('open',new Event('open'));});}
  emit(type,event){this['on'+type]?.(event);this.dispatchEvent(event);}
  receive(data){if(this.readyState===1)this.emit('message',new MessageEvent('message',{data}));}
  send(text){const request=JSON.parse(text);if(request.type==='request.state'){this.receive(packet);return;}
   if(request.id)queueMicrotask(()=>this.receive(JSON.stringify({id:request.id,ok:true,result:request.method==='sources.list'?{sources:[{id:tag,name:tag,type:'topological_map',active:true}]}:{success:true}})));
  }
  close(){this.readyState=3;sockets.delete(this);this.emit('close',new Event('close'));}
 }
 const original_socket=window.WebSocket;window.WebSocket=fixture_socket;
 window.performance_fixture={tag,packet_bytes:packet.byteLength,get sequence(){return sequence;},start(){
  if(stream_hz)timer=setInterval(()=>{sequence++;view.setUint32(16,sequence,true);view.setFloat32(offset+32,.65+(sequence%10)*.001,true);for(const socket of sockets)socket.receive(packet);},1000/stream_hz);
 },stop(){clearInterval(timer);for(const socket of sockets)socket.close();window.WebSocket=original_socket;}};
}
