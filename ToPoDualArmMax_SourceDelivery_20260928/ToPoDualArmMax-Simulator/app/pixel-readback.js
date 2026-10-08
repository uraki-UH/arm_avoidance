// センサ画像群の一括読出し。描画時のフレームバッファから同一バッファへ転送。
export class pixel_readback_pool {
 constructor(context){this.context=context;this.entries=new Set();this.is_disposed=false;}
 begin(byte_length){
  if(this.is_disposed)throw Error('画像読出しバッファは破棄済みです');
  const context=this.context;
  let entry=[...this.entries].find(value=>!value.is_pending);
  if(!entry){entry={buffer:context.createBuffer(),byte_length:0,is_pending:false};if(!entry.buffer)throw Error('画像読出しバッファを確保できません');this.entries.add(entry);}
  entry.is_pending=true;
  try{
   context.bindBuffer(context.PIXEL_PACK_BUFFER,entry.buffer);
   if(entry.byte_length<byte_length){context.bufferData(context.PIXEL_PACK_BUFFER,byte_length,context.STREAM_READ);entry.byte_length=byte_length;}
  }catch(error){entry.is_pending=false;throw error;}
  finally{context.bindBuffer(context.PIXEL_PACK_BUFFER,null);}
  return new pixel_readback_batch(this,entry,byte_length);
 }
 dispose(){
  this.is_disposed=true;
  for(const entry of this.entries)this.context.deleteBuffer(entry.buffer);
  this.entries.clear();
 }
}

class pixel_readback_batch {
 constructor(pool,entry,byte_length){
  this.pool=pool;this.entry=entry;this.bytes=new Uint8Array(byte_length);this.byte_offset=0;this.is_finished=false;this.sync=null;
 }
 allocate(array_type,length){
  const byte_length=length*array_type.BYTES_PER_ELEMENT;
  if(this.byte_offset+byte_length>this.bytes.byteLength)throw Error('画像読出し領域が不足しています');
  const array=new array_type(this.bytes.buffer,this.byte_offset,length);this.byte_offset+=byte_length;return array;
 }
 enqueue(array,width,height,format,type){
  const context=this.pool.context;
  context.bindBuffer(context.PIXEL_PACK_BUFFER,this.entry.buffer);
  try{context.readPixels(0,0,width,height,format,type,array.byteOffset);}
  finally{context.bindBuffer(context.PIXEL_PACK_BUFFER,null);}
 }
 async finish(){
  const context=this.pool.context;
  try{
   this.sync=context.fenceSync(context.SYNC_GPU_COMMANDS_COMPLETE,0);
   if(!this.sync)throw Error('画像読出しの完了待ちを開始できません');
   context.flush();
   const start=performance.now();
   // 最後の転送完了までイベントループへ返却。後続の画像ごとの同期待機なし。
   while(true){
    await new Promise(resolve=>setTimeout(resolve,4));
    if(this.pool.is_disposed||context.isContextLost())throw Error('画像読出しが中断されました');
    const status=context.clientWaitSync(this.sync,0,0);
    if(status===context.ALREADY_SIGNALED||status===context.CONDITION_SATISFIED)break;
    if(status===context.WAIT_FAILED||performance.now()-start>10000)throw Error('画像読出しの完了待ちに失敗しました');
   }
   context.bindBuffer(context.PIXEL_PACK_BUFFER,this.entry.buffer);
   context.getBufferSubData(context.PIXEL_PACK_BUFFER,0,this.bytes);
  }finally{context.bindBuffer(context.PIXEL_PACK_BUFFER,null);this.release();}
 }
 release(){
  if(this.is_finished)return;
  if(this.sync)this.pool.context.deleteSync(this.sync);
  this.entry.is_pending=false;this.is_finished=true;
 }
}
