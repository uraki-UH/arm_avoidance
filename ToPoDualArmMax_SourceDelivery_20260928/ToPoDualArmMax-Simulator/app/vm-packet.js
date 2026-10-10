const TYPES = { f32: Float32Array, u32: Uint32Array, i32: Int32Array, u8: Uint8Array };
const REQUIRED = { points: ['f32',3], nodes:['f32',3], nodeLabels:['u8',1], nodeClusterIds:['i32',1], edges:['u32',2], fvgAdd:['f32',10], fvgDelete:['f32',10], fvgMemory:['f32',10] };
export function encode_input(meta,points){
 const json=new TextEncoder().encode(JSON.stringify(meta)),offset=8+Math.ceil(json.length/4)*4;
 const bytes=new Uint8Array(offset+points.byteLength);bytes.set([84,80,67,49]);new DataView(bytes.buffer).setUint32(4,json.length,true);bytes.set(json,8);bytes.set(new Uint8Array(points.buffer,points.byteOffset,points.byteLength),offset);return bytes;
}
export function decodeFrame(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 12 || buffer.byteLength > 32*1024*1024) throw new Error('不正なフレームサイズ');
  const header = new DataView(buffer);
  if (header.getUint32(0,true) !== 0x31564654) throw new Error('未対応のフレーム形式');
  const length=header.getUint32(4,true), start=8+Math.ceil(length/4)*4;
  if (length>1024*1024 || start>buffer.byteLength) throw new Error('フレーム情報が不正です');
  const meta=JSON.parse(new TextDecoder('utf-8',{fatal:true}).decode(new Uint8Array(buffer,8,length)));
  if (meta.version!==1 || !Number.isSafeInteger(meta.sequence) || !Array.isArray(meta.clusters)) throw new Error('フレームの版または番号が不正です');
  const frame={meta};
  for(const [key,d] of Object.entries(meta.arrays||{})) {
    const T=TYPES[d.type];
    if(!T || !Number.isSafeInteger(d.offset) || !Number.isSafeInteger(d.count) || d.offset<0 || d.count<0 || d.offset%T.BYTES_PER_ELEMENT || start+d.offset+d.count*T.BYTES_PER_ELEMENT>buffer.byteLength) throw new Error('不正な配列: '+key);
    if (!Number.isSafeInteger(d.components) || d.components<1 || d.count%d.components) throw new Error('不正な配列構成: '+key);
    frame[key]=new T(buffer,start+d.offset,d.count);
  }
  for(const [key,[type,size]] of Object.entries(REQUIRED)) if(!frame[key] || meta.arrays[key].type!==type || meta.arrays[key].components!==size) throw new Error('必要な配列がありません: '+key);
  if(frame.nodes.length/3!==frame.nodeLabels.length || frame.nodeClusterIds.length!==frame.nodeLabels.length) throw new Error('ノード配列が一致しません');
  if(frame.points.length/3!==meta.pointCount || frame.nodes.length/3!==meta.nodeCount || frame.edges.length/2!==meta.edgeCount) throw new Error('フレームの要素数が一致しません');
  for(const c of meta.clusters) if(!Number.isSafeInteger(c.id) || !Array.isArray(c.centroid) || c.centroid.length!==3 || !Array.isArray(c.scale) || c.scale.length!==3 || !Array.isArray(c.quat) || c.quat.length!==4 || ![...c.centroid,...c.scale,...c.quat].every(Number.isFinite)) throw new Error('クラスタ座標が不正です');
  return frame;
}
