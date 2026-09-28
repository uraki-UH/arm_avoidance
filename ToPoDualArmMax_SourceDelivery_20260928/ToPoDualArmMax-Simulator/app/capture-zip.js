// Small ZIP writer (stored entries). All payloads are produced locally by the simulator.
const table=new Uint32Array(256);for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=c&1?0xedb88320^(c>>>1):c>>>1;table[n]=c;}
function crc32(bytes){let c=0xffffffff;for(const b of bytes)c=table[(c^b)&255]^(c>>>8);return (c^0xffffffff)>>>0;}
export async function zipFiles(entries){
 const local=[],central=[];let offset=0,totalCentral=0;
 for(const [name,payload] of Object.entries(entries)){
  const filename=new TextEncoder().encode(name),bytes=payload instanceof Blob?new Uint8Array(await payload.arrayBuffer()):typeof payload==='string'?new TextEncoder().encode(payload):payload instanceof ArrayBuffer?new Uint8Array(payload):new Uint8Array(payload.buffer,payload.byteOffset,payload.byteLength),crc=crc32(bytes);
  const h=new Uint8Array(30+filename.length),v=new DataView(h.buffer);v.setUint32(0,0x04034b50,true);v.setUint16(4,20,true);v.setUint16(6,0x800,true);v.setUint32(14,crc,true);v.setUint32(18,bytes.length,true);v.setUint32(22,bytes.length,true);v.setUint16(26,filename.length,true);h.set(filename,30);local.push(h,bytes);
  const c=new Uint8Array(46+filename.length),cv=new DataView(c.buffer);cv.setUint32(0,0x02014b50,true);cv.setUint16(4,20,true);cv.setUint16(6,20,true);cv.setUint16(8,0x800,true);cv.setUint32(16,crc,true);cv.setUint32(20,bytes.length,true);cv.setUint32(24,bytes.length,true);cv.setUint16(28,filename.length,true);cv.setUint32(42,offset,true);c.set(filename,46);central.push(c);totalCentral+=c.length;offset+=h.length+bytes.length;
 }
 const end=new Uint8Array(22),ev=new DataView(end.buffer);ev.setUint32(0,0x06054b50,true);ev.setUint16(8,central.length,true);ev.setUint16(10,central.length,true);ev.setUint32(12,totalCentral,true);ev.setUint32(16,offset,true);return new Blob([...local,...central,end],{type:'application/zip'});
}
