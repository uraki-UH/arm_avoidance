import {jt128_channels} from './jt128-scan.js';

// 分配済み交点結果の出力順復元。乱数系列と近距離除外は全スロットで統一
export function frame_from_hits(c,responses,start=0){
 const begin=performance.now(),num_workers=responses.length;
 const xyz=new Float32Array(c.beams*3),range=new Float32Array(c.beams),time=new Float32Array(c.beams),beamIndex=new Uint32Array(c.beams),objectId=new Uint32Array(c.beams),reflectance=new Uint8Array(c.beams),slotStatus=new Uint8Array(c.beams),slotRange=new Float32Array(c.beams);
 let count=0,nearRejected=0,unknownDirections=0,noReturn=0,rangeRejected=0,seed=c.seed>>>0;
 const random=()=>{seed=(1664525*seed+1013904223)>>>0;return(seed+.5)/4294967296;};
 for(let idx=0;idx<c.beams;idx++){
  const packet=responses[idx%num_workers],slot=Math.floor(idx/num_workers),dx=packet.beam_directions[slot*3],dy=packet.beam_directions[slot*3+1],dz=packet.beam_directions[slot*3+2];
  if(dx===0&&dy===0&&dz===0){unknownDirections++;slotStatus[idx]=1;continue;}
  let dist=packet.distances[slot];if(dist===Infinity){noReturn++;slotStatus[idx]=2;continue;}
  const min_range=c.sensor_type==='jt128'?Math.max(c.minRange,jt128_channels[(start+idx)%128][2]):c.minRange;
  if(dist<min_range){nearRejected++;slotStatus[idx]=3;continue;}
  if(c.mode==='noise')dist+=c.noiseSigma*Math.sqrt(-2*Math.log(random()))*Math.cos(2*Math.PI*random());
  if(dist<min_range||dist>c.maxRange){rangeRejected++;slotStatus[idx]=4;continue;}
  slotRange[idx]=dist;xyz[count*3]=dx*dist;xyz[count*3+1]=dy*dist;xyz[count*3+2]=dz*dist;range[count]=dist;time[count]=idx*c.duration/c.beams;beamIndex[count]=idx;objectId[count]=packet.object_ids[slot];reflectance[count]=packet.reflectances[slot];count++;
 }
 return {xyz:xyz.slice(0,count*3),range:range.slice(0,count),time:time.slice(0,count),beamIndex:beamIndex.slice(0,count),objectId:objectId.slice(0,count),reflectance:reflectance.slice(0,count),count,nearRejected,unknownDirections,noReturn,rangeRejected,slotStatus,slotRange,ms:performance.now()-begin};
}
