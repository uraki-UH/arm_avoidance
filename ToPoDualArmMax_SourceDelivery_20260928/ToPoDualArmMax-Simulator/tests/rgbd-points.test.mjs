import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {nominalCalibration,extrinsicMatrix} from '../app/rgbd-core.js';
import {load_points_kernel,process_points_js} from '../app/rgbd-points.js';

const arrays=['depth','z16','xyz','colors','colorValid','pixels'];
function compare(expected,actual){
 for(const name of arrays)assert.deepEqual(new Uint8Array(actual[name].buffer,actual[name].byteOffset,actual[name].byteLength),new Uint8Array(expected[name].buffer,expected[name].byteOffset,expected[name].byteLength),name);
 for(const name of ['valid','colored','stereoRejected','min','max'])assert.equal(actual[name],expected[name],name);
}
function inputs(width,height){
 const calibration=nominalCalibration(width,height,width+17,height+9),count=width*height,color_count=(width+17)*(height+9);
 const raw=new Float32Array(count),color_depth=new Float32Array(color_count),rgba=new Uint8Array(color_count*4),right_depth=new Float32Array(count),target_depth=new Float32Array(count);
 let seed=7;
 for(let idx=0;idx<count;idx++){
  seed=(Math.imul(seed,1664525)+1013904223)>>>0;
  raw[idx]=[0,NaN,Infinity,-Infinity,-1,.01,4,1,1.0005,1.002,1.5,2][seed%12];
  right_depth[idx]=idx%13===0?NaN:raw[idx];target_depth[idx]=idx%7===0?0:raw[idx];
 }
 for(let idx=0;idx<color_count;idx++)color_depth[idx]=idx%9===0?NaN:idx%3===0?0:1;
 for(let idx=0;idx<rgba.length;idx++)rgba[idx]=idx*17%256;
 return {calibration,raw,color_depth,rgba,right_depth,target_depth};
}

test('WebAssemblyの画素順・丸め・遮蔽・欠損・領域再利用と参照版の完全一致',async t=>{
 const bytes=await readFile(new URL('../app/rgbd-points.wasm',import.meta.url));
 t.mock.method(globalThis,'fetch',async()=>new Response(bytes));
 const kernel=await load_points_kernel();assert.ok(kernel);
 let retained,retained_copy;
 for(const [width,height] of [[97,65],[1280,720],[16,16],[424,240]]){
  const data=inputs(width,height),matrix=extrinsicMatrix(data.calibration.depth_to_color).elements;
  for(const enable_direct_depth of [false,true])for(const enable_stereo of [false,true])for(const enable_target of [false,true]){
   const args=[data.calibration,matrix,data.raw,data.color_depth,data.rgba,enable_stereo?data.right_depth:null,enable_target?data.target_depth:null,enable_direct_depth];
   const expected=process_points_js(...args),actual=kernel.process(...args);compare(expected,actual);
   if(!retained){retained=actual;retained_copy=structuredClone(actual);}
   compare(retained_copy,retained);
  }
 }
 // 半画素の直前・境界・直後と、空フレームによる統計・前回データの消去
 const calibration=nominalCalibration(16,16,16,16),matrix=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
 Object.assign(calibration.depth,{fx:1,fy:1,ppx:0,ppy:0});Object.assign(calibration.color,{fx:1,fy:1,ppx:0,ppy:0});
 const raw=new Float32Array(256).fill(1),color_depth=raw.slice(),rgba=Uint8Array.from({length:1024},(_,idx)=>idx%256);
 for(const shift of [.49999999999999994,.5,.5000000000000001,-.5]){
  matrix[12]=shift;
  const args=[calibration,matrix,raw,color_depth,rgba,null,null,false];compare(process_points_js(...args),kernel.process(...args));
 }
 raw.fill(0);const args=[calibration,matrix,raw,color_depth,rgba,null,null,true];compare(process_points_js(...args),kernel.process(...args));
});

test('WebAssemblyの取得・コンパイル失敗時のJavaScript代替',async t=>{
 for(const [idx,response] of [new Response('',{status:404}),new Response('invalid wasm')].entries()){
  t.mock.method(globalThis,'fetch',async()=>response);
  const module=await import('../app/rgbd-points.js?failure='+idx);
  assert.equal(module.ready_points_kernel(),null);assert.equal(await module.load_points_kernel(),null);
  t.mock.restoreAll();
 }
});
