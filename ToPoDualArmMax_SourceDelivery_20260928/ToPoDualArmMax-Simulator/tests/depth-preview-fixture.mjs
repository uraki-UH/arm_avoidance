// 表示用GPU色付けの上下方向・欠損色・境界値・解像度変更・CPU代替の検証。
export async function verify_depth_preview(){
 const {depth_preview}=await import('/depth-preview.js'),preview=new depth_preview(),results=[];
 const canvas=document.createElement('canvas'),context=canvas.getContext('2d');
 const check=(is_valid,message)=>{if(!is_valid)throw Error(message);};
 const reference=frame=>{
  const pixels=new Uint8ClampedArray(frame.depth.length*4),lo=frame.calibration.min_depth_m,hi=frame.calibration.max_depth_m;
  for(let idx=0;idx<frame.depth.length;idx++){
   const z=frame.depth[idx],offset=idx*4,t=Math.max(0,Math.min(1,(z-lo)/(hi-lo)));
   pixels[offset]=z?255*Math.max(0,1-Math.abs(t*3-2)):18;
   pixels[offset+1]=z?255*Math.max(0,1-Math.abs(t*3-1)):20;
   pixels[offset+2]=z?255*Math.max(0,1-Math.abs(t*3)):29;pixels[offset+3]=255;
  }
  return pixels;
 };
 try{
  for(const [width,height,lo,hi] of [[97,65,.195,3],[424,240,.105,3],[848,480,.195,3],[1280,720,.28,3],[97,65,.01,20]]){
   const depth=new Float32Array(width*height),frame={depth,calibration:{depth:{width,height},min_depth_m:lo,max_depth_m:hi}};
   for(let idx=0;idx<depth.length;idx++)depth[idx]=idx%17===0?0:idx%19===0?lo:idx%23===0?hi:lo+(hi-lo)*((idx*997)%65536)/65535;
   canvas.width=width;canvas.height=height;check(preview.draw(context,frame),'GPUプレビュー生成失敗');
   const expected=reference(frame),actual=context.getImageData(0,0,width,height).data;let max_error=0;
   for(let idx=0;idx<actual.length;idx++)max_error=Math.max(max_error,Math.abs(actual[idx]-expected[idx]));
   check(max_error<=1,`GPUプレビューの画素差: ${max_error}`);results.push({width,height,lo,hi,max_error});
  }
  check(!preview.draw(context,{depth:new Float32Array([1]),calibration:{depth:{width:1,height:1},min_depth_m:1,max_depth_m:1.000001}}),'狭い深度範囲のCPU代替失敗');
  results.push({has_narrow_range_fallback:true});
  const workspace=simulator.rgbd,frame=workspace.sensor.lastFrame,display=document.getElementById('depth-preview'),display_context=display.getContext('2d');
  const enable_gpu=workspace.enable_gpu_depth_preview;
  try{
   workspace.enable_gpu_depth_preview=true;workspace.depth_preview.has_failed=true;workspace.paint(frame);
   const expected=reference(frame),actual=display_context.getImageData(0,0,display.width,display.height).data;
   check(actual.every((value,idx)=>value===expected[idx]),'GPU利用不可時のCPU代替失敗');
  }finally{workspace.depth_preview.has_failed=false;workspace.enable_gpu_depth_preview=enable_gpu;workspace.paint(frame);}
  results.push({has_cpu_fallback:true});
  return results;
 }finally{preview.dispose();}
}
