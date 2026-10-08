// 従来float描画との比較。解像度・ステレオ・遮蔽・対象抽出の独立確認。
export async function verify_depth_readback(){
 const THREE=await import('three'),{RGBDSensor,nominalCalibration}=await import('/rgbd-core.js');
 const scene=new THREE.Scene(),material=new THREE.MeshBasicMaterial({color:'#a04020',side:THREE.DoubleSide});
 const plane=new THREE.Mesh(new THREE.PlaneGeometry(20,20),material),box=new THREE.Mesh(new THREE.BoxGeometry(.35,.3,.25),material);
 plane.position.z=2;plane.rotation.set(.13,.19,0);box.position.set(.04,.02,1);scene.add(plane,box);
 const reference=new RGBDSensor(simulator.renderer,scene,{depth_readback:'float',enable_batched_readback:false}),candidate=new RGBDSensor(simulator.renderer,scene,{depth_readback:'packed'});
 const results=[];
 const compare_frames=(left,right,label)=>{
  for(const key of ['depth','z16','xyz','colors','colorValid','pixels','rgba']){
   const a=new Uint8Array(left[key].buffer,left[key].byteOffset,left[key].byteLength),b=new Uint8Array(right[key].buffer,right[key].byteOffset,right[key].byteLength);
   if(a.length!==b.length||!a.every((value,idx)=>value===b[idx]))throw Error(`画像読出し不一致: ${label}/${key}`);
  }
  for(const key of ['valid','colored','stereoRejected','min','max'])if(left[key]!==right[key])throw Error('深度統計の不一致: '+key);
 };
 try{
  for(const width of [96,424,848,1280])for(const mode of ['ideal','stereo'])for(const enable_async_read of [false,true]){
   const calibration=nominalCalibration(width,width===96?64:width===424?240:width===848?480:720);
   calibration.color={...calibration.depth};reference.configure(calibration);candidate.configure(calibration);
   const options={mode,enable_async_read},world=new THREE.Matrix4();
   const left=await reference.capture(world,options),right=await candidate.capture(world,options);
   compare_frames(left,right,`${width}/${mode}/${enable_async_read}`);
   results.push({width,mode,enable_async_read,num_points:right.valid});
  }
  const world=new THREE.Matrix4(),options={target_group:box,enable_async_read:true};
  const left=await reference.capture(world,options),right=await candidate.capture(world,options);
  if(!left.valid)throw Error('対象抽出の欠落');compare_frames(left,right,'対象抽出');
  results.push({target_group:'box',num_points:right.valid});
  // 異なる解像度・外部回転・移動中の対象・重複取得でのバッファ混線検査。
  const moving_calibration=nominalCalibration(97,65);
  moving_calibration.color=nominalCalibration(151,99).depth;
  moving_calibration.depth_to_color.rotation=new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationY(.08)).transpose().toArray();
  reference.configure(moving_calibration);candidate.configure(moving_calibration);
  for(let idx=0;idx<3;idx++){
   box.position.x=.04+idx*.07;box.rotation.y=idx*.1;
   const pose=new THREE.Matrix4().makeRotationZ(idx*.03);pose.setPosition(.01*idx,.02*idx,0);
   const expected=await reference.capture(pose,{mode:'stereo',enable_async_read:true});
   const pending=[candidate.capture(pose,{mode:'stereo',enable_async_read:true}),candidate.capture(pose,{mode:'stereo',enable_async_read:true})];
   for(const frame of await Promise.all(pending))compare_frames(expected,frame,'重複取得');
   results.push({moving_pose:idx,num_points:expected.valid});
  }
  // 単一成分読出し非対応GPUの代替経路。形式照会だけの差替え。
  const context=simulator.renderer.getContext(),get_parameter=context.getParameter;
  const fallback=new RGBDSensor(simulator.renderer,scene);
  try{
   fallback.configure(reference.calibration);
   context.getParameter=function(parameter){return parameter===this.IMPLEMENTATION_COLOR_READ_FORMAT?this.RGBA:get_parameter.call(this,parameter);};
   const frame=await fallback.capture(world,{enable_async_read:true}),expected=await candidate.capture(world,{enable_async_read:true});
   if(fallback.targets.depth.texture.type!==THREE.UnsignedByteType||frame.valid!==expected.valid||!frame.depth.every((value,idx)=>value===expected.depth[idx]))throw Error('自動代替経路の不一致');
   const target=fallback.targets.depth;await fallback.capture(world,{enable_async_read:true});if(fallback.targets.depth!==target)throw Error('代替ターゲットの不要な再生成');
   results.push({fallback:'RGBA8',num_points:frame.valid});
   const wide=new RGBDSensor(simulator.renderer,scene,{depth_readback:'float'});
   try{wide.configure(reference.calibration);const wide_frame=await wide.capture(world,{enable_async_read:true});compare_frames(expected,wide_frame,'RGBA float');results.push({fallback:'RGBA float',num_points:wide_frame.valid});}
   finally{wide.dispose();}
  }finally{context.getParameter=get_parameter;fallback.dispose();}
  // 定常取得時のGPUバッファ再利用とCPUへの一括コピーの検査。
  const create_buffer=context.createBuffer,copy_buffer=context.getBufferSubData;
  let num_allocations=0,num_copies=0;
  try{
   context.createBuffer=function(...args){num_allocations++;return create_buffer.apply(this,args);};
   context.getBufferSubData=function(...args){num_copies++;return copy_buffer.apply(this,args);};
   await candidate.capture(world,{mode:'stereo',enable_async_read:true});
   if(num_allocations!==0||num_copies!==1)throw Error(`一括読出しの退行: allocations=${num_allocations}, copies=${num_copies}`);
   results.push({num_readback_allocations:num_allocations,num_cpu_copies:num_copies});
  }finally{context.createBuffer=create_buffer;context.getBufferSubData=copy_buffer;}
  // 待機失敗後の再取得と破棄中断。既存の描画コンテキストは維持。
  const wait_sync=context.clientWaitSync;
  try{
   context.clientWaitSync=()=>context.WAIT_FAILED;
   let has_rejected=false;try{await candidate.capture(world,{enable_async_read:true});}catch{has_rejected=true;}
   if(!has_rejected||context.getParameter(context.PIXEL_PACK_BUFFER_BINDING)!==null)throw Error('読出し失敗時の後始末不良');
  }finally{context.clientWaitSync=wait_sync;}
  compare_frames(await reference.capture(world,{enable_async_read:true}),await candidate.capture(world,{enable_async_read:true}),'失敗後の復旧');
  const disposable=new RGBDSensor(simulator.renderer,scene);disposable.configure(reference.calibration);
  const pending=disposable.capture(world,{enable_async_read:true});disposable.dispose();
  let has_rejected=false;try{await pending;}catch{has_rejected=true;}
  if(!has_rejected||context.getParameter(context.PIXEL_PACK_BUFFER_BINDING)!==null)throw Error('破棄中の読出し中断不良');
  results.push({has_failure_recovery:true,has_dispose_cancellation:true});
  const qa=await(await import('/rgbd-qa.js')).runSensorQA(simulator.renderer);
  if(!qa.passed)throw Error(JSON.stringify(qa));
  return {comparisons:results,geometry_qa:qa};
 }finally{reference.dispose();candidate.dispose();plane.geometry.dispose();box.geometry.dispose();material.dispose();}
}
