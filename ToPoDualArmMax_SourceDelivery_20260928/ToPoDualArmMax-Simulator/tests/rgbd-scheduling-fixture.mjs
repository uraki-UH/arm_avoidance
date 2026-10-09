// 読出し待ち中の表示継続と、視点変更時の描画優先の実ブラウザ検査。
export async function verify_rgbd_scheduling(){
 const {rgbd,renderer,camera,scene}=simulator,context=renderer.getContext();
 const render=renderer.render,wait_sync=context.clientWaitSync,position=camera.position.clone(),quaternion=camera.quaternion.clone();
 const enable_priority=rgbd.enable_readback_priority,is_live=rgbd.live;
 let is_hold=true,pending;const view_frames=[];
 try{
  rgbd.live=false;rgbd.enable_readback_priority=true;
  while(rgbd.is_capture_pending)await new Promise(resolve=>setTimeout(resolve,10));
  await new Promise(resolve=>setTimeout(resolve,120));
  context.clientWaitSync=function(...args){return is_hold?this.TIMEOUT_EXPIRED:wait_sync.apply(this,args);};
  renderer.render=function(...args){if(args[0]===scene&&args[1]===camera&&rgbd.sensor.readback_pool.has_pending)view_frames.push({time:performance.now(),x:camera.position.x});return render.apply(this,args);};
  pending=rgbd.capture();await new Promise(resolve=>setTimeout(resolve,300));
  if(view_frames.length<2)throw Error('RGB-D待機中の3D画面の停止');
  const num_static_frames=view_frames.length;camera.position.x+=.05;
  await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));
  if(view_frames.length===num_static_frames||Math.abs(view_frames.at(-1).x-position.x)<.01)throw Error('RGB-D待機中の視点変更の欠落');
  is_hold=false;const frame=await pending;if(!frame)throw Error('描画優先検査後の取得失敗');
  return {num_static_frames,num_input_frames:view_frames.length-num_static_frames,has_completed_capture:true};
 }finally{
  is_hold=false;context.clientWaitSync=wait_sync;
  if(pending)await pending;
  renderer.render=render;camera.position.copy(position);camera.quaternion.copy(quaternion);
  rgbd.enable_readback_priority=enable_priority;rgbd.live=is_live;
 }
}
