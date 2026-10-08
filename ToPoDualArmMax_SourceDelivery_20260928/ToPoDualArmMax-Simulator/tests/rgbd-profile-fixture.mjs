// 実際の選択UIとGPU取得による、プロファイル切替時の画像・校正・保留状態の検証。
export async function verify_rgbd_profile(){
 const workspace=simulator.rgbd,sensor=workspace.sensor,$=id=>document.getElementById(id),results=[];
 const check=(is_valid,message)=>{if(!is_valid)throw Error(message);};
 const wait_for=async(condition,label)=>{
  const start=performance.now();
  while(!condition()){
   if(performance.now()-start>8000)throw Error(`${label}: ${JSON.stringify({profile:$('sensor-profile').value,calibration:sensor.calibration.depth,frame:workspace.lastSummary.frame,canvas:[$('depth-preview').width,$('depth-preview').height]})}`);
   await new Promise(resolve=>setTimeout(resolve,20));
  }
 };
 const select_profile=value=>{$('sensor-profile').value=String(value);$('sensor-profile').dispatchEvent(new Event('change',{bubbles:true}));};
 const show_preview=()=>{document.querySelector('[data-panel=sensors]').click();document.querySelector('[data-sensor-panel=sensor]').click();};
 const wait_frame=async(width,height)=>{
  await wait_for(()=>!workspace.is_capture_pending&&sensor.lastFrame?.calibration.depth.width===width&&sensor.lastFrame?.calibration.depth.height===height,'新しい深度フレームの取得待機超過');
  const frame=sensor.lastFrame;
  check(frame.depth.length===width*height&&frame.z16.length===width*height,'深度配列の解像度不一致');
  check(workspace.last_scene_frame===frame&&workspace.lastSummary.frame.width===width&&workspace.lastSummary.frame.height===height,'送信用フレーム・公開状態の解像度不一致');
  check(!$('sensor-export').disabled,'新規フレームの保存が無効');
  return frame;
 };
 const check_preview=(width,height,color_width=width,color_height=height)=>{
  const depth=$('depth-preview'),rgb=$('rgb-preview');
  check(depth.width===width&&depth.height===height,'深度プレビューの解像度不一致');
  check(rgb.width===color_width&&rgb.height===color_height,'RGB解像度の不一致');
  const frame=sensor.lastFrame;
  check(frame.calibration.color.width===color_width&&frame.calibration.color.height===color_height&&frame.rgba.length===color_width*color_height*4,'RGB取得画像と表示解像度の不一致');
  for(const key of ['color','colorDepth']){const target=sensor.targets[key];check(target.width===color_width&&target.height===color_height,'RGB描画ターゲットの解像度不一致');}
  check($('depth-resolution').textContent.includes(`${width} × ${height}`)&&$('rgb-resolution').textContent===`${color_width} × ${color_height}`,'プレビューの解像度表示不一致');
  check(Number($('pixel-u').max)===width-1&&Number($('pixel-v').max)===height-1,'画素指定範囲の不一致');
  for(const canvas of [depth,rgb])check(canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data.some(value=>value!==0),'プレビュー画像の欠落');
 };
 const check_cleared=()=>{
  check(sensor.lastFrame===null&&workspace.last_scene_frame===null&&workspace.pending_preview_frame===null,'変更前のフレームが残存');
  check(workspace.lastSummary.frame===null&&$('sensor-export').disabled,'変更前の保存・公開状態が残存');
  check(!workspace.cloud.geometry.getAttribute('position'),'変更前の点群が残存');
  for(const id of ['rgb-preview','depth-preview']){const canvas=$(id);check(canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data.every(value=>value===0),'変更前のプレビューが残存');}
 };
 show_preview();check(!workspace.live,'試験開始時に連続取得中');
 check_preview(848,480);results.push({mode:'initial',width:848,height:480,color_width:848,color_height:480});
 for(const [width,height] of [[424,240],[1280,720]]){
  select_profile(width);
  await wait_frame(width,height);check_preview(width,height);
  const frame=sensor.lastFrame;await new Promise(resolve=>setTimeout(resolve,250));
  check(sensor.lastFrame===frame&&!workspace.live,'停止中のプロファイル変更による余分な連続取得');

  const radians=Math.PI/180,calibration=sensor.lastFrame.calibration;
  for(const [key,horizontal_deg,vertical_deg] of [['depth',87,58],['color',69,42]]){
   const intrinsics=calibration[key];
   check(Math.abs(2*Math.atan(intrinsics.width/(2*intrinsics.fx))/radians-horizontal_deg)<1e-9&&Math.abs(2*Math.atan(intrinsics.height/(2*intrinsics.fy))/radians-vertical_deg)<1e-9,'解像度変更による画角の変化');
   check(intrinsics.ppx===(width-1)/2&&intrinsics.ppy===(height-1)/2,'画素中心の校正不一致');
  }
  results.push({mode:'stopped',width,height,color_width:width,color_height:height});
 }
 // 旧取得の完了を遅延させ、連続した変更での旧フレーム復活・多重取得の検査。
 const original_capture=sensor.capture,original_paint=workspace.paint,painted_widths=[];
 let release,has_old_frame=false,num_calls=0,num_active=0,max_active=0;
 const gate=new Promise(resolve=>{release=resolve;});
 let old_capture;
 try{
  sensor.capture=async function(...args){
   const is_first=++num_calls===1;num_active++;max_active=Math.max(max_active,num_active);
   try{const frame=await original_capture.apply(this,args);if(is_first){has_old_frame=true;await gate;}return frame;}
   finally{num_active--;}
  };
  workspace.paint=function(frame){painted_widths.push(frame.calibration.depth.width);return original_paint.call(this,frame);};
  $('cloud-show').checked=true;$('cloud-show').dispatchEvent(new Event('change'));
  old_capture=workspace.capture();await wait_for(()=>has_old_frame,'旧フレームの待機超過');
  select_profile(424);select_profile(848);check_cleared();
  release();check(await old_capture===null,'変更前の取得結果が再採用');
  const frame=await wait_frame(848,480);check_preview(848,480);
  check(num_calls===2&&max_active===1&&painted_widths.length===1&&painted_widths[0]===848,'切替中の余分な取得・旧画像の描画');
  check(workspace.cloud.geometry.getAttribute('position').count===frame.valid,'点群の解像度切替失敗');
  results.push({mode:'in_flight',num_calls,max_active,painted_widths});
 }finally{
  release();if(old_capture)await old_capture;sensor.capture=original_capture;workspace.paint=original_paint;
  $('cloud-show').checked=false;$('cloud-show').dispatchEvent(new Event('change'));
 }
 document.querySelector('[data-panel=robot]').click();await workspace.capture();
 check(workspace.pending_preview_frame===sensor.lastFrame,'非表示中の旧フレーム保留失敗');
 select_profile(424);check_cleared();const hidden_frame=await wait_frame(424,240);
 check(workspace.pending_preview_frame===hidden_frame,'非表示中の新フレーム保留失敗');
 show_preview();await wait_for(()=>workspace.pending_preview_frame===null,'再表示の待機超過');check_preview(424,240);
 results.push({mode:'hidden',width:424,height:240});
 $('sensor-live').click();select_profile(1280);
 await wait_for(()=>sensor.lastFrame?.calibration.depth.width===1280,'連続取得中の切替待機超過');
 check(workspace.live,'プロファイル変更による連続取得停止');$('sensor-live').click();
 await wait_frame(1280,720);check_preview(1280,720);results.push({mode:'live',width:1280,height:720});
 const {nominalCalibration}=await import('/rgbd-core.js'),custom=nominalCalibration(97,65);
 custom.color=nominalCalibration(151,99).depth;
 $('calibration-json').value=JSON.stringify(custom);$('calibration-apply').click();check_cleared();
 await wait_frame(97,65);check_preview(97,65,151,99);check($('sensor-profile').value==='custom','カスタム校正の選択状態不一致');
 results.push({mode:'custom',width:97,height:65,color_width:151,color_height:99});
 const frame=sensor.lastFrame;custom.depth.width=0;$('calibration-json').value=JSON.stringify(custom);$('calibration-apply').click();
 check(sensor.lastFrame===frame&&!workspace.is_refresh_pending,'無効な校正による有効フレームの破棄');
 $('calibration-reset').click();await wait_frame(848,480);check_preview(848,480);
 check($('sensor-profile').value==='848'&&!workspace.live,'公称校正復帰時の状態不一致');
 results.push({mode:'reset',width:848,height:480});
 return results;
}
