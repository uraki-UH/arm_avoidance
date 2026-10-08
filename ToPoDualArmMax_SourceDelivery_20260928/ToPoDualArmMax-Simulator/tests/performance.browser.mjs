import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import fs from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {parseArgs} from 'node:util';
import {install_graph_fixture} from './performance-fixture.mjs';

// 専用ChromeとHTTPサーバーによる有限時間の性能試験。既存ROS・ブラウザへの接続なし。
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const {values:options}=parseArgs({options:{output:{type:'string'},gpu:{type:'string',default:'mesa'},cases:{type:'string',default:'idle,frustum,lidar,rgbd,graph_static,graph_stream'},'duration-ms':{type:'string',default:'3000'},captures:{type:'string',default:'10'},nodes:{type:'string',default:'10000'},profile:{type:'boolean',default:false},baseline:{type:'string'},'max-regression-percent':{type:'string'}}});
const duration_ms=Number(options['duration-ms']),num_captures=Number(options.captures),num_nodes=Number(options.nodes);
assert.ok(duration_ms>=500&&duration_ms<=60000);assert.ok(Number.isInteger(num_captures)&&num_captures>=2&&num_captures<=100);assert.ok(Number.isInteger(num_nodes)&&num_nodes>=102&&num_nodes<=65535);
assert.ok(['mesa','nvidia','software'].includes(options.gpu));
if(options['max-regression-percent']!==undefined)assert.ok(Number.isFinite(Number(options['max-regression-percent']))&&Number(options['max-regression-percent'])>=0);
const case_names=options.cases.split(','),allowed_cases=['idle','frustum','lidar','rgbd','graph_static','graph_stream'];
assert.ok(case_names.every(name=>allowed_cases.includes(name)));
const output=options.output?path.resolve(options.output):await fs.mkdtemp('/tmp/topo-performance-');
await fs.mkdir(output,{recursive:true});
await fs.writeFile(path.join(output,'run.lock'),String(process.pid),{flag:'wx'});
const profile_directory=await fs.mkdtemp('/tmp/topo-performance-chrome-');
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const source_files=['app/app.js','app/lidar-core.js','app/lidar-ui.js','app/rgbd-core.js','app/rgbd-ui.js','app/generated/ros-results.js'];
const read_source_hashes=async()=>Object.fromEntries(await Promise.all(source_files.map(async filename=>[filename,createHash('sha256').update(await fs.readFile(path.join(root,filename))).digest('hex')])));

const processes=[],report={schema:1,created_at:new Date().toISOString(),enable_profile:options.profile,config:{gpu:options.gpu,duration_ms,num_captures,num_nodes,case_names,viewport:[1440,1000],model:'long'},cases:{},commands:[]};
let socket,deadline;
const stop=async child=>{
 if(!child||child.exitCode!==null||child.signalCode!==null)return;
 const exited=once(child,'exit');child.kill('SIGTERM');
 let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,3000);})]);clearTimeout(timer);
 if(child.exitCode===null&&child.signalCode===null){child.kill('SIGKILL');await exited;}
};
const cleanup=async()=>{socket?.close();for(const child of [...processes].reverse())await stop(child);await fs.rm(profile_directory,{recursive:true,force:true});};
for(const signal of ['SIGINT','SIGTERM'])process.once(signal,()=>{void cleanup().finally(()=>process.exit(130));});
try {
 report.source_sha256_start=await read_source_hashes();
 const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
 const launch=(command,args,env)=>{report.commands.push({command,args,port});const child=spawn(command,args,{cwd:root,env,stdio:['ignore','ignore','pipe']});processes.push(child);let tail='';child.stderr.on('data',data=>{tail=(tail+data).slice(-6000);});child.on('error',error=>{report.process_error=String(error);});child.on('exit',()=>{if(child.exitCode)report.process_error=tail;});return child;};
 launch(process.execPath,['app/server.mjs'],{...process.env,PORT:String(port)});
 for(let idx=0;idx<100;idx++){try{if((await fetch(`http://127.0.0.1:${port}/api/health`)).ok)break;}catch{}await pause(100);}
 const chrome_env={...process.env};delete chrome_env.LD_LIBRARY_PATH;
 if(options.gpu!=='software')Object.assign(chrome_env,{__NV_PRIME_RENDER_OFFLOAD:options.gpu==='nvidia'?'1':'0',__GLX_VENDOR_LIBRARY_NAME:options.gpu==='nvidia'?'nvidia':'mesa',__EGL_VENDOR_LIBRARY_FILENAMES:options.gpu==='nvidia'?'/usr/share/glvnd/egl_vendor.d/10_nvidia.json':'/usr/share/glvnd/egl_vendor.d/50_mesa.json'});
 launch(process.env.CHROME_BIN||'/usr/bin/google-chrome',['--headless=new','--no-first-run','--disable-background-timer-throttling','--disable-renderer-backgrounding','--disable-dev-shm-usage',...(options.gpu==='software'?['--use-angle=swiftshader','--enable-unsafe-swiftshader']:['--use-gl=angle','--use-angle=gl-egl']),'--window-size=1440,1000','--remote-debugging-port=0','--user-data-dir='+profile_directory,'about:blank'],chrome_env);
 let pages;
 for(let idx=0;idx<100;idx++){try{const debug_port=(await fs.readFile(profile_directory+'/DevToolsActivePort','utf8')).split('\n')[0];pages=await(await fetch(`http://127.0.0.1:${debug_port}/json`)).json();if(pages.some(page=>page.type==='page'))break;}catch{}await pause(100);}
 assert.ok(pages,'Chromeの起動失敗');socket=new WebSocket(pages.find(page=>page.type==='page').webSocketDebuggerUrl);await once(socket,'open');
 let sequence=0;const pending=new Map(),exceptions=[];
 socket.onmessage=event=>{const message=JSON.parse(event.data);if(message.id){pending.get(message.id)?.(message);pending.delete(message.id);}if(message.method==='Runtime.exceptionThrown')exceptions.push(message.params);};
 const call=(method,params={})=>new Promise((resolve,reject)=>{const id=++sequence,timer=setTimeout(()=>{pending.delete(id);reject(Error('CDP時間超過: '+method));},60000);pending.set(id,message=>{clearTimeout(timer);message.error?reject(Error(JSON.stringify(message.error))):resolve(message.result);});socket.send(JSON.stringify({id,method,params}));});
 const evaluate=async expression=>{const result=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;};
 const wait_for=async expression=>{for(let idx=0;idx<200;idx++){if(await evaluate(expression))return;await pause(100);}throw Error('起動待機超過: '+expression);};
 await call('Runtime.enable');await call('Performance.enable');
 deadline=setTimeout(()=>{void cleanup().finally(()=>process.exit(124));},600000);
 for(const name of case_names){
  await call('Page.navigate',{url:`http://127.0.0.1:${port}/?model=long`});await wait_for('!!window.simulator?.diagnostics.ready');await pause(600);
  const environment=await evaluate(`(()=>{const gl=simulator.renderer.getContext(),ext=gl.getExtension('WEBGL_debug_renderer_info');return {renderer:ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):'unknown',user_agent:navigator.userAgent,triangles:simulator.robot.triangles,viewport:[innerWidth,innerHeight],canvas:[gl.drawingBufferWidth,gl.drawingBufferHeight]};})()`);
  if(options.gpu!=='software')assert.ok(!/SwiftShader|llvmpipe|softpipe|unknown/i.test(environment.renderer),'指定GPUが使用されていません: '+environment.renderer);
  report.environment=environment;
  if(name.startsWith('graph_')){
   await evaluate(`(${install_graph_fixture.toString()})(${num_nodes},${name==='graph_stream'?10:0});simulator.ros_results.open()`);
   await wait_for('!!simulator.ros_results.api');await evaluate('simulator.ros_results.api.connect()');
   await wait_for(`simulator.ros_results.api.graphData['/performance/graph']?.nodes.length===${num_nodes}`);
   await wait_for(`!!simulator.ros_results.scene.root_scene.getObjectByName('/performance/graph')`);
   const graph=await evaluate(`(()=>{const group=simulator.ros_results.scene.root_scene.getObjectByName('/performance/graph');let num_instances=0;group.traverse(object=>{if(object.isInstancedMesh&&object.visible)num_instances+=object.count;});return {num_nodes:simulator.ros_results.api.graphData['/performance/graph'].nodes.length,num_edges:simulator.ros_results.api.graphData['/performance/graph'].edges.length/2,num_instances,has_transform:group.userData.has_transform,packet_bytes:performance_fixture.packet_bytes};})()`);
   assert.ok(graph.has_transform&&graph.num_instances>=num_nodes);report.cases[name]={graph};await evaluate('performance_fixture.start()');
  }
  if(name==='frustum')await evaluate('simulator.rgbd.frustum.visible=true');
  if(name==='lidar')await evaluate('simulator.lidar.config.enabled=true;simulator.lidar.apply();simulator.lidar.sequence=0;simulator.lidar.scanTime=0');
  // 初回のシェーダー・BVH構築を定常計測から分離。
  if(name==='lidar'||name==='rgbd'){
   const warmup=await evaluate(`(async()=>{const start=performance.now(),frame=await simulator.${name}.capture();if(!frame)throw Error('センサ取得失敗');return {wall_ms:performance.now()-start,compute_ms:frame.ms};})()`);
   report.cases[name]={warmup};
  }
  await pause(500);await call('HeapProfiler.collectGarbage');
  const heap_before=await call('Runtime.getHeapUsage'),metrics_before=await call('Performance.getMetrics');
  // 描画呼出し時間はCPU送信・待機込み。GPU単体の実行時間とは別の値。
  await evaluate(`(()=>{window.performance_samples={frames:[],render:[],methods:{},is_running:true};const samples=performance_samples;let last;const tick=now=>{if(!samples.is_running)return;if(last!==undefined)samples.frames.push(now-last);last=now;requestAnimationFrame(tick);};requestAnimationFrame(tick);const renderer=simulator.renderer,render=renderer.render;renderer.render=function(...args){const start=performance.now();try{return render.apply(this,args);}finally{samples.render.push(performance.now()-start);}};for(const [object,method] of [[simulator.rgbd,'paint'],[simulator.rgbd,'updateFrustum'],[simulator.rgbd,'updateCloud'],[simulator.lidar,'updateCloud'],[simulator.lidar,'drawPattern']]){const original=object[method],key=(object===simulator.rgbd?'rgbd.':'lidar.')+method;samples.methods[key]=[];object[method]=function(...args){const start=performance.now();try{return original.apply(this,args);}finally{samples.methods[key].push(performance.now()-start);}};}})()`);
  if(options.profile){await call('Profiler.enable');await call('Profiler.start');}
  const started=Date.now();let captures;
  if(name==='lidar'||name==='rgbd')captures=await evaluate(`(async()=>{const values=[],capture_start=performance.now();for(let idx=0;idx<${num_captures}||performance.now()-capture_start<${duration_ms};idx++){${name==='lidar'?'simulator.lidar.sequence=0;simulator.lidar.scanTime=0;':''}const start=performance.now(),frame=await simulator.${name}.capture();if(!frame)throw Error('センサ取得失敗');values.push({wall_ms:performance.now()-start,compute_ms:frame.ms,render_ms:frame.renderMs??null,num_points:frame.count??frame.valid});await new Promise(resolve=>requestAnimationFrame(resolve));}return values;})()`);
  if(Date.now()-started<duration_ms)await pause(duration_ms-(Date.now()-started));
  if(options.profile){const profile=await call('Profiler.stop');await fs.writeFile(path.join(output,name+'.cpuprofile'),JSON.stringify(profile.profile));}
  const samples=await evaluate(`(()=>{performance_samples.is_running=false;return {...performance_samples,geometry_count:simulator.renderer.info.memory.geometries,texture_count:simulator.renderer.info.memory.textures,num_messages:window.performance_fixture?.sequence??0,errors:simulator.diagnostics.errors};})()`);
  const metrics_after=await call('Performance.getMetrics');await call('HeapProfiler.collectGarbage');const heap_after=await call('Runtime.getHeapUsage');
  const stats=values=>{if(!values.length)return {num:0};const sorted=[...values].sort((a,b)=>a-b);return {num:values.length,mean_ms:values.reduce((a,b)=>a+b,0)/values.length,p50_ms:sorted[Math.floor(sorted.length*.5)],p95_ms:sorted[Math.min(sorted.length-1,Math.floor(sorted.length*.95))],max_ms:sorted.at(-1)};};
  const before=Object.fromEntries(metrics_before.metrics.map(value=>[value.name,value.value])),after=Object.fromEntries(metrics_after.metrics.map(value=>[value.name,value.value]));
  const result={...report.cases[name],frame:stats(samples.frames),render_submit:stats(samples.render),methods:Object.fromEntries(Object.entries(samples.methods).map(([key,value])=>[key,stats(value)])),num_frames_over_50_ms:samples.frames.filter(value=>value>50).length,main_thread_task_ms:(after.TaskDuration-before.TaskDuration)*1000,elapsed_ms:Date.now()-started,heap_delta_bytes:heap_after.usedSize-heap_before.usedSize,geometry_count:samples.geometry_count,texture_count:samples.texture_count,num_messages:samples.num_messages};
  if(captures){
   result.output_signature=await evaluate(`(async()=>{const frame=${name==='lidar'?'simulator.lidar.last':'simulator.rgbd.sensor.lastFrame'},signatures={};for(const key of ${JSON.stringify(name==='lidar'?['xyz','range','slotStatus','objectId']:['xyz','depth','z16','colors','pixels'])}){const value=frame[key];signatures[key]=[value.length,Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',value))).map(byte=>byte.toString(16).padStart(2,'0')).join('')];}return signatures;})()`);
   result.captures=captures;result.capture_wall=stats(captures.map(value=>value.wall_ms));result.capture_compute=stats(captures.map(value=>value.compute_ms));assert.ok(captures.every(value=>value.num_points>0));}
  assert.ok(result.frame.num>=2);assert.deepEqual(samples.errors,[]);if(name==='graph_stream')assert.ok(result.num_messages>=2);
  // 非表示中の遅延更新と再表示時の内容一致。計測区間外での確認。
  if(name==='lidar'){
   await evaluate(`simulator.lidar.drawPattern(.3,simulator.lidar.config.duration,60000);document.querySelector('[data-panel=sensors]').click();document.querySelector('[data-sensor-panel=lidar]').click()`);
   await wait_for(`document.getElementById('lidar-pattern-preview').dataset.scanStart==='0.3'`);
   const is_preview_equal=await evaluate(`(()=>{const canvas=document.getElementById('lidar-pattern-preview'),context=canvas.getContext('2d'),before=context.getImageData(0,0,canvas.width,canvas.height).data;simulator.lidar.drawPattern(.3,simulator.lidar.config.duration,60000);const after=context.getImageData(0,0,canvas.width,canvas.height).data;return before.every((value,idx)=>value===after[idx]);})()`);
   assert.ok(is_preview_equal,'遅延描画と直接描画の不一致');result.is_preview_equal=true;
  }
  if(name==='rgbd'){
   const is_preview_equal=await evaluate(`(()=>{const sensor=simulator.rgbd,frame=sensor.sensor.lastFrame,canvases=['rgb-preview','depth-preview'].map(id=>document.getElementById(id));sensor.paint(frame);const before=canvases.map(canvas=>canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data);for(const canvas of canvases){canvas.width=1;canvas.height=1;}sensor.paint(frame);return canvases.every((canvas,idx)=>{const after=canvas.getContext('2d').getImageData(0,0,canvas.width,canvas.height).data;return after.length===before[idx].length&&after.every((value,idx_byte)=>value===before[idx][idx_byte]);});})()`);
   assert.ok(is_preview_equal,'解像度変更後のプレビュー復元失敗');result.is_preview_equal=true;
  }
  report.cases[name]=result;console.log(JSON.stringify({case:name,frame_p95_ms:result.frame.p95_ms,render_mean_ms:result.render_submit.mean_ms,capture_mean_ms:result.capture_wall?.mean_ms}));
 }
 assert.deepEqual(exceptions,[]);
 report.source_sha256=await read_source_hashes();assert.deepEqual(report.source_sha256,report.source_sha256_start,'計測中のソース変更');
 if(options.baseline){
  const baseline=JSON.parse(await fs.readFile(options.baseline,'utf8'));assert.equal(report.enable_profile,baseline.enable_profile??false,'プロファイル条件の不一致');assert.deepEqual(report.config,baseline.config,'比較条件の不一致');assert.deepEqual(report.environment,baseline.environment,'描画環境の不一致');
  report.comparison={};for(const [name,result] of Object.entries(report.cases)){
   const reference=baseline.cases[name];if(result.output_signature)assert.deepEqual(result.output_signature,reference.output_signature,name+': 出力の不一致');const metric=result.capture_wall?'capture_wall':'frame',key=metric==='frame'?'p95_ms':'mean_ms';
   const change_percent=(result[metric][key]/reference[metric][key]-1)*100;report.comparison[name]={metric:metric+'.'+key,change_percent};
   if(options['max-regression-percent']!==undefined&&change_percent>Number(options['max-regression-percent']))report.has_regression=true;
  }
 }
 report.status=report.has_regression?'regression':'passed';
} catch(error){report.status='failed';report.error=String(error);process.exitCode=1;console.error(error);}
finally{clearTimeout(deadline);await cleanup();report.has_stopped_processes=true;await fs.writeFile(path.join(output,'report.json'),JSON.stringify(report,null,2)+'\n');console.log('REPORT: '+path.join(output,'report.json'));}
if(report.has_regression)process.exitCode=1;
