import fs from 'node:fs';
import path from 'node:path';
import {spawn} from 'node:child_process';
import {randomUUID} from 'node:crypto';
import {createInterface} from 'node:readline';
import {isDeepStrictEqual} from 'node:util';

const allowed_keys=['policy','obstacle_shape','target_link','target_offset_m','duration_sec','seed','radius_m','speed_ratios','max_ball_acceleration','max_ball_deceleration','max_ball_turn_acceleration','execution_mode','speed_ramp_sec','enable_proximal_guard'];

export function validate_experiment(value,link_names){
 if(!value||Array.isArray(value)||typeof value!=='object'||Object.keys(value).some(key=>!allowed_keys.includes(key)))throw Error('未対応の実験設定');
 const config={policy:'rollout_cv',obstacle_shape:'sphere',target_link:'L_tcp',target_offset_m:[0,0,0],duration_sec:18,seed:401,radius_m:.06,speed_ratios:[.4,1.3],max_ball_acceleration:3,max_ball_deceleration:8,max_ball_turn_acceleration:10,execution_mode:'recorded',...value};
 if(!['recorded','continuous'].includes(config.execution_mode))throw Error('実行方式の不正');
 if(config.execution_mode==='continuous'){
  if(Object.hasOwn(value,'duration_sec')&&value.duration_sec!==0)throw Error('連続実験の時間指定不可');
  config.duration_sec=0;
 }
 if(!Object.hasOwn(value,'speed_ramp_sec'))config.speed_ramp_sec=config.execution_mode==='continuous'?18:config.duration_sec;
 if(!['rollout_cv','rollout_reactive','cbf_qp','oscbf'].includes(config.policy)||!['sphere','linked_arm'].includes(config.obstacle_shape)||!link_names.includes(config.target_link))throw Error('方式・形状・リンク名の不正');
 const bounded=(number,min,max)=>typeof number==='number'&&Number.isFinite(number)&&number>=min&&number<=max;
 for(const [key,min,max] of [['duration_sec',config.execution_mode==='continuous'?0:1,60],['speed_ramp_sec',1,60],['seed',0,2147483647],['radius_m',.015,.15],['max_ball_acceleration',.1,30],['max_ball_deceleration',.1,30],['max_ball_turn_acceleration',.1,30]])if(!bounded(config[key],min,max))throw Error('設定の範囲外: '+key);
 if(!Number.isInteger(config.seed)||!Array.isArray(config.speed_ratios)||config.speed_ratios.length!==2||!config.speed_ratios.every(v=>bounded(v,.05,3))||config.speed_ratios[0]>config.speed_ratios[1])throw Error('seed・速度比の不正');
 if(!Array.isArray(config.target_offset_m)||config.target_offset_m.length!==3||!config.target_offset_m.every(v=>bounded(v,-.5,.5))||Math.hypot(...config.target_offset_m)>.5)throw Error('追跡点オフセットの不正');
 if(Object.hasOwn(config,'enable_proximal_guard')&&typeof config.enable_proximal_guard!=='boolean')throw Error('近位部の迂回条件は真偽値');
 return {...config,enable_proximal_guard:config.enable_proximal_guard??true,enable_learning:false,enable_periodic_angles:false,enable_fixed_guard:true,start_dist_m:.45};
}

export class experiment_frame_window{
 constructor(max_num_frames=120){
  if(!Number.isSafeInteger(max_num_frames)||max_num_frames<1||max_num_frames>20000)throw Error('表示フレーム上限の不正');
  this.max_num_frames=max_num_frames;this.frames=[];this.num_frames=0;
 }
 append(frame){
  if(!Number.isFinite(frame?.time_sec)||frame.time_sec<0||frame.time_sec<=(this.frames.at(-1)?.time_sec??-1))return false;
  this.frames.push(frame);this.num_frames++;
  if(this.frames.length>this.max_num_frames)this.frames.splice(0,this.frames.length-this.max_num_frames);
  return true;
 }
 read(after){
  if(!Number.isSafeInteger(after)||after<0||after>this.num_frames)throw Error('逐次表示の受信位置不一致');
  const first_available=this.num_frames-this.frames.length,first_frame_idx=Math.max(after,first_available);
  return {first_frame_idx,num_frames:this.num_frames,frames:this.frames.slice(first_frame_idx-first_available),has_gap:after<first_available};
 }
}

export function delete_recording(root,value){
 if(!value||typeof value!=='object'||Array.isArray(value)||Object.keys(value).length!==1||!/^\/exports\/avoidance-[a-f0-9]{20}\.json$/.test(value.recording_url))throw Error('実験記録の指定不正');
 const directory=path.join(root,'exports'),lock=path.join(directory,'.avoidance-catalog.lock');
 try{fs.mkdirSync(lock);}catch(error){if(error.code==='EEXIST')throw Error('実験一覧を更新中です。再度操作してください');throw error;}
 const read_json=filename=>{const info=fs.lstatSync(filename);if(!info.isFile()||info.isSymbolicLink()||info.size>16*1024*1024)throw Error('記録ファイルの形式・容量の不正');return JSON.parse(fs.readFileSync(filename,'utf8'));};
 try{
  const catalog_path=path.join(directory,'avoidance-replays.json'),catalog=read_json(catalog_path);
  if(catalog.format!=='topo-avoidance-catalog/1'||!Array.isArray(catalog.runs))throw Error('実験一覧の形式不正');
  const entries=catalog.runs.filter(run=>run.url===value.recording_url);
  if(!entries.length)throw Error('一覧にない実験の削除不可');
  const recording_path=path.join(directory,path.basename(value.recording_url));
  if(fs.lstatSync(recording_path).isSymbolicLink()||!fs.lstatSync(recording_path).isFile())throw Error('通常の記録ファイルのみ削除可能');
  const recording=read_json(recording_path);
  if(!Array.isArray(recording.frames)||!recording.frames.length)throw Error('元データと照合できない記録');
  const workspace=path.resolve(root,'../../..'),artifact_root=path.join(workspace,'log/artifacts');
  const case_dirs=[...new Set(entries.map(entry=>entry.source_case_dir))].map(relative=>{
   if(typeof relative!=='string')throw Error('旧一覧のため再構築が必要です。--replay付きの起動で更新してください');
   const target=path.resolve(workspace,relative);
   if(!target.startsWith(artifact_root+path.sep)||fs.realpathSync(target)!==target||!fs.statSync(target).isDirectory())throw Error('元実験データのパス不正');
   const details=read_json(path.join(target,'details.json')),trajectory=path.join(target,'trajectory.jsonl');
   const info=fs.lstatSync(trajectory);
   if(!info.isFile()||info.isSymbolicLink()||info.size>16*1024*1024)throw Error('軌跡ファイルの形式・容量の不正');
   const frames=fs.readFileSync(trajectory,'utf8').trim().split('\n').map(line=>JSON.parse(line));
   const keys=Object.keys(recording.frames[0]);
   const expected=frames.map(frame=>Object.fromEntries(keys.map(key=>[key,frame[key]])));
   if(details.config?.policy!==recording.policy||details.config?.seed!==recording.seed||!isDeepStrictEqual(details.metrics,recording.metrics)||!isDeepStrictEqual(expected,recording.frames))throw Error('選択記録と元実験データの不一致。完全削除の中止');
   return target;
  });
  if(case_dirs.some(target=>case_dirs.some(other=>target!==other&&target.startsWith(other+path.sep))))throw Error('元実験ディレクトリの包含関係の不正');
  const urls=new Set(catalog.runs.filter(entry=>entries.some(selected=>entry.source_case_dir===selected.source_case_dir)).map(entry=>entry.url));
  const targets=[...case_dirs,...[...urls].map(url=>{
   if(!/^\/exports\/avoidance-[a-f0-9]{20}\.json$/.test(url))throw Error('関連記録のパス不正');
   return path.join(directory,path.basename(url));
  }).filter(filename=>fs.existsSync(filename))];
  const stage=fs.mkdtempSync(path.join(directory,'.avoidance-deleting-'));
  const deleted_dir=path.join(directory,'.avoidance-deleted'),temporary=path.join(stage,'catalog.tmp'),moved=[],markers=[];
  let has_committed=false;
  try{
   // 確定済みの個別対象だけの一時退避。確定前の失敗では元位置への復元。
   for(const [idx,target] of targets.entries()){
    const staged=path.join(stage,String(idx));fs.renameSync(target,staged);moved.push([target,staged]);
   }
   fs.mkdirSync(deleted_dir,{recursive:true});
   for(const url of urls){const marker=path.join(deleted_dir,path.basename(url));if(!fs.existsSync(marker)){fs.writeFileSync(marker,'',{flag:'wx'});markers.push(marker);}}
   catalog.runs=catalog.runs.filter(entry=>!urls.has(entry.url));
   if(!catalog.runs.some(entry=>entry.url===catalog.selected))catalog.selected=catalog.runs[0]?.url??null;
   fs.writeFileSync(temporary,JSON.stringify(catalog,null,2)+'\n',{flag:'wx',mode:0o644});
   fs.renameSync(temporary,catalog_path);has_committed=true;
   // 一覧確定後の完全消去。保持対象は再掲載防止用の空マーカーのみ。
   fs.rmSync(stage,{recursive:true});
   return {catalog,recording_url:value.recording_url,deleted_recording_urls:[...urls],num_deleted_cases:case_dirs.length,num_deleted_recordings:urls.size};
  }catch(error){
   if(!has_committed){
    for(const [target,staged] of moved.reverse())fs.renameSync(staged,target);
    for(const marker of markers)fs.unlinkSync(marker);
    fs.rmSync(stage,{recursive:true});
   }
   throw error;
  }
 }finally{fs.rmdirSync(lock);}
}

export function create_experiment_service(root,port){
 const simulator=path.resolve(root,'..'),workspace=path.resolve(simulator,'../..');
 const link_names=[...fs.readFileSync(path.join(root,'source.urdf'),'utf8').matchAll(/<link\s+name="([^"]+)"/g)].map(match=>match[1]);
 const is_available=fs.existsSync('/.dockerenv')&&fs.existsSync('/opt/ros/'+(process.env.ROS_DISTRO||'humble')+'/setup.bash');
 let job=null,child=null,timer=null,kill_timer=null;
 let live_metadata=null,live_window=new experiment_frame_window();
 const reply=(res,code,value)=>{res.writeHead(code,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store','X-Content-Type-Options':'nosniff'});res.end(JSON.stringify(value));};
 const snapshot=(after=null)=>{
  const live=after===null?null:live_window.read(after);
  return {is_available,live_protocol_version:1,archive_protocol_version:2,continuous_protocol_version:1,job:job?{...job,output:job.state==='failed'?job.output.slice(-1800):undefined,
   live:live?{...live,metadata:after===0||live.has_gap||!['running','stopping'].includes(job.state)?live_metadata:undefined}:undefined}:null};
 };
 const stop=reason=>{
  if(!child||child.exitCode!==null||child.signalCode!==null)return;
  job.state='stopping';job.stop_reason=reason;
  try{process.kill(-child.pid,'SIGINT');}catch(error){if(error.code!=='ESRCH')throw error;}
  const owned=child;
  kill_timer=setTimeout(()=>{try{process.kill(-owned.pid,'SIGKILL');}catch(error){if(error.code!=='ESRCH')console.error(error);}},3000);kill_timer.unref();
 };
 const read_body=req=>new Promise((resolve,reject)=>{
  let body='';req.setEncoding('utf8');req.on('data',chunk=>{body+=chunk;if(Buffer.byteLength(body)>8192){reject(Error('実験設定の容量超過'));req.destroy();}});
  req.on('end',()=>{try{resolve(JSON.parse(body));}catch{reject(Error('設定JSONの不正'));}});req.on('error',reject);
 });
 const start=config=>{
  const is_continuous=config.execution_mode==='continuous';
  const job_id=randomUUID(),relative_dir=is_continuous?null:'log/artifacts/pursuit_v2/manual_gui_'+job_id,case_dir=relative_dir?path.join(workspace,relative_dir):null;
  if(case_dir){fs.mkdirSync(case_dir,{recursive:true});fs.writeFileSync(path.join(case_dir,'config.json'),JSON.stringify(config,null,2)+'\n',{flag:'wx'});}
  job={job_id,state:'running',config,case_dir:relative_dir,started_ms:Date.now(),output:''};
  live_metadata=null;live_window=new experiment_frame_window(is_continuous?120:Math.ceil(config.duration_sec*20)+2);
  // シミュレーション専用の固定コマンド。シェル補間・ROS出力指定の受付なし。
  const command=[path.join(simulator,'start_ros.sh'),'--simulate','--scenario','chasing_sphere','--stream-frames','--scenario-file',is_continuous?'-':relative_dir+'/config.json'];
  if(!is_continuous)command.push('--case-dir',relative_dir);
  child=spawn('bash',command,{cwd:workspace,detached:true,env:{...process.env,PYTHONDONTWRITEBYTECODE:'1',OPENBLAS_NUM_THREADS:'1',OMP_NUM_THREADS:'1'},stdio:[is_continuous?'pipe':'ignore','pipe','pipe']});
  const owned=child,record=job;
  if(is_continuous){owned.stdin.on('error',error=>{if(error.code!=='EPIPE')record.output+=error.message;});owned.stdin.end(JSON.stringify(config));}
  const log=case_dir?fs.createWriteStream(path.join(case_dir,'execution.log'),{flags:'wx'}):null;
  const output=chunk=>{record.output=(record.output+chunk.toString()).slice(-32768);log?.write(chunk);};
  owned.stdout.on('data',output);owned.stderr.on('data',output);
  // GUIとは独立した標準出力の受信。低速な表示クライアントによる計算停止なし。
  const lines=createInterface({input:owned.stdout});
  lines.on('line',line=>{
   if(line.length>16384)return;
   let value;try{value=JSON.parse(line);}catch{return;}
   if(value.type==='experiment_ready'&&value.recording?.format==='topo-avoidance-replay/1')live_metadata=value.recording;
   if(value.type==='experiment_frame'&&live_metadata)live_window.append(value.frame);
   if(is_continuous&&value.type==='experiment_finished'){
    record.metrics=value.metrics;record.failure_reason=value.failure_reason;record.viewer_url='/?model=long&replay=latest';
    if(live_metadata)live_metadata={...live_metadata,metrics:value.metrics,failure_reason:value.failure_reason,failure_details:value.failure_details,is_valid_scenario:value.is_valid_scenario,outcome:value.outcome};
   }
   if(value.type==='replay'&&/^\/exports\/avoidance-[a-f0-9]+\.json$/.test(value.recording_url))record.recording_url=value.recording_url;
  });
  owned.on('error',error=>{record.output+=error.message;});
  owned.on('close',(code,signal)=>{
   clearTimeout(timer);clearTimeout(kill_timer);log?.end();child=null;
   record.finished_ms=Date.now();record.exit_code=code;record.signal=signal;
   record.state=record.stop_reason?'stopped':code===0?'completed':'failed';
   if(is_continuous){
    if(record.state==='completed'&&!record.metrics){record.state='failed';record.output+='連続実験の終了情報なし';}
    return;
   }
   if(record.state==='completed'){
    try{const details=JSON.parse(fs.readFileSync(path.join(case_dir,'details.json'),'utf8'));record.metrics=details.metrics;record.failure_reason=details.failure_reason;record.viewer_url='/?model=long&replay=latest';
     if(live_metadata)live_metadata={...live_metadata,metrics:details.metrics,failure_reason:details.failure_reason,failure_details:details.failure_details,is_valid_scenario:details.is_valid_scenario,outcome:details.outcome};}
    catch(error){record.state='failed';record.output+=error.message;}
   }
   if(live_metadata&&record.state!=='completed'){
    live_metadata={...live_metadata,is_partial:true,completion_state:record.state};
    if(live_window.frames.length)fs.writeFileSync(path.join(case_dir,'partial-recording.json'),JSON.stringify({...live_metadata,frames:live_window.frames})+'\n',{flag:'wx'});
   }
   fs.writeFileSync(path.join(case_dir,'execution.json'),JSON.stringify({...record,output:undefined},null,2)+'\n',{flag:'wx'});
  });
  if(!is_continuous){timer=setTimeout(()=>stop('wall_time_limit'),180000);timer.unref();}
 };
 return {
  async handle(req,res,pathname){
   if(req.method==='GET'&&pathname==='/api/avoidance-experiments'){
    const params=new URL(req.url,'http://localhost').searchParams,value=params.get('after');
    if(value!==null&&(!/^\d{1,16}$/.test(value)||!Number.isSafeInteger(Number(value))||Number(value)>live_window.num_frames)){reply(res,409,{message:'逐次表示の受信位置不一致'});return;}
    if(params.has('job_id')&&params.get('job_id')!==job?.job_id){reply(res,409,{message:'実験の切替。逐次表示の再同期が必要'});return;}
    reply(res,200,snapshot(value===null?null:Number(value)));return;
   }
   if(req.method!=='POST'||req.headers['x-topo-experiment']!=='1'||![`http://127.0.0.1:${port}`,`http://localhost:${port}`].includes(req.headers.origin)||!String(req.headers['content-type']).startsWith('application/json')){reply(res,403,{message:'同一画面からの実験操作のみ受付'});return;}
   try{
    const value=await read_body(req);
    if(pathname==='/api/avoidance-experiments/delete'){
     if(child){reply(res,409,{message:'実行中の削除不可。実験終了後に操作してください'});return;}
     const result=delete_recording(root,value);
     if(job?.recording_url&&result.deleted_recording_urls.includes(job.recording_url)){job=null;live_metadata=null;live_window=new experiment_frame_window();}
     reply(res,200,result);return;
    }
    if(pathname==='/api/avoidance-experiments/stop'){
     if(!job||value.job_id!==job.job_id){reply(res,409,{message:'この画面で開始した実験のみ停止可能'});return;}
     stop('user');reply(res,200,snapshot());return;
    }
    if(pathname!=='/api/avoidance-experiments'){reply(res,404,{message:'未対応の操作'});return;}
    if(!is_available){reply(res,503,{message:'実験の実行にはROSコンテナ内の画面サーバーが必要です'});return;}
    if(child){reply(res,409,{message:'実験を実行中です。終了後に開始してください'});return;}
    start(validate_experiment(value,link_names));reply(res,202,snapshot());
   }catch(error){if(!res.destroyed)reply(res,400,{message:error.message});}
  },
  async close(){stop('server_shutdown');if(child)await new Promise(resolve=>child.once('close',resolve));}
 };
}
