import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import fs from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {createRequire} from 'node:module';

// 専用ROS domain・MuJoCo・サーバー・Chromeによる物理フォロワーの通し試験
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const socket_class=globalThis.WebSocket??createRequire(import.meta.url)('undici').WebSocket;
const directory=await fs.mkdtemp('/tmp/topo-leader-browser-');
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const free_port=async()=>{const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));return port;};
const port=await free_port(),joint_port=await free_port(),origin=`http://127.0.0.1:${port}`;
let server,fixture,chrome,ws;
const stopped=async process=>{
 if(!process||process.exitCode!==null||process.signalCode!==null)return;
 const exited=once(process,'exit');process.kill('SIGTERM');
 let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,5000);})]);clearTimeout(timer);
 if(process.exitCode===null&&process.signalCode===null){process.kill('SIGKILL');await exited;}
};
try{
 server=spawn(process.execPath,['app/server.mjs'],{cwd:root,env:{...process.env,PORT:String(port)},stdio:'ignore'});
 const fixture_path='/ros2_ws/src/ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator/tests/leader_follower_fixture.py';
 fixture=spawn('docker',['exec','-i','-e','ROS_DOMAIN_ID=98','-e','ROS_LOCALHOST_ONLY=1','gng_cpu_container','bash','-lc',
  `source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec timeout -s INT -k 5 110 python3 -B ${fixture_path} --port ${joint_port} --origin ${origin}`],{stdio:['pipe','pipe','pipe']});
 let fixture_log='';fixture.stdout.on('data',data=>{fixture_log+=data;});fixture.stderr.on('data',data=>{fixture_log+=data;});
 for(let idx=0;idx<50;idx++){try{if((await fetch(origin+'/api/health')).ok)break;}catch{}await pause(100);}
 chrome=spawn(process.env.CHROME_BIN||'/usr/bin/google-chrome',['--headless=new','--no-first-run','--disable-dev-shm-usage',
  '--use-angle=swiftshader','--enable-unsafe-swiftshader','--window-size=1440,1000','--remote-debugging-port=0',
  '--user-data-dir='+directory,origin+'/?model=long'],{stdio:'ignore'});
 let pages;
 for(let idx=0;idx<100;idx++){try{const debug_port=(await fs.readFile(directory+'/DevToolsActivePort','utf8')).split('\n')[0];pages=await(await fetch(`http://127.0.0.1:${debug_port}/json`)).json();if(pages.some(page=>page.type==='page'))break;}catch{}await pause(100);}
 assert.ok(pages,'Chrome起動失敗');
 ws=new socket_class(pages.find(page=>page.type==='page').webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
 let seq=0;const requests=new Map();
 ws.onmessage=event=>{const value=JSON.parse(event.data);if(value.id){requests.get(value.id)?.(value);requests.delete(value.id);}};
 const evaluate=expression=>new Promise((resolve,reject)=>{
  const id=++seq,timer=setTimeout(()=>{requests.delete(id);reject(Error('ブラウザ試験時間超過'));},30000);
  requests.set(id,value=>{clearTimeout(timer);value.error||value.result.exceptionDetails?reject(Error(JSON.stringify(value))):resolve(value.result.result.value);});
  ws.send(JSON.stringify({id,method:'Runtime.evaluate',params:{expression,returnByValue:true,awaitPromise:true}}));
 });
 let is_ready=false;
 for(let idx=0;idx<150;idx++){is_ready=await evaluate('!!window.simulator?.diagnostics.ready');if(is_ready)break;await pause(150);}
 assert.ok(is_ready,'シミュレータ起動失敗');
 assert.equal(fixture.exitCode,null,fixture_log);
 const result=await evaluate(`(async()=>{
  const s=simulator,p=s.physics_panel,robot=s.robot,$=id=>document.getElementById(id);
  const check=(value,message)=>{if(!value)throw Error(message);};
  const wait_for=async(predicate,max_ms=15000)=>{const until=performance.now()+max_ms;while(performance.now()<until){if(predicate())return;await new Promise(resolve=>setTimeout(resolve,30));}throw Error(JSON.stringify({status:$('physics-status').textContent,joints:$('ros-joints-status').textContent,pose_source:robot.pose_source,stamp:s.ros_points.robot_panel.joint_stream.latest_stamp_sec,wall_sec:Date.now()/1000,anchor:p.leader_anchor,actual:p.actual.R_joint1,target:p.targets.R_joint1}));};
  $('ros-endpoint').value='http://127.0.0.1:${joint_port-1}';
  // ソフトウェア描画の負荷除去。MuJoCoのロボット形状・駆動への変更なし
  $('robot-visible').checked=false;$('robot-visible').dispatchEvent(new Event('change'));
  if(s.renderer.shadowMap.enabled)$('enable-shadows').click();
  let leader_q=.5,is_sending=true;
  const sender=new WebSocket('ws://127.0.0.1:${joint_port}/joints');
  await new Promise((resolve,reject)=>{sender.onopen=resolve;sender.onerror=reject;});
  sender.send(JSON.stringify({type:'config',model:'long',hz:100,receive:false}));
  await new Promise((resolve,reject)=>{sender.onmessage=event=>JSON.parse(event.data).type==='ready'?resolve():reject(Error(event.data));});
  const timer=setInterval(()=>{if(is_sending&&sender.readyState===1)sender.send(JSON.stringify({type:'joints',pose:{R_joint1:leader_q}}));},40);
  try{
   $('robot-pose-source').value='leader';$('robot-pose-source').dispatchEvent(new Event('change'));
   await wait_for(()=>s.ros_points.robot_panel.joint_stream.is_ready);
   p.start();await wait_for(()=>!!p.actual_joints&&!!p.leader_anchor);
   const initial=p.actual.R_joint1;check(Math.abs(initial)<.03,'開始時の姿勢飛び');
   leader_q=.7;
   await wait_for(()=>p.targets.R_joint1>.15);
   const immediate=p.actual.R_joint1,goal=p.targets.R_joint1;
   check(Math.abs(immediate-goal)>.005,'物理実姿勢の目標への直接代入');
   await wait_for(()=>Math.abs(p.actual.R_joint1-.2)<.02);
   await wait_for(()=>Math.abs(robot.getPose().R_joint1-p.actual.R_joint1)<.02);
   const visible_q=robot.getPose().R_joint1;robot.setJoint('R_joint1',1.);check(robot.getPose().R_joint1===visible_q,'手動操作の排他');
   is_sending=false;await wait_for(()=>p.is_leader_stopped);
   const stopped_target=p.targets.R_joint1;
   leader_q=1.;is_sending=true;await new Promise(resolve=>setTimeout(resolve,400));
   check(p.is_leader_stopped&&p.targets.R_joint1===stopped_target,'通信復旧時の自動再開');
   p.start();await wait_for(()=>!!p.actual_joints&&!!p.leader_anchor&&!p.is_leader_stopped);
   check(Math.abs(p.targets.R_joint1-p.actual.R_joint1)<.03,'再開始時の基準姿勢');
   s.ros_points.robot_panel.joint_stream.stop();check(p.is_leader_stopped,'通信OFF時の追従停止');
   return {initial,immediate,goal,stopped_target,has_actual_physics:!!p.actual_joints};
  }finally{clearInterval(timer);sender.close();p.stop();s.ros_points.robot_panel.stop();}
 })()`);
 console.log('PASS: ROS→物理モータ追従・実姿勢描画・無跳躍開始・入力途絶停止・非自動再開',JSON.stringify(result));
}finally{
 ws?.close();await stopped(chrome);await stopped(server);
 if(fixture&&fixture.exitCode===null){const exited=once(fixture,'exit');fixture.stdin.end();let timer;await Promise.race([exited,new Promise(resolve=>{timer=setTimeout(resolve,120000);})]);clearTimeout(timer);assert.notEqual(fixture.exitCode,null,'試験用ROS・物理ブリッジの残留');}
 await fs.rm(directory,{recursive:true,force:true});
}
