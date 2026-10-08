import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import net from 'node:net';
import {fileURLToPath} from 'node:url';

// 既存のサーバー・ブラウザから独立した描画と計測の検証
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=fileURLToPath(new URL('../',import.meta.url));
const probe=net.createServer();probe.listen(0,'127.0.0.1');await once(probe,'listening');
const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
const server=spawn(process.execPath,['app/server.mjs'],{cwd:root,env:{...process.env,PORT:String(port)},stdio:'ignore'});
const base=`http://127.0.0.1:${port}`;
let browser;
const deadline=setTimeout(()=>{process.exitCode=1;void browser?.close();server.kill('SIGTERM');},180000);
try {
  for(let iter=0;iter<100;iter++){
    try{if((await fetch(base+'/api/health')).ok)break;}catch{}
    await new Promise(resolve=>setTimeout(resolve,50));
  }
  browser=await chromium.launch({headless:true,executablePath:process.env.CHROME_PATH||'/usr/bin/google-chrome',
    args:['--use-gl=angle','--use-angle=gl-egl','--enable-unsafe-swiftshader'],
    env:{...process.env,__NV_PRIME_RENDER_OFFLOAD:'0',__GLX_VENDOR_LIBRARY_NAME:'mesa',__EGL_VENDOR_LIBRARY_FILENAMES:'/usr/share/glvnd/egl_vendor.d/50_mesa.json'}});
  const page=await browser.newPage({viewport:{width:1400,height:1000}}),errors=[];
  page.on('pageerror',error=>errors.push(error.message));
  page.on('response',response=>{
    if(response.status()>=400&&new URL(response.url()).pathname!=='/api/status')errors.push(`${response.status()} ${response.url()}`);
  });
  await page.goto(base+'/?model=long');
  await page.waitForFunction(()=>window.simulator?.diagnostics.ready,null,{timeout:90000});
  const result=await page.evaluate(async()=>{
    const {robot,lidar,scene}=window.simulator;
    const check=(condition,message)=>{if(!condition)throw Error(message);};
    const near=(a,b)=>a.length===b.length&&a.every((value,idx)=>Math.abs(value-b[idx])<1e-10);
    const visible_sensor_meshes=()=>{const found=[];scene.traverseVisible(mesh=>{if(mesh.isMesh&&mesh.name.includes('chest_lidar_color_'))found.push(mesh);});return found;};
    const colors=()=>visible_sensor_meshes().map(mesh=>mesh.material.color.getHexString()).sort();
    // 実際の描画キャッシュ・片面材質による青色球面の全周確認
    const T=await import('/vendor/three/build/three.module.js');
    const blue=visible_sensor_meshes().find(mesh=>mesh.name.includes('chest_lidar_color_0'));
    const probe=new T.Mesh(blue.geometry,blue.material),ray=new T.Raycaster();
    let num_cover_rays=0;
    for(const z of [18,24,30])for(let deg=.37;deg<360;deg+=10){
      const angle=deg*Math.PI/180,direction=new T.Vector3(-Math.cos(angle),-Math.sin(angle),0);
      ray.set(new T.Vector3(40*Math.cos(angle),40*Math.sin(angle),z),direction);
      const hits=ray.intersectObject(probe,false);
      check(hits.length>0,`青色曲面の欠損: ${z} mm / ${deg} deg`);
      const expected=Math.sqrt(22**2-(z-13.48)**2);
      check(Math.abs(40-hits[0].distance-expected)<=.08,'STEP球面との寸法不一致');
      num_cover_rays++;
    }
    check(visible_sensor_meshes().length===4,'無効時の本体4材質');
    check(colors().join(',')==='110197,a0a0a0,dcdcdc,e0f2ff','STEP面色のsRGB変換');
    check(near(lidar.config.position,[.069326157159,0,.120847255132]),'LongのURDF由来配置');
    check(!lidar.support.visible&&!lidar.mountPlate.visible,'旧ブラケットの非表示');
    check(!lidar.config.enabled,'初期取得は無効');
    await lidar.configure({...lidar.config,enabled:true,scanPattern:'low-discrepancy',beams:5000,duration:.025});
    scene.updateMatrixWorld(true);
    check(visible_sensor_meshes().length===4,'有効時の二重表示なし');
    check(!lidar.model.visible&&!lidar.modelPromise,'旧STLの未読込');
    check(near(lidar.mount.matrixWorld.toArray(),robot.links.chest_lidar_link.matrixWorld.toArray()),'計測位置と筐体原点の一致');
    const original_pose=robot.getPose();
    window.simulator.apply_ros_pose({...original_pose,waist_joint:.6});scene.updateMatrixWorld(true);
    check(near(lidar.mount.matrixWorld.toArray(),robot.links.chest_lidar_link.matrixWorld.toArray()),'腰Yaw追従');
    const frame=await lidar.capture();
    check(frame?.count>0,'点群取得');
    check(near(frame.pose,robot.links.chest_lidar_link.matrixWorld.toArray()),'取得姿勢の固定');
    check(!Object.values(frame.names).some(name=>name.includes('chest_lidar_color_')),'センサ本体の自己交差除外');
    check(Object.values(frame.names).some(name=>name.includes('chest_lidar_mount_45')),'ブラケットの遮蔽対象化');
    lidar.live=true;
    const start=performance.now();
    while(lidar.last.id<frame.id+3){
      check(performance.now()-start<30000,'連続取得の時間上限');
      await new Promise(resolve=>setTimeout(resolve,50));
    }
    lidar.live=false;
    while(lidar.busy)await new Promise(resolve=>setTimeout(resolve,50));
    const pending=lidar.capture();lidar.resetForRobot(robot);await pending;
    check(!lidar.busy&&!lidar.pending,'取得中リセット');
    const custom={...lidar.config,parent:'world',position:[.3,.2,.7],rpy:[0,0,0]};
    await lidar.configure(custom);scene.updateMatrixWorld(true);
    check(visible_sensor_meshes().length===4,'任意配置の二重表示なし');
    check(!robot.links.chest_lidar_mount_link.visible,'任意配置の固定ブラケット非表示');
    check(near(lidar.mount.position.toArray(),custom.position),'任意配置の保持');
    await lidar.configure({...custom,enabled:false});
    check(visible_sensor_meshes().length===4&&robot.links.chest_lidar_mount_link.visible,'無効化でロボット本体の表示復帰');
    document.getElementById('lidar-waist-preset').click();
    await lidar.configure({...lidar.config,enabled:true});
    await window.simulator.switchModel('standard');
    check(near(lidar.config.position,[.07769,0,.105]),'標準の従来配置');
    await lidar.loadModel();
    check(lidar.model.visible&&lidar.has_legacy_model&&lidar.support.visible,'標準の従来描画');
    check(visible_sensor_meshes().length===0,'標準へのCAD表示漏れなし');
    await window.simulator.switchModel('long');
    check(near(lidar.config.position,[.069326157159,0,.120847255132]),'Long既定配置の復帰');
    check(visible_sensor_meshes().length===4&&!lidar.model.visible,'Long復帰の二重表示なし');
    await lidar.configure(custom);await window.simulator.switchModel('standard');await window.simulator.switchModel('long');
    check(near(lidar.config.position,custom.position)&&lidar.config.parent==='world','モデル切替時の任意配置保持');
    document.getElementById('lidar-waist-preset').click();
    await lidar.configure({...lidar.config,enabled:false});
    window.simulator.apply_ros_pose(original_pose);
    const {runLidarQA}=await import('/lidar-qa.js');const qa=await runLidarQA();
    check(qa.passed,JSON.stringify(qa));
    return {colors:colors(),points:frame.count,geometry_tests:qa.tests.length,num_cover_rays};
  });
  if(process.env.CHEST_LIDAR_SCREENSHOT)await page.screenshot({path:process.env.CHEST_LIDAR_SCREENSHOT});
  assert.deepEqual(errors,[],'ブラウザ・配信エラー');
  console.log('胸部LiDAR描画・計測・モデル切替: PASS',JSON.stringify(result));
} finally {
  clearTimeout(deadline);
  try {await browser?.close();} finally {
    if(server.exitCode===null&&server.signalCode===null){const exited=once(server,'exit');server.kill('SIGTERM');await exited;}
  }
  console.log('試験起動: node app/server.mjs / Chrome headless。専用プロセス停止済み');
}
