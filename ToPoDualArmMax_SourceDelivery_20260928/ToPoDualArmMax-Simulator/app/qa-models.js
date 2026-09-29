import {runQA} from './qa-tests.js';

// Browser integration test: actual URDFs, GPU RGB-D and the triangle-raycast worker.
export async function runModelQA(sim){
 const tests=[],models={},poses={},frames={},savedModel=sim.model.id;
 const assert=(ok,message)=>{if(!ok)throw Error(message);};
 const test=async(name,fn)=>{try{tests.push({name,passed:true,details:await fn()});}catch(e){tests.push({name,passed:false,error:e.message});}};
 const waitUntil=async(fn)=>{const start=performance.now();while(!fn()){if(performance.now()-start>30000)throw Error('Sensor timeout');await new Promise(r=>setTimeout(r,30));}};
 const waitCapture=async()=>{const id=sim.lidar.last?.id;await sim.lidar.capture();await waitUntil(()=>!sim.lidar.busy&&sim.lidar.last?.id!==id);assert(sim.lidar.last,'Missing LiDAR frame');return sim.lidar.last;};
 const environment=JSON.stringify(sim.workspace.getState()),camera=sim.camera.matrixWorld.toArray();
 for(const id of ['standard','long']){
  await sim.switchModel(id);
  await test(id+': numerical kinematics',async()=>{const report=await runQA(sim);models[id]=report;assert(report.passed,JSON.stringify(report.tests.filter(t=>!t.passed)));return {tests:report.tests.length};});
  await test(id+': wrist axis and URDF joint type',()=>{for(const side of ['L','R']){const j=sim.robot.joints[side+'_joint7'];assert(j.type===(id==='standard'?'continuous':'revolute'),'Wrist type');assert(j.axis.toArray().join(',')===(id==='standard'?'0,0,1':'1,0,0'),'Wrist axis');sim.robot.setJoint(j.name,7);assert(Math.abs(j.q-(id==='standard'?7:j.upper))<1e-10,'Wrist rotation range');}sim.reset();return 'Both wrists checked';});
  await test(id+': only active robot and one set of controls',()=>{assert(sim.scene.children.filter(o=>o.modelId).length===1,'Stale robot in scene');assert(document.querySelectorAll('#joint-controls input').length===14,'Duplicate arm controls');assert(document.querySelectorAll('#body-controls input[type=number]').length===3,'Duplicate neck/waist controls');assert(sim.lidar.mount.parent===sim.robot.links.torso_link,'LiDAR parent');assert(sim.lidar.support.parent===sim.robot.links.torso_link,'Bracket parent');assert(sim.rgbd.robot===sim.robot&&sim.ai.robot===sim.robot,'Stale sensor/AI robot');return true;});
  await test(id+': RGB-D after model switch',async()=>{document.getElementById('cloud-show').checked=true;sim.rgbd.setCloud();const f=await sim.rgbd.capture();assert(f?.valid>100,'Empty RGB-D');assert(f.robotModel===id,'Wrong RGB-D model');assert(sim.rgbd.cloud.visible,'Point cloud not visible');assert(sim.rgbd.exclude.includes(sim.robot.links.camera_link),'Camera self-exclusion');return {model:f.robotModel,points:f.valid};});
  await test(id+': MID-360 after model switch',async()=>{await sim.lidar.configure({...sim.lidar.config,enabled:true});const f=await waitCapture();assert(f.count>100,'Empty LiDAR');assert(f.robotModel===id,'Wrong LiDAR model');return {model:f.robotModel,points:f.count,slots:f.config.beams};});
  sim.robot.setPose({...sim.presetPose('ready'),neck_pan_joint:id==='standard'?.25:-.35,waist_joint:id==='standard'?.3:-.4});sim.syncTargets();
  poses[id]=sim.robot.getPose();sim.keyframes.splice(0,sim.keyframes.length,poses[id]);frames[id]=structuredClone(sim.keyframes);
 }
 await test('In-flight old LiDAR data is discarded',async()=>{await sim.lidar.capture();assert(sim.lidar.busy,'Expected in-flight scan');await sim.switchModel('standard');await new Promise(r=>setTimeout(r,600));assert(sim.lidar.last===null&&!sim.lidar.busy,'Stale scan survived');assert(sim.rgbd.sensor.lastFrame===null,'Stale RGB-D survived');assert(sim.ai.frame===null&&!sim.ai.group.visible,'Stale AI overlay survived');return true;});
 await test('Model-specific pose and keyframe restore across four switches',async()=>{for(const id of ['long','standard','long','standard']){await sim.switchModel(id);assert(JSON.stringify(sim.robot.getPose())===JSON.stringify(poses[id]),id+' pose');assert(JSON.stringify(sim.keyframes)===JSON.stringify(frames[id]),id+' keyframes');}return true;});
 await test('Environment and view remain unchanged',async()=>{await sim.lidar.configure({...sim.lidar.config,enabled:false});assert(JSON.stringify(sim.workspace.getState())===environment,'Workspace changed');assert(sim.camera.matrixWorld.toArray().every((x,i)=>Math.abs(x-camera[i])<1e-10),'View changed');return true;});
 await sim.switchModel(savedModel);sim.reset();sim.keyframes.length=0;document.getElementById('clear-frames').click();
 document.getElementById('cloud-show').checked=false;sim.rgbd.setCloud();
 const report={timestamp:new Date().toISOString(),passed:tests.every(t=>t.passed),tests,models};
 const pre=document.createElement('pre');pre.id='model-qa-results';pre.hidden=true;pre.textContent=JSON.stringify(report,null,2);document.body.append(pre);
 document.documentElement.dataset.modelQaPassed=String(report.passed);console.log('Model switching QA',report);return report;
}
