import * as THREE from 'three';
import { orientationError } from './robot.js';

export async function runQA(sim) {
 const robot=sim.robot, results=[], saved=robot.getPose();
 const test=(name,fn)=>{try{const details=fn();results.push({name,passed:true,details});}catch(e){results.push({name,passed:false,error:e.message});}};
 const assert=(condition,message)=>{if(!condition)throw new Error(message);};
 const close=(a,b,tolerance,message)=>assert(Math.abs(a-b)<tolerance,`${message}: ${a} vs ${b}`);
 const reference=await fetch(sim.model?.reference||'qa-fk-reference.json').then(r=>r.json());
 test('URDF structural completeness',()=>{assert(robot.actuated.length===19,'Expected 19 independent joints');assert(robot.chain('L').length===7&&robot.chain('R').length===7,'Expected seven joints per arm');assert(robot.renderMeshes.length===robot.visuals.length,'Every visual must render');return {links:Object.keys(robot.links).length,joints:Object.keys(robot.joints).length,independentJoints:robot.actuated.length,visuals:robot.renderMeshes.length,triangles:robot.triangles};});
 test('Independent home TCP positions for selected model',()=>{robot.setPose(sim.homePose());for(const side of ['L','R']){const p=robot.tcp(side).position,m=reference.home.tcp[side];close(p.x,m[12],1e-8,'X');close(p.y,m[13],1e-8,'Y');close(p.z,m[14],1e-8,'Z');}return reference.home.tcp;});
 test('Forward kinematics vs independent NumPy implementation',()=>{let maxError=0;for(const item of reference.cases){robot.setPose(item.pose);for(const side of ['L','R']){robot.tcp(side);const actual=robot.links[side+'_tcp'].matrixWorld.elements;const expected=item.tcp[side];for(let i=0;i<16;i++)maxError=Math.max(maxError,Math.abs(actual[i]-expected[i]));}}assert(maxError<1e-9,'Transform mismatch '+maxError);return {cases:reference.cases.length,maximumMatrixError:maxError};});
 test('Revolute limits clamp and continuous joint remains continuous',()=>{for(const j of robot.actuated){if(j.type!=='continuous'){robot.setJoint(j.name,100);close(j.q,j.upper,1e-10,j.name+' upper');robot.setJoint(j.name,-100);close(j.q,j.lower,1e-10,j.name+' lower');}}robot.setJoint('waist_joint',7);close(robot.joints.waist_joint.q,7,1e-10,'continuous');return 'All bounded joints checked at both limits';});
 test('Left/right gripper mimic joints',()=>{for(const side of ['L','R'])for(const q of [0,.3,.785398]){robot.setJoint(side+'_gripper_joint',q);close(robot.joints[side+'_gripper_mimic'].q,-q,1e-10,side+' finger mimic');}return 'Six checks: opposing finger angles and limits';});
 test('Independent neck Yaw / Pitch and waist Yaw transforms',()=>{
  robot.setPose(sim.homePose());const wrists={L:robot.tcp('L').position,R:robot.tcp('R').position};
  const cameraLink=robot.links.camera_link,initialCamera=cameraLink.getWorldQuaternion(new THREE.Quaternion());
  robot.setJoint('neck_pan_joint',.55);robot.updateMatrixWorld(true);
  close(cameraLink.getWorldQuaternion(new THREE.Quaternion()).angleTo(initialCamera),.55,1e-8,'neck yaw');
  for(const s of ['L','R'])close(robot.tcp(s).position.distanceTo(wrists[s]),0,1e-9,'neck yaw must not move arm');
  const afterYaw=cameraLink.getWorldQuaternion(new THREE.Quaternion());
  robot.setJoint('neck_tilt_joint',-.35);robot.updateMatrixWorld(true);
  close(cameraLink.getWorldQuaternion(new THREE.Quaternion()).angleTo(afterYaw),.35,1e-8,'neck pitch');
  for(const s of ['L','R'])close(robot.tcp(s).position.distanceTo(wrists[s]),0,1e-9,'neck pitch must not move arm');
  robot.setJoint('waist_joint',Math.PI/2);robot.updateMatrixWorld(true);
  for(const s of ['L','R']){const p=robot.tcp(s).position,b=wrists[s];close(p.x,-b.y,1e-8,'waist X');close(p.y,b.x,1e-8,'waist Y');close(p.z,b.z,1e-8,'waist Z');}
  return 'Neck axes independently rotate camera; waist rotates both arm TCPs by exactly 90 degrees';
 });
 for(const orientation of [false,true])test(orientation?'6D IK: reachable position and orientation':'3D IK: reachable position',()=>{
  const cases=[];for(const side of ['L','R'])for(let n=0;n<12;n++){
   const base=sim.presetPose('ready');robot.setPose(base);
   for(let j=1;j<=7;j++)robot.setJoint(`${side}_joint${j}`,base[`${side}_joint${j}`]+Math.sin(n*1.9+j*.77)*.18);
   const target=robot.tcp(side);robot.setPose(base);
   const result=sim.solveIK(robot,side,target,{orientation,iterations:160,damping:.004});
   const positionMm=result.position*1000,angleDeg=result.angle*180/Math.PI;
   assert(positionMm<1,`${side} case ${n}: ${positionMm} mm`);
   if(orientation)assert(angleDeg<1,`${side} case ${n}: ${angleDeg} deg`);
   cases.push({side,positionMm,angleDeg});
  }return {cases:cases.length,maxPositionMm:Math.max(...cases.map(x=>x.positionMm)),maxAngleDeg:orientation?Math.max(...cases.map(x=>x.angleDeg)):null};
 });
 test('Unreachable target stays finite and within limits',()=>{robot.setPose(sim.presetPose('ready'));for(const side of ['L','R']){const target={position:new THREE.Vector3(2,side==='L'?2:-2,2),quaternion:new THREE.Quaternion()};const result=sim.solveIK(robot,side,target,{orientation:true,iterations:160});assert(result.position>1,'Unreachable target must have residual');for(const j of robot.chain(side))assert(Number.isFinite(j.q)&&j.q>=j.lower&&j.q<=j.upper,'Invalid joint '+j.name);}return 'Both arms retain finite, limited joint angles';});
 test('Pose format validation',()=>{const pose=sim.homePose();sim.validatePose(pose);for(const bad of [{...pose,L_joint2:10},{...pose,L_joint1:NaN},{...pose,L_joint1:'0'}]){let rejected=false;try{sim.validatePose(bad);}catch{rejected=true;}assert(rejected,'Invalid pose accepted');}return 'Valid pose accepted; limit, NaN and type errors rejected';});
 robot.setPose(saved);sim.syncTargets();sim.renderer.shadowMap.needsUpdate=true;
 const report={timestamp:new Date().toISOString(),passed:results.every(r=>r.passed),tests:results,state:sim.getState()};
 const pre=document.getElementById('qa-results')||document.createElement('pre');pre.id='qa-results';pre.hidden=true;pre.textContent=JSON.stringify(report,null,2);document.body.append(pre);
 document.documentElement.dataset.qaPassed=String(report.passed);
 console.log('ToPo numerical QA',report);
 return report;
}
