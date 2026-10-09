import assert from 'node:assert/strict';
import test from 'node:test';
import {RosJointStream} from '../app/ros-joints.js';

// 通信worker・DOMの代替による、実測受信から表示更新までの検証
function stream_fixture(t,source='ros'){
 const saved={document:globalThis.document,window:globalThis.window,Worker:globalThis.Worker};
 t.after(()=>{for(const [name,value] of Object.entries(saved)){if(value===undefined)delete globalThis[name];else globalThis[name]=value;}});
 const elements=Object.fromEntries(['ros-joints-receive','robot-pose-source','ros-joints-hz','ros-endpoint','ros-joints-topic','ros-joints-status','ros-state-all','ros-joints-send','ros-base-send','ros-tf-send'].map(name=>[name,{checked:false,value:''}]));
 Object.assign(elements['ros-joints-receive'],{checked:true});
 elements['robot-pose-source'].value=source;
 elements['ros-joints-hz'].value='100';elements['ros-endpoint'].value='http://127.0.0.1:8879';elements['ros-joints-topic'].value='/follower/joint_states';
 globalThis.document={getElementById:name=>elements[name]};
 const rendered=[],targets=[];
 globalThis.window={simulator:{apply_ros_pose:pose=>rendered.push(pose),physics_panel:{max_leader_age_ms:1000,set_leader_target:pose=>{targets.push(pose);return true;}}}};
 globalThis.Worker=class {postMessage(){} terminate(){}};
 const max_position=Math.PI/4;
 const robot={modelId:'long',pose_source:source,joints:{
  R_joint2:{lower:-2,upper:2},R_gripper_joint:{lower:0,upper:max_position},L_gripper_joint:{lower:0,upper:max_position},
  R_gripper_mimic:{lower:-max_position,upper:0},L_gripper_mimic:{lower:-max_position,upper:0}}};
 const stream=Object.assign(Object.create(RosJointStream.prototype),{socket:null,panel:{rgbd:{robot},instance_panel:{set_source:value=>{robot.pose_source=value;}}}});
 stream.connect();
 const receive=pose=>stream.socket.onmessage({data:{type:'joints',pose,stamp_sec:Date.now()/1000-.01}});
 return {stream,receive,rendered,targets,elements,max_position};
}

test('実測グリッパーの開閉端表示と閉じ方向への継続追従',t=>{
 const f=stream_fixture(t),rad=Math.PI/180;
 f.receive({R_gripper_joint:40*rad,L_gripper_joint:46*rad,R_gripper_mimic:-40*rad,L_gripper_mimic:-46*rad});f.stream.tick();
 assert.equal(f.rendered[0].R_gripper_joint,40*rad);
 assert.equal(f.rendered[0].L_gripper_joint,f.max_position);
 assert.equal(f.rendered[0].R_gripper_mimic,-40*rad);
 assert.equal(f.rendered[0].L_gripper_mimic,-f.max_position);
 f.receive({R_gripper_joint:0,L_gripper_joint:0});f.stream.tick();
 assert.deepEqual(f.rendered[1],{R_gripper_joint:0,L_gripper_joint:0});
});

test('可動域外の腕を保持したままグリッパー表示の更新',t=>{
 const f=stream_fixture(t);
 f.receive({R_joint2:3,R_gripper_joint:.2,L_gripper_joint:.3});f.stream.tick();
 assert.deepEqual(f.rendered[0],{R_gripper_joint:.2,L_gripper_joint:.3});
 assert.match(f.elements['ros-joints-status'].textContent,/R_joint2/);
});

test('リーダー制御入力の丸め込みなし',t=>{
 const f=stream_fixture(t,'leader'),pose={L_gripper_joint:1};
 f.receive(pose);
 assert.deepEqual(f.targets,[pose]);
 assert.deepEqual(f.rendered,[]);
});

test('非有限のグリッパー実測による受信停止',t=>{
 const f=stream_fixture(t);
 f.receive({R_gripper_joint:NaN});
 assert.equal(f.stream.socket,null);
 assert.equal(f.elements['ros-joints-receive'].checked,false);
 assert.deepEqual(f.rendered,[]);
});
