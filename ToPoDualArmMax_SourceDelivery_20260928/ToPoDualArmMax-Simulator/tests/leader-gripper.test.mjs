import assert from 'node:assert/strict';
import test from 'node:test';
import {PhysicsPanel} from '../app/physics-ui.js';

// DOM・ROS・実機通信なしの物理目標換算
function panel_fixture(){
 const joints={R_joint1:{name:'R_joint1',lower:-2,upper:2},
  R_gripper_joint:{name:'R_gripper_joint',lower:0,upper:Math.PI/4},
  L_gripper_joint:{name:'L_gripper_joint',lower:0,upper:Math.PI/4}};
 const robot={joints,actuated:Object.values(joints)};
 return Object.assign(Object.create(PhysicsPanel.prototype),{
  socket:{},enable_dynamics:true,actual_joints:{},robot:()=>robot,robot_model:robot,
  is_leader_stopped:false,leader_anchor:null,
  actual:{R_joint1:.2,R_gripper_joint:0,L_gripper_joint:0},
  targets:{R_joint1:.2,R_gripper_joint:0,L_gripper_joint:0}});
}

test('腕の相対追従とグリッパー開度の絶対追従',()=>{
 const panel=panel_fixture(),actual={...panel.actual};
 panel.set_leader_target({R_joint1:1,R_gripper_joint:Math.PI/4,L_gripper_joint:Math.PI/4},100);
 assert.ok(Math.abs(panel.targets.R_joint1-.2)<1e-12);
 assert.equal(panel.targets.R_gripper_joint,Math.PI/4);
 assert.equal(panel.targets.L_gripper_joint,Math.PI/4);
 panel.set_leader_target({R_joint1:1.1,R_gripper_joint:Math.PI/8,L_gripper_joint:0},200);
 assert.ok(Math.abs(panel.targets.R_joint1-.3)<1e-12);
 assert.equal(panel.targets.R_gripper_joint,Math.PI/8);
 assert.equal(panel.targets.L_gripper_joint,0);
 assert.deepEqual(panel.actual,actual);
});

test('グリッパー可動域外の入力による目標更新の拒否',()=>{
 const panel=panel_fixture();
 panel.set_leader_target({R_joint1:0,R_gripper_joint:0},100);
 const targets={...panel.targets};
 assert.throws(()=>panel.set_leader_target({R_joint1:.1,R_gripper_joint:-.1},200),/可動域超過/);
 assert.deepEqual(panel.targets,targets);
 assert.equal(panel.leader_received_ms,100);
});

test('途中欠測による関節集合の変更の拒否',()=>{
 const panel=panel_fixture();
 panel.set_leader_target({R_joint1:0,R_gripper_joint:0,L_gripper_joint:0},100);
 assert.throws(()=>panel.set_leader_target({R_joint1:.1,R_gripper_joint:0},200),/関節集合の変更/);
});
