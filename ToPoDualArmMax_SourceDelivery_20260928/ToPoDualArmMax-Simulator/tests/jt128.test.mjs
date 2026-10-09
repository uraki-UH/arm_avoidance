import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../app/vendor/three/build/three.module.js';
import {jt128_channels,jt128_scan} from '../app/jt128-scan.js';
import {lidar_preset,validateLidarConfig,scanDirection,scan,buildGeometry,prepareMeshes} from '../app/lidar-core.js';

test('JT128の128方向・900列・1回転の再現とMID-360旧設定の互換性',()=>{
 const config=validateLidarConfig({sensor_type:'jt128'});
 assert.equal(config.beams,115200);assert.equal(config.maxRange,60);assert.equal(config.scanPattern,'jt128');
 assert.equal(jt128_channels.length,128);assert.equal(jt128_scan.nominal_rays_per_sec,config.beams/config.duration);
 for(let idx=0;idx<128;idx++){
  const ray=scanDirection(idx,new THREE.Vector3(),'jt128');
  assert.ok(Math.abs(ray.length()-1)<1e-12);
  assert.ok(Math.abs(Math.asin(ray.z)*180/Math.PI-jt128_channels[idx][1])<1e-9);
  assert.ok(ray.distanceTo(scanDirection(idx+115200,new THREE.Vector3(),'jt128'))<1e-12);
 }
 const a=scanDirection(0,new THREE.Vector3(),'jt128'),b=scanDirection(128,new THREE.Vector3(),'jt128');
 assert.ok(Math.abs(Math.atan2(b.y,b.x)-Math.atan2(a.y,a.x)+.4*Math.PI/180)<1e-12);
 assert.equal(validateLidarConfig({parent:'neck_tilt_link'}).sensor_type,'mid360');
 for(const invalid of [{sensor_type:'unknown'},{sensor_type:'jt128',scanPattern:'measured'},{sensor_type:'jt128',maxRange:61},{sensor_type:'mid360',scanPattern:'jt128'}])assert.throws(()=>validateLidarConfig(invalid));
});

test('近距離のチャンネル制限・遮蔽物の不透過・分割取得の連続性',()=>{
 const geometry=new THREE.BoxGeometry(.4,.4,.4),packed=buildGeometry({position:geometry.attributes.position.array,index:geometry.index.array}),matrix=new THREE.Matrix4().toArray();
 const meshes=prepareMeshes([{id:1,geometry:'room',matrix}],new Map([['room',packed]]));
 try{
  const config={...lidar_preset('jt128'),duration:128/1152000,beams:128};
  const frame=scan(config,matrix,meshes);
  assert.equal(frame.count,63);assert.equal(frame.nearRejected,65);assert.equal(frame.noReturn,0);
  assert.equal(frame.count+frame.nearRejected,128);
  for(const beam_idx of frame.beamIndex)assert.equal(jt128_channels[beam_idx][2],0);
  const all=scan({...config,beams:256,duration:256/1152000},matrix,meshes);
  const second=scan(config,matrix,meshes,128,128/1152000);
  assert.deepEqual(all.xyz,new Float32Array([...frame.xyz,...second.xyz]));
  const empty=scan(config,matrix,[]);assert.equal(empty.count,0);assert.equal(empty.noReturn,128);
 }finally{geometry.dispose();packed.dispose();}
});
