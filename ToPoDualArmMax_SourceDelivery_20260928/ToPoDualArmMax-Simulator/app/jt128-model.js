import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';

// 公式CAD由来の筐体。計測原点・軸への変換は描画メッシュのみ
export async function load_jt128_cad(){
 const url=new URL('./assets/jt128/model.json',import.meta.url);
 const response=await fetch(url);
 if(!response.ok)throw Error('JT128 CAD: HTTP '+response.status);
 const metadata=await response.json(),group=new THREE.Group(),loader=new STLLoader();
 group.name='Hesai JT128 official CAD';
 const results=await Promise.allSettled(metadata.parts.map(async part=>{
  const geometry=await loader.loadAsync(new URL(part.file,url).href);
  // 曲面の滑らかな陰影と、溝・筐体境界の稜線の維持
  toCreasedNormals(geometry,Math.PI/4);
  geometry.scale(.001,.001,.001);
  geometry.rotateZ(metadata.cad_to_sensor_rot_z_deg*Math.PI/180);
  geometry.translate(0,0,-metadata.lidar_origin_height_mm*.001);
  const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:part.color,metalness:part.metalness,roughness:part.roughness}));
  mesh.name=part.name;mesh.castShadow=mesh.receiveShadow=true;
  return mesh;
 }));
 if(results.some(result=>result.status==='rejected')){
  for(const result of results)if(result.status==='fulfilled'){result.value.geometry.dispose();result.value.material.dispose();}
  throw results.find(result=>result.status==='rejected').reason;
 }
 for(const result of results)group.add(result.value);
 group.userData.cad=metadata;
 return group;
}
