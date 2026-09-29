import * as THREE from 'three';

const surface_cache=new WeakMap();
const yield_ui=()=>new Promise(resolve=>setTimeout(resolve,0));

// 変換後の三角形面積に比例する表面サンプリング。裏面・内側のメッシュ面も対象
export async function sample_object_surface(group,num_points){
 if(!Number.isInteger(num_points)||num_points<1||num_points>200000)throw Error('点数は1〜200000を指定してください');
 group.updateWorldMatrix(true,true);
 const meshes=[];
 group.traverse(mesh=>{if(mesh.isMesh&&mesh.geometry?.attributes.position)meshes.push(mesh);});
 const matrices=meshes.map(mesh=>mesh.matrixWorld.clone());
 const signature=JSON.stringify(meshes.map(m=>[m.geometry.uuid,m.geometry.attributes.position.version,m.geometry.index?.version,...m.matrixWorld.elements]));
 let cached=surface_cache.get(group);
 if(!cached||cached.signature!==signature||cached.num_points!==num_points){
  const triangles=[],areas=[];let total_area=0;
  const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3(),ab=new THREE.Vector3(),ac=new THREE.Vector3();
  for(const [mesh_idx,mesh] of meshes.entries()){
   const pos=mesh.geometry.attributes.position,idx=mesh.geometry.index,matrix=matrices[mesh_idx],num=idx?idx.count:pos.count;
   for(let i=0;i+2<num;i+=3){
    a.fromBufferAttribute(pos,idx?idx.getX(i):i).applyMatrix4(matrix);
    b.fromBufferAttribute(pos,idx?idx.getX(i+1):i+1).applyMatrix4(matrix);
    c.fromBufferAttribute(pos,idx?idx.getX(i+2):i+2).applyMatrix4(matrix);
    const area=ab.subVectors(b,a).cross(ac.subVectors(c,a)).length()*.5;
    if(area>0&&Number.isFinite(area)){total_area+=area;areas.push(total_area);triangles.push([...a.toArray(),...b.toArray(),...c.toArray()]);}
    if(i%12288===0)await yield_ui();
   }
  }
  if(!total_area)throw Error('サンプリング可能な面がありません');
  let state=42;
  const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return (state>>>0)/4294967296;};
  const points=new Float32Array(num_points*3);
  for(let i=0;i<num_points;i++){
   const choice=random()*total_area;let lo=0,hi=areas.length-1;
   while(lo<hi){const mid=(lo+hi)>>1;if(choice<areas[mid])hi=mid;else lo=mid+1;}
   const tri=triangles[lo],u=Math.sqrt(random()),v=random();
   for(let k=0;k<3;k++)points[i*3+k]=(1-u)*tri[k]+u*(1-v)*tri[k+3]+u*v*tri[k+6];
   if(i%8192===0)await yield_ui();
  }
  cached={signature,num_points,points};surface_cache.set(group,cached);
 }
 return cached.points;
}
