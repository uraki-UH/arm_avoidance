import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {DRACOLoader} from 'three/addons/loaders/DRACOLoader.js';
export const VEHICLES={ferrari:'Ferrari 458 Italia',concept:'コンセプトクーペ'};
const draco=new DRACOLoader().setDecoderPath('./vendor/three/examples/jsm/libs/draco/gltf/');
const loader=new GLTFLoader().setDRACOLoader(draco),cache=new Map();
export async function createVehicle(type,color){
 if(!VEHICLES[type])throw Error('未対応の車種です');
 if(!cache.has(type))cache.set(type,loader.loadAsync(type==='ferrari'?'./assets/vehicles/ferrari.glb':'./assets/vehicles/CarConcept.glb').catch(e=>{cache.delete(type);throw e;}));
 const source=(await cache.get(type)).scene.clone(true),oriented=new THREE.Group();
 // glTF +Y up; Ferrari front -Z, CarConcept front +Z. Both become +X forward, +Z up.
 oriented.rotation.set(Math.PI/2,0,type==='concept'?Math.PI/2:-Math.PI/2,'ZYX');oriented.add(source);
 const g=new THREE.Group();g.add(oriented);g.updateMatrixWorld(true);
 const box=new THREE.Box3().setFromObject(g,true),size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3());
 oriented.position.set(-center.x,-center.y,-box.min.z);g.userData.nativeSize=size.toArray();g.userData.paint=[];
 g.traverse(o=>{if(!o.isMesh)return;o.castShadow=o.receiveShadow=true;o.geometry=o.geometry.clone();
  const process=m=>{m=m.clone();if(o.name==='body'||/Body_Color|Paint [12]/i.test(m.name)){
   if(!m.isMeshPhysicalMaterial)m=new THREE.MeshPhysicalMaterial({name:'Body paint',color,metalness:.65,roughness:.24,clearcoat:1,clearcoatRoughness:.08});
   m.color.set(color);m.clearcoat=1;m.roughness=Math.max(.2,m.roughness);g.userData.paint.push(m);
  }if(type==='ferrari'&&o.name==='glass')m=new THREE.MeshPhysicalMaterial({color:'#a9bbc8',metalness:.15,roughness:.07,transparent:true,opacity:.35,depthWrite:false,side:THREE.DoubleSide,clearcoat:1});return m;};o.material=Array.isArray(o.material)?o.material.map(process):process(o.material);
 });g.userData.primary=g.userData.paint[0];return g;
}
