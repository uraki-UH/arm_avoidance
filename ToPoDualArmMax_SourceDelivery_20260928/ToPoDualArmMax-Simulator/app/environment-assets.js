import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {DRACOLoader} from 'three/addons/loaders/DRACOLoader.js';

const $=id=>document.getElementById(id);
export const environment_assets={littlest_tokyo:{
 name:'街並み：Littlest Tokyo',url:'./assets/environments/littlest_tokyo/LittlestTokyo.glb',
 author:'Glen Fox / glenatron',source:'https://artstation.com/artwork/1AGwX',
 license:'CC BY 4.0',license_url:'https://creativecommons.org/licenses/by/4.0/',width_m:12
}};

// 固定の表示・センサ用環境。物体の編集・物理対象とは独立した管理。
export class EnvironmentAssets {
 constructor(environment){
  this.environment=environment;this.asset_id='none';this.group=null;this.cache=new Map();this.req=0;
  const draco=new DRACOLoader().setDecoderPath('./vendor/three/examples/jsm/libs/draco/gltf/');
  this.loader=new GLTFLoader().setDRACOLoader(draco);
  const panel=document.createElement('section');panel.id='environment-assets';
  panel.innerHTML=`<div class="section-label">外部環境 <span>表示・センサ用</span></div><label>環境シーン<select id="environment-asset"><option value="none">なし</option>${Object.entries(environment_assets).map(([id,asset])=>`<option value="${id}">${asset.name}</option>`).join('')}</select></label><div class="row-actions"><button id="environment-asset-apply">環境を読み込む</button><button id="environment-asset-focus" disabled>環境全体を表示</button></div><output id="environment-asset-status" role="status">外部環境なし</output><p id="environment-asset-credit" class="sub-note"></p>`;
  $('environment-panel').querySelector('.panel-heading').insertAdjacentElement('afterend',panel);
  $('environment-asset-apply').onclick=async()=>{
   const button=$('environment-asset-apply');button.disabled=true;$('environment-asset-status').textContent='環境を読み込み中…';
   try{await this.set($('environment-asset').value);}
   catch(error){$('environment-asset-status').textContent='読込失敗（現在の環境は保持）：'+error.message;}
   finally{button.disabled=false;}
  };
  $('environment-asset-focus').onclick=()=>{if(this.group){const box=new THREE.Box3().setFromObject(this.group);box.expandByPoint(new THREE.Vector3(0,0,0));environment.focusBounds(box);}};
 }
 async prepare(asset_id){
  if(asset_id==='none')return null;
  if(!Object.hasOwn(environment_assets,asset_id))throw Error('未対応の環境シーンです');
  if(!this.cache.has(asset_id)){
   const asset=environment_assets[asset_id];
   const task=this.loader.loadAsync(asset.url).then(gltf=>{
    const group=new THREE.Group();group.name=asset.name;
    // glTFのY-upから本アプリのZ-upへの変換。元GLBへの変更なし。
    const oriented=new THREE.Group();oriented.rotation.x=Math.PI/2;oriented.add(gltf.scene);group.add(oriented);
    group.updateMatrixWorld(true);
    const bounds=new THREE.Box3().setFromObject(group),size=bounds.getSize(new THREE.Vector3());
    const width=Math.max(size.x,size.y);
    if(!Number.isFinite(width)||width<=0)throw Error('環境の寸法が不正です');
    oriented.scale.setScalar(asset.width_m/width);group.updateMatrixWorld(true);bounds.setFromObject(group);
    const center=bounds.getCenter(new THREE.Vector3());
    // 足元Z=-0.14 m、環境の左端X=1.5 m。既存の作業台と離した配置。
    group.position.set(1.5-bounds.min.x,-center.y,-.14-bounds.min.z);
    group.traverse(object=>{if(object.isMesh){object.castShadow=false;object.receiveShadow=true;}});
    group.updateMatrixWorld(true);return group;
   }).catch(error=>{this.cache.delete(asset_id);throw error;});
   this.cache.set(asset_id,task);
  }
  return this.cache.get(asset_id);
 }
 apply(asset_id,group){
  ++this.req;this.group?.removeFromParent();this.group=group;this.asset_id=asset_id;
  if(group)this.environment.scene.add(group);
  $('environment-asset').value=asset_id;$('environment-asset-focus').disabled=!group;
  const asset=environment_assets[asset_id],credit=$('environment-asset-credit');credit.replaceChildren();
  if(asset){
   const source=document.createElement('a');source.href=asset.source;source.textContent='Littlest Tokyo — '+asset.author;source.target='_blank';source.rel='noopener noreferrer';
   const license=document.createElement('a');license.href=asset.license_url;license.textContent=asset.license;license.target='_blank';license.rel='noopener noreferrer';credit.append(source,' / ',license);
  }
  $('environment-asset-status').textContent=asset?`${asset.name}：読込完了。水平最大寸法 ${asset.width_m} m（試験用）。物理衝突なし。`:'外部環境なし';
  this.environment.changed();
 }
 async set(asset_id){
  const req=++this.req,group=await this.prepare(asset_id);
  if(req!==this.req)return;
  this.apply(asset_id,group);
 }
}
