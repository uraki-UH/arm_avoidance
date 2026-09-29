import * as THREE from 'three';

const rad = Math.PI / 180;
export function nominalCalibration(width=848,height=480) {
  const intrinsics=(w,h,hfov,vfov)=>({width:w,height:h,fx:w/(2*Math.tan(hfov*rad/2)),fy:h/(2*Math.tan(vfov*rad/2)),ppx:(w-1)/2,ppy:(h-1)/2,model:'none',coeffs:[0,0,0,0,0]});
  return {version:1,label:'D435i 公称画角・近似外部パラメータ（未実機校正）',depth:intrinsics(width,height,87,58),color:intrinsics(1280,720,69,42),
    depth_to_color:{rotation:[1,0,0,0,1,0,0,0,1],translation:[.015,0,0]},
    mount:{translation:[0,0,0],rpy:[0,0,0]},baseline_m:.05,depth_scale:.001,min_depth_m:width===848?.195:width===424?.105:.28,max_depth_m:3};
}
export function validateCalibration(value) {
  const c=structuredClone(value),finite=a=>Array.isArray(a)&&a.every(Number.isFinite);
  for(const name of ['depth','color']){
    const k=c[name];if(!k||![k.width,k.height].every(Number.isInteger)||k.width<16||k.height<16||k.width>1920||k.height>1080||![k.fx,k.fy,k.ppx,k.ppy].every(Number.isFinite)||k.fx<=0||k.fy<=0)throw Error(name+' の内部パラメータが不正です');
    if(k.model!=='none'||!finite(k.coeffs)||k.coeffs.length!==5||k.coeffs.some(x=>x!==0))throw Error('歪み補正済みの model:none / coeffs:0 のみ対応します');
    if(k.ppx<-.5||k.ppx>k.width-.5||k.ppy<-.5||k.ppy>k.height-.5)throw Error('主点が画像外です');
  }
  const ex=c.depth_to_color;if(!ex||!finite(ex.translation)||ex.translation.length!==3||ex.translation.some(x=>Math.abs(x)>.2)||!finite(ex.rotation)||ex.rotation.length!==9)throw Error('depth_to_color が不正です');
  const r=ex.rotation,m=new THREE.Matrix4().set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);
  const ident=m.clone().transpose().multiply(m).elements;
  if(ident.some((x,i)=>Math.abs(x-(i%5===0?1:0))>1e-5)||Math.abs(m.determinant()-1)>1e-5)throw Error('外部回転行列は右手系の正規直交行列にしてください');
  if(!c.mount||!finite(c.mount.translation)||c.mount.translation.length!==3||c.mount.translation.some(x=>Math.abs(x)>.5)||!finite(c.mount.rpy)||c.mount.rpy.length!==3)throw Error('mount が不正です');
  if(![c.baseline_m,c.depth_scale,c.min_depth_m,c.max_depth_m].every(Number.isFinite)||c.baseline_m<=0||c.baseline_m>.2||c.depth_scale<1e-6||c.depth_scale>.01||c.min_depth_m<.01||c.max_depth_m<=c.min_depth_m||c.max_depth_m>20||c.max_depth_m/c.depth_scale>65535)throw Error('距離範囲・深度単位・基線長が不正です');
  return c;
}
export function extrinsicMatrix(ex) {const r=ex.rotation,t=ex.translation;return new THREE.Matrix4().set(r[0],r[1],r[2],t[0],r[3],r[4],r[5],t[1],r[6],r[7],r[8],t[2],0,0,0,1);}
export function calibratedCamera(k,opticalToWorld,near=.005,far=20) {
  const camera=new THREE.PerspectiveCamera();camera.matrixAutoUpdate=false;
  camera.matrix.copy(opticalToWorld).multiply(new THREE.Matrix4().makeRotationX(Math.PI));
  camera.updateMatrixWorld(true);camera.near=near;camera.far=far;
  // SDK pixel coordinates refer to pixel centres; GL raster centres lie at u+.5/v+.5.
  camera.projectionMatrix.set(2*k.fx/k.width,0,1-2*(k.ppx+.5)/k.width,0,0,2*k.fy/k.height,2*(k.ppy+.5)/k.height-1,0,0,0,-(far+near)/(far-near),-2*far*near/(far-near),0,0,-1,0);
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();return camera;
}
export function deproject(u,v,z,k,out=new THREE.Vector3()) {return out.set((u-k.ppx)*z/k.fx,(v-k.ppy)*z/k.fy,z);}
export function project(p,k) {return {u:k.fx*p.x/p.z+k.ppx,v:k.fy*p.y/p.z+k.ppy,z:p.z};}

export class RGBDSensor {
  constructor(renderer,scene){
    this.renderer=renderer;this.scene=scene;this.calibration=validateCalibration(nominalCalibration());this.targets={};this.frameNumber=0;
    if(!renderer.extensions.has('EXT_color_buffer_float'))throw Error('RGB-DにはEXT_color_buffer_float対応GPUが必要です');
    this.depthMaterial=new THREE.ShaderMaterial({vertexShader:'varying float metricZ; void main(){vec4 p=modelViewMatrix*vec4(position,1.0);metricZ=-p.z;gl_Position=projectionMatrix*p;}',fragmentShader:'varying float metricZ; void main(){gl_FragColor=vec4(metricZ,0.0,0.0,1.0);}',side:THREE.DoubleSide,toneMapped:false});
  }
  configure(c){this.calibration=validateCalibration(c);}
  target(name,k,depth){
    let t=this.targets[name];
    if(t&&(t.width!==k.width||t.height!==k.height)){t.dispose();t=null;}
    if(!t){
      t=new THREE.WebGLRenderTarget(k.width,k.height,{type:depth?THREE.FloatType:THREE.UnsignedByteType,format:depth?THREE.RedFormat:THREE.RGBAFormat,minFilter:THREE.NearestFilter,magFilter:THREE.NearestFilter,depthBuffer:true,stencilBuffer:false});
      t.texture.colorSpace=depth?THREE.NoColorSpace:THREE.SRGBColorSpace;
      if(depth){
        // 単一成分読み出し非対応GPUでは従来のRGBA形式を使用
        const previous_target=this.renderer.getRenderTarget(),gl=this.renderer.getContext();
        try{this.renderer.setRenderTarget(t);if(gl.getParameter(gl.IMPLEMENTATION_COLOR_READ_FORMAT)!==gl.RED){t.dispose();t.texture.format=THREE.RGBAFormat;}}
        finally{this.renderer.setRenderTarget(previous_target);}
      }
      this.targets[name]=t;
    }
    return t;
  }
  render(k,world,name,depth,enable_async_read=false){
    const r=this.renderer,s=this.scene,target=this.target(name,k,depth),camera=calibratedCamera(k,world,.005,Math.max(20,this.calibration.max_depth_m+.1));
    const state={target:r.getRenderTarget(),override:s.overrideMaterial,background:s.background,fog:s.fog,clear:r.getClearColor(new THREE.Color()),alpha:r.getClearAlpha(),tone:r.toneMapping,auto:r.autoClear,shadows:r.shadowMap.enabled};
    const num_channels=depth&&target.texture.format===THREE.RedFormat?1:4;
    const raw=depth?new Float32Array(k.width*k.height*num_channels):new Uint8Array(k.width*k.height*4);
    let read_result;
    try {s.overrideMaterial=depth?this.depthMaterial:null;if(depth){s.background=null;s.fog=null;r.toneMapping=THREE.NoToneMapping;r.shadowMap.enabled=false;}r.autoClear=true;r.setClearColor(0,0);r.setRenderTarget(target);r.clear();r.render(s,camera);read_result=enable_async_read?r.readRenderTargetPixelsAsync(target,0,0,k.width,k.height,raw):r.readRenderTargetPixels(target,0,0,k.width,k.height,raw);}
    finally {const gl=r.getContext();gl.bindBuffer(gl.PIXEL_PACK_BUFFER,null);r.setRenderTarget(state.target);s.overrideMaterial=state.override;s.background=state.background;s.fog=state.fog;r.setClearColor(state.clear,state.alpha);r.toneMapping=state.tone;r.autoClear=state.auto;r.shadowMap.enabled=state.shadows;}
    const finish_read=()=>{
    const out=depth?new Float32Array(k.width*k.height):new Uint8ClampedArray(raw.length);
    for(let v=0;v<k.height;v++){const from=(k.height-1-v)*k.width;if(depth){for(let u=0;u<k.width;u++)out[v*k.width+u]=raw[(from+u)*num_channels];}else out.set(raw.subarray(from*4,(from+k.width)*4),v*k.width*4);}
    return out;
    };
    return enable_async_read?read_result.then(finish_read):finish_read();
  }
  opticalToWorld(urdfOpticalWorld){const c=this.calibration,mount=new THREE.Matrix4().compose(new THREE.Vector3(...c.mount.translation),new THREE.Quaternion().setFromEuler(new THREE.Euler(...c.mount.rpy,'ZYX')),new THREE.Vector3(1,1,1));return urdfOpticalWorld.clone().multiply(mount);}
  capture(urdfOpticalWorld,{mode='ideal',exclude=[],enable_async_read=false,target_group=null}={}){
    const start=performance.now(),c=structuredClone(this.calibration),k=c.depth,kc=c.color;
    const depthWorld=this.opticalToWorld(urdfOpticalWorld),depthToColor=extrinsicMatrix(c.depth_to_color),colorWorld=depthWorld.clone().multiply(depthToColor.clone().invert());
    const rightWorld=depthWorld.clone().multiply(new THREE.Matrix4().makeTranslation(c.baseline_m,0,0));
    const visible=exclude.map(x=>x.visible);let raw,colorZ,rgba,rightZ,target_depth;
    try{exclude.forEach(x=>x.visible=false);raw=this.render(k,depthWorld,'depth',true,enable_async_read);colorZ=this.render(kc,colorWorld,'colorDepth',true,enable_async_read);rgba=this.render(kc,colorWorld,'color',false,enable_async_read);if(mode==='stereo')rightZ=this.render(k,rightWorld,'rightDepth',true,enable_async_read);
      if(target_group){
        // 同じ姿勢の対象のみの深度とシーン全体の最前面深度を照合
        const members=new Set(),hidden=[];target_group.traverse(o=>members.add(o));
        this.scene.traverse(o=>{if(o.isMesh&&!members.has(o)){hidden.push([o,o.visible]);o.visible=false;}});
        try{target_depth=this.render(k,depthWorld,'targetDepth',true,enable_async_read);}
        finally{for(const [o,visible] of hidden)o.visible=visible;}
      }
}
    finally{exclude.forEach((x,i)=>x.visible=visible[i]);}
    const finish_capture=([raw,colorZ,rgba,rightZ,target_depth])=>{
    const count=k.width*k.height,depth=new Float32Array(count),z16=new Uint16Array(count),xyz=new Float32Array(count*3),colors=new Uint8Array(count*3),colorValid=new Uint8Array(count),pixels=new Uint32Array(count);
    const renderMs=performance.now()-start,em=depthToColor.elements,colorTolerance=.75/Math.min(kc.fx,kc.fy);let valid=0,colored=0,min=Infinity,max=0,stereoRejected=0;
    for(let v=0;v<k.height;v++)for(let u=0;u<k.width;u++){
      const i=v*k.width+u;let z=raw[i];if(target_depth&&target_depth[i]!==z)continue;if(!Number.isFinite(z)||z<c.min_depth_m||z>c.max_depth_m)continue;
      if(rightZ){const ur=Math.round(u-k.fx*c.baseline_m/z),zr=ur>=0&&ur<k.width?rightZ[v*k.width+ur]:0;if(!zr||Math.abs(zr-z)>Math.max(.002,z/k.fx)){stereoRejected++;continue;}z=Math.round(z/c.depth_scale)*c.depth_scale;if(z<c.min_depth_m||z>c.max_depth_m)continue;}
      depth[i]=z;z16[i]=Math.min(65535,Math.max(1,Math.round(z/c.depth_scale)));const x=(u-k.ppx)*z/k.fx,y=(v-k.ppy)*z/k.fy,j=valid*3;xyz[j]=x;xyz[j+1]=y;xyz[j+2]=z;pixels[valid]=i;
      const rx=em[0]*x+em[4]*y+em[8]*z+em[12],ry=em[1]*x+em[5]*y+em[9]*z+em[13],rz=em[2]*x+em[6]*y+em[10]*z+em[14],cu=Math.round(kc.fx*rx/rz+kc.ppx),cv=Math.round(kc.fy*ry/rz+kc.ppy);let hasColor=false;
      if(rz>0&&cu>=0&&cu<kc.width&&cv>=0&&cv<kc.height){const ci=cv*kc.width+cu,cz=colorZ[ci];hasColor=cz>0&&Math.abs(cz-rz)<=Math.max(.001,colorTolerance*rz);if(hasColor){const si=ci*4;colors[j]=rgba[si];colors[j+1]=rgba[si+1];colors[j+2]=rgba[si+2];colored++;}}
      if(!hasColor){colors[j]=155;colors[j+1]=165;colors[j+2]=175;}else colorValid[valid]=1;
      min=Math.min(min,z);max=Math.max(max,z);valid++;
    }
    const frame={id:++this.frameNumber,timestamp:new Date().toISOString(),calibration:c,mode,depth,z16,rgba,xyz:xyz.slice(0,valid*3),colors:colors.slice(0,valid*3),colorValid:colorValid.slice(0,valid),pixels:pixels.slice(0,valid),depthWorld:depthWorld.toArray(),colorWorld:colorWorld.toArray(),valid,colored,stereoRejected,min:valid?min:0,max:valid?max:0,renderMs,ms:performance.now()-start};
    if(!enable_async_read)this.lastFrame=frame;return frame;
    };
    // 同一姿勢の描画を発行後、GPU完了待ち中にブラウザへ制御を返却
    const reads=[raw,colorZ,rgba,rightZ,target_depth];
    return enable_async_read?Promise.all(reads).then(finish_capture):finish_capture(reads);
  }
  dispose(){Object.values(this.targets).forEach(x=>x.dispose());this.depthMaterial.dispose();}
}

export function binaryPLY(frame,world=false){
  const header=new TextEncoder().encode(`ply\nformat binary_little_endian 1.0\ncomment units meters\ncomment frame ${world?'base_footprint':'camera_depth_optical_frame'}\ncomment capture_id ${frame.id}\nelement vertex ${frame.valid}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar color_valid\nproperty uint pixel_index\nend_header\n`);
  const buffer=new ArrayBuffer(header.length+frame.valid*20);new Uint8Array(buffer).set(header);const view=new DataView(buffer),m=new THREE.Matrix4().fromArray(frame.depthWorld),p=new THREE.Vector3();
  for(let i=0;i<frame.valid;i++){p.fromArray(frame.xyz,i*3);if(world)p.applyMatrix4(m);const j=header.length+i*20;view.setFloat32(j,p.x,true);view.setFloat32(j+4,p.y,true);view.setFloat32(j+8,p.z,true);for(let c=0;c<3;c++)view.setUint8(j+12+c,frame.colors[i*3+c]);view.setUint8(j+15,frame.colorValid[i]);view.setUint32(j+16,frame.pixels[i],true);}return buffer;
}
