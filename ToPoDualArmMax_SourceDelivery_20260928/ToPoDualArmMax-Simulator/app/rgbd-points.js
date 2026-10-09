// 点群生成のJavaScript参照経路。WebAssemblyと同じ演算順・画素順。
export function process_points_js(c,em,raw,color_depth,rgba,right_depth,target_depth,enable_direct_depth){
 const k=c.depth,kc=c.color;
    const count=k.width*k.height,depth=new Float32Array(count),z16=new Uint16Array(count),xyz=new Float32Array(count*3),colors=new Uint8Array(count*3),color_valid=new Uint8Array(count),pixels=new Uint32Array(count);
    const color_tolerance=.75/Math.min(kc.fx,kc.fy);let valid=0,colored=0,min=Infinity,max=0,num_stereo_rejected=0;
    for(let v=0;v<k.height;v++){const row=(enable_direct_depth?k.height-1-v:v)*k.width;for(let u=0;u<k.width;u++){
      const source_idx=row+u,i=v*k.width+u;let z=raw[source_idx];if(target_depth&&target_depth[source_idx]!==z)continue;if(!Number.isFinite(z)||z<c.min_depth_m||z>c.max_depth_m)continue;
      if(right_depth){const ur=Math.round(u-k.fx*c.baseline_m/z),zr=ur>=0&&ur<k.width?right_depth[row+ur]:0;if(!zr||Math.abs(zr-z)>Math.max(.002,z/k.fx)){num_stereo_rejected++;continue;}z=Math.round(z/c.depth_scale)*c.depth_scale;if(z<c.min_depth_m||z>c.max_depth_m)continue;}
      depth[i]=z;z16[i]=Math.min(65535,Math.max(1,Math.round(z/c.depth_scale)));const x=(u-k.ppx)*z/k.fx,y=(v-k.ppy)*z/k.fy,j=valid*3;xyz[j]=x;xyz[j+1]=y;xyz[j+2]=z;pixels[valid]=i;
      const rx=em[0]*x+em[4]*y+em[8]*z+em[12],ry=em[1]*x+em[5]*y+em[9]*z+em[13],rz=em[2]*x+em[6]*y+em[10]*z+em[14],cu=Math.round(kc.fx*rx/rz+kc.ppx),cv=Math.round(kc.fy*ry/rz+kc.ppy);let has_color=false;
      if(rz>0&&cu>=0&&cu<kc.width&&cv>=0&&cv<kc.height){const ci=cv*kc.width+cu,cz=color_depth[(enable_direct_depth?kc.height-1-cv:cv)*kc.width+cu];has_color=cz>0&&Math.abs(cz-rz)<=Math.max(.001,color_tolerance*rz);if(has_color){const si=ci*4;colors[j]=rgba[si];colors[j+1]=rgba[si+1];colors[j+2]=rgba[si+2];colored++;}}
      if(!has_color){colors[j]=155;colors[j+1]=165;colors[j+2]=175;}else color_valid[valid]=1;
      min=Math.min(min,z);max=Math.max(max,z);valid++;
    }}
 return {depth,z16,xyz:xyz.slice(0,valid*3),colors:colors.slice(0,valid*3),colorValid:color_valid.slice(0,valid),pixels:pixels.slice(0,valid),valid,colored,stereoRejected:num_stereo_rejected,min:valid?min:0,max:valid?max:0};
}

let kernel=null,kernel_loading=null;
export function ready_points_kernel(){return kernel;}
export function load_points_kernel(){
 if(!kernel_loading)kernel_loading=(async()=>{
  try{
   const response=await fetch(new URL('./rgbd-points.wasm',import.meta.url));
   if(!response.ok)throw Error('点群処理の読込失敗');
   const {instance}=await WebAssembly.instantiate(await response.arrayBuffer());
   kernel=new points_kernel(instance.exports);return kernel;
  }catch{return null;}
 })();
 return kernel_loading;
}
class points_kernel {
 constructor(exports){this.exports=exports;this.memory=exports.memory;this.start=Number(exports.__heap_base.value);}
 process(c,matrix,raw,color_depth,rgba,right_depth,target_depth,enable_direct_depth){
  const count=c.depth.width*c.depth.height,layout={};let offset=this.start;
  const reserve=(name,array_type,length)=>{offset=Math.ceil(offset/8)*8;layout[name]={offset,array_type,length};offset+=length*array_type.BYTES_PER_ELEMENT;};
  reserve('calibration',Float64Array,33);
  for(const [name,array] of Object.entries({raw,color_depth,rgba,right_depth,target_depth}))if(array)reserve(name,array.constructor,array.length);
  for(const [name,type,num] of [['depth',Float32Array,count],['z16',Uint16Array,count],['xyz',Float32Array,count*3],['colors',Uint8Array,count*3],['colorValid',Uint8Array,count],['pixels',Uint32Array,count],['statistics',Float64Array,5]])reserve(name,type,num);
  const num_pages=Math.ceil((offset-this.memory.buffer.byteLength)/65536);if(num_pages>0)this.memory.grow(num_pages);
  const view=name=>{const field=layout[name];return new field.array_type(this.memory.buffer,field.offset,field.length);};
  const k=c.depth,color=c.color;
  view('calibration').set([k.width,k.height,k.fx,k.fy,k.ppx,k.ppy,color.width,color.height,color.fx,color.fy,color.ppx,color.ppy,c.baseline_m,c.depth_scale,c.min_depth_m,c.max_depth_m,enable_direct_depth?1:0,...matrix]);
  for(const [name,array] of Object.entries({raw,color_depth,rgba,right_depth,target_depth}))if(array)view(name).set(array);
  this.exports.process_points(...['calibration','raw','color_depth','rgba','right_depth','target_depth','depth','z16','xyz','colors','colorValid','pixels','statistics'].map(name=>layout[name]?.offset??0));
  const [valid,colored,num_stereo_rejected,min,max]=view('statistics');
  // 次の呼出しによる上書きを防ぐ、公開配列の独立所有
  return {depth:view('depth').slice(),z16:view('z16').slice(),xyz:view('xyz').slice(0,valid*3),colors:view('colors').slice(0,valid*3),colorValid:view('colorValid').slice(0,valid),pixels:view('pixels').slice(0,valid),valid,colored,stereoRejected:num_stereo_rejected,min,max};
 }
}
