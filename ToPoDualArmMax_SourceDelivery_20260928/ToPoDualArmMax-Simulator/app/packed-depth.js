import * as THREE from 'three';

// float32のビット列をRGBA8へ格納。深度の量子化・間引きなし。
export function create_packed_depth_material(){
 return new THREE.ShaderMaterial({
  glslVersion:THREE.GLSL3,side:THREE.DoubleSide,toneMapped:false,blending:THREE.NoBlending,
  vertexShader:`out highp float metric_z;
   void main(){vec4 point=modelViewMatrix*vec4(position,1.0);metric_z=-point.z;gl_Position=projectionMatrix*point;}`,
  fragmentShader:`in highp float metric_z;
   out highp vec4 packed_depth;
   void main(){highp uint bits=floatBitsToUint(metric_z);
    packed_depth=vec4(uvec4(bits,bits>>8u,bits>>16u,bits>>24u)&uvec4(255u))/255.0;}`,
 });
}

const is_little_endian=new Uint8Array(new Uint32Array([1]).buffer)[0]===1;
// GPU下端原点のRGBA8から上端原点のfloat32配列への復元。
export function unpack_depth(raw,width,height){
 const output=new Float32Array(width*height);
 if(is_little_endian){
  const values=new Float32Array(raw.buffer,raw.byteOffset,raw.byteLength/4);
  for(let row=0;row<height;row++)output.set(values.subarray((height-1-row)*width,(height-row)*width),row*width);
 }else{
  const view=new DataView(raw.buffer,raw.byteOffset,raw.byteLength);
  for(let row=0;row<height;row++)for(let column=0;column<width;column++)output[row*width+column]=view.getFloat32(((height-1-row)*width+column)*4,true);
 }
 return output;
}
