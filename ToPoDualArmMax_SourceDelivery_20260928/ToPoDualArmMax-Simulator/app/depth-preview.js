// GPUによる表示用深度ヒートマップ。計測用の深度・点群配列への変更なし。
export class depth_preview {
 constructor(){
  this.canvas=document.createElement('canvas');
  try{this.context=this.canvas.getContext('webgl2',{alpha:false,antialias:false,depth:false,stencil:false,preserveDrawingBuffer:false});}catch{this.context=null;}
  this.program=null;this.texture=null;this.width=0;this.height=0;this.has_failed=false;
  this.canvas.addEventListener('webglcontextlost',event=>event.preventDefault());
  this.canvas.addEventListener('webglcontextrestored',()=>{this.program=null;this.texture=null;this.width=0;this.height=0;this.has_failed=false;});
 }
 initialize(){
  const gl=this.context,shaders=[];let program;
  try{
   for(const [type,source] of [[gl.VERTEX_SHADER,`#version 300 es
    void main(){vec2 position=vec2((gl_VertexID<<1)&2,gl_VertexID&2);gl_Position=vec4(position*2.0-1.0,0.0,1.0);}`],
    [gl.FRAGMENT_SHADER,`#version 300 es
    precision highp float;
    uniform highp sampler2D depth_image;
    uniform vec2 depth_range;
    out vec4 pixel;
    void main(){
     ivec2 size=textureSize(depth_image,0);
     float z=texelFetch(depth_image,ivec2(int(gl_FragCoord.x),size.y-1-int(gl_FragCoord.y)),0).r;
     if(z==0.0){pixel=vec4(18.0/255.0,20.0/255.0,29.0/255.0,1.0);return;}
     float t=clamp((z-depth_range.x)/(depth_range.y-depth_range.x),0.0,1.0);
     pixel=vec4(max(vec3(0.0),1.0-abs(t*3.0-vec3(2.0,1.0,0.0))),1.0);
    }`]]){
    const shader=gl.createShader(type);shaders.push(shader);gl.shaderSource(shader,source);gl.compileShader(shader);
    if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw Error('深度プレビューのシェーダー初期化失敗');
   }
   program=gl.createProgram();for(const shader of shaders)gl.attachShader(program,shader);gl.linkProgram(program);
   if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw Error('深度プレビューのプログラム初期化失敗');
   this.texture=gl.createTexture();if(!this.texture)throw Error('深度プレビューの画像領域確保失敗');gl.bindTexture(gl.TEXTURE_2D,this.texture);
   gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
   gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
   this.program=program;this.depth_range=gl.getUniformLocation(program,'depth_range');
  }catch(error){if(program)gl.deleteProgram(program);throw error;}
  finally{for(const shader of shaders)if(shader)gl.deleteShader(shader);}
 }
 draw(context,frame){
  const gl=this.context;if(!gl||this.has_failed||gl.isContextLost())return false;
  // float32の丸め差が大きい狭い表示範囲のCPU代替
  const lo=frame.calibration.min_depth_m,hi=frame.calibration.max_depth_m,min_relative_span=.001;
  if(hi-lo<min_relative_span*Math.max(Math.abs(lo),Math.abs(hi)))return false;
  try{
   if(!this.program)this.initialize();
   const {width,height}=frame.calibration.depth;
   gl.bindTexture(gl.TEXTURE_2D,this.texture);
   if(width!==this.width||height!==this.height){
    this.canvas.width=width;this.canvas.height=height;this.width=width;this.height=height;
    gl.texImage2D(gl.TEXTURE_2D,0,gl.R32F,width,height,0,gl.RED,gl.FLOAT,frame.depth);
   }else gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,width,height,gl.RED,gl.FLOAT,frame.depth);
   gl.viewport(0,0,width,height);gl.useProgram(this.program);gl.disable(gl.DITHER);
   gl.uniform2f(this.depth_range,frame.calibration.min_depth_m,frame.calibration.max_depth_m);gl.drawArrays(gl.TRIANGLES,0,3);
   context.drawImage(this.canvas,0,0);return true;
  }catch{this.has_failed=true;return false;}
 }
 dispose(){const gl=this.context;if(gl){if(this.program)gl.deleteProgram(this.program);if(this.texture)gl.deleteTexture(this.texture);}this.has_failed=true;}
}
