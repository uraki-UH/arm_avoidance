// Geometry and radiometric RGB remain untouched; heatmaps are a separate view.
export const colorStops=[[48,18,110],[48,78,205],[22,169,224],[40,210,147],[175,231,52],[253,193,41],[237,70,32],[122,4,3]];
const linear=Float32Array.from({length:256},(_,i)=>{const c=i/255;return c<=.04045?c/12.92:((c+.055)/1.055)**2.4;});
export function cloudColors(xyz,matrix,settings,source=null){
 const count=xyz.length/3,values=new Float32Array(count),rgb=new Uint8Array(count*3),colors=new Float32Array(count*3);let lo=Infinity,hi=-Infinity;
 for(let i=0;i<count;i++){const j=i*3,x=xyz[j],y=xyz[j+1],z=xyz[j+2];const v=settings.mode==='height'?matrix[2]*x+matrix[6]*y+matrix[10]*z+matrix[14]:settings.mode==='reflectance'?(source?.[i]??0):Math.hypot(x,y,z);values[i]=v;lo=Math.min(lo,v);hi=Math.max(hi,v);}
 if(!count){lo=0;hi=1;}if(!settings.auto){lo=settings.min;hi=settings.max;}
 const span=Math.max(1e-9,hi-lo);
 for(let i=0;i<count;i++){const j=i*3;if(settings.mode==='rgb'&&source){rgb.set(source.subarray(j,j+3),j);}else{const t=Math.max(0,Math.min(1,(values[i]-lo)/span))*(colorStops.length-1),a=Math.min(colorStops.length-2,Math.floor(t)),u=t-a;for(let c=0;c<3;c++)rgb[j+c]=Math.round(colorStops[a][c]*(1-u)+colorStops[a+1][c]*u);}for(let c=0;c<3;c++)colors[j+c]=linear[rgb[j+c]];}
 return{rgb,colors,summary:{mode:settings.mode,auto:settings.mode==='rgb'?false:settings.auto,min:settings.mode==='rgb'?0:lo,max:settings.mode==='rgb'?255:hi,units:settings.mode==='rgb'?'RGB8':settings.mode==='reflectance'?'0–255':'m',reference:settings.mode==='height'?'base_footprint Z':settings.mode==='range'?'Euclidean distance from captured sensor origin':settings.mode,palette:settings.mode==='rgb'?'source-rgb':'spectral-8',count}};
}
export class CloudColorControls{
 constructor(host,prefix,original,onChange){
  this.settings={mode:'height',auto:true,min:0,max:2};this.prefix=prefix;
  host.innerHTML=`<label class="field-label">点群の色<select id="${prefix}-color-mode"><option value="height">高さ · 基準座標 Z [m]</option><option value="range">距離 · センサから [m]</option><option value="${original}">${original==='rgb'?'RGB · カメラの色':'反射強度 · 模擬値'}</option></select></label><div class="row-actions"><label><input id="${prefix}-color-auto" type="checkbox" checked>色範囲を自動調整</label></div><div class="field-grid"><label>下限<input id="${prefix}-color-min" type="number" step="0.1" value="0" disabled></label><label>上限<input id="${prefix}-color-max" type="number" step="0.1" value="2" disabled></label></div><div class="cloud-color-bar"></div><div class="cloud-color-legend" id="${prefix}-color-legend" role="status">高さ · 取得待ち</div>`;
  const el=k=>document.getElementById(prefix+'-color-'+k);this.el=el;
  const change=()=>{const mode=el('mode').value,auto=el('auto').checked,min=el('min').valueAsNumber,max=el('max').valueAsNumber;el('min').disabled=el('max').disabled=auto||mode==='rgb';el('auto').disabled=mode==='rgb';if(!auto&&(!Number.isFinite(min)||!Number.isFinite(max)||max<=min)){el('legend').textContent='上限を下限より大きくしてください';return;}this.settings={mode,auto,min,max};onChange();};
  for(const k of ['mode','auto','min','max'])el(k).onchange=change;for(const k of ['min','max'])el(k).oninput=change;
 }
 compute(xyz,matrix,source){const result=cloudColors(xyz,matrix,this.settings,source),s=result.summary;this.summary=s;this.el('legend').textContent=s.mode==='rgb'?'RGB · 色のない画素は灰色':`${s.mode==='height'?'高さ':s.mode==='range'?'距離':'模擬反射強度'} ${s.min.toFixed(3)} 〜 ${s.max.toFixed(3)} ${s.units} · ${s.count.toLocaleString()} 点`;if(this.settings.auto){this.el('min').value=s.min.toFixed(3);this.el('max').value=s.max.toFixed(3);}this.el('legend').dataset.color=JSON.stringify(s);return result;}
}
export function displayPLY(xyz,matrix,rgb,world){
 const n=xyz.length/3,h=new TextEncoder().encode(`ply\nformat binary_little_endian 1.0\ncomment Display colors only; raw measurements are in points.ply\nelement vertex ${n}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n`),b=new ArrayBuffer(h.length+n*15),d=new DataView(b);new Uint8Array(b).set(h);
 for(let i=0;i<n;i++){const j=i*3,o=h.length+i*15,x=xyz[j],y=xyz[j+1],z=xyz[j+2];for(let c=0;c<3;c++){d.setFloat32(o+c*4,world?matrix[c]*x+matrix[c+4]*y+matrix[c+8]*z+matrix[c+12]:xyz[j+c],true);d.setUint8(o+12+c,rgb[j+c]);}}return b;
}
