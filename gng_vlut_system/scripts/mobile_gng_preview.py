#!/usr/bin/env python3
"""保存済み移動状態GNGの、yaw・速度・角速度選択と原点配置の独立表示。"""
import argparse
import json
from pathlib import Path


def write_preview(model_path, output_path):
    data = json.loads(Path(model_path).read_text())
    if data.get('schema') != 'mobile_gng_state_v1':
        raise ValueError('対応する状態GNGではありません')
    payload = json.dumps(data, ensure_ascii=False).replace('</', '<\\/')
    html = '''<!doctype html><html lang="ja"><meta charset="utf-8"><title>FuzzBot state GNG</title>
<style>body{margin:0;background:#101827;color:#e5edf8;font:15px system-ui}main{display:grid;grid-template-columns:310px 1fr;height:100vh}aside{padding:18px;overflow:auto;border-right:1px solid #3c4c65}h1{font-size:20px}label{display:block;margin:12px 0}input[type=number]{width:80px}input[type=range]{width:165px}input,button{background:#24344c;color:#eef5ff;border:1px solid #52657b;padding:5px}pre{white-space:pre-wrap;font-size:13px}canvas{width:100%;height:100%;display:block}small{color:#aabbd1}b{color:#7cd8e5}</style>
<main><aside><h1>FuzzBot state GNG</h1><p id="summary"></p><small>原点は初期ロボット、Xは前方。GNG近傍辺の走行可否・衝突判定は未検査。</small>
<label><input type="checkbox" id="enable_normals" checked> 前方ベクトル（yaw）</label><label><input type="checkbox" id="enable_velocity"> 速度ベクトル</label><label><input type="checkbox" id="enable_edges"> GNG近傍辺</label>
<label>色 <select id="color_field"><option value="yaw">相対yaw</option><option value="speed">速度</option><option value="angular">角速度</option><option value="support">サンプル数</option></select></label>
<label>相対yaw中心 <input id="yaw_center" type="range" min="-180" max="180" value="0"><output id="yaw_center_label"></output></label>
<label>yaw半幅 <input id="yaw_width" type="range" min="5" max="180" value="180"><output id="yaw_width_label"></output></label>
<label>速度範囲 [m/s]<br><input id="min_speed" type="number" value="-0.2" step="0.05"> ～ <input id="max_speed" type="number" value="0.4" step="0.05"></label>
<label>角速度範囲 [rad/s]<br><input id="min_angular" type="number" value="-1.2" step="0.1"> ～ <input id="max_angular" type="number" value="1.2" step="0.1"></label>
<label>表示原点の世界yaw [deg] <input id="origin_yaw" type="number" value="0" step="15"></label>
<small>世界yaw変更は配置のみ。保存済み状態・学習結果の変更なし。</small><p id="visible"></p><pre id="selected">ノードをクリックすると、状態と代表サンプル数を表示。</pre><button id="reset">表示をリセット</button></aside><canvas id="graph"></canvas></main>
<script>const data=__DATA__;const element=id=>document.getElementById(id),canvas=element('graph'),context=canvas.getContext('2d');let points=[],selected=-1;
element('summary').textContent=`${data.states.length} nodes / ${data.edges.length} edges / ${data.num_training_samples} samples`;
const wrap=value=>(value+Math.PI)%(2*Math.PI)+((value+Math.PI)%(2*Math.PI)<0?2*Math.PI:0)-Math.PI;
function redraw(){
 const width=canvas.clientWidth,height=canvas.clientHeight,ratio=devicePixelRatio||1;canvas.width=width*ratio;canvas.height=height*ratio;context.setTransform(ratio,0,0,ratio,0,0);context.clearRect(0,0,width,height);
 const extent=data.motion_config.radius+.3,scale=Math.min(width,height)/(2*extent),screen=(x,y)=>[width/2+x*scale,height/2-y*scale],origin=Number(element('origin_yaw').value)*Math.PI/180;
 const cos=Math.cos(origin),sin=Math.sin(origin),transform=state=>[cos*state[0]-sin*state[1],sin*state[0]+cos*state[1]];
 context.font='12px system-ui';context.lineWidth=1;
 for(let value=-Math.floor(extent*2)/2;value<=extent;value+=.5){let start=screen(value,-extent),end=screen(value,extent);context.strokeStyle=value===0?'#5c718e':'#25334a';context.beginPath();context.moveTo(...start);context.lineTo(...end);context.stroke();start=screen(-extent,value);end=screen(extent,value);context.beginPath();context.moveTo(...start);context.lineTo(...end);context.stroke();context.fillStyle='#8295b0';context.fillText(value.toFixed(1),...screen(value,-.05));}
 const center=Number(element('yaw_center').value)*Math.PI/180,half=Number(element('yaw_width').value)*Math.PI/180;
 element('yaw_center_label').textContent=element('yaw_center').value+'°';element('yaw_width_label').textContent=element('yaw_width').value+'°';
 const ids=data.states.flatMap((state,idx)=>Math.abs(wrap(state[2]-center))<=half+1e-9&&state[3]>=Number(element('min_speed').value)&&state[3]<=Number(element('max_speed').value)&&state[4]>=Number(element('min_angular').value)&&state[4]<=Number(element('max_angular').value)?[idx]:[]),allowed=new Set(ids);
 points=ids.map(idx=>({idx,position:screen(...transform(data.states[idx]))}));element('visible').textContent=`表示 ${ids.length} / ${data.states.length} ノード`;
 if(element('enable_edges').checked){context.strokeStyle='#6f88a840';context.lineWidth=.8;context.beginPath();for(const [source,target]of data.edges){if(!allowed.has(source)||!allowed.has(target))continue;context.moveTo(...screen(...transform(data.states[source])));context.lineTo(...screen(...transform(data.states[target])));}context.stroke();}
 const arrow=(position,angle,length,color)=>{const end=[position[0]+Math.cos(angle)*length,position[1]-Math.sin(angle)*length];context.strokeStyle=color;context.lineWidth=1.1;context.beginPath();context.moveTo(...position);context.lineTo(...end);context.lineTo(end[0]-4*Math.cos(angle-.55),end[1]+4*Math.sin(angle-.55));context.moveTo(...end);context.lineTo(end[0]-4*Math.cos(angle+.55),end[1]+4*Math.sin(angle+.55));context.stroke();};
 for(const {idx,position}of points){const state=data.states[idx],field=element('color_field').value;let value=field==='yaw'?(state[2]+Math.PI)/(2*Math.PI):field==='speed'?(state[3]-data.motion_config.min_speed)/(data.motion_config.max_speed-data.motion_config.min_speed):field==='angular'?(state[4]+data.motion_config.max_angular_speed)/(2*data.motion_config.max_angular_speed):Math.min(1,data.sample_counts[idx]/150);const color=`hsl(${field==='yaw'?value*360:240-240*value} 85% 65%)`;
 if(element('enable_normals').checked)arrow(position,state[2]+origin,.07*scale,color);if(element('enable_velocity').checked&&Math.abs(state[3])>.005)arrow(position,state[2]+origin+(state[3]<0?Math.PI:0),Math.abs(state[3])*.4*scale,'#ffffff');context.fillStyle=idx===selected?'#fff':color;context.beginPath();context.arc(...position,idx===selected?5:2.2,0,Math.PI*2);context.fill();}
 arrow(screen(0,0),origin,.28*scale,'#66ddff');context.fillStyle='#66ddff';context.fillText('初期ロボット前方',...screen(Math.cos(origin)*.33,Math.sin(origin)*.33));
 if(selected>=0){const state=data.states[selected],world=transform(state);element('selected').textContent=`node ${selected}\n位置: (${state[0].toFixed(3)}, ${state[1].toFixed(3)}) m\n相対yaw: ${(state[2]*180/Math.PI).toFixed(1)}°\nv: ${state[3].toFixed(3)} m/s\nω: ${state[4].toFixed(3)} rad/s\n前方: (${Math.cos(state[2]).toFixed(3)}, ${Math.sin(state[2]).toFixed(3)}, 0)\nサンプル数: ${data.sample_counts[selected]}\n世界配置: (${world[0].toFixed(3)}, ${world[1].toFixed(3)}) m / ${((state[2]+origin)*180/Math.PI).toFixed(1)}°`;}
}
for(const input of document.querySelectorAll('input,select'))input.addEventListener('input',redraw);
canvas.addEventListener('click',event=>{const rectangle=canvas.getBoundingClientRect(),x=event.clientX-rectangle.left,y=event.clientY-rectangle.top;let min_dist=12;selected=-1;for(const point of points){const dist=Math.hypot(point.position[0]-x,point.position[1]-y);if(dist<min_dist){min_dist=dist;selected=point.idx;}}redraw();});
element('reset').onclick=()=>{element('yaw_center').value=0;element('yaw_width').value=180;element('origin_yaw').value=0;element('min_speed').value=data.motion_config.min_speed;element('max_speed').value=data.motion_config.max_speed;element('min_angular').value=-data.motion_config.max_angular_speed;element('max_angular').value=data.motion_config.max_angular_speed;redraw();};
element('reset').click();new ResizeObserver(redraw).observe(canvas);window.mobile_gng_preview={data,redraw,get_visible_ids:()=>points.map(point=>point.idx)};</script></html>'''
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html.replace('__DATA__', payload))
    return output


def main():
    parser = argparse.ArgumentParser(description='移動状態GNGの独立プレビュー')
    parser.add_argument('model_path')
    parser.add_argument('output_path')
    args = parser.parse_args()
    print(write_preview(args.model_path, args.output_path))


if __name__ == '__main__':
    main()
