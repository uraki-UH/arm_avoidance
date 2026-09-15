#!/usr/bin/env python3
"""保存済み局所FK結果の、Plotlyによる単独HTML表示。"""

import argparse
import csv
import json
from pathlib import Path


html_template = r'''<!doctype html><html lang="ja"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Local FK — Plotly 3D</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f4f7fb;color:#23364c;font:14px system-ui,sans-serif}header{padding:18px 24px;background:#18334d;color:white}h1{font-size:21px;margin:0 0 5px}header p{margin:0;color:#cad9e7}.controls{display:flex;align-items:center;flex-wrap:wrap;gap:12px;padding:12px 24px;background:white;border-bottom:1px solid #dce4ee}button,select{padding:7px 10px;border:1px solid #bccddb;border-radius:5px;background:white;color:#18334d;cursor:pointer}button.is_selected{background:#236f88;color:white;border-color:#236f88}input{accent-color:#236f88}input[type=range]{width:210px;vertical-align:middle}#plot{height:calc(100vh - 295px);min-height:440px}#count{padding:10px 24px;font-weight:600}footer{padding:8px 24px 18px;color:#546b80;font-size:12px;line-height:1.8}#status{color:#9b341e}a{color:inherit}
</style>
<header><h1>局所FKの3Dグラフ — Plotly</h1><p id="subtitle"></p></header>
<div class="controls"><span>表示：</span><button class="kind is_selected" data-idx="0">並進的な動き</button><button class="kind" data-idx="1">回転的な動き</button></div>
<div class="controls"><label>関節角差 L2 <input id="radius" type="range" min="0" step="0.1"><b id="radius_label"></b></label><button id="play">順に追加</button><label><input id="edges" type="checkbox" checked>グラフの辺</label><label><input id="all_points" type="checkbox">全採用点</label><select id="projection"><option value="orthographic">正投影</option><option value="perspective">透視投影</option></select><span>視点：</span><button class="view" data-view="oblique">斜め</button><button class="view" data-view="xy">XY</button><button class="view" data-view="xz">XZ</button><button class="view" data-view="yz">YZ</button></div>
<div id="count"></div><div id="plot"></div>
<footer>ドラッグ：回転 ／ Shift＋ドラッグ：平行移動 ／ ホイール：拡大縮小 ／ 点にカーソル：関節角差・残差。右上のカメラアイコン：画像保存。<br><span id="conditions"></span><br>全採用点を表示しても、辺は元の表示用グラフのみ。辺の追加構築なし。衝突は未評価。回転座標は R0ᵀR = Rz(yaw)Ry(pitch)Rx(roll)。<br><span id="status"></span> <a href="https://plotly.com/javascript/3d-scatter-plots/">Plotly 3D Scatter</a></footer>
<script>__PLOTLY__</script><script>
const data=__DATA__,summary=data.summary,plot=document.getElementById('plot');
const slider=document.getElementById('radius'),edge_box=document.getElementById('edges'),all_box=document.getElementById('all_points');
let group_idx=0,is_busy=false,is_dirty=false,is_playing=false,play_radius=0,play_time=0;
slider.max=summary.max_joint_norm_deg;slider.value=slider.max;
document.getElementById('subtitle').textContent=`GNG node ${summary.node_id} ｜ ${summary.valid_fk_num.toLocaleString()} FK samples ｜ 並進：元TCP座標系 [mm] ／ 回転：相対RPY [deg]`;
document.getElementById('conditions').textContent=`並進的：回転量 ≤ ${summary.max_orientation_dev_deg}°。回転的：並進量 ≤ ${summary.max_position_dev_mm} mm。混合した動き ${summary.mixed_motion_num.toLocaleString()}件は対象外。ほぼ動かない ${summary.overlap_num}件は両方に含む。`;
function stop_play(){is_playing=false;document.getElementById('play').textContent='順に追加';}
function coordinates(points){return {x:points.map(p=>p[0]),y:points.map(p=>p[1]),z:points.map(p=>p[2])};}
function camera(view='oblique'){
 const settings={oblique:[[1.45,1.45,1.1],[0,0,1]],xy:[[0,0,2],[0,1,0]],xz:[[0,-2,0],[0,0,1]],yz:[[2,0,0],[0,0,1]]};
 const [eye,up]=settings[view],as_xyz=v=>({x:v[0],y:v[1],z:v[2]});
 return {eye:as_xyz(eye),up:as_xyz(up),projection:{type:document.getElementById('projection').value}};
}
async function render(){
 if(is_busy){is_dirty=true;return;}is_busy=true;is_dirty=false;
 try{
  const group=data.groups[group_idx],limit=Number(slider.value),points=(all_box.checked?group.all_points:group.points).filter(p=>p[3]<=limit+1e-8);
  const labels=group.kind==='translation'?['Δx [mm]','Δy [mm]','Δz [mm]']:['Δroll [deg]','Δpitch [deg]','Δyaw [deg]'];
  const edges={x:[],y:[],z:[]};let edge_num=0;
  if(group.kind==='translation'&&edge_box.checked)for(const [a,b] of group.edges){const p=group.points[a],q=group.points[b];if(p[3]>limit+1e-8||q[3]>limit+1e-8)continue;edge_num++;['x','y','z'].forEach((key,i)=>edges[key].push(p[i],q[i],null));}
  const hover=labels.map((label,i)=>`${label}: %{${['x','y','z'][i]}:.3f}`).join('<br>')+'<br>Δq L2: %{customdata[0]:.3f}°<br>回転量: %{customdata[1]:.4f}°<br>並進量: %{customdata[2]:.4f} mm<br>sample: %{customdata[10]}<br>Δq [deg]: '+summary.joint_names.map((name,i)=>`${name}=%{customdata[${i+3}]:.1f}`).join(', ')+'<extra></extra>';
  const traces=[{type:'scatter3d',mode:'lines',...edges,line:{color:'#8c9fab',width:1},opacity:.3,hoverinfo:'skip',name:'接続',showlegend:false},
   {type:'scatter3d',mode:'markers',...coordinates(points),customdata:points.map(p=>p.slice(3)),hovertemplate:hover,name:'候補',marker:{size:all_box.checked?2:3,color:points.map(p=>p[3]),cmin:0,cmax:summary.max_joint_norm_deg,colorscale:'Viridis',opacity:.85,colorbar:{title:{text:'Δq L2 [deg]'},thickness:15,len:.7}},showlegend:false},
   {type:'scatter3d',mode:'markers',x:[0],y:[0],z:[0],marker:{size:6,color:'#ed7046',symbol:'cross'},name:'元ノード',hovertemplate:'元ノード<extra></extra>',showlegend:false}];
  const extent=group.all_points.reduce((value,p)=>Math.max(value,Math.abs(p[0]),Math.abs(p[1]),Math.abs(p[2])),.1)*1.08;
  const axis=(title)=>({title:{text:title},range:[-extent,extent],gridcolor:'#dce5ee',zerolinecolor:'#9eafbf',showbackground:true,backgroundcolor:'#f7f9fc'});
  const current_camera=plot.layout?.uirevision===group_idx?plot._fullLayout?.scene.camera:camera();
  await Plotly.react(plot,traces,{margin:{l:0,r:85,t:5,b:0},paper_bgcolor:'#f4f7fb',font:{family:'sans-serif',color:'#23364c'},uirevision:group_idx,scene:{xaxis:axis(labels[0]),yaxis:axis(labels[1]),zaxis:axis(labels[2]),aspectmode:'cube',camera:current_camera,dragmode:'orbit'}},{responsive:true,displaylogo:false,scrollZoom:true,toImageButtonOptions:{filename:'local_fk_'+group_idx,width:1400,height:1000}});
  let accepted_num=0;for(const [radius,count] of group.cumulative){if(radius>limit+1e-8)break;accepted_num=count;}
  document.getElementById('count').textContent=`採用 ${accepted_num.toLocaleString()} 点 ／ 表示 ${points.length.toLocaleString()} 点 ／ 辺 ${edge_num.toLocaleString()} 本`;
  document.getElementById('radius_label').textContent=limit.toFixed(1)+'°';edge_box.disabled=group.kind==='rotation';
 }catch(error){document.getElementById('status').textContent=String(error);throw error;}finally{is_busy=false;}
 if(is_dirty)await render();
}
document.querySelectorAll('.kind').forEach(button=>button.onclick=()=>{group_idx=Number(button.dataset.idx);document.querySelectorAll('.kind').forEach(b=>b.classList.toggle('is_selected',b===button));render();});
slider.oninput=()=>{stop_play();render();};edge_box.onchange=render;all_box.onchange=render;
document.querySelectorAll('.view').forEach(button=>button.onclick=()=>Plotly.relayout(plot,{'scene.camera':camera(button.dataset.view)}));
document.getElementById('projection').onchange=()=>Plotly.relayout(plot,{'scene.camera.projection.type':document.getElementById('projection').value});
async function animate(now){if(!is_playing)return;if(!play_time)play_time=now;play_radius=Math.min(Number(slider.max),play_radius+(now-play_time)*Number(slider.max)/16000);play_time=now;slider.value=play_radius;await render();if(!is_playing)return;if(play_radius>=Number(slider.max))stop_play();else requestAnimationFrame(animate);}
document.getElementById('play').onclick=()=>{if(is_playing){stop_play();return;}is_playing=true;document.getElementById('play').textContent='一時停止';if(Number(slider.value)>=Number(slider.max))slider.value=0;play_radius=Number(slider.value);play_time=0;requestAnimationFrame(animate);};
async function start(){
 await render();
 if(location.hash==='#selftest'){
  slider.value=0;await render();if(plot.data[1].x.length!==1)throw Error('zero radius');
  slider.value=slider.max;all_box.checked=true;await render();if(plot.data[1].x.length!==data.groups[0].accepted_num)throw Error('all translation samples');
  for(const idx of [1]){group_idx=idx;await render();if(plot.data[1].x.length!==data.groups[idx].accepted_num)throw Error('group count');}
  if(plot.data[0].x.length)throw Error('rotation edges');
  group_idx=0;all_box.checked=false;await render();
  await Plotly.relayout(plot,{'scene.camera':camera('xy')});if(plot._fullLayout.scene.camera.eye.z!==2)throw Error('camera');
  await Plotly.relayout(plot,{'scene.camera':camera()});
  if(!plot.querySelector('canvas'))throw Error('no WebGL canvas');
  document.body.dataset.selftest='passed';
 }
 document.body.dataset.ready='true';
}
start().catch(error=>{document.body.dataset.selftest='failed';document.getElementById('status').textContent=String(error);});
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results_dir', type=Path)
    parser.add_argument('--plotly-js', type=Path)
    args = parser.parse_args()
    output = args.results_dir
    plotly_path = args.plotly_js or output/'plotly-4.0.0.min.js'
    payload = json.loads((output/'graphs.json').read_text())
    if payload['summary'].get('format_version') != 2:
        raise ValueError('expected translation/rotation format 2; regenerate FK results')
    groups = payload['groups']
    for group in groups:
        # 既存グラフの添字・辺を保持した表示点と元サンプルIDの対応
        group['points'] = [point+[sample_id] for point, sample_id in zip(group['points'], group['sample_ids'])]
        group['all_points'] = []
    joint_columns = ['delta_'+name+'_deg' for name in payload['summary']['joint_names']]
    with (output/'samples.csv').open() as stream:
        for row in csv.DictReader(stream):
            errors = [float(row[key]) for key in ['joint_norm_deg','orientation_dev_deg','position_dev_mm']+joint_columns]
            for idx, flag in enumerate(['is_translation_like','is_rotation_like']):
                if row[flag] != '1':
                    continue
                columns = ['dx_mm','dy_mm','dz_mm'] if idx == 0 else ['roll_deg','pitch_deg','yaw_deg']
                groups[idx]['all_points'].append([float(row[key]) for key in columns]+errors+[int(row['sample_id'])])
    for group in groups:
        if len(group['all_points']) != group['accepted_num']:
            raise ValueError('CSV and graph counts differ')
    # Plotly本体と全データを埋め込んだ、オフラインで表示可能な単独HTML
    script = plotly_path.read_text().replace('</script', r'<\/script')
    data = json.dumps(payload,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/')
    destination = output/'plotly.html'
    destination.write_text(html_template.replace('__PLOTLY__',script).replace('__DATA__',data))
    print('Saved: '+str(destination))


if __name__ == '__main__':
    main()
