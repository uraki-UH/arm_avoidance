#!/usr/bin/env python3
"""保存GNGの一姿勢を中心とする、関節微小格子のFKと局所運動の可視化。"""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as xml

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def load_node(path, node_id):
    # 現行GNG version 9、little-endian、64 bit Eigen::Index、float配列の読み込み
    with path.open('rb') as stream:
        def read(fmt):
            return struct.unpack(fmt, stream.read(struct.calcsize(fmt)))

        def eigen():
            rows, cols = read('<qq')
            if not 0 < rows*cols <= 1024:
                raise ValueError('invalid Eigen dimensions')
            return np.frombuffer(stream.read(4*rows*cols), dtype='<f4').astype(float)

        version, layer_num, node_num = read('<Iii')
        if version != 9 or layer_num != 1 or not 0 < node_num <= 1000000:
            raise ValueError('expected version 9 GNG with one coordinate layer')
        for _ in range(node_num):
            current_id, _, _ = read('<iff')
            joints, stored_position = eigen(), eigen()
            coord_num, = read('<i')
            if not 0 <= coord_num <= 16:
                raise ValueError('invalid coordinate count')
            for _ in range(coord_num):
                eigen()
            read('<i')
            states = read('<5?')
            direction = eigen()
            read('<fff?ff?')
            if current_id == node_id:
                if not states[3] or not np.isfinite(joints).all():
                    raise ValueError('seed node is inactive or invalid')
                return joints, {'node_id': node_id, 'node_num': node_num,
                    'stored_position': stored_position.tolist(), 'stored_direction': direction.tolist(),
                    'stored_self_collision_free': states[2]}
    raise ValueError('node id not found')


class chain:
    def __init__(self, path, root_link, tcp_link):
        robot = xml.parse(path).getroot()
        parents = {entry.find('child').get('link'): entry for entry in robot.findall('joint')}
        route, current = [], tcp_link
        while current != root_link:
            entry = parents[current]
            route.append(entry)
            current = entry.find('parent').get('link')
            if len(route) > len(parents):
                raise ValueError('invalid URDF tree')
        self.steps, self.names, self.min_joints, self.max_joints = [], [], [], []
        for entry in reversed(route):
            origin = entry.find('origin')
            origin_values = {} if origin is None else origin.attrib
            position = np.fromstring(origin_values.get('xyz', '0 0 0'), sep=' ')
            angles = np.fromstring(origin_values.get('rpy', '0 0 0'), sep=' ')
            rotation = Rotation.from_euler('xyz', angles).as_matrix()
            kind, joint_idx, axis = entry.get('type'), -1, np.zeros(3)
            if kind != 'fixed':
                if kind not in ('revolute', 'continuous') or entry.find('mimic') is not None:
                    raise ValueError('experiment requires independent revolute arm joints')
                axis_entry = entry.find('axis')
                axis = np.fromstring('1 0 0' if axis_entry is None else axis_entry.get('xyz'), sep=' ')
                axis /= np.linalg.norm(axis)
                joint_idx = len(self.names)
                self.names.append(entry.get('name'))
                limits = entry.find('limit')
                self.min_joints.append(-np.inf if kind == 'continuous' else float(limits.get('lower')))
                self.max_joints.append(np.inf if kind == 'continuous' else float(limits.get('upper')))
            self.steps.append((position, rotation, joint_idx, axis))
        self.min_joints, self.max_joints = np.array(self.min_joints), np.array(self.max_joints)

    def fk(self, joints):
        joints = np.atleast_2d(joints)
        positions = np.zeros((len(joints), 3))
        rotations = np.broadcast_to(np.eye(3), (len(joints), 3, 3)).copy()
        for position, rotation, joint_idx, axis in self.steps:
            positions += rotations @ position
            rotations = rotations @ rotation
            if joint_idx >= 0:
                x, y, z = axis
                cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                outer = np.outer(axis, axis)
                angle = joints[:, joint_idx, None, None]
                move = outer+np.cos(angle)*(np.eye(3)-outer)+np.sin(angle)*cross
                rotations = rotations @ move
        return positions, rotations


def measure(model, joints, origin_position, origin_rotation):
    position, rotation = model.fk(joints)
    # 並進と相対回転の共通基準は、元ノードのTCP座標系
    translation_mm = (position-origin_position) @ origin_rotation*1000
    relative = origin_rotation.T @ rotation
    # 特定の手先軸によらない、姿勢全体の相対回転角
    relative_rotation = Rotation.from_matrix(relative)
    rotation_deg = np.degrees(relative_rotation.magnitude())
    # R0^T R = Rz(yaw) Ry(pitch) Rx(roll) に対応する局所RPY
    angles_deg = relative_rotation.as_euler('xyz', degrees=True)
    return translation_mm, angles_deg, rotation_deg


def ordered_grid(joint_num, half_steps, step_deg, max_norm_deg):
    side = 2*half_steps+1
    total = side**joint_num
    if total > 6000000:
        raise ValueError('grid exceeds 6,000,000 samples; increase joint_step_deg or reduce joint span')
    ids = np.arange(total, dtype=np.uint32)
    squared = np.zeros(total, dtype=np.uint16)
    for joint_idx in range(joint_num):
        digit = ((ids//side**joint_idx) % side).astype(np.int16)-half_steps
        squared += (digit*digit).astype(np.uint16)
    selected = np.flatnonzero(squared*step_deg**2 <= max_norm_deg**2+1e-10)
    selected = selected[np.argsort(squared[selected], kind='stable')]
    return selected.astype(np.uint32), squared[selected], side


def decode(ids, side, half_steps, joint_num, step_deg):
    return np.column_stack([((ids//side**j) % side).astype(np.int16)-half_steps
        for j in range(joint_num)])*step_deg


def display_indices(mask, max_points):
    selected = np.flatnonzero(mask)
    if len(selected) > max_points:
        selected = selected[np.linspace(0, len(selected)-1, max_points, dtype=int)]
    return selected


def graph_edges(model, seed, origin_position, origin_rotation, differences, positions,
                max_orientation_dev_deg, max_joint_step_deg, max_dist_mm):
    # 表示点の関節空間近傍と、辺内部7点のFKによる姿勢条件の検査。衝突検査なし
    if len(differences) < 2:
        return np.empty((0, 2), dtype=int), 0
    tree = cKDTree(differences)
    dists, neighbors = tree.query(differences, k=min(9, len(differences)), distance_upper_bound=max_joint_step_deg)
    pairs = set()
    for a in range(len(differences)):
        for dist, b in zip(dists[a, 1:], neighbors[a, 1:]):
            if np.isfinite(dist) and np.linalg.norm(positions[a]-positions[b]) <= max_dist_mm:
                pairs.add(tuple(sorted((a, int(b)))))
    pairs = np.array(sorted(pairs), dtype=int).reshape(-1, 2)
    accepted = []
    for offset in range(0, len(pairs), 2000):
        current = pairs[offset:offset+2000]
        a, b = differences[current[:, 0]], differences[current[:, 1]]
        interpolated = a[:, None, :]+np.linspace(0, 1, 9)[None, 1:-1, None]*(b-a)[:, None, :]
        _, _, orientation = measure(model, seed+np.radians(interpolated.reshape(-1, len(seed))), origin_position, origin_rotation)
        accepted.extend(current[np.max(orientation.reshape(-1, 7), axis=1) <= max_orientation_dev_deg+1e-9].tolist())
    return np.array(accepted, dtype=int).reshape(-1, 2), len(pairs)*7


def make_plot(output, groups, summary):
    os.environ.setdefault('MPLCONFIGDIR', str(output/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = 'Noto Sans CJK JP'
    figure = plt.figure(figsize=(12, 6), facecolor='#f6f8fb')
    titles = ['並進的な動き：回転成分が小さい候補', '回転的な動き：並進成分が小さい候補']
    for idx, (group, title) in enumerate(zip(groups, titles)):
        axis = figure.add_subplot(1, 2, idx+1, projection='3d')
        points = np.array(group['points'])
        for a, b in group['edges']:
            axis.plot(*points[[a, b], :3].T, color='#7b91aa', linewidth=.25, alpha=.3)
        scatter = axis.scatter(*points[:, :3].T, c=points[:, 3], cmap='viridis', s=4, vmin=0, vmax=summary['max_joint_norm_deg'], alpha=.65)
        axis.scatter(0, 0, 0, marker='*', s=90, c='#ed6947', edgecolors='black')
        extent = max(.1, np.abs(points[:, :3]).max()*1.08)
        for setter in (axis.set_xlim, axis.set_ylim, axis.set_zlim):
            setter(-extent, extent)
        axis.set_box_aspect((1, 1, 1))
        labels = ('Δx [mm]', 'Δy [mm]', 'Δz [mm]') if idx == 0 else ('Δroll [deg]', 'Δpitch [deg]', 'Δyaw [deg]')
        axis.set_xlabel(labels[0]); axis.set_ylabel(labels[1]); axis.set_zlabel(labels[2])
        axis.set_title(f'{title}\n採用 {group["accepted_num"]:,} / 表示 {len(points):,}', fontsize=10)
    figure.suptitle(f'GNG node {summary["node_id"]} ｜ 微小関節差のFKによる局所運動\n'
        f'並進時の回転量 {summary["max_orientation_dev_deg"]}° / 回転時の並進量 {summary["max_position_dev_mm"]} mm ｜ 衝突未評価', fontsize=14)
    figure.colorbar(scatter, ax=figure.axes, fraction=.018, pad=.06, label='元ノードからの関節角差 L2 [deg]')
    figure.savefig(output/'overview.png', dpi=170, bbox_inches='tight')
    plt.close(figure)


html_template = r'''<!doctype html><html lang="ja"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Local FK motion</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#edf2f8;color:#182b42;font:14px system-ui,sans-serif}header{padding:22px 28px;background:#142c46;color:white}h1{margin:0 0 8px;font-size:24px}header p{margin:4px 0;color:#c6d7eb}.controls{padding:18px 28px;background:white;display:flex;gap:16px;align-items:center;flex-wrap:wrap}input[type=range]{width:280px;accent-color:#26738c}button{border:0;border-radius:7px;background:#246d86;color:white;padding:9px 16px;cursor:pointer}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:14px;padding:18px}.panel{background:white;border-radius:12px;overflow:hidden;box-shadow:0 3px 12px #2030500b}.panel h2{font-size:16px;margin:15px 16px 6px}.count{margin:0 16px;color:#586c83;font-size:12px}canvas{display:block;width:100%;height:430px;touch-action:none;cursor:grab}.footer{padding:0 26px 18px;color:#40516b;line-height:1.8}.legend{display:inline-block;width:110px;height:10px;background:linear-gradient(90deg,#385bd0,#20ada0,#eda247);border-radius:6px}.info{padding:10px 16px;border-top:1px solid #e8edf4;min-height:65px;font:12px ui-monospace,monospace;white-space:pre-line}.small{font-size:12px}details{margin-top:10px}pre{white-space:pre-wrap}.active{color:#1b867d}@media(max-width:950px){.grid{grid-template-columns:1fr}canvas{height:400px}}
</style>
<header><h1>関節の微小変化から見る、手先の局所運動</h1><p id="subtitle"></p><p>FKのみ・IK補正なし ｜ 関節角差の小さい順に追加 ｜ 元ノードのTCP座標系で表示</p></header>
<div class="controls"><button id="play">順に追加</button><label>関節角差 L2 <input id="radius" type="range" min="0" step="0.05"><b id="radius_label"></b></label><label><input id="edges" type="checkbox" checked> 並進グラフの辺</label><button id="reset">視点を戻す</button><span class="small"><span class="legend"></span> 小さい角度差 → 大きい角度差</span></div>
<div class="grid">
<section class="panel"><h2>1. 並進的な動き</h2><p class="count" id="count0"></p><canvas id="view0"></canvas><div class="info" id="info0">回転成分が小さい候補の並進グラフ</div></section>
<section class="panel"><h2>2. 回転的な動き</h2><p class="count" id="count1"></p><canvas id="view1"></canvas><div class="info" id="info1">並進成分が小さい候補の回転点群</div></section>
</div><div class="footer"><b>操作：</b>ドラッグで3D回転、ホイールで拡大、点にカーソルを重ねると関節差・残差を表示。橙の十字は元ノード。<br><span id="conditions"></span><br>辺は近傍関節姿勢の補間7点でも姿勢条件を確認した接続。衝突・速度・経路全体は未評価。サンプルのない部分は未確認。<details><summary>基準姿勢と測定条件</summary><pre id="metadata"></pre></details></div>
<script>
const data=__DATA__;
const summary=data.summary, groups=data.groups;
const slider=document.getElementById('radius'); slider.max=summary.max_joint_norm_deg; slider.value=slider.max;
document.getElementById('subtitle').textContent=`ToPoDualArm / GNG node ${summary.node_id} / ${summary.valid_fk_num.toLocaleString()} FK samples / 各関節 ±${summary.max_joint_delta_deg}°・${summary.joint_step_deg}°刻み`;
document.getElementById('conditions').textContent=`並進的：回転量 ≤ ${summary.max_orientation_dev_deg}°。回転的：並進量 ≤ ${summary.max_position_dev_mm} mm。各表示は最大${summary.max_display_points.toLocaleString()}点に間引き、採用数は全サンプルで集計。RPYは R0ᵀR = Rz(yaw)Ry(pitch)Rx(roll)。`;
document.getElementById('metadata').textContent=JSON.stringify(summary,null,2);
const scenes=groups.map((group,i)=>({group,i,canvas:document.getElementById('view'+i),yaw:-.65,pitch:.52,zoom:1,projected:[],extent:Math.max(.1,...group.points.map(p=>Math.max(...p.slice(0,3).map(Math.abs))))*1.17}));
function color(value){const t=value/summary.max_joint_norm_deg;return `hsl(${225-190*t} 62% ${43+12*t}%)`;}
function draw(scene){
 const {canvas,group,i}=scene, rect=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
 canvas.width=rect.width*dpr;canvas.height=rect.height*dpr;const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);
 const w=rect.width,h=rect.height,scale=Math.min(w,h)*.34*scene.zoom/scene.extent,limit=Number(slider.value);
 const cy=Math.cos(scene.yaw),sy=Math.sin(scene.yaw),cp=Math.cos(scene.pitch),sp=Math.sin(scene.pitch);
 function project(p){let x=cy*p[0]-sy*p[1],y=sy*p[0]+cy*p[1];return [w/2+scale*x,h/2-scale*(cp*p[2]-sp*y),sp*p[2]+cp*y];}
 ctx.clearRect(0,0,w,h);ctx.lineWidth=.6;ctx.font='11px system-ui';
 for(let k=-2;k<=2;k++){let t=k/2*scene.extent;for(const pair of [[[t,-scene.extent,0],[t,scene.extent,0]],[[-scene.extent,t,0],[scene.extent,t,0]]]){let a=project(pair[0]),b=project(pair[1]);ctx.strokeStyle='#e9eef5';ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();}}
 const names=i===0?['Δx mm','Δy mm','Δz mm']:['Δroll °','Δpitch °','Δyaw °'];
 for(let k=0;k<3;k++){let p=[0,0,0];p[k]=scene.extent;let a=project([0,0,0]),b=project(p);ctx.strokeStyle=['#cb5d69','#33936e','#477ec4'][k];ctx.lineWidth=1.2;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.fillText(names[k],b[0]+3,b[1]-4);p[k]=scene.extent*.5;const m=project(p);ctx.fillStyle='#8192a6';ctx.fillText((scene.extent*.5).toFixed(1),m[0]+3,m[1]+12);}
 if(document.getElementById('edges').checked){ctx.lineWidth=.5;ctx.strokeStyle='#718ba02b';ctx.beginPath();for(const [a,b] of group.edges){if(group.points[a][3]>limit+1e-8||group.points[b][3]>limit+1e-8)continue;let pa=project(group.points[a]),pb=project(group.points[b]);ctx.moveTo(pa[0],pa[1]);ctx.lineTo(pb[0],pb[1]);}ctx.stroke();}
 scene.projected=[];group.points.forEach((p,idx)=>{if(p[3]<=limit+1e-8)scene.projected.push({p,idx,screen:project(p)});});
 scene.projected.sort((a,b)=>a.screen[2]-b.screen[2]);
 for(const point of scene.projected){const [x,y]=point.screen;ctx.fillStyle=color(point.p[3]);ctx.globalAlpha=.72;ctx.beginPath();ctx.arc(x,y,1.9,0,Math.PI*2);ctx.fill();}ctx.globalAlpha=1;
 let origin=project([0,0,0]);ctx.strokeStyle='#e3763b';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(origin[0]-5,origin[1]);ctx.lineTo(origin[0]+5,origin[1]);ctx.moveTo(origin[0],origin[1]-5);ctx.lineTo(origin[0],origin[1]+5);ctx.stroke();
 let cumulative=0;for(const [radius,count] of group.cumulative){if(radius>limit+1e-8)break;cumulative=count;}
 document.getElementById('count'+i).textContent=`採用 ${cumulative.toLocaleString()} 点 / 表示 ${scene.projected.length.toLocaleString()} 点`;
}
function draw_all(){document.getElementById('radius_label').textContent=Number(slider.value).toFixed(2)+'°';scenes.forEach(draw);}
for(const scene of scenes){let drag=null;const canvas=scene.canvas;canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};canvas.onpointerup=()=>drag=null;canvas.onpointercancel=()=>drag=null;
 canvas.onpointermove=e=>{if(drag){scene.yaw+=(e.clientX-drag[0])*.009;scene.pitch=Math.max(-1.5,Math.min(1.5,scene.pitch+(e.clientY-drag[1])*.009));drag=[e.clientX,e.clientY];draw(scene);return;}const rect=canvas.getBoundingClientRect(),x=e.clientX-rect.left,y=e.clientY-rect.top;let best=null,dist=100;for(const p of scene.projected){let d=(p.screen[0]-x)**2+(p.screen[1]-y)**2;if(d<dist){best=p;dist=d;}}if(best){const p=best.p;document.getElementById('info'+scene.i).textContent=`Δq L2=${p[3].toFixed(3)}°  回転量=${p[4].toFixed(3)}°\n並進量=${p[5].toFixed(3)} mm / Δq [deg]=${p.slice(6).map(v=>v.toFixed(1)).join(', ')}`;}};
 canvas.addEventListener('wheel',e=>{e.preventDefault();scene.zoom=Math.max(.4,Math.min(5,scene.zoom*Math.exp(-e.deltaY*.001)));draw(scene);},{passive:false});}
slider.oninput=draw_all;document.getElementById('edges').onchange=draw_all;window.onresize=draw_all;
document.getElementById('reset').onclick=()=>{scenes.forEach(s=>{s.yaw=-.65;s.pitch=.52;s.zoom=1;});draw_all();};
let is_playing=false,previous=0,play_radius=0;document.getElementById('play').onclick=()=>{is_playing=!is_playing;document.getElementById('play').textContent=is_playing?'一時停止':'順に追加';if(is_playing){if(Number(slider.value)>=Number(slider.max))slider.value=0;play_radius=Number(slider.value);previous=0;requestAnimationFrame(animate);}};
function animate(now){if(!is_playing)return;if(!previous)previous=now;play_radius=Math.min(Number(slider.max),play_radius+(now-previous)*Number(slider.max)/16000);slider.value=play_radius;previous=now;draw_all();if(play_radius>=Number(slider.max)){is_playing=false;document.getElementById('play').textContent='順に追加';}else requestAnimationFrame(animate);}
draw_all();
if(location.hash==='#selftest'){const full=scenes.map(s=>s.projected.length);slider.value=0;slider.dispatchEvent(new Event('input'));if(scenes.some(s=>s.projected.length!==1))throw Error('zero radius must show only seed');slider.value=slider.max;slider.dispatchEvent(new Event('input'));if(scenes.some((s,i)=>s.projected.length!==full[i]))throw Error('radius restoration failed');document.getElementById('play').click();animate(1000);animate(1200);if(Number(slider.value)<=0)throw Error('animation did not advance');is_playing=false;document.getElementById('play').textContent='順に追加';slider.value=slider.max;document.getElementById('edges').click();document.getElementById('edges').click();document.getElementById('reset').click();document.body.dataset.selftest='passed';}
</script></html>'''


def main():
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gng-file', type=Path, default=workspace/'gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin')
    parser.add_argument('--urdf', type=Path, default=workspace/'dual_arm_urdf/dual_arm_robot.urdf')
    parser.add_argument('--node-id', type=int, default=0)
    parser.add_argument('--root-link', default='L_shoulder_mount')
    parser.add_argument('--tcp-link', default='L_tcp')
    parser.add_argument('--joint-step-deg', type=float, default=.5)
    parser.add_argument('--max-joint-delta-deg', type=float, default=2.)
    parser.add_argument('--max-joint-norm-deg', type=float, default=4.)
    parser.add_argument('--max-orientation-dev-deg', type=float, default=.25)
    parser.add_argument('--max-position-dev-mm', type=float, default=.5)
    parser.add_argument('--max-graph-joint-step-deg', type=float, default=1.5)
    parser.add_argument('--max-graph-dist-mm', type=float, default=5.)
    parser.add_argument('--max-display-points', type=int, default=3000)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    for name, value in vars(args).items():
        if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
            raise ValueError('invalid '+name)
    if args.max_display_points < 2 or args.max_display_points > 20000:
        raise ValueError('max_display_points must be in [2,20000]')
    half_steps = round(args.max_joint_delta_deg/args.joint_step_deg)
    if not 1 <= half_steps <= 100 or abs(half_steps*args.joint_step_deg-args.max_joint_delta_deg)>1e-8:
        raise ValueError('joint span must be an integer multiple of the step')
    seed, source = load_node(args.gng_file, args.node_id)
    model = chain(args.urdf, args.root_link, args.tcp_link)
    if len(seed) != len(model.names) or np.any(seed<model.min_joints) or np.any(seed>model.max_joints):
        raise ValueError('seed does not fit the URDF chain')
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    (output/'matplotlib_cache').mkdir(exist_ok=True)
    origin_position, origin_rotation = model.fk(seed)
    origin_position, origin_rotation = origin_position[0], origin_rotation[0]
    begin = time.perf_counter()
    ids, squared, side = ordered_grid(len(seed), half_steps, args.joint_step_deg, args.max_joint_norm_deg)
    print(f'Seed: node={args.node_id} joints={len(seed)} Samples: {len(ids):,}', flush=True)
    kept, kept_ids, valid_num, fk_sec = [], [], 0, 0.
    batch_size = 32768
    for offset in range(0, len(ids), batch_size):
        current_ids = ids[offset:offset+batch_size]
        differences = decode(current_ids, side, half_steps, len(seed), args.joint_step_deg)
        joints = seed+np.radians(differences)
        is_valid = np.all((joints>=model.min_joints)&(joints<=model.max_joints), axis=1)
        joints, differences, current_ids = joints[is_valid], differences[is_valid], current_ids[is_valid]
        valid_num += len(joints)
        if not len(joints):
            continue
        started = time.perf_counter()
        translation, angles, orientation = measure(model, joints, origin_position, origin_rotation)
        fk_sec += time.perf_counter()-started
        position = np.linalg.norm(translation, axis=1)
        norm = np.linalg.norm(differences, axis=1)
        is_kept = (orientation<=args.max_orientation_dev_deg+1e-9)|(position<=args.max_position_dev_mm+1e-9)
        # 元ノードも各分類に含む、誤差許容による重複可能な分類
        kept.append(np.column_stack((translation, angles, norm, orientation, position, differences))[is_kept])
        kept_ids.append(current_ids[is_kept])
        if offset//batch_size % 16 == 0:
            print(f'FK: {min(offset+batch_size,len(ids)):,}/{len(ids):,}', flush=True)
    samples, sample_ids = np.concatenate(kept), np.concatenate(kept_ids)
    if np.any(np.diff(samples[:, 6]) < -1e-9):
        raise AssertionError('joint differences are not sorted')
    masks = [samples[:, 7]<=args.max_orientation_dev_deg+1e-9,
             samples[:, 8]<=args.max_position_dev_mm+1e-9]
    np.savez_compressed(output/'samples.npz', sample_ids=sample_ids, samples=samples, seed=seed,
        min_joints=model.min_joints, max_joints=model.max_joints)
    columns = ['dx_mm','dy_mm','dz_mm','roll_deg','pitch_deg','yaw_deg','joint_norm_deg',
               'orientation_dev_deg','position_dev_mm']+['delta_'+name+'_deg' for name in model.names]
    with (output/'samples.csv').open('w') as stream:
        writer = csv.writer(stream); writer.writerow(['sample_id']+columns+['is_translation_like','is_rotation_like'])
        for idx, row in enumerate(samples):
            writer.writerow([int(sample_ids[idx])]+row.tolist()+[int(mask[idx]) for mask in masks])
    groups, graph_fk_num = [], 0
    for idx, mask in enumerate(masks):
        selected = display_indices(mask, args.max_display_points)
        current = samples[selected]
        edges = np.empty((0, 2), dtype=int)
        if idx == 0:
            edges, evaluated = graph_edges(model, seed, origin_position, origin_rotation, current[:, 9:], current[:, :3],
                args.max_orientation_dev_deg, args.max_graph_joint_step_deg, args.max_graph_dist_mm)
            graph_fk_num += evaluated
        radii, counts = np.unique(np.round(samples[mask, 6], 10), return_counts=True)
        coordinates = current[:, :3] if idx == 0 else current[:, 3:6]
        groups.append({'kind': 'translation' if idx == 0 else 'rotation',
            'accepted_num': int(mask.sum()), 'sample_ids': sample_ids[selected].tolist(),
            'points': np.round(np.column_stack((coordinates,current[:, 6:9],current[:, 9:])), 7).tolist(),
            'edges': edges.tolist(), 'cumulative': np.column_stack((radii,np.cumsum(counts))).tolist()})
    summary = {**source, **{key: str(value) if isinstance(value,Path) else value for key,value in vars(args).items()},
        'gng_sha256': hashlib.sha256(args.gng_file.read_bytes()).hexdigest(),
        'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
        'joint_names': model.names, 'seed_joint_rad': seed.tolist(), 'seed_joint_deg': np.degrees(seed).tolist(),
        'seed_position_root_m': origin_position.tolist(), 'seed_rotation_root': origin_rotation.tolist(),
        'enumerated_num': len(ids), 'valid_fk_num': valid_num, 'graph_fk_num': graph_fk_num,
        'translation_like_num': int(masks[0].sum()), 'rotation_like_num': int(masks[1].sum()),
        'overlap_num': int(np.sum(masks[0]&masks[1])), 'mixed_motion_num': valid_num-len(samples),
        'fk_and_measure_sec': fk_sec,
        'analysis_sec': time.perf_counter()-begin, 'has_collision_check': False,
        'format_version': 2, 'sample_columns': columns,
        'max_translation_like_mm': float(samples[masks[0],8].max()),
        'max_rotation_like_deg': float(samples[masks[1],7].max())}
    payload = {'summary':summary,'groups':groups}
    (output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    (output/'graphs.json').write_text(json.dumps(payload,ensure_ascii=False))
    (output/'index.html').write_text(html_template.replace('__DATA__',json.dumps(payload,ensure_ascii=False).replace('</','<\\/')))
    make_plot(output,groups,summary)
    print(json.dumps({key:summary[key] for key in ('node_id','valid_fk_num','translation_like_num',
        'rotation_like_num','overlap_num','mixed_motion_num','max_translation_like_mm',
        'max_rotation_like_deg','fk_and_measure_sec','analysis_sec')},indent=2),flush=True)
    print('Saved: '+str(output/'index.html'),flush=True)


if __name__ == '__main__':
    main()
