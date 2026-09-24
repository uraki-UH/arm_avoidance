"""既知姿勢の欠損セダンと保存済み交差点クラスタの再評価。"""
import json
from pathlib import Path
import sys
import numpy as np

root = Path(__file__).resolve().parents[2]
package = root/'ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer'
sys.path.insert(0,str(package/'scripts'))
from vehicle_registration import load_models, register_vehicle, yaw_rotation

models = load_models()
points = models[0]['points']
points = points[(points[:,1]<-.4)&(points[:,2]>.25)][::3] @ yaw_rotation(.7).T + [8,-4,1]
points += np.random.default_rng(3).normal(0,.015,points.shape)
synthetic = {'source_id':'/topological_map', 'frame_id':'map', 'title':'欠損セダン・既知姿勢の検証',
             'selection':{'kind':'cluster','id':1}, 'min_position':points.min(0).tolist(),
             'max_position':points.max(0).tolist(), 'graph': {'timestamp':1,'frameId':'map',
             'edges':[],'clusters':[], 'nodes':[dict(x=x,y=y,z=z,id=idx,nx=0,ny=0,nz=0,label=3,age=0)
                                              for idx,(x,y,z) in enumerate(points)]}}
live = json.loads((package/'test/fixtures/vehicle_intersection_cluster.json').read_text())
for name, snapshot in [('synthetic_partial',synthetic), ('live_57',live)]:
    result = register_vehicle(snapshot,models)
    (Path(__file__).parent/(name+'.json')).write_text(json.dumps({'snapshot':snapshot,'result':result},ensure_ascii=False)+'\n')
    print(name, result['message'], round(result['elapsed_ms']), 'ms')
    for candidate in result['candidates']:
        print(candidate['model_id'], {key:round(candidate[key],4) if candidate[key] is not None else None
              for key in ['match_ratio','support_ratio','unmatched_ratio','inlier_rms_m','compatibility']})
