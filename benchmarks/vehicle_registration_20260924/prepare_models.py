"""取得済みDrivAerMLと寸法仮定の簡易車体からの面積比例サンプリング。"""
import hashlib
import json
from pathlib import Path
import numpy as np

root = Path(__file__).resolve().parents[2]
out = root / 'ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/config/vehicle_models'
source = Path('/home/uraki/datasets/vehicle_models/drivaerml/run_1/drivaer_1.stl')
with source.open() as stream:
    vertices = np.fromiter((float(value) for line in stream if line.lstrip().startswith('vertex ') for value in line.split()[1:4]), dtype=float).reshape(-1, 3, 3)

def sample(triangles, seed=24):
    area = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]), axis=1)
    rng = np.random.default_rng(seed)
    selected = triangles[rng.choice(len(triangles), 3500, p=area/area.sum())]
    u = np.sqrt(rng.random((3500, 1)))
    v = rng.random((3500, 1))
    return (1-u)*selected[:, 0]+u*(1-v)*selected[:, 1]+u*v*selected[:, 2]

def extrude(profile, width):
    polygon = np.array([[x, -width/2, z] for x,z in profile] + [[x, width/2, z] for x,z in profile])
    num = len(profile)
    faces=[]
    for idx in range(1,num-1):
        faces.extend([[0,idx,idx+1],[num,num+idx,num+idx+1]])
    for idx in range(num):
        nxt=(idx+1)%num
        faces.extend([[idx,nxt,nxt+num],[idx,nxt+num,idx+num]])
    return polygon[np.array(faces)]

low, high = vertices.min(axis=(0,1)), vertices.max(axis=(0,1))
print('DrivAer original bounds:',low, high, 'dimensions:',high-low)
vertices -= np.array([(low[0]+high[0])/2,(low[1]+high[1])/2,low[2]])
models=[]
def add(model_id,label,note,triangles):
    dims=np.ptp(triangles.reshape(-1,3),axis=0)
    models.append({'id':model_id,'label':label,'note':note,'dimensions_m':dims.round(4).tolist(),'points':sample(triangles).round(5).tolist()})
add('drivaer_sedan','乗用車（セダン）','DrivAerML run_1の表面。Ashton et al., 2024 / CC BY-SA 4.0。XY中心・底面基準へ移動、面積比例サンプリング。',vertices)
add('compact_tall','軽自動車相当（背高）','3.35 × 1.45 × 1.75 mを仮定した簡易形状。実車種・法的区分の判定用ではありません。',extrude([(-1.675,0),(1.675,0),(1.675,.85),(1.25,1.0),(.85,1.75),(-1.4,1.75),(-1.675,1.4)],1.45))
add('passenger_van','乗用車（バン）','4.6 × 1.8 × 1.9 mを仮定した簡易形状。',extrude([(-2.3,0),(2.3,0),(2.3,.85),(1.8,1.1),(1.35,1.9),(-2.1,1.9),(-2.3,1.6)],1.8))
add('box_truck','トラック（箱型）','5.4 × 1.9 × 2.7 mを仮定した簡易形状。大型車や平ボディ車は対象外。',extrude([(-2.7,0),(2.7,0),(2.7,1.3),(2.2,2.2),(1.1,2.2),(1.1,2.7),(-2.7,2.7)],1.9))
out.mkdir(parents=True,exist_ok=True)
(out/'models.json').write_text(json.dumps({'source':'DrivAerML run_1, Ashton et al. 2024, https://huggingface.co/datasets/neashton/drivaerml','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'models':models},ensure_ascii=False,separators=(',',':'))+'\n')
(out/'DrivAerML_LICENSE.txt').write_text((source.parent.parent/'LICENSE.txt').read_text().rstrip()+'\n')
(out/'ATTRIBUTION.md').write_text('# 車両表面モデル\n\n- `drivaer_sedan`: DrivAerML run_1, Ashton et al. (2024), [取得元](https://huggingface.co/datasets/neashton/drivaerml)。CC BY-SA 4.0。XY中心化・底面基準移動・面積比例サンプリングの派生データ。ライセンス全文は同梱。\n- その他: この検証で作成した寸法仮定の簡易形状。実車の再現モデルではない。\n- 各モデル3500点。+Xが長手、Zが上。寸法・出典・点座標は `models.json`。\n')
print([(m['id'],m['dimensions_m']) for m in models])
