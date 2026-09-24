"""GNG version 9の保存座標の読取専用抽出。Eigen添字は64bit、座標はfloat32。"""
from pathlib import Path
import hashlib
import json
import math
import struct
root=Path(__file__).resolve().parents[2]
source=root/'gng_vlut_system/gng_results/ToPoDualArm10000/gng.bin'
out=root/'artifacts/goal_selection_efficiency_20260924'
with source.open('rb') as stream:
    def read(fmt):
        size=struct.calcsize(fmt)
        data=stream.read(size)
        if len(data)!=size: raise ValueError('保存データの終端')
        return struct.unpack(fmt,data)
    def vector():
        rows,cols=read('<qq')
        if not 0<=rows*cols<=1000: raise ValueError('Eigen行列の不正なサイズ')
        return read('<'+'f'*(rows*cols))
    version,layers,count=read('<iii')
    if version!=9 or not 0<count<1000000: raise ValueError('対象はversion 9の有限件数のみ')
    points=[]
    for _ in range(count):
        idx,error_angle,error_coord=read('<iff')
        angle=vector();coordinate=vector()
        num_coords,=read('<i')
        for coord_idx in range(num_coords):
            value=vector()
            if coord_idx==0: coordinate=value
        level,=read('<i')
        is_surface,is_active_surface,is_free,is_active,is_boundary=read('<?????')
        direction=vector()
        manip,min_value,joint_score,is_valid,dynamic_manip=read('<fff?f')
        rot_manip,is_rot_valid=read('<f?')
        if is_active and len(coordinate)==3 and all(math.isfinite(x) for x in coordinate): points.append((idx,coordinate))
    num_edges,=read('<i')
    if not 0<=num_edges<10000000: raise ValueError('ノード終端位置の不一致')
points.sort()
(out/'robot.txt').write_text(''.join(' '.join(format(value,'.17g') for value in xyz)+'\n' for _,xyz in points))
(out/'robot_source.json').write_text(json.dumps({'source':str(source.relative_to(root)),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'version':version,'nodes':len(points),'coordinate_layer':0,'note':'保存座標のみ。比較用ラベル・法線・特徴量・候補は共通の合成条件。'},ensure_ascii=False,indent=2)+'\n')
print('Robot fixture nodes=',len(points))
