"""Extract time-ordered measured unit directions, including unknown (zero) slots.
The official recording is input data, never executable code. No missing direction
is invented. Raw mm quantisation and sample-specific dropouts remain limitations.
"""
from pathlib import Path
import argparse,hashlib,json,struct
import numpy as np

p=Path(__file__).resolve().parents[1]/'app/assets/mid360'
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path,help='Official LVX2 file or first 16 MiB prefix');args=parser.parse_args()
b=args.recording.read_bytes()
assert b[:10]==b'livox_tech' and b[16]==2 and b[28]==1
o=29+63*b[28]; chunks=[]; times=[]
dt=np.dtype([('xyz','<i4',3),('r','u1'),('tag','u1')])
while o+24<len(b):
    cur,nxt,idx=struct.unpack_from('<QQQ',b,o)
    assert cur==o and nxt>o
    if nxt>len(b): break
    q=o+24
    while q<nxt:
        kind=b[q+17];size=struct.unpack_from('<I',b,q+18)[0]
        assert size<2000 and q+27+size<=nxt
        if kind==1:
            assert size%14==0
            a=np.frombuffer(b,dt,size//14,q+27)
            chunks.append(a['xyz'].copy())
            t=struct.unpack_from('<Q',b,q+7)[0]
            times.append(t+np.arange(len(a),dtype=np.int64)*5000)
        q+=27+size
    assert q==nxt
    o=nxt
xyz=np.concatenate(chunks)[:1000000];t=np.concatenate(times)[:len(xyz)]
assert len(xyz)==1000000 and np.all(np.diff(t)==5000)
norm=np.linalg.norm(xyz.astype(float),axis=1);known=norm>0
directions=np.zeros_like(xyz,dtype='<f4');directions[known]=xyz[known]/norm[known,None]
dest=p/'measured-directions.f32';directions.tofile(dest)
e=np.rad2deg(np.arcsin(directions[known,2]))
meta={'model':'livox-official-indoor-directions-v1','source':'https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Indoor_sampledata.lvx2','source_page':'https://www.livoxtech.com/mid-360/downloads','source_prefix_bytes':len(b),'source_prefix_sha256':hashlib.sha256(b).hexdigest(),'asset':'assets/mid360/measured-directions.f32','sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'slots':len(xyz),'known_directions':int(known.sum()),'unknown_directions':int((~known).sum()),'interval_s':.000005,'duration_s':5,'source_first_timestamp_ns':int(t[0]),'format':'little-endian float32 xyz unit vectors; 0,0,0 means direction unknown','order':'original LVX2 packet and point order, no spatial resampling','loop':'replays same five-second sample; not an infinitely non-repetitive physical scanner','elevation_observed_deg':[float(e.min()),float(e.max())],'limitations':'Includes source-recording invalid slots and mm range quantisation; no reconstruction of missing ray angles. New geometry is raycast only along known measured directions. Not individual-device calibration.'}
(p/'measured-directions.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
print(json.dumps(meta))
