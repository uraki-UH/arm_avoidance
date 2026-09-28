"""Rebuild both render caches from the included URDF/STL source assets."""
from pathlib import Path
import argparse,hashlib,json,struct,xml.etree.ElementTree as ET
import numpy as np

def rebuild(app,kind):
    folder=app if kind=='long' else app/'models/standard'
    xml=folder/'source.urdf';root=ET.parse(xml).getroot()
    old=json.loads((folder/'assets.json').read_text(encoding='utf-8'))
    out={k:v for k,v in old.items() if k!='meshes'};out.update(model=kind,urdf_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),meshes={})
    dt=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
    names=sorted({x.attrib['filename'] for x in root.findall('./link/visual/geometry/mesh')})
    for name in names:
        source=(folder/name).resolve();source.relative_to(folder.resolve());data=source.read_bytes()
        n=struct.unpack_from('<I',data,80)[0]
        if len(data)!=84+n*50:raise ValueError('Invalid binary STL: '+name)
        raw=np.frombuffer(data,dt,count=n,offset=84)
        p=raw['vertices'].reshape(-1,3).copy()
        face=np.cross(raw['vertices'][:,1]-raw['vertices'][:,0],raw['vertices'][:,2]-raw['vertices'][:,0])
        face/=np.maximum(np.linalg.norm(face,axis=1)[:,None],1e-20);fn=np.repeat(face,3,axis=0)
        _,inv=np.unique(np.round(p,5),axis=0,return_inverse=True)
        avg=np.zeros((inv.max()+1,3),np.float32);np.add.at(avg,inv,fn);avg/=np.maximum(np.linalg.norm(avg,axis=1)[:,None],1e-20)
        normals=np.where((np.sum(avg[inv]*fn,axis=1)>np.cos(np.deg2rad(35)))[:,None],avg[inv],fn)
        combined=np.concatenate([p,normals],axis=1)
        _,first,index=np.unique(np.round(combined,5),axis=0,return_index=True,return_inverse=True)
        vertices=combined[first].astype('<f4');index=index.astype('<u4');sha=hashlib.sha256(data).hexdigest()
        dest=folder/'cache'/(Path(name).stem+'-'+sha[:10]+'.bin');dest.parent.mkdir(exist_ok=True)
        dest.write_bytes(struct.pack('<II',len(vertices),len(index))+vertices.tobytes()+index.tobytes())
        out['meshes'][name]={'file':dest.relative_to(app).as_posix(),'triangles':n,'vertices':len(vertices),'source_sha256':sha}
    (folder/'assets.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(kind+': '+str(len(names))+' render meshes rebuilt')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',choices=['standard','long','all'],default='all')
    parser.add_argument('--app-dir',type=Path,default=Path(__file__).resolve().parents[1]/'app')
    args=parser.parse_args()
    for kind in (['long','standard'] if args.model=='all' else [args.model]):rebuild(args.app_dir.resolve(),kind)
