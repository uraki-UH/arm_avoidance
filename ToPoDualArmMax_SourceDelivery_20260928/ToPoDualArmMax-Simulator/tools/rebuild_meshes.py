"""同梱URDF/STLからの描画キャッシュ生成。未変更メッシュは再利用。"""
from pathlib import Path
import argparse,copy,hashlib,json,re,shutil,struct,xml.etree.ElementTree as ET
import numpy as np

def sync_visual_source(app, kind, source_urdf):
    """ROS原本のvisual・材質・メッシュの同期。collision・関節・慣性の保持。"""
    folder = app if kind == 'long' else app/'models/standard'
    source_urdf = source_urdf.resolve()
    source_root = ET.parse(source_urdf).getroot()
    target_urdf = folder/'source.urdf'
    text = target_urdf.read_bytes().decode('utf-8-sig')
    target_root = ET.fromstring(text)
    source_links = {link.attrib['name']: link for link in source_root.findall('link')}
    target_links = {link.attrib['name']: link for link in target_root.findall('link')}
    if source_links.keys() != target_links.keys():
        raise ValueError('原本と同梱URDFのリンク構成が不一致')
    materials = {item.attrib['name']: item for item in source_root.findall('material')}
    target_materials = {item.attrib['name']: item for item in target_root.findall('material')}
    def canonical(items):
        return [ET.canonicalize(ET.tostring(item, encoding='unicode').strip(), strip_text=True) for item in items]
    visuals = {}
    files = {}
    for name, link in source_links.items():
        visuals[name] = copy.deepcopy(link.findall('visual'))
        for visual in visuals[name]:
            material = visual.find('material')
            if material is not None and len(material) == 0 and material.get('name') in materials:
                source_material = materials[material.get('name')]
                target_material = target_materials.get(material.get('name'))
                if target_material is None or canonical([source_material]) != canonical([target_material]):
                    material.extend(copy.deepcopy(list(source_material)))
            for mesh in visual.findall('geometry/mesh'):
                relative = Path(mesh.attrib['filename'])
                source = (source_urdf.parent/relative).resolve()
                target = (folder/relative).resolve()
                source.relative_to(source_urdf.parent)
                target.relative_to(folder.resolve())
                if not source.is_file():
                    raise FileNotFoundError(source)
                files[target] = source
    def replace_link(match):
        block = match.group(0)
        name = ET.fromstring(block).attrib['name']
        previous = target_links[name].findall('visual')
        if canonical(previous) == canonical(visuals[name]):
            return block
        if block.endswith('/>'):
            block = block[:-2] + '></link>'
        block = re.sub(r'\s*<visual\b[^>]*(?:/>|>.*?</visual>)', '', block, flags=re.S)
        replacement = []
        for visual in visuals[name]:
            ET.indent(visual, space='  ', level=2)
            replacement.append('    '+ET.tostring(visual, encoding='unicode').strip())
        return re.sub(r'\s*</link>', '\n'+'\n'.join(replacement)+'\n  </link>', block)
    updated = re.sub(r'<link\b[^>]*?(?:/>|>.*?</link>)', replace_link, text, flags=re.S)
    ET.fromstring(updated)
    num_copied = 0
    for target, source in files.items():
        if target.is_file() and hashlib.sha256(target.read_bytes()).digest() == hashlib.sha256(source.read_bytes()).digest():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        num_copied += 1
    if updated != text:
        target_urdf.write_bytes(updated.encode('utf-8'))
    print(f'{kind}: ROS原本のvisual同期、メッシュ更新 {num_copied} 件')

def rebuild(app,kind):
    folder=app if kind=='long' else app/'models/standard'
    xml=folder/'source.urdf';root=ET.parse(xml).getroot()
    old=json.loads((folder/'assets.json').read_text(encoding='utf-8'))
    out={k:v for k,v in old.items() if k!='meshes'};out.update(model=kind,urdf_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),meshes={})
    dt=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
    names=sorted({x.attrib['filename'] for x in root.findall('./link/visual/geometry/mesh')})
    for name in names:
        source=(folder/name).resolve();source.relative_to(folder.resolve());data=source.read_bytes()
        sha=hashlib.sha256(data).hexdigest()
        previous=old['meshes'].get(name)
        if previous and previous.get('source_sha256')==sha and (app/previous['file']).is_file():
            out['meshes'][name]=previous
            continue
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
    parser.add_argument('--source-urdf',type=Path,help='visual・色・メッシュの共通原本。--model long/standardの指定必須')
    args=parser.parse_args()
    if args.source_urdf:
        if args.model == 'all':parser.error('--source-urdfには--model long/standardが必要')
        sync_visual_source(args.app_dir.resolve(),args.model,args.source_urdf)
    for kind in (['long','standard'] if args.model=='all' else [args.model]):rebuild(args.app_dir.resolve(),kind)
