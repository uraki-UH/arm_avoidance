"""Independent NumPy URDF transform oracle for browser FK verification."""
from pathlib import Path
import math, json, xml.etree.ElementTree as ET,sys
import numpy as np
root_dir=Path(__file__).resolve().parents[1]/'app'
if '--standard' in sys.argv:root_dir=root_dir/'models/standard'
root=ET.parse(root_dir/'source.urdf').getroot()
joints=root.findall('joint')
actuated=[j for j in joints if j.attrib['type']!='fixed' and j.find('mimic') is None]
def values(e,attr,default='0 0 0'):
    return np.array([float(x) for x in (e.attrib.get(attr,default) if e is not None else default).split()])
def rx(t):
    c,s=math.cos(t),math.sin(t);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(t):
    c,s=math.cos(t),math.sin(t);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(t):
    c,s=math.cos(t),math.sin(t);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
cases=[]
for n in range(-1,13):
    pose={}
    for i,j in enumerate(actuated):
        name=j.attrib['name']; q=0 if n==-1 else math.sin(n*1.37+i*.8)*.5
        if name=='L_joint2':q-=math.pi/2
        if name=='R_joint2':q+=math.pi/2
        lim=j.find('limit')
        if j.attrib['type']!='continuous':q=max(float(lim.attrib['lower']),min(float(lim.attrib['upper']),q))
        pose[name]=q
    transforms={'base_footprint':np.eye(4)};pending=list(joints)
    while pending:
        progress=False
        for j in pending[:]:
            parent=j.find('parent').attrib['link']
            if parent not in transforms:continue
            o=j.find('origin');r,p,y=values(o,'rpy');t=np.eye(4);t[:3,:3]=rz(y)@ry(p)@rx(r);t[:3,3]=values(o,'xyz')
            if j.attrib['type']!='fixed':
                q=pose.get(j.attrib['name'],0);mimic=j.find('mimic')
                if mimic is not None:q=pose[mimic.attrib['joint']]*float(mimic.attrib.get('multiplier',1))+float(mimic.attrib.get('offset',0))
                a=values(j.find('axis'),'xyz','1 0 0');a/=np.linalg.norm(a)
                k=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
                motion=np.eye(4);motion[:3,:3]=np.eye(3)+math.sin(q)*k+(1-math.cos(q))*(k@k)
                t=t@motion
            transforms[j.find('child').attrib['link']]=transforms[parent]@t
            pending.remove(j);progress=True
        if not progress:raise ValueError('Disconnected URDF tree')
    cases.append({'pose':pose,'tcp':{s:transforms[s+'_tcp'].flatten(order='F').tolist() for s in ['L','R']}})
(root_dir/'qa-fk-reference.json').write_text(json.dumps({'home':cases[0],'cases':cases[1:]},indent=2),encoding='utf-8')
print('Generated',len(cases),'independent FK reference cases')
