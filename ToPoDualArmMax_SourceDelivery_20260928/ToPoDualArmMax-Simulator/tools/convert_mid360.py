"""Convert the original Livox STEP to binary STL, without changing units (mm)."""
import sys,json,tempfile,shutil
from pathlib import Path

from OCP.STEPControl import STEPControl_Reader
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
p=Path(__file__).resolve().parents[1]/'app/assets/mid360'
r=STEPControl_Reader();r.ReadFile(str(p/'mid-360-asm.stp'));r.TransferRoots();shape=r.OneShape()
BRepMesh_IncrementalMesh(shape,.04,False,.12,True).Perform()
w=StlAPI_Writer();w.ASCIIMode=False
staging=tempfile.TemporaryDirectory(prefix='topo-mid360-')
def write_stl(shape,name):
 target=Path(staging.name)/name
 if not w.Write(shape,str(target)):raise RuntimeError('STL write failed: '+name)
 shutil.copyfile(target,p/name)
write_stl(shape,'mid-360.stl')
parts=[];it=TopExp_Explorer(shape,TopAbs_SOLID)
while it.More():
 s=it.Current();b=Bnd_Box();BRepBndLib.Add_s(s,b);name=f'part-{len(parts):02}.stl';write_stl(s,name);parts.append({'file':name,'bounds_mm':[b.CornerMin().X(),b.CornerMin().Y(),b.CornerMin().Z(),b.CornerMax().X(),b.CornerMax().Y(),b.CornerMax().Z()]});it.Next()
b=Bnd_Box();BRepBndLib.Add_s(shape,b)
result={'source':'https://www.livoxtech.com/mid-360/downloads','unit':'mm','chord_tolerance_mm':.04,'angular_tolerance_rad':.12,'bounds_mm':[b.CornerMin().X(),b.CornerMin().Y(),b.CornerMin().Z(),b.CornerMax().X(),b.CornerMax().Y(),b.CornerMax().Z()],'parts':parts}
(p/'model.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
