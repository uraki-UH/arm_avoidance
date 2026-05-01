import numpy as np
import os

def load_stl(filename):
    try:
        with open(filename, 'rb') as f:
            f.seek(80)
            count_data = f.read(4)
            count = np.frombuffer(count_data, dtype='uint32')[0]
            dtype = np.dtype([('normal', 'f4', (3,)), 
                             ('v', 'f4', (3, 3)), 
                             ('attr', 'u2')])
            data = np.fromfile(f, dtype=dtype, count=count)
            return data['v']
    except Exception as e:
        print(f"Error: {e}")
        return None

path = "~/uraki_ws/gng_vlut_system/urdf/topoarm_description/meshes/topoarm/link1.stl"
v = load_stl(path)
if v is not None:
    flat = v.reshape(-1, 3)
    print(f"Bounds: Min {flat.min(axis=0)}, Max {flat.max(axis=0)}")
    print(f"Count: {len(v)} triangles")
else:
    print("Failed to load")
