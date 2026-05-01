import xml.etree.ElementTree as ET
import numpy as np
import json
import os

def parse_origin(origin_tag):
    if origin_tag is None: return np.eye(4)
    xyz = [float(x) for x in origin_tag.attrib.get('xyz', '0 0 0').split()]
    rpy = [float(x) for x in origin_tag.attrib.get('rpy', '0 0 0').split()]
    c, s = np.cos(rpy), np.sin(rpy)
    R_x = np.array([[1, 0, 0], [0, c[0], -s[0]], [0, s[0], c[0]]])
    R_y = np.array([[c[1], 0, s[1]], [0, 1, 0], [-s[1], 0, c[1]]])
    R_z = np.array([[c[2], -s[2], 0], [s[2], c[2], 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = R_z @ R_y @ R_x; T[:3, 3] = xyz
    return T

def load_stl(filename):
    if not os.path.exists(filename): return None
    try:
        with open(filename, 'rb') as f:
            f.seek(80); count = np.frombuffer(f.read(4), dtype='uint32')[0]
            data = np.fromfile(f, dtype=np.dtype([('n','f4',(3,)),('v','f4',(3,3)),('a','u2')]), count=count)
            return data['v'].tolist()
    except: return None

class MeshExporter:
    def __init__(self, urdf_path):
        self.workspace_root = "/Users/kubotamacbookair/Desktop/arm_avoidance"
        self.tree = ET.parse(urdf_path)
        self.meshes = {}
        self._parse_urdf()

    def _parse_urdf(self):
        for link in self.tree.getroot().findall('link'):
            name = link.attrib['name']
            link_data = []
            for col in link.findall('collision'):
                origin = parse_origin(col.find('origin')).tolist()
                geom = col.find('geometry')
                if geom.find('mesh') is not None:
                    fname = geom.find('mesh').attrib['filename'].replace("package://topoarm_description/", f"{self.workspace_root}/gng_vlut_system/urdf/topoarm_description/").replace("package://gng_vlut_system/", f"{self.workspace_root}/gng_vlut_system/")
                    scale = [float(x) for x in geom.find('mesh').attrib.get('scale', '1 1 1').split()]
                    v = load_stl(fname)
                    if v: link_data.append({'origin': origin, 'vertices': v, 'scale': scale})
            if link_data: self.meshes[name] = link_data

    def export(self):
        out = "/Users/kubotamacbookair/Desktop/arm_avoidance/robot_meshes.json"
        with open(out, 'w') as f:
            json.dump(self.meshes, f)
        print(f"Exported mesh data to {out}")

if __name__ == "__main__":
    MeshExporter("/Users/kubotamacbookair/Desktop/arm_avoidance/gng_vlut_system/urdf/temp_robot.urdf").export()
