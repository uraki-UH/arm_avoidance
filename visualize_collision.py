import os
import json
import numpy as np
import xml.etree.ElementTree as ET

def read_stl_binary(path):
    try:
        if not os.path.exists(path): return None
        with open(path, 'rb') as f:
            f.seek(80) 
            num_faces_bytes = f.read(4)
            if not num_faces_bytes: return None
            num_faces = np.frombuffer(num_faces_bytes, dtype=np.uint32)[0]
            dtype = np.dtype([('normal', 'f4', (3,)), ('vertices', 'f4', (3, 3)), ('attr', 'u2')])
            data = np.frombuffer(f.read(num_faces * 50), dtype=dtype)
            return data['vertices']
    except Exception as e:
        return None

def get_bounding_capsule(vertices):
    if vertices is None or len(vertices) == 0: return {"r": 0.01, "h": 0.01, "oz": 0}
    v_flat = vertices.reshape(-1, 3)
    mins, maxs = np.min(v_flat, axis=0), np.max(v_flat, axis=0)
    center = (mins + maxs) / 2
    dims = maxs - mins
    
    # Auto-detect longest axis
    axis_idx = int(np.argmax(dims)) # Convert np.int64 to int
    h = float(dims[axis_idx])
    other_axes = [i for i in range(3) if i != axis_idx]
    dist_sq = np.sum((v_flat[:, other_axes] - center[other_axes])**2, axis=1)
    r = float(np.sqrt(np.max(dist_sq))) if len(dist_sq) > 0 else 0.01
    
    return {
        "r": float(max(0.005, r)),
        "h": float(max(0.005, h - 2*r)),
        "oz": float(center[axis_idx]),
        "axis": axis_idx 
    }

def parse_urdf(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    links, joints = {}, []
    
    workspace_root = "/Users/kubotamacbookair/Desktop/arm_avoidance"
    mesh_map = {}
    for r, d, f in os.walk(workspace_root):
        for file in f:
            if file.endswith(".stl"): mesh_map[file.lower()] = os.path.join(r, file)

    for link in root.findall('link'):
        name = link.get('name')
        links[name] = {"meshes": [], "capsule": None}
        all_v = []
        tags = link.findall('collision') + link.findall('visual')
        for tag in tags:
            geom = tag.find('geometry')
            if geom is None: continue
            mesh_tag = geom.find('mesh')
            if mesh_tag is not None:
                filename = os.path.basename(mesh_tag.get('filename')).lower()
                full_path = mesh_map.get(filename)
                if full_path:
                    scale = [float(x) for x in mesh_tag.get('scale', '1 1 1').split()]
                    v = read_stl_binary(full_path)
                    if v is not None:
                        scaled_v = (v * scale).astype(np.float64) # Ensure float64
                        all_v.append(scaled_v)
                        origin = tag.find('origin')
                        links[name]["meshes"].append({
                            "vertices": scaled_v.tolist(),
                            "scale": scale,
                            "origin": {
                                "xyz": [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0,0,0],
                                "rpy": [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0,0,0]
                            }
                        })
        if all_v:
            links[name]["capsule"] = get_bounding_capsule(np.vstack(all_v))

    for joint in root.findall('joint'):
        origin = joint.find('origin')
        axis_tag = joint.find('axis')
        joints.append({
            "name": joint.get('name'), "type": joint.get('type'),
            "parent": joint.find('parent').get('link'), "child": joint.find('child').get('link'),
            "xyz": [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0,0,0],
            "rpy": [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0,0,0],
            "axis": [float(x) for x in axis_tag.get('xyz', '0 0 0').split()] if axis_tag is not None else [0,0,1]
        })
    return {"links": links, "joints": joints}

if __name__ == "__main__":
    urdf_file = "/Users/kubotamacbookair/Desktop/arm_avoidance/gng_vlut_system/urdf/temp_robot.urdf"
    robot_data = parse_urdf(urdf_file)
    with open("robot_model.json", "w") as f:
        json.dump(robot_data, f)
    print(f"Successfully exported robot_model.json with {len(robot_data['links'])} links.")
