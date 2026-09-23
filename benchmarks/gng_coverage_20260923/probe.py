import ctypes as ct, sqlite3, time
import numpy as np, yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
class vec(ct.Structure): _fields_=[("x",ct.c_float),("y",ct.c_float),("z",ct.c_float)]
class quat(ct.Structure): _fields_=[("x",ct.c_float),("y",ct.c_float),("z",ct.c_float),("w",ct.c_float)]
class lidar(ct.Structure): _fields_=[("pos",vec),("quat",quat),("point_step",ct.c_uint32)]
class node(ct.Structure): _fields_=[("id",ct.c_uint16),("pos",vec),("normal",vec),("rho",ct.c_float),("label",ct.c_uint8),("frame",ct.c_uint32),("inpcl_ids",ct.c_void_p),("inpcl_num",ct.c_uint32)]
class tmap(ct.Structure): _fields_=[("frame",ct.c_uint32),("num",ct.c_uint32),("clusters_num",ct.c_uint32),("edges_num",ct.c_uint32),("nodes",ct.POINTER(node)),("clusters",ct.c_void_p),("edges",ct.c_void_p),("dist",ct.c_void_p)]
import sys
lib=ct.CDLL(sys.argv[2] if len(sys.argv)>2 else "/ros2_ws/install/gng_cpu/lib/libgng_cpu.so")
lib.gng_setParameter.argtypes=[ct.c_char_p,ct.c_uint32,ct.c_float]
lib.gng_setPointCloud.argtypes=[ct.c_void_p,ct.c_uint32,ct.POINTER(lidar)]
lib.gng_getTopologicalMap.restype=tmap
params=yaml.safe_load(open("/ros2_ws/src/ais_gng_cpu/src/ais_gng/config/gng_cpu/at128.yaml"))["ais_gng_node"]["ros__parameters"]
import sys
grid=float(sys.argv[1]);params["node.grid"]=grid
for k,v in params.items():
 for idx,val in enumerate(v if isinstance(v,list) else [v]):
  if isinstance(val,(int,float,bool)):lib.gng_setParameter(k.encode(),idx,float(val))
assert lib.gng_init()==0
p="/rosbag/fuzzy/Macnica_交差点分析/algo_0000_ros2/algo_0000_ros2.db3"
db=sqlite3.connect("file:"+p+"?mode=ro",uri=True)
m=deserialize_message(db.execute("select data from messages order by timestamp limit 1").fetchone()[0],PointCloud2)
data=(ct.c_uint8*len(m.data)).from_buffer_copy(m.data)
config=lidar(vec(0,0,0),quat(0,0,0,1),m.point_step)
for idx in range(30):
 lib.gng_setPointCloud(data,m.width*m.height,ct.byref(config));lib.gng_exec()
 if idx in [0,4,14,29]:
  result=lib.gng_getTopologicalMap()
  a=np.array([[result.nodes[j].pos.x,result.nodes[j].pos.y,result.nodes[j].pos.z] for j in range(result.num)])
  print("RESULT",grid,idx,result.num,"bounds",a.min(axis=0).tolist() if len(a) else [],a.max(axis=0).tolist() if len(a) else [],"far_nodes",int((a[:,0]>50).sum()) if len(a) else 0,flush=True)
