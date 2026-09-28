"""Simulator HTTP service + PointCloud2 input, isolated ROS domain 57."""
import collections
import json
import os
import platform
import signal
import struct
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, JointState
from std_msgs.msg import String, ByteMultiArray
from visualization_msgs.msg import MarkerArray
from ais_gng_msgs.msg import TopologicalMap
from server import Store, Joiner, make_handler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[2] / "app"
PORT = int(os.environ.get("TOPO_VM_PORT", "8878"))
DOMAIN = int(os.environ.get("ROS_DOMAIN_ID", "57"))


class RobotStore(Store):
    def __init__(self):
        super().__init__()
        self.input_lock = threading.Lock()
        self.owner = None
        self.owner_time = 0.
        self.input_count = 0
        self.inputs = collections.OrderedDict()
        self.last_input = None

    def status(self):
        state = super().status()
        with self.input_lock:
            state.update(service='topo-robot-vm', domain=DOMAIN, hostname=platform.node(),
                         algorithm='ais_gng_fullrange', fvg_mode='read_only_candidates',
                         execution={'http':'Ubuntu VM','ais_gng':'Ubuntu VM',
                                    'fvg':'Ubuntu VM','sensor_and_render':'connected browser'},
                         input_count=self.input_count, last_input=self.last_input,
                         input_age_ms=(time.monotonic()-self.owner_time)*1000 if self.owner else None)
        return state


def parse_input(raw):
    if len(raw)<12 or raw[:4] != b'TPC1':
        raise ValueError('TPC1 input required')
    size, = struct.unpack_from('<I', raw, 4)
    if size>64000 or 8+size>len(raw):
        raise ValueError('Invalid metadata size')
    meta=json.loads(raw[8:8+size])
    offset=8+((size+3)//4)*4
    n=meta.get('count')
    if not isinstance(n,int) or not 1<=n<=1000000 or len(raw)-offset!=n*12:
        raise ValueError('Point count/length mismatch')
    if meta.get('frame_id')!='base_footprint' or meta.get('sensor') not in ('mid360','d435i'):
        raise ValueError('World-frame sensor input required')
    owner=meta.get('client','')
    if not isinstance(owner,str) or not 16<=len(owner)<=80:
        raise ValueError('Invalid client identity')
    points=np.frombuffer(raw,dtype='<f4',offset=offset).reshape(-1,3)
    if not np.isfinite(points).all() or np.any(np.abs(points)>200):
        raise ValueError('Invalid world coordinates')
    pose=meta.get('robot_pose',{})
    if not isinstance(pose,dict) or len(pose)>64 or any(not isinstance(k,str) or not isinstance(v,(int,float)) or not np.isfinite(v) for k,v in pose.items()):
        raise ValueError('Invalid robot pose')
    return meta, raw[offset:]


def handler(store,node,publisher,joints):
    base=make_handler(store,ROOT,PORT)
    class Handler(base):
        def do_GET(self,head=False):
            if urlsplit(self.path).path=='/api/health':
                if not self.allowed():return self.respond(403)
                return self.respond(200,json.dumps(store.status()).encode(),'application/json',head)
            return super().do_GET(head)

        def do_POST(self):
            if not self.allowed() or self.headers.get('X-ToPo-VM')!='1':
                self.close_connection=True
                return self.respond(403)
            if urlsplit(self.path).path!='/api/input':
                self.close_connection=True
                return self.respond(404)
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 12<=length<=13000000:raise ValueError('Input too large')
                self.connection.settimeout(15)
                raw=self.rfile.read(length)
                meta,data=parse_input(raw)
                with store.input_lock:
                    now=time.monotonic()
                    if store.owner and store.owner!=meta['client'] and now-store.owner_time<5:
                        return self.respond(409,b'Another browser is publishing; this tab can view the result','text/plain')
                    stamp=time.time_ns()
                    message=PointCloud2()
                    message.header.frame_id='base_footprint'
                    message.header.stamp.sec=stamp//1000000000
                    message.header.stamp.nanosec=stamp%1000000000
                    message.height=1;message.width=meta['count']
                    message.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
                    message.is_bigendian=False;message.point_step=12
                    message.row_step=len(data);message.is_dense=True;message.data=data
                    pose=JointState();pose.header=message.header
                    pose.name=list(meta['robot_pose']);pose.position=[float(v) for v in meta['robot_pose'].values()]
                    publisher.publish(message);joints.publish(pose)
                    store.owner=meta['client'];store.owner_time=now;store.input_count+=1
                    record=dict(stamp_ns=str(stamp),count=meta['count'],sensor=meta['sensor'],
                                sensor_frame=meta.get('sensor_frame'),robot_model=meta.get('robot_model'),robot_pose=meta['robot_pose'],
                                received_wall_ms=time.time()*1000)
                    store.last_input=record;store.inputs[str(stamp)]=record
                    while len(store.inputs)>128:store.inputs.popitem(last=False)
                self.respond(202,json.dumps(record).encode(),'application/json')
            except (ValueError,KeyError,TypeError,TimeoutError) as exc:
                self.close_connection=True
                self.respond(400,str(exc).encode(),'text/plain')
    return Handler


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', str(DOMAIN))
    rclpy.init()
    node=Node('topo_robot_sim_bridge',start_parameter_services=False,enable_rosout=False)
    store=RobotStore();joiner=Joiner(store)
    qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=4,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs=[node.create_subscription(ByteMultiArray,'/ais_gng/fvg_frame',lambda m:joiner.ingest('snapshot',m),qos,raw=True),
          node.create_subscription(TopologicalMap,'/topological_map',lambda m:joiner.ingest('map',m),qos,raw=True),
          node.create_subscription(String,'/ais_gng_fvg/processing_metrics',lambda m:joiner.ingest('metrics',m),qos)]
    for kind in ('add','delete','memory'):
        subs.append(node.create_subscription(MarkerArray,'/fvg_observer/'+kind,lambda m,k=kind:joiner.ingest(k,m),qos,raw=True))
    publisher=node.create_publisher(PointCloud2,'/scan',2)
    joints=node.create_publisher(JointState,'/topo/joint_states',2)
    http=ThreadingHTTPServer(('127.0.0.1',PORT),handler(store,node,publisher,joints));http.daemon_threads=True
    threading.Thread(target=http.serve_forever,daemon=True).start()
    print(json.dumps({'service':'topo-robot-vm','port':PORT,'domain':DOMAIN,'pid':os.getpid()}),flush=True)
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:pass
    finally:
        joiner.stop.set();http.shutdown();http.server_close();joiner.worker.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
