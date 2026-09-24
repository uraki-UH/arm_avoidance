"""隔離ROSドメインでの受信更新・選択出力・終了確認。"""
from pathlib import Path
import copy
import json
import os
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID']='173'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/goal_selection_efficiency_20260924/roslogs'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from ais_gng_feature_msgs.msg import TopologicalNodeFeatureArray, TopologicalNodeFeature
from gng_control_msgs.msg import GraspCandidateArray, GraspCandidate
from std_msgs.msg import Int32MultiArray

out=Path('/ros2_ws/src/artifacts/goal_selection_efficiency_20260924')
command=['/ros2_ws/install/gng_vlut_system/lib/gng_vlut_system/topological_map_goal_selector_node','--ros-args','-p','candidate_count:=1','-p','goal_update_hz:=40.0','-p','topological_map_topic:=/selection_test/map','-p','candidate_topic:=/selection_test/candidates','-p','node_feature_topic:=/selection_test/features','-p','output_topic:=/selection_test/output','-p','goal_candidate_ids_topic:=/selection_test/ids']
(out/'ros_command.json').write_text(json.dumps({'environment':{key:os.environ[key] for key in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_LOG_DIR']},'command':command},indent=2)+'\n')
process=None
node=None
checks=[]
try:
    rclpy.init()
    node=rclpy.create_node('selection_efficiency_test')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub=node.create_publisher(TopologicalMap,'/selection_test/map',qos)
    source_pub=node.create_publisher(GraspCandidateArray,'/selection_test/candidates',qos)
    feature_pub=node.create_publisher(TopologicalNodeFeatureArray,'/selection_test/features',10)
    received={}
    output_sub=node.create_subscription(TopologicalMap,'/selection_test/output',lambda msg:received.update(map=msg),qos)
    ids_sub=node.create_subscription(Int32MultiArray,'/selection_test/ids',lambda msg:received.update(ids=list(msg.data)),qos)
    def wait_for(predicate):
        end=time.monotonic()+15
        while time.monotonic()<end:
            if process is not None and process.poll() is not None: raise RuntimeError('目標選択ノードの早期終了')
            rclpy.spin_once(node,timeout_sec=0.05)
            if predicate(): return
        raise RuntimeError('ROS出力待ちの期限超過')
    def verify(name,ids,positions,frame):
        wait_for(lambda:received.get('ids')==ids and 'map' in received and received['map'].frame_number==frame and [n.id for n in received['map'].nodes]==([7,7] if ids==[7] else ids) and all(abs(a.pos.x-b)<1e-6 for a,b in zip(received['map'].nodes,positions)))
        checks.append(name)
    with (out/'ros_node.log').open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        wait_for(lambda:map_pub.get_subscription_count()>0 and source_pub.get_subscription_count()>0 and feature_pub.get_subscription_count()>0)
        map_msg=TopologicalMap();map_msg.header.frame_id='base';map_msg.frame_number=1
        for idx,x in [(65535,0.01),(7,0.04),(7,5.0)]:
            value=TopologicalNode();value.id=idx;value.label=1;value.pos.x=x;value.pos.y=value.pos.z=0.01;value.normal.z=1.0;map_msg.nodes.append(value)
        source=GraspCandidateArray();source.header.frame_id=source.evaluation_header.frame_id='base';source.voxel_size=0.1
        value=GraspCandidate();value.state=GraspCandidate.INSIDE;value.pose.position.x=0.035;value.pose.position.y=value.pose.position.z=0.01;value.pose.orientation.w=1.0;source.candidates.append(value)
        features=TopologicalNodeFeatureArray()
        for idx,condition in [(65535,10.0),(7,99.0),(7,2.0)]:
            value=TopologicalNodeFeature();value.node_id=idx;value.manip_valid=True;value.manip_condition_number=condition;features.features.append(value)
        feature_pub.publish(features);map_pub.publish(map_msg);source_pub.publish(source)
        verify('duplicate_ids_and_last_feature',[7],[0.04,5.0],1)
        features.features[-1].manip_valid=False;feature_pub.publish(features)
        verify('feature_update',[65535],[0.01],1)
        map_msg=copy.deepcopy(map_msg);map_msg.frame_number=9;map_msg.nodes[0].id=42;map_msg.nodes=[map_msg.nodes[2],map_msg.nodes[0],map_msg.nodes[1]];map_pub.publish(map_msg)
        verify('map_id_and_order_update',[42],[0.01],9)
        feature_pub.publish(TopologicalNodeFeatureArray())
        verify('empty_features',[7],[5.0,0.04],9)
        map_msg.nodes.pop();map_msg.frame_number=10;map_pub.publish(map_msg)
        verify('removed_node',[42],[0.01],10)
        value=TopologicalNode();value.id=900;value.label=1;value.pos.x=0.035;value.pos.y=value.pos.z=0.01;value.normal.z=1.0;map_msg.nodes.append(value);map_msg.frame_number=11;map_pub.publish(map_msg)
        verify('added_node',[900],[0.035],11)
        map_msg.nodes[-1].pos.x=9.0;map_msg.frame_number=12;map_pub.publish(map_msg)
        verify('moved_node',[42],[0.01],12)
        source.candidates.clear();source_pub.publish(source)
        verify('empty_candidates',[],[],12)
finally:
    if process is not None:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
    if node is not None: node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    (out/'ros_smoke.json').write_text(json.dumps({'checks':checks,'node_pid':process.pid if process else None,'node_exit':process.returncode if process else None,'is_stopped':process is None or process.poll() is not None},indent=2)+'\n')
print('ROS integration checks='+str(len(checks))+' node stopped',flush=True)
