import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {updateNodeInstances,updateEdgeInstances} from '@topo/visualization/gng_graphics';
import type {GraphNode} from '@topo/visualization/graph_types';

const make_node=(id:number):GraphNode=>({id,x:id*.03,y:Math.sin(id),z:Math.cos(id),nx:0,ny:0,nz:1,label:1,age:0});
const make_mesh=()=>new THREE.InstancedMesh(new THREE.BoxGeometry(),new THREE.MeshBasicMaterial(),128);
const dispose_mesh=(mesh:THREE.InstancedMesh)=>{mesh.dispose();mesh.geometry.dispose();(mesh.material as THREE.Material).dispose();};

test('GNG差分更新: 不変時のGPU転送抑止と姿勢・色・並替え・属性交換への追従',()=>{
 const nodes=Array.from({length:64},(_,idx)=>make_node(idx));
 const node_mesh=make_mesh(),edge_mesh=make_mesh();let scale=.01,width=.003;
 let edges=nodes.flatMap((_,idx)=>[idx,(idx+1)%nodes.length]);
 try{
  const verify=()=>{
   updateNodeInstances(node_mesh,nodes,scale);updateEdgeInstances(edge_mesh,edges,nodes,width);
   const reference_nodes=make_mesh(),reference_edges=make_mesh();
   try{
    updateNodeInstances(reference_nodes,nodes,scale);updateEdgeInstances(reference_edges,edges,nodes,width);
    assert.deepEqual(node_mesh.instanceMatrix.array.slice(0,node_mesh.count*16),reference_nodes.instanceMatrix.array.slice(0,reference_nodes.count*16));
    assert.deepEqual(node_mesh.instanceColor?.array,reference_nodes.instanceColor?.array);
    assert.deepEqual(edge_mesh.instanceMatrix.array.slice(0,edge_mesh.count*16),reference_edges.instanceMatrix.array.slice(0,reference_edges.count*16));
    const versions=[node_mesh.instanceMatrix.version,node_mesh.instanceColor!.version,edge_mesh.instanceMatrix.version];
    updateNodeInstances(node_mesh,nodes,scale);updateEdgeInstances(edge_mesh,edges,nodes,width);
    assert.deepEqual([node_mesh.instanceMatrix.version,node_mesh.instanceColor!.version,edge_mesh.instanceMatrix.version],versions);
   }finally{dispose_mesh(reference_nodes);dispose_mesh(reference_edges);}
  };
  verify();nodes[1].x+=.123;nodes[5].label=3;verify();
  nodes.reverse();edges=edges.reverse();verify();
  scale=.02;width=.008;verify();
  node_mesh.instanceMatrix=new THREE.InstancedBufferAttribute(new Float32Array(128*16),16);node_mesh.instanceColor=null;verify();
  edge_mesh.instanceMatrix.array.fill(0);edge_mesh.instanceMatrix.needsUpdate=true;verify();
  nodes[2].x=nodes[3].x;nodes[2].y=nodes[3].y;nodes[2].z=nodes[3].z;verify();
 }finally{dispose_mesh(node_mesh);dispose_mesh(edge_mesh);}
});

test('GNG辺: 切断・再接続・クラスタ配色解除の更新',()=>{
 const nodes=[make_node(0),make_node(1)],mesh=make_mesh();
 try{
  updateEdgeInstances(mesh,[0,1],nodes,.002,new Map([[0,'#ff0000']]));
  const original=mesh.instanceMatrix.array.slice(0,16);
  updateEdgeInstances(mesh,[0,99],nodes,.002);
  assert.equal(mesh.instanceMatrix.array[0],0);
  updateEdgeInstances(mesh,[0,1],nodes,.002);
  assert.deepEqual(mesh.instanceMatrix.array.slice(0,16),original);
  assert.deepEqual(Array.from(mesh.instanceColor!.array.slice(0,3)),[1,1,1]);
 }finally{dispose_mesh(mesh);}
});
