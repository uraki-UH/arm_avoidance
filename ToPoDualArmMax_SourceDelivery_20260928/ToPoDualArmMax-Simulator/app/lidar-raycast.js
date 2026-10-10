import {MeshBVH} from './vendor/three-mesh-bvh/index.module.js';

// 同梱MeshBVHの直接索引形式専用。UV・法線・面情報の生成を省略
const geometry_queries=new WeakMap();
function geometry_query(geometry){
 let query=geometry_queries.get(geometry);if(query)return query;
 const data=MeshBVH.serialize(geometry.boundsTree,{cloneBuffers:false});
 if(data.indirectBuffer)throw Error('LiDAR索引は直接三角形索引のみ対応');
 query={positions:geometry.attributes.position.array,index:data.index,roots:data.roots.map(buffer=>({f32:new Float32Array(buffer),u32:new Uint32Array(buffer),u16:new Uint16Array(buffer)})),stack:[]};
 geometry_queries.set(geometry,query);return query;
}
function intersects_node(idx,bounds,ox,oy,oz,ix,iy,iz,max_dist){
 let near=(bounds[idx+(ix>=0?0:3)]-ox)*ix,far=(bounds[idx+(ix>=0?3:0)]-ox)*ix;
 const y_near=(bounds[idx+(iy>=0?1:4)]-oy)*iy,y_far=(bounds[idx+(iy>=0?4:1)]-oy)*iy;
 if(near>y_far||y_near>far)return false;
 if(y_near>near||near!==near)near=y_near;
 if(y_far<far||far!==far)far=y_far;
 const z_near=(bounds[idx+(iz>=0?2:5)]-oz)*iz,z_far=(bounds[idx+(iz>=0?5:2)]-oz)*iz;
 if(near>z_far||z_near>far)return false;
 if(z_near>near||near!==near)near=z_near;
 if(z_far<far||far!==far)far=z_far;
 return near<=max_dist+1e-7&&far>=0;
}

export function raycast_surface(geometry,ray,max_dist,target){
 const query=geometry_query(geometry),{positions:p,index,stack}=query;
 const {x:ox,y:oy,z:oz}=ray.origin,{x:dx,y:dy,z:dz}=ray.direction,ix=1/dx,iy=1/dy,iz=1/dz;
 let best_dist=Infinity,has_hit=false;
 for(const root of query.roots){
  const {f32,u32,u16}=root;stack.length=0;stack.push(0);
  while(stack.length){
   // 32バイトのBVHノード。32ビット要素で8個、葉の識別値は末尾の16ビット
   const idx=stack.pop();if(!intersects_node(idx,f32,ox,oy,oz,ix,iy,iz,Math.min(max_dist,best_dist)))continue;
   if(u16[idx*2+15]!==65535){
    const axis=u32[idx+7],left=idx+8,right=u32[idx+6],is_forward=(axis===0?dx:axis===1?dy:dz)>=0;
    stack.push(is_forward?right:left,is_forward?left:right);continue;
   }
   const end=u32[idx+6]+u16[idx*2+14];
   for(let tri=u32[idx+6];tri<end;tri++){
    const a=index[tri*3]*3,b=index[tri*3+1]*3,c=index[tri*3+2]*3;
    const ax=p[a],ay=p[a+1],az=p[a+2],ex=p[b]-ax,ey=p[b+1]-ay,ez=p[b+2]-az,fx=p[c]-ax,fy=p[c+1]-ay,fz=p[c+2]-az;
    const nx=ey*fz-ez*fy,ny=ez*fx-ex*fz,nz=ex*fy-ey*fx;
    let denom=dx*nx+dy*ny+dz*nz;if(denom===0)continue;
    const sign=denom>0?1:-1;if(sign<0)denom=-denom;
    const qx=ox-ax,qy=oy-ay,qz=oz-az;
    const u=sign*(dx*(qy*fz-qz*fy)+dy*(qz*fx-qx*fz)+dz*(qx*fy-qy*fx));if(u<0)continue;
    const v=sign*(dx*(ey*qz-ez*qy)+dy*(ez*qx-ex*qz)+dz*(ex*qy-ey*qx));if(v<0||u+v>denom)continue;
    const numerator=-sign*(qx*nx+qy*ny+qz*nz);if(numerator<0)continue;
    const t=numerator/denom,px=dx*t+ox,py=dy*t+oy,pz=dz*t+oz;
    const rx=px-ox,ry=py-oy,rz=pz-oz,dist=Math.sqrt(rx*rx+ry*ry+rz*rz);
    if(dist<=max_dist&&dist<best_dist){best_dist=dist;has_hit=true;target.set(px,py,pz);}
   }
  }
 }
 return has_hit;
}
