#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include <iostream>
#include <algorithm>
using namespace fuzzrobo::topological_plane::incremental;
using ais_gng_msgs::msg::TopologicalMap;
TopologicalMap make_map() {
  TopologicalMap map;
  for (unsigned y=0;y<6;++y) for (unsigned x=0;x<8;++x) {
    ais_gng_msgs::msg::TopologicalNode node;
    node.id=y*8+x; node.frame=1; node.pos.x=x*.2; node.pos.y=y*.2;
    node.normal.z=1; node.label=TopologicalMap::SAFE_TERRAIN;
    map.nodes.push_back(node);
    if (x) {map.edges.push_back(y*8+x-1); map.edges.push_back(y*8+x);}
    if (y) {map.edges.push_back((y-1)*8+x); map.edges.push_back(y*8+x);}
  }
  return map;
}
int main() {
  for (const bool has_fresh_ids : {false,true}) {
    Clusterizer engine;
    auto map=make_map();
    for (unsigned f=1;f<=20;++f) {map.frame_number=f; engine.update(map);}
    std::vector<uint16_t> edges;
    for (size_t i=0;i<map.edges.size();i+=2) {
      if ((map.edges[i]%8<4)==(map.edges[i+1]%8<4)) {
        edges.push_back(map.edges[i]); edges.push_back(map.edges[i+1]);
      }
    }
    map.edges=edges;
    for (auto &node:map.nodes) if(node.id%8>=4) {
      node.pos.x+=20; node.frame=21;
      if(has_fresh_ids) node.id+=100;
    }
    for (unsigned f=21;f<=25;++f) {
      map.frame_number=f;
      const auto result=engine.update(map);
      float extent=0;
      for(const auto &c:result.clusters.clusters) extent=std::max(extent,std::max(c.extent_u,c.extent_v));
      std::cout<<"fresh_ids="<<has_fresh_ids<<" frame="<<f<<" clusters="<<result.clusters.clusters.size()<<" max_extent_m="<<extent<<"\n";
    }
  }
}
