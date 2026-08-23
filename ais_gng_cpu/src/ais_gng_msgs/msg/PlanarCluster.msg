# A connected planar patch derived from the existing GNG node graph.
# `node_indices` refer to indices in the TopologicalMap.nodes array that
# produced this message.  They deliberately remain map-local: no persistent
# global-node identifier is assumed by the extractor.
uint32 id
uint32[] node_indices

geometry_msgs/Point32 centroid
geometry_msgs/Vector3 normal
geometry_msgs/Vector3 tangent_u
geometry_msgs/Vector3 tangent_v

# Convex 2-D boundary projected back into 3-D.  It is a compact diagnostic
# footprint, not an assertion that the whole convex hull was observed.
geometry_msgs/Point32[] boundary

float32 area
float32 extent_u
float32 extent_v
float32 local_spacing
float32 planarity
float32 residual_ratio
