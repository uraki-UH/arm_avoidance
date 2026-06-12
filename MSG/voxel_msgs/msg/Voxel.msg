std_msgs/Header header

# Voxel interpretation metadata
float32 voxel_size
# Grid origin in world coordinates. Leave as (0, 0, 0) when unused.
float32 origin_x
float32 origin_y
float32 origin_z
int32 x_shift
int32 y_shift
int32 z_shift
int64 offset

# Flattened voxel IDs
int64[] data
