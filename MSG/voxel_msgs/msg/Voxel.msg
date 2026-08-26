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

# 全量snapshotの単調増加更新番号
# 0は更新番号なしのstream
uint32 revision

# Flattened voxel IDs
int64[] data

# Optional per-voxel labels aligned with data. Publishers that do not classify
# voxels leave this empty; when populated, labels.length must equal data.length.
uint8[] labels
