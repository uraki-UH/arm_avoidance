std_msgs/Header header

# Grid metadata. These fields must match the Voxel snapshot for this revision.
float32 voxel_size
float32 origin_x
float32 origin_y
float32 origin_z
int32 x_shift
int32 y_shift
int32 z_shift
int64 offset

# Monotonic revision shared with the corresponding Voxel snapshot.
uint32 revision

# Changed flattened voxel IDs. old_labels and new_labels are aligned with data.
# A label value of 255 means that the voxel was absent before or after the update.
int64[] data
uint8[] old_labels
uint8[] new_labels
