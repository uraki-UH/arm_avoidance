uint8 STATE_RELEASED=0
uint8 STATE_GRASPED=1

uint8 SHAPE_BOX=0

std_msgs/Header header
uint8 state
string object_id
string eef_link
uint8 shape_type
geometry_msgs/Pose object_pose_in_eef
geometry_msgs/Vector3 dimensions
