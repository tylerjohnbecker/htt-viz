# Control message type
# 0: state and control
# 1: state only

NodeBitmask sender
int32 type
float32 activation_level
float32 activation_potential
bool done
bool active
NodeBitmask highest
int32 parent_type
bool collision
bool peerPlacing
bool selfPlacing
bool peerUndone
string simstate_obj_name
geometry_msgs/Pose simstate_obj_pose
uint8 simstate_robot_id
geometry_msgs/Pose simstate_robot_pose
geometry_msgs/Pose simstate_robot_goal
string peer_issue