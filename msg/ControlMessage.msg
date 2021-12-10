# Control message type
# 0: state and control
# 1: state only
# TB: added for working with locks that need to be retained through multiple actions (see ActionBehavior.cc)
#	locks_to_retain, prev_name

NodeBitmask sender
int32 type
float32 activation_level
float32 activation_potential
bool done
bool active
NodeBitmask highest
int32 parent_type
bool collision
bool robotPlacing
bool humanPlacing
bool peerUndone
string simstate_obj_name
geometry_msgs/Pose simstate_obj_pose
uint8 simstate_robot_id
geometry_msgs/Pose simstate_robot_pose
geometry_msgs/Pose simstate_robot_goal
htt_viz/Actuator[] actuators_to_retain
#actuatorState
htt_viz/ActuatorState state
#actuatorRequest
htt_viz/ActuatorRequest[] requests