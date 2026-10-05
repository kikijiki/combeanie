uint8 TASK_IDLE=0
uint8 TASK_VALIDATING_SCENE=1
uint8 TASK_EXECUTING=2
uint8 TASK_RECOVERING=3
uint8 TASK_FAULT=4
uint8 TASK_REQUESTING_OPERATOR=5

uint8 FAULT_NONE=0
uint8 FAULT_RECOVERABLE=1
uint8 FAULT_NON_RECOVERABLE=2
uint8 FAULT_EXTERNAL_INCONSISTENCY=3

float64[6] joint_positions
float64[6] joint_velocities
float64 rail_position
float64 rail_velocity
float64[2] gripper_joint_positions
float64[2] gripper_joint_velocities
bool has_held_object
uint64 held_object
# Pose of the held product's body frame in the gripper's grasp_center frame, fixed when the grasp
# was committed. The product is rigidly attached, so this does not change while held: composed
# with the robot's kinematics it gives the product's pose without observing it. Identity when
# has_held_object is false.
geometry_msgs/Pose grasp_center_from_held_object
uint8 task_phase
uint8 fault_state
builtin_interfaces/Time telemetry_time
string telemetry_source_id
uint64 telemetry_revision
uint64 revision
