# Normalized, telemetry-only robot state input. Semantic task, fault, attachment, and
# world-revision fields are absent and remain authority-owned.

string JOINT_1_NAME=shoulder_pan_joint
string JOINT_2_NAME=shoulder_lift_joint
string JOINT_3_NAME=elbow_joint
string JOINT_4_NAME=wrist_1_joint
string JOINT_5_NAME=wrist_2_joint
string JOINT_6_NAME=wrist_3_joint
string LEFT_FINGER_JOINT_NAME=left_finger_joint
string RIGHT_FINGER_JOINT_NAME=right_finger_joint

builtin_interfaces/Time stamp
string source_id
string[6] joint_names
float64[6] joint_positions
float64[6] joint_velocities
float64 rail_position
float64 rail_velocity
string[2] gripper_joint_names
float64[2] gripper_joint_positions
float64[2] gripper_joint_velocities
