# Token-free evidence from the simulated physical-attachment boundary.

uint8 PHASE_UNKNOWN=0
uint8 PHASE_DETACHED=1
uint8 PHASE_VALIDATING_ATTACH=2
uint8 PHASE_APPLYING_ATTACH=3
uint8 PHASE_VERIFYING_ATTACH=4
uint8 PHASE_ATTACHED=5
uint8 PHASE_VALIDATING_DETACH=6
uint8 PHASE_APPLYING_DETACH=7
uint8 PHASE_VERIFYING_DETACH=8
uint8 PHASE_INCONSISTENT=9

uint8 MOTION_GATE_UNKNOWN=0
uint8 MOTION_GATE_INHIBITED=1
uint8 MOTION_GATE_VALID=2

string simulator_epoch
uint64 sequence
string operation_id
uint64 reservation_id
uint64 object_id
string object_source_id
string parent_model
string parent_link
string child_model
string child_link
uint8 phase
bool joint_observed
# This boundary is only one input to the coordinator's overall motion gate.
uint8 motion_gate
geometry_msgs/Pose parent_to_child
geometry_msgs/Pose child_pose_in_world
# Measured disagreement between the grasp the caller authorized and the grasp the simulator
# observed when the jaws closed. Reported on every attach that reached the comparison, pass or
# fail. False when the attach was refused before the comparison, or for a detach.
bool has_fidelity
# Euclidean norm of the translation difference, and the quaternion angular distance.
float64 fidelity_translation_residual_m
float64 fidelity_rotation_residual_rad
# The bound the translation residual was held to: the covariance-derived budget, or the configured
# constant when the caller stated no uncertainty, clamped to the geometric ceiling.
float64 fidelity_translation_budget_m
# Standard deviation the authorizing observation claimed, as the square root of the largest
# eigenvalue of the translational block of its covariance. Zero when none was stated.
float64 fidelity_claimed_translation_sigma_m
# Largest lateral offset at which the closing jaws still capture this product. The budget never
# exceeds this.
float64 fidelity_translation_ceiling_m
# Linear and angular child velocity relative to the parent, expressed in the parent link frame.
geometry_msgs/Twist relative_twist_in_parent
uint64 simulator_iteration
builtin_interfaces/Time observed_at
restocker_interfaces/SimulationAttachmentOperationStatus status
