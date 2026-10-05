# Measured disagreement between one camera-derived lane observation and the simulator's own
# ground-truth observation of the same lane, to check that the measured available_depth_m agrees
# with ground truth within a stated bound.
#
# Lane-depth counterpart of PoseErrorSample, with the same use: evaluation output only. Nothing
# on the execution path may consume it, and ground truth must never reach authoritative world
# state through it; ground truth stays on its own topic so it can measure the camera without
# being the camera.
#
# available_depth_error_m is signed. available_depth_m is the gap at the rear of the lane, so a
# positive error means the camera read the lane as emptier than it is, which opens the capacity
# gate on a lane with no room. Do not take the absolute value before looking at the sign.

uint8 STATUS_OK=0
# No ground-truth observation of this lane was held when the measurement arrived.
uint8 STATUS_NO_GROUND_TRUTH=1
# The two observations disagree about which lane they describe, or about the frame it is in.
uint8 STATUS_LANE_MISMATCH=2
uint8 STATUS_STALE_PAIRING=3
# The measurement itself declined to answer (insufficient coverage, or a producer error). The
# pair is reported rather than dropped, since a refused measurement is a result.
uint8 STATUS_MEASUREMENT_UNUSABLE=4
uint8 STATUS_INTERNAL_ERROR=255

std_msgs/Header header
string lane_id
float64 measured_available_depth_m
float64 ground_truth_available_depth_m
# measured minus ground truth. Positive reads emptier than the lane is.
float64 available_depth_error_m
# The measurement's own self-reported coverage, carried through so a distribution of the error
# can be conditioned on it rather than pooled over measurements of unequal quality.
float32 coverage
bool measured_obstructed
bool ground_truth_obstructed
builtin_interfaces/Time measurement_stamp
builtin_interfaces/Time ground_truth_stamp
string backend_name
string backend_version
uint8 status
string status_detail
