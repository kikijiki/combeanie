# Measured 6-DoF error of one perception pose estimate against simulator ground truth.
#
# Both poses are compared in header.frame_id. A sample is only produced when the estimate and the
# ground truth were resolved into that same frame; otherwise status reports why no comparison was
# made.
#
# Evaluation output only. Nothing on the execution path may consume it, and ground truth must
# never reach authoritative world state through it.

uint8 STATUS_OK=0
uint8 STATUS_NO_GROUND_TRUTH=1
uint8 STATUS_FRAME_MISMATCH=2
uint8 STATUS_STALE_PAIRING=3
uint8 STATUS_INTERNAL_ERROR=255

# Whether the scored estimate contained an orientation estimate at all (separate from whether the
# sample could be produced).
#
# ORIENTATION_NOT_ESTIMATED means the estimate's own covariance declares its orientation
# unmeasured, so axis_error_rad is NaN. Reporting 0.0 instead would make an orientation-blind
# pipeline and a perfect one produce the same number.
uint8 ORIENTATION_NOT_ESTIMATED=0
uint8 ORIENTATION_AXIS_ESTIMATED=1

std_msgs/Header header
string source_object_id
float64 translation_error_m
# Straight-line range from the estimate camera's optical origin to the estimated product centre
# at the estimate stamp, in metres. NaN when the evaluator was not configured with a camera frame
# or TF could not resolve that stamp.
float64 observation_range_m
# Angle between the estimated and true symmetry axes, in [0, pi/2], or NaN when
# orientation_status says the estimate did not carry one.
#
# Only the axis is scored: every catalogued product is a cylinder, so its yaw is unobservable.
# NaN (not 0.0) for the unestimated case, so it does not average into a mean as a good result.
float64 axis_error_rad
uint8 orientation_status
builtin_interfaces/Time estimate_stamp
builtin_interfaces/Time ground_truth_stamp
string backend_name
string backend_version
uint8 status
string status_detail
