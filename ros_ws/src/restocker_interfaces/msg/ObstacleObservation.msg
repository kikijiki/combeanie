# Unmodeled obstacles recovered from one depth image.
#
# The whole message describes a single instant: header.stamp is the depth image's own stamp, and
# every box was transformed into header.frame_id using the sensor pose looked up at exactly that
# stamp. A consumer that mixes these boxes with a transform from any other time reintroduces the
# frame-versus-instant defect, so the sensor frame and the stamp are both carried explicitly.

# stamp: the depth image stamp. frame_id: the planning frame the boxes are expressed in.
std_msgs/Header header

# Optical frame the depth image was produced in. Recorded for provenance only; the boxes are
# already in header.frame_id.
string sensor_frame

# Monotonic count of published observations, starting at 1. A restart of the publisher restarts
# the sequence, which is how a consumer tells a restarted pipeline from a stalled one.
uint64 sequence

# True when the robot links used for self-filtering were stationary across the probe window
# ending at header.stamp. A consumer may refuse to adopt new geometry from a moving robot: a link
# that moved during the exposure can survive self-filtering and be published as an obstacle.
bool robot_static

# Surviving clusters, ordered deterministically by centre so an unchanged scene yields an
# unchanged message.
ObstacleBox[] boxes
