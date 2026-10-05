# One normalized observation of a configured shelf lane.
# The observation uses header.frame_id at header.stamp. Geometry fields are
# expressed in meters. observed_source_object_ids is evidence only: consumers
# must validate it and must not overwrite authoritative ShelfLane.contents.

uint8 STATUS_OK=0
uint8 STATUS_INVALID_GEOMETRY=1
uint8 STATUS_DUPLICATE_SOURCE=2
uint8 STATUS_MISSING_TIMESTAMP=3
# The producer looked and could not see enough of the lane to answer. This is a distinct status,
# not a low confidence: low confidence is a weak answer, this is a refusal to give one.
#
# A lane is read as empty when nothing returns inside its window, so a producer that cannot see
# (blinded lens, missed viewpoint, lane in shadow) would report the emptiest possible lane with
# no evidence, and "empty" opens the capacity gate on a lane that is full. A camera-derived
# producer must be able to say "I could not see" in a form a consumer cannot mistake for "it is
# empty"; available_depth_m is not that form.
uint8 STATUS_INSUFFICIENT_COVERAGE=4
uint8 STATUS_INTERNAL_ERROR=255

std_msgs/Header header
string lane_id
float64 available_depth_m
bool obstructed
# Successful producers publish this sequence in lexicographic order without duplicates.
string[] observed_source_object_ids
# What the producer claims for this observation, on [0, 1]; consumers gate on it.
#
# A producer that measures a lane geometrically reports coverage here: the fraction of the
# lane's depth between the rear entrance and the nearest surface it resolved. A lane reported
# empty at coverage 1.0 was seen to be empty along its whole length; at coverage 0.05 it was
# not seen at all. It is not a detection posterior and must not be compared against a threshold
# derived from one.
float32 confidence
string backend_name
string backend_version
uint8 status
string status_detail
