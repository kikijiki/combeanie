# Per-station acquisition accounting for one visited tray overview station (Card 050).
#
# Every count is stamp-windowed on the station's own dwell window: the instant the station's
# motion stopped plus acquisition_dwell_ms, through overview_collection_ms. Reporting only —
# which frames the survey admits is unchanged by these counts; they exist so a station that
# admits zero frames names the stage that lost them instead of being silent about it.
#
# The stage a zero read from here points at:
#   images = 0 (with the tap configured)     -> the input stream delivered nothing in the window
#   images > 0, detection_frames = 0         -> the duty is not processing acquisitions
#   detection_frames > 0, frames_with_detections = 0 -> the colour stage found nothing in view
#   frames_with_detections > 0, published = 0 -> estimation or identity dropped every proposal
#   published > 0, admitted = 0              -> backend, status or stamp-window mismatch
# and admitted is exactly the count the survey has always logged as frame(s).

# The station these counts belong to (parallel to overview_stations_visited).
string station

# Whether the survey node has a subscription on the corresponding input topic at all. False
# means the count below cannot be read as evidence about that stage — the tap does not exist.
bool image_tap_configured
bool detection_tap_configured

# Colour image frames stamped inside the window (sensor_msgs/Image header stamps).
uint32 images
# Overview-duty detection frames stamped inside the window (PerceptionFrame on the duty's
# detection topic), and how many of those carried at least one proposal.
uint32 detection_frames
uint32 frames_with_detections
# Overview-topic observation messages stamped inside the window, regardless of backend or
# status: what the duty published into the window.
uint32 published
# What the survey admitted: overview backend, status OK, stamp inside the window.
uint32 admitted

# Card 050's zero-dwell drain: true when the first snapshot admitted nothing and the survey
# waited one further overview_collection_ms of arrival time before declaring the station. The
# stamp window never grows — only the arrival deadline of a would-be-zero station moves.
bool drained
# In-window frames/observations that arrived only during that drain (after arrival_grace_ms
# past the window end). Zero when not drained; the totals above include them.
uint32 late_images
uint32 late_published
