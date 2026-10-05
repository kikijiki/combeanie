# Every detection extracted from one RGB-D acquisition, delivered atomically.
#
# header.stamp is the acquisition time and header.frame_id is the camera optical
# frame in which every bounding box and mask was measured. Consumers must not
# merge detections across PerceptionFrame messages or assume a detection is
# valid at another stamp or frame.
#
# Backend-neutral: backend tensors, weights, and image encodings do not appear here.

uint8 STATUS_OK=0
uint8 STATUS_INVALID_FRAME=1
uint8 STATUS_BACKEND_UNAVAILABLE=2
uint8 STATUS_MISSING_TIMESTAMP=3
uint8 STATUS_INTERNAL_ERROR=255

std_msgs/Header header
uint16 image_width
uint16 image_height
ObjectDetection[] detections
string backend_name
string backend_version
uint8 status
string status_detail
