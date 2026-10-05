# One normalized semantic object observation from any perception backend.
# The pose uses header.frame_id at header.stamp. Covariance order follows
# geometry_msgs/PoseWithCovariance: x, y, z, rotation about X, Y, Z.

uint8 PRODUCT_CLASS_UNKNOWN=0
uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3

uint8 ORIENTATION_UNKNOWN=0
uint8 ORIENTATION_UPRIGHT=1
uint8 ORIENTATION_HORIZONTAL=2
uint8 ORIENTATION_TILTED=3

uint8 STATUS_OK=0
uint8 STATUS_INVALID_POSE=1
uint8 STATUS_DUPLICATE_SOURCE=2
uint8 STATUS_MISSING_TIMESTAMP=3
uint8 STATUS_INTERNAL_ERROR=255

std_msgs/Header header
string source_object_id
uint8 product_class
bool has_sku
string sku
geometry_msgs/PoseWithCovariance pose
uint8 orientation
float32 confidence
string backend_name
string backend_version
uint8 status
string status_detail
