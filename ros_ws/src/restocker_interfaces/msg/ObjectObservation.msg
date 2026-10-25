# One normalized semantic object observation from any perception backend.
# The pose uses header.frame_id at header.stamp. Covariance order follows
# geometry_msgs/PoseWithCovariance: x, y, z, rotation about X, Y, Z.

uint8 PRODUCT_CLASS_UNKNOWN=0
uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3
uint8 PRODUCT_CLASS_BOX=4

uint8 ORIENTATION_UNKNOWN=0
uint8 ORIENTATION_UPRIGHT=1
uint8 ORIENTATION_HORIZONTAL=2
uint8 ORIENTATION_TILTED=3

# What the identity channel concluded about this proposal (CMB-113). A producer that does not
# run the identity channel leaves IDENTITY_NOT_ASSESSED (0), which is every producer before this
# field existed. IDENTITY_UNKNOWN_PRODUCT is a proposal the identity channel could not name
# (it said unknown, it abstained, or it was unavailable; identity_reason says which): has_sku must
# be false, and the proposal is still a real object to avoid or reject, never a forced SKU.
# IDENTITY_IDENTIFIED requires has_sku. identity_evidence_* are parallel arrays (kind: embedding,
# ocr, barcode or geometry).
uint8 IDENTITY_NOT_ASSESSED=0
uint8 IDENTITY_IDENTIFIED=1
uint8 IDENTITY_UNKNOWN_PRODUCT=2

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
uint8 identity_state
string[] identity_evidence_kinds
float32[] identity_evidence_scores
string identity_reason
