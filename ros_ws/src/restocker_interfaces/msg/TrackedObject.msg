uint8 PRODUCT_CLASS_UNKNOWN=0
uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3

uint8 ORIENTATION_UNKNOWN=0
uint8 ORIENTATION_UPRIGHT=1
uint8 ORIENTATION_HORIZONTAL=2
uint8 ORIENTATION_TILTED=3

uint8 TRACKING_TRACKED=0
uint8 TRACKING_OCCLUDED=1
uint8 TRACKING_LOST=2
uint8 TRACKING_REMOVED=3

uint8 GRASP_FREE=0
uint8 GRASP_ATTACHED=1

uint64 id
string source_object_id
uint8 product_class
bool has_sku
string sku
geometry_msgs/PoseWithCovariance pose
uint8 orientation
uint8 tracking_state
uint8 grasp_state
builtin_interfaces/Time observation_time
builtin_interfaces/Time transition_time
uint64 revision
