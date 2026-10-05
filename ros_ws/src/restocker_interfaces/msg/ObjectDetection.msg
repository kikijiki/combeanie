# One 2D detection, with optional segmentation, inside a single RGB-D
# acquisition. Product classes mirror ObjectObservation so a detection and the
# observation derived from it cannot disagree about category.
#
# A detection carries no stamp or frame: PerceptionFrame owns both for all of
# its detections, so siblings cannot differ in time or frame.

uint8 PRODUCT_CLASS_UNKNOWN=0
uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3

# Pixel bounding box, half-open [min, max) in the owning frame's image.
uint16 x_min
uint16 y_min
uint16 x_max
uint16 y_max
uint8 product_class
float32 score
# Stable tracking identity across frames; 0 means this detection is untracked.
uint32 instance_id
# Source identity this detection resolves to; empty when unresolved.
string source_object_id
bool has_mask
ObjectSegmentation mask
