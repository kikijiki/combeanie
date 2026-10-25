# What is in one lane and how many: the stock estimator's statement (CMB-114). It sits beside
# LaneObservation and changes nothing about it: LaneObservation is the depth measurement this is
# derived from, and its consumers are unaffected.
#
# Counts are estimates from geometry (occupied depth over product length) and are never
# authoritative: ShelfLane.contents stays the ledger. Every identity field is optional evidence:
# a lane whose products were not or could not be named still has a count. Nothing here is a
# posterior; the consumer reads `state`, `count_min` and `count_max`, not a confidence.

# state: the estimator either answers or says it cannot. A cannot-tell is not an empty lane: when
# state is STATE_CANNOT_TELL every count, depth and composition field below it is not an answer
# and must be ignored; `reason` says why.
uint8 STATE_ESTIMATED=0
uint8 STATE_CANNOT_TELL=1

uint8 REASON_NONE=0
# The depth producer refused (LaneObservation.status was not OK) or sent no observation.
uint8 REASON_NO_EVIDENCE=1
# LaneObservation.confidence (the fraction of the lane's depth the camera accounted for) was
# below the estimator's floor.
uint8 REASON_LOW_COVERAGE=2
# Something protrudes out of the lane mouth, so the column cannot be read as a column.
uint8 REASON_OBSTRUCTED=3
# The occupied depth is not a whole number of products at any catalogued length, or the rear
# product's measured height contradicts the lane policy's class by an amount no class explains.
uint8 REASON_INCONSISTENT_GEOMETRY=4
# No lane policy (expected product class) is known, so a product length cannot be chosen.
uint8 REASON_NO_POLICY=5

# Composition: what the evidence says about the products' identity against the lane policy. It
# is deliberately not a boolean "mixed": most products in a column are hidden behind the rear one
# and an unnamed product is not a conforming product.
uint8 COMPOSITION_UNKNOWN=0
# Every product in the column was named and every name is the expected SKU.
uint8 COMPOSITION_CONFORMING=1
# At least one product was named and every name is the expected SKU; the rest were not named.
uint8 COMPOSITION_PARTIAL=2
# At least one named product is a different SKU, or the rear product's measured geometry is of a
# different class than the lane policy. `misplaced` is set.
uint8 COMPOSITION_MISPLACED=3
# The lane is empty (and was seen to be).
uint8 COMPOSITION_EMPTY=4

# Identity outcome per visible product face, in `face_skus` order. UNAVAILABLE means no identity
# channel was reachable (sidecar absent or no crop could be framed), ABSTAIN means the channel
# looked and would not name one product, UNKNOWN means it has positive evidence the product is
# not registered.
uint8 IDENTITY_UNAVAILABLE=0
uint8 IDENTITY_NAMED=1
uint8 IDENTITY_ABSTAIN=2
uint8 IDENTITY_UNKNOWN=3

std_msgs/Header header
string lane_id
uint8 state
uint8 reason
string reason_detail

# Lane policy as given by the owner: what should be here.
uint8 expected_product_class
bool has_expected_sku
string expected_sku

# Geometry: free depth is LaneObservation.available_depth_m, occupied is usable depth minus it.
float64 free_depth_m
float64 occupied_depth_m
float64 usable_depth_m
float32 coverage
# The product length along the lane (its diameter, plus the contact pitch factor) the count was
# divided by, and the class it belongs to (expected_product_class unless the rear product's
# measured height says otherwise).
float64 product_length_m
uint8 product_class_assumed

# The count. count_estimate is the nearest whole number of product lengths; the closed range
# [count_min, count_max] is what the geometry and the visible faces together allow, and is the
# quantity to compare against a tolerance. count_consistent is false when the faces disagree with
# the depth by more than one product, in which case the range is widened to cover both.
uint32 count_estimate
uint32 count_min
uint32 count_max
bool count_consistent
# Products the camera resolved as separate visible faces (a cross-check on the depth count, not a
# count of the lane: hidden products are not seen).
uint32 visible_faces

# Identity, parallel arrays with one entry per visible face the estimator tried to name.
bool identity_channel_reachable
string[] face_skus
uint8[] face_identity_status
uint32 named_count
uint32 unverified_count

uint8 composition
# Set when a named product is not the expected SKU, or the rear product's class contradicts the
# policy. Never set from an abstention.
bool misplaced
uint32 misplaced_named_count
string[] misplaced_skus

# Provenance of the identity channel that was consulted, so a stored statement says which
# registry and calibration produced its names. Empty when no channel was reachable.
string identity_registry_sha256
string identity_calibration_sha256
string backend_name
string backend_version
