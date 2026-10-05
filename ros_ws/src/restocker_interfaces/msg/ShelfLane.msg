uint8 PRODUCT_CLASS_UNKNOWN=0
uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3

string id
uint8 expected_product_class
bool has_expected_sku
string expected_sku
# Desired stocking: expected_product_class and expected_sku say what the owner wants here,
# target_count says how many. Zero is a declared "want none", not an unset default.
uint32 target_count
# Semantic FIFO ledger of the products the robot placed: back() is the rearmost (last placed),
# front() is the customer-facing entry that a measured shortfall pops first.
uint64[] contents
# Fresh occupancy evidence is source identity, separate from semantic contents.
string[] observed_source_object_ids
float64 depth_m
float64 available_depth_m
bool obstructed
builtin_interfaces/Time last_verified
# Event-driven invalidation of the lane's depletion evidence. A placement into the lane, an
# operator intervention, or a policy change sets this; the next accepted LaneObservation clears
# it. Aged-but-valid evidence may still be selected against; invalidated evidence may not.
bool evidence_invalidated
builtin_interfaces/Time evidence_invalidated_at
# Revision of the latest observation evidence; zero means no observation has been accepted.
uint64 evidence_revision
uint64 revision
# Identity trust for the contents ledger: set when the measured column length diverges beyond
# half a product pitch from what the settled contents (plus any in-flight Attached placement on
# the growth side only) can explain — unexplained growth, or a shortfall that is not a whole
# number of front-entry sales. Once set it stays set. Geometry stays authoritative — depth
# evidence, the occupied-volume projection, and geometry-safe placements continue — but
# reported contents are withdrawn from the deficit calculation and the condition is surfaced
# to the operator as a LANE_LEDGER_UNRELIABLE event.
bool ledger_unreliable
