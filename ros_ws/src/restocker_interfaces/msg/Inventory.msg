# The queryable stock inventory (CMB-115): per-SKU totals with explicit uncertainty, per-lane
# contents, observation age against a declared horizon, and discrepancy against the authoritative
# world state. It aggregates LaneStock (the lane stock estimator) and the tray; it changes nothing
# upstream, and ShelfLane.contents stays the ledger.
#
# Nothing here is a posterior. A consumer reads `complete`, the ranges and the bases, not a
# confidence number.

uint8 TRAY_SOURCE_NONE=0
# The tray's products are listed by the world state; no sensor reading backs them (in simulation
# the world state is fed ground truth, which is not a sensor).
uint8 TRAY_SOURCE_WORLD_STATE=1
# The tray's products come from the perception path (geometry proposals and the identity channel).
uint8 TRAY_SOURCE_PERCEPTION=2

std_msgs/Header header
# Observations older than this at the header stamp are stale and excluded from totals.
float64 stale_after_s
# The filters this inventory was computed under (a query's, or the node's defaults).
float64 max_age_s
uint8 min_confidence

InventoryLane[] lanes
InventorySku[] skus

uint8 tray_source
uint32 tray_count
builtin_interfaces/Time tray_observed_at
float64 tray_age_s
bool tray_stale

bool world_available
uint64 world_revision
uint32 lanes_total
uint32 lanes_fresh
uint32 lanes_stale
uint32 lanes_unread
uint32 discrepancy_count
