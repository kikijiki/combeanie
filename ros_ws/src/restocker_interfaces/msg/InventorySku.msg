# One SKU's line in the inventory (CMB-115): how many of it are in stock, split by how well each
# is known, and whether the total can be trusted as complete.
#
# The total is a range over three kinds of product:
#   confirmed       named as this SKU by the identity channel (exact);
#   expected_*      counted by a sensor in a lane whose policy says this SKU and nobody contradicted
#                   it (a range: the count's own range, and widened where a lane is contested);
#   reported        listed by the world state only (no sensor looked), the tray's fallback source.
# An empty `sku` with is_unknown_bucket set holds the products whose SKU nobody knows.

string sku
bool is_unknown_bucket

uint32 confirmed_count
uint32 expected_count_min
uint32 expected_count_max
uint32 reported_count
uint32 total_min
uint32 total_max
uint32 total_estimate

# complete: every lane that could hold this SKU (its policy lanes and any lane it was named in)
# was read and is fresh. When false the total is what the read lanes hold, not what is in stock:
# `unread_lanes` were refused or never read, `stale_lanes` are older than the horizon, and
# `contested_lanes` hold a wrong product so their share is widened.
bool complete
string[] unread_lanes
string[] stale_lanes
string[] contested_lanes

# Of the total: how many sit on the tray, and where the tray's figure came from
# (Inventory.TRAY_SOURCE_*).
uint32 tray_count
uint8 tray_source

# The world state's figure for this SKU (ledger contents of lanes plus tray), when it has one.
bool has_world
uint32 world_count
# total_estimate minus world_count; meaningful only when `complete` and `has_world`.
int32 world_difference
