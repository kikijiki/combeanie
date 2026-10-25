# One lane's line in the inventory (CMB-115): what the lane stock estimator last said about it,
# how old that is, what it is believed to hold and how far that belief is evidence, and whether the
# authoritative world state agrees.
#
# Counts are estimates from geometry and never authoritative (ShelfLane.contents stays the ledger).
# Every identity field is optional evidence: a lane whose products were not or could not be named
# still has a count, and `sku_basis` says how much of its SKU is policy expectation.

# observation: whether the lane was ever read, and whether the reading is still fresh. A stale
# lane keeps its last-known fields so an operator can see them, but it is left out of every total.
uint8 OBSERVATION_NEVER=0
uint8 OBSERVATION_FRESH=1
uint8 OBSERVATION_STALE=2

# sku_basis: what supports the SKU of the lane's products. Counting is by geometry; the SKU is
# the lane policy's unless the identity channel named the products.
# UNKNOWN: no policy and nobody named the products (or the lane is empty or was not read).
uint8 SKU_BASIS_UNKNOWN=0
# EXPECTED_ONLY: the policy's SKU is assumed; no product was named.
uint8 SKU_BASIS_EXPECTED_ONLY=1
# PARTLY_CONFIRMED: at least one product was named as the expected SKU, the rest are assumed.
uint8 SKU_BASIS_PARTLY_CONFIRMED=2
# CONFIRMED: every product in the lane was named as the expected SKU.
uint8 SKU_BASIS_CONFIRMED=3
# CONTESTED: a wrong product was found (named SKU or class); the policy SKU holds for no product in
# particular and the lane's totals are widened.
uint8 SKU_BASIS_CONTESTED=4

# count_discrepancy: the observed count against the world state's ledger (ShelfLane.contents).
uint8 COUNT_DISCREPANCY_NONE=0
# The observed count range lies above the ledger's count: product the ledger does not know.
uint8 COUNT_DISCREPANCY_OBSERVED_HIGHER=1
# The observed count range lies below the ledger's count: product the ledger believes is there.
uint8 COUNT_DISCREPANCY_OBSERVED_LOWER=2
# No comparison was made: the lane was not read, is stale, or the world state has no such lane.
# This is not agreement.
uint8 COUNT_DISCREPANCY_NOT_COMPARABLE=3

string lane_id

uint8 observation
# Stamp of the LaneStock this line came from, and its age at the inventory's stamp.
builtin_interfaces/Time observed_at
float64 age_s

# True when the lane was read but the estimator could not answer (a refusal): the count fields are
# not an answer. `cannot_tell_reason` is LaneStock.REASON_*.
bool cannot_tell
uint8 cannot_tell_reason

bool has_expected_sku
string expected_sku
# What the products are. confirmed_count products were named by the identity channel (their SKUs
# are in confirmed_skus, one entry per named product), the rest of the count rests on
# `sku_basis`. unknown_count products are known not to be a registered SKU (the channel said so).
uint8 sku_basis
uint32 confirmed_count
string[] confirmed_skus
uint32 unknown_count
bool misplaced
string[] misplaced_skus

uint32 count_estimate
uint32 count_min
uint32 count_max
bool empty

# The world state's side.
bool has_world
uint32 world_count
bool world_ledger_unreliable
uint8 count_discrepancy
# Signed distance from the ledger to the nearest edge of the observed range (positive: more seen
# than recorded). Zero unless there is a count discrepancy.
int32 count_delta
# True when a product the identity channel named is not the SKU the world state records for the
# lane's rearmost product. Never raised from an abstention.
bool sku_discrepancy
string discrepancy_detail
