# Public diagnostic summary of an active task reservation.
# The mutation capability token is not part of this message.

uint8 STAGE_RESERVED=0
uint8 STAGE_ATTACHED=1
uint8 STAGE_DETACHED=2

uint8 PRODUCT_CLASS_UNKNOWN=0
uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3

uint64 reservation_id
string request_id
uint64 object_id
string object_source_id
uint8 product_class
bool has_sku
string sku
bool has_source_lane
string source_lane_id
string destination_lane_id
uint8 stage
# Meaningful only in STAGE_DETACHED; false also represents safe release without membership.
bool placed_in_destination
builtin_interfaces/Time created_at
uint64 created_revision
uint64 admitted_robot_telemetry_revision
uint64 revision
# The destination lane's policy at the instant this reservation was granted. Lane intent may
# change while the transfer is in flight; revalidation compares against this captured policy,
# not the lane's live one, so a granted reservation completes under the policy it was granted.
uint8 destination_expected_product_class
bool has_destination_expected_sku
string destination_expected_sku
