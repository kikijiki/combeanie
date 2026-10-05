# Stable result code for planning-scene lease services.
# Consumers branch on code, never on detail.

uint16 UNSET=0
uint16 DRAINING=1
uint16 GRANTED=2
uint16 VALID=3
uint16 RELEASE_ACCEPTED=4
uint16 INVALID_ARGUMENT=5
uint16 CONFLICT=6
uint16 TOKEN_MISMATCH=7
uint16 IDEMPOTENCY_CONFLICT=8
uint16 RESOURCE_EXHAUSTED=9
uint16 INTERNAL_ERROR=255

uint16 code
string detail
