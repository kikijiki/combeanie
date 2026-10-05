# Stable result code for world-state reservation services.
# Consumers branch on code, never on detail.

uint16 UNSET=0
uint16 OK=1
uint16 INVALID_ARGUMENT=2
uint16 NOT_FOUND=3
uint16 REVISION_CONFLICT=4
uint16 RESERVATION_CONFLICT=5
uint16 TOKEN_MISMATCH=6
uint16 PREDICATE_FAILED=7
uint16 INVALID_TRANSITION=8
uint16 IDEMPOTENCY_CONFLICT=9
uint16 RESOURCE_EXHAUSTED=10
# Attach-only refusal: no mutation on this call; a bounded-future admitted object observation
# can be re-evaluated by the same immutable attachment transaction within its steady deadline.
uint16 ATTACHMENT_CLOCK_NOT_READY=11
# No current temporal continuation authority. An attachment reply may separately carry an
# immutable historical commit receipt; this code never authorizes lease release or motion.
uint16 CLOCK_AUTHORITY_INHIBITED=12
uint16 INTERNAL_ERROR=255

uint16 code
string detail
