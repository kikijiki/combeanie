uint8 STATE_STARTING=0
uint8 STATE_SYNCHRONIZING=1
uint8 STATE_APPLIED=2
uint8 STATE_DEGRADED=3
uint8 STATE_TRANSACTION_DRAINING=4
uint8 STATE_TRANSACTION_HELD=5
uint8 STATE_TRANSACTION_RELEASING=6

uint16 ERROR_NONE=0
uint16 ERROR_CONFIGURATION=1
uint16 ERROR_WORLD_STATE_UNAVAILABLE=2
uint16 ERROR_PLANNING_SCENE_UNAVAILABLE=3
uint16 ERROR_TRANSFORM_UNAVAILABLE=4
uint16 ERROR_INVALID_SNAPSHOT=5
uint16 ERROR_APPLY_REJECTED=6
uint16 ERROR_VERIFICATION_FAILED=7
uint16 ERROR_APPLY_OUTCOME_UNKNOWN=8
# Depth-derived obstacle evidence has aged out. The geometry already in the scene is retained,
# because a stale obstacle observation means "I can no longer see", not "nothing is there". The
# projector reports this state instead of certifying the projection, and every consumer that
# gates on STATE_APPLIED refuses to execute.
uint16 ERROR_OBSTACLE_EVIDENCE_STALE=9
# A product observation in the world snapshot is stamped beyond the projector's future-skew
# tolerance. The geometry already in the scene is retained, the cycle is abandoned rather than
# certified, and the condition clears once now() advances past the stamp (under load the
# projector's clock can lag the producer's by 50 to 100 ms, which is a transient, not a property
# of the data). A product observation that is merely old is not an error: the product is projected
# at its last known pose and named in the applied detail (Milestone 10 §6, Card 071). Reported
# separately from ERROR_INVALID_SNAPSHOT, which means the snapshot cannot be read at all (a
# permanent property of the data that fails identically on every retry), so a momentary gap or
# clock-lag skew in the observation stream is distinguishable from an unusable world.
uint16 ERROR_OBSERVATION_EVIDENCE_STALE=10

std_msgs/Header header
string projector_epoch
uint8 state
uint64 desired_revision
uint64 applied_revision
uint64 verification_epoch
uint64 scene_content_generation
uint64 lease_id
uint8 lease_phase
uint16 error_code
string detail
