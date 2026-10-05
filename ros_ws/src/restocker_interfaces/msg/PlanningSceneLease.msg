# Public diagnostic summary of a planning-scene transaction lease.
# The mutation capability token is not part of this message.

uint8 PHASE_NONE=0
uint8 PHASE_DRAINING=1
uint8 PHASE_HELD=2
uint8 PHASE_RELEASING=3

uint64 lease_id
string acquisition_operation_id
uint64 minimum_applied_revision
uint64 granted_applied_revision
uint64 verification_epoch
builtin_interfaces/Time acquired_at
uint8 phase
