uint8 OBJECT_OBSERVED=0
uint8 OBJECT_TRACKING_CHANGED=1
uint8 OBJECT_REMOVED=2
uint8 LANE_CONFIGURED=3
uint8 LANE_VERIFIED=4
uint8 ROBOT_SEMANTIC_STATE_CHANGED=5
uint8 OBJECT_ATTACHED=6
uint8 OBJECT_DETACHED=7
uint8 TASK_RESERVED=8
uint8 TASK_CHECKPOINTED=9
uint8 TASK_RESERVATION_RELEASED=10
uint8 LANE_POLICY_SET=11
# Identity-trust withdrawal: measured lane geometry and the contents ledger diverged beyond
# half a product pitch. Appended, never renumbered; geometry-safe operation continues.
uint8 LANE_LEDGER_UNRELIABLE=12

uint64 revision
builtin_interfaces/Time event_time
uint8 kind
bool has_object_id
uint64 object_id
bool has_lane_id
string lane_id
string detail
