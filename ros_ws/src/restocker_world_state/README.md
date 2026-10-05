# restocker_world_state

Owns the authoritative deterministic C++ model for tracked objects, shelf lanes, robot execution
state, freshness, uncertainty, snapshots, atomic attachment semantics, and event history.

The exported domain targets are `restocker_world_state::world_state`,
`restocker_world_state::lane_config`, and `restocker_world_state::ros_conversions`. Callers submit
observations already transformed into the configured planning frame and receive typed `Result<T>`
failures for normal validation outcomes. Immutable value snapshots are safe to retain after later
mutations. Shelf-lane snapshots keep observed source identities separate from authoritative
numeric object membership. Perception may refresh evidence, obstruction, depth, and time; only
semantic attachment transactions push into a lane's FIFO `contents`, and only the store's own
reconciliation of an accepted survey may pop from its front.

`config/baseline_lanes.yaml` owns task-level lane compatibility. The lane configuration loader
joins that policy with `restocker_description/config/workcell_geometry.yaml`, which remains the
only source for surveyed lane depth. Startup rejects missing or extra lane IDs, unknown product
classes, empty configured SKUs, missing or malformed `target_count`, and invalid geometry instead
of substituting defaults. Each lane declares three things: `expected_product_class` and
`expected_sku` say what the owner wants there, `target_count` says how many.

Lane intent is editable at runtime through `/world_state/set_lane_policy` (`SetLanePolicy`): lane
id, expected class, optional SKU, target count, and the caller's expected lane revision. The
change is revision-fenced and all-or-nothing: a stale expected revision, an invalid policy, or a
failed write refuses without changing the lane or the policy file. An accepted change bumps the
lane revision (so selections and un-granted reservations built from the old revision fail on the
existing revision-conflict path), invalidates that lane's depletion evidence until the next
accepted lane observation, and appends one `LANE_POLICY_SET` event. An already-granted
reservation keeps the policy it was granted under: the reservation captures the destination
policy at grant, and every later revalidation compares against that capture rather than the
lane's live intent, so changing a lane mid-transfer never strands the in-flight task.

Accepted changes are persisted atomically to the `lane_policy_state` document (same schema as
`baseline_lanes.yaml`: `schema_version` plus the lane map with `target_count`): the full table is
written and fsynced to a temporary file in the same directory, renamed over the target, and the
directory fsynced, all before the in-memory mutation applies. At startup the document, when
present, overlays the shipped baseline lane by lane — identical lane IDs are required — so a
restart resumes the owner's intent instead of the shipped default. The document lives next to the
baseline by default (`…/restocker_world_state/config/lane_policy_state.yaml`, launch argument
`lane_policy_state`); keep it paired with the `lane_semantics` file it was written for, because a
saved intent for one policy will overlay another. When `lane_policy_state` is empty the service
refuses policy changes rather than making ones that would not survive the next restart.

The shelf carries six lanes and the baseline assigns two of them to each product class, so a
product always has a compatible alternative when one of its lanes is occupied or obstructed.
Task selection scores every eligible pair and orders them by deficit against `target_count`, largest first, with ties broken by rail distance
from the carriage. It then keeps the best under the front-row and travel-cost rules. This is a real
decision only because a class has more than one destination.

`contents` is that lane's semantic FIFO ledger of what the robot placed: push_back on placement,
front() is the customer-facing entry. Every accepted lane survey reconciles the measured column
length against the settled contents at the catalogued pitch (half-pitch tolerance): a shortfall of
whole pitches pops that many front entries as sales (the sold objects are marked Removed), while
growth beyond what the contents plus an in-flight Attached placement can explain latches
`ledger_unreliable`, appends one `LANE_LEDGER_UNRELIABLE` event, and withdraws contents from the
deficit calculation — measured geometry, the occupied-volume projection, and geometry-safe
placement continue unchanged. An in-flight placement counts only on the growth side, so the
pre-insert surveys the continuous evidence topic publishes while the arm is still carrying the
product neither invent a sale nor withdraw trust. A ledger whose entries the current lane policy
does not accept is left untouched by length reconciliation (the new intent's pitch does not
describe the old column); selection reports that conflict as `lane_wrong_product` instead.

`config/same_shape_lanes.yaml` is the same policy with one field changed: `lane_04` expects
`SIM-CAN-CITRUS` instead of a second `SIM-CAN-STD`. It goes with
`restocker_gazebo/config/same_shape_products.yaml`, which stocks two cans of identical dimensions
that differ only in the label. Two lanes of one class then disagree about SKU, so recognising the
label is the only way to route either can. An object whose SKU no lane accepts is skipped by task
selection and the run ends in `NoEligiblePair`, a clean outcome rather than a fault. A run that
reads the labels wrong therefore delivers nothing and reports no error.

The package also owns a thin ROS process boundary. `world_state_node` validates normalized object
and lane observations, initializes the configured lane policy, owns one long-lived store, and
exposes typed snapshot reads through `/world_state/get_snapshot`. Lane observations refresh only
evidence, available depth, obstruction, and verification time.

Two confidence floors sit on that boundary, `minimum_confidence` for objects and
`minimum_lane_confidence` for lanes, and both are trust gates, not provenance gates. A
below-floor message is rejected outright by `object_observation_from_message` /
`lane_observation_from_message` (a typed `InvalidArgument` failure, logged and dropped, never
clamped or admitted with a marker). The comparison is `>=`, so a producer emitting exactly the
floor is admitted. Neither floor authorises anyone: `confidence` is self-reported, and any producer
that would be refused at 0.7 can publish 1.0 instead. Provenance is a separate gate:
`on_observation`, `on_lane_observation`, and `on_robot_telemetry` each require exactly one
publisher, latch the publisher GID plus a stable source field (`backend_name` for object and lane
observations, `source_id` for telemetry), and reject identity changes after latch. That matters
because lane evidence reaches reservation admission, the placement capacity gate, and the
placement-acceptance proof in `commit_reserved_detachment`, and object observations seed the
tracked objects those paths depend on.

`minimum_confidence` defaults to 0.60, derived from the measured detection posteriors on this
cell. `minimum_lane_confidence` defaults to **0.90**, the lane survey's coverage floor: valid
wrist-camera readings score 1.000 and a blinded or mis-aimed acquisition scores 0.000 (0.463 for
half a blinded frame on an empty lane). The object gate's 0.60 should not be reused here, because
object confidence is a detection posterior and lane confidence is a coverage fraction.
`lane_observation_topic` defaults to `/perception/lane_observations`, which the wrist-camera survey
publishes. Setting it to `/perception/ground_truth/lane_observations` switches to continuous
simulator ground truth. Both comparisons are made at the wire type's float precision, so a producer emitting exactly the
configured floor is admitted at every floor rather than at the ones that happen to round upward.

Consumers use `snapshot_from_message` to cross the ROS boundary back into the domain model. The
decoder fails closed on frame or timestamp errors, invalid closed enums, non-positive-semidefinite
covariance, duplicate identities or lane membership, inconsistent held-object state, and malformed
reservation stages. Task code receives a coherent immutable snapshot and does not revalidate wire fields in
callbacks. The same strict reservation decoder is public for
reservation-service consumers, preventing snapshot and service-response validation from drifting.

One opaque, non-expiring task reservation may be active. Its token is returned only to the caller;
snapshots and events expose a token-free diagnostic summary. Reservation mutations use a bounded,
non-evicting process-lifetime idempotency journal configured by `operation_journal_capacity`.
Exact replays return the original revision, while operation-ID reuse with a different semantic
payload fails closed. The ROS boundary exposes:

- `/world_state/reserve_task`
- `/world_state/validate_reservation`
- `/world_state/checkpoint_task`
- `/world_state/commit_attachment`
- `/world_state/commit_detachment`
- `/world_state/release_reservation`
- `/world_state/invalidate_lane_evidence`
- `/world_state/set_lane_policy`

A placement commit is the one mutation whose evidence has to be dated as well as read. The
ground-truth backend reports every product pose, held or not, so the destination lane can name the
product several seconds before the gripper lets go, because the pre-insert carries it into the
lane volume. Lane occupancy alone therefore proves presence, not placement. `commit_detachment`
closes that gap by carrying the instant the physical boundary verified the release, and accepting
a placement only when destination evidence is inside its validity horizon, not invalidated,
unobstructed, naming nothing the reservation did not already account for, recorded strictly after
that release, and — with the default `placement_require_column_growth` — showing the free rear
depth shrank against the grant-time baseline by at least one catalogued product pitch less
`placement_growth_tolerance_m`. That growth proof is geometry alone, so a wrist-camera lane
producer that measures depth and never names identities in a lane can prove a placement. With the
flag false the landing proof is instead that the post-release evidence names the target (the
ground-truth simulator opt-out the dense demo keeps). The caller re-asks the commit while the
evidence catches up.

Legacy attachment, detachment, reserved-object lifecycle, and robot-semantic mutations are fenced
while a reservation is active. Object and lane evidence may continue advancing; stage predicates
are revalidated from current semantics. One admission rule narrows that advancing: while the
reservation is at its pre-attach `Reserved` stage, an admitted observation for the **reserved**
object still advances its freshness stamp, tracking state, identity fill-in, revision, and an
event, but `pose_in_world` is replaced only when the observation lies farther than
`reserved_pose_divergence_bound_m` (default 0.020 m) from the held pose. The bound compares
**translation only**: a rotational divergence is caught downstream by the attachment boundary's
rotation residual. Inside the bound, confirm-frame noise cannot move the obstacle a staged grasp aims at — staging, validation, and
the planning-scene projection all read the same snapshot pose. Beyond the bound the observation's
pose is applied and becomes the new held pose, so a real movement is surfaced rather than hidden
and the existing scene-content fence, collision checks, and the attachment boundary's physical
hold-target check fail the goal closed. The hold ends when the reservation attaches the product
and when the reservation is released under any outcome. The package contains no Gazebo adapter,
MoveIt projection, or task state machine; those components consume this authority without
replacing it.
