# restocker_interfaces

Owns custom ROS messages, services, and actions. This leaf package never depends on application
packages.

`ObjectObservation` is the backend-neutral perception boundary. Its header is the acquisition
timestamp and source frame; its provenance, confidence, uncertainty, identity, and status fields
must be validated before authoritative state changes. Category and exact SKU remain separate.

`PerceptionFrame` is the backend-neutral detection and segmentation boundary. Its header is the
single authority for when and in which frame every `ObjectDetection` and `ObjectSegmentation` it
contains was measured; those messages carry no header of their own, so a detection cannot be read
at an instant or in a frame other than its siblings'.
Detection product classes mirror `ObjectObservation` and a contract test holds the two sets equal.
There is no separate pose-estimation message: 6-DoF estimates are published as `ObjectObservation`,
which already carries frame, stamp, confidence, covariance, and backend provenance.

`PoseErrorSample` is evaluation output only. It records measured pose error against simulator ground
truth, both stamps it compared, and the frame it compared them in. Nothing on the execution path may
consume it, and ground truth must never reach authoritative world state through it.

The rotational error is `axis_error_rad`, paired with `orientation_status`, rather than a full
rotation error. Every catalogued product is a cylinder, so its yaw is unobservable and scoring it
would measure the scenario file rather than the pipeline. An estimator that publishes a constant
orientation is reported as `ORIENTATION_NOT_ESTIMATED` with NaN, not 0.0, which an orientation-blind
pipeline would otherwise share with a perfect one. The producing backend declares which case
applies through the rotational block of the covariance on its `ObjectObservation`.

`LaneObservation` is the corresponding backend-neutral evidence boundary for a configured shelf
lane. It reports measured free depth, obstruction evidence, and sorted source identities. It does
not itself rewrite `ShelfLane.contents`: when the world state accepts one it reconciles the
measured column against the FIFO ledger internally — popping front entries as sales or marking
`ledger_unreliable` — and only validated semantic world-state transactions may push new
membership.

 `ShelfLane` carries the owner's desired stocking alongside the observed evidence:
 `expected_product_class` and `expected_sku` say what belongs in the lane, `target_count` says how
 many, and the evidence fields say what the sensor last saw. `contents` is the FIFO ledger of what
 the robot placed (back() rearmost, front() customer-facing); `ledger_unreliable` is the latched
 withdrawal of identity trust when measured geometry and the settled contents length diverge
 beyond half a product pitch (an in-flight placement counts toward growth only) — geometry stays
 authoritative and the withdrawal is surfaced as a
 `LANE_LEDGER_UNRELIABLE` event. Intent changes arrive through
 `SetLanePolicy`, which takes the caller's expected lane revision; a stale revision is refused
 without changing the lane or the persisted policy document. `TaskReservation` records the
 destination policy it was granted under (`destination_expected_product_class` and
`destination_expected_sku`), because lane intent may change while a transfer is in flight and a
granted reservation must complete under the policy it was granted rather than the lane's live
one.

Simulator protobufs, model-specific tensors, and application behavior do not belong here.

`GetWorldState` returns the complete semantic snapshot shape: tracked objects, shelf lanes, robot
execution state, and optionally bounded event history. Empty domains remain present and typed so
downstream code needs no interim object-only API.

`PlanningSceneProjectionStatus` is the synchronization gate for planning consumers. Consumers
require `STATE_APPLIED` and an `applied_revision` at least as new as the world-state snapshot they
intend to use; log messages are not part of this contract. Its transaction states
inhibit planning while the projector drains, cedes managed geometry to one lease holder, or
reconciles after release. `projector_epoch` fences process restarts, while
`scene_content_generation` changes only after verified collision-content mutation or lease release.

`ERROR_OBSTACLE_EVIDENCE_STALE` is the one projector error that does not describe a failure to
apply geometry: depth-derived obstacle evidence has aged out and the obstacle geometry already
applied is retained. It does not mean the scene is empty; it means the sensor can no longer see.

`ObstacleObservation` carries the unmodeled obstacles one depth image shows, as axis-aligned
`ObstacleBox` entries in the frame its header names. The whole message describes a single instant:
`header.stamp` is the depth image's own stamp and every box was placed using the sensor pose
resolved at exactly that stamp. `sensor_frame` records the optical frame it came from, `sequence`
distinguishes a restarted publisher from a stalled one, and `robot_static` says whether the links
used for self-filtering moved across the probe window ending at that stamp, so a consumer may
refuse to adopt new geometry derived from a moving robot. Boxes are ordered deterministically, so
an unchanged scene yields an unchanged message.

`AcquirePlanningSceneLease`, `ValidatePlanningSceneLease`, and `ReleasePlanningSceneLease` expose
the exclusive managed-geometry boundary. `PlanningSceneLease` is a token-free diagnostic summary;
the opaque capability appears only in an acquisition response and holder-supplied requests.

The world-state reservation API uses six typed services and a shared numeric status message.
Mutation requests carry an opaque capability token and caller-generated idempotency ID. Public
snapshots expose reservation diagnostics but omit the token.

`SetSimulationAttachment` is the pollable, capability-authorized request boundary for simulated
physical coupling. `GetSimulationAttachmentState` reads current or journaled evidence. Their shared
state records exact object/link identity, simulator epoch, applied transforms, relative motion, and
typed outcome without exposing reservation or planning-scene lease tokens.

`RestockProduct` is the public task action. Goals use explicit presence flags for optional object
and lane selectors; unset selectors request deterministic selection. Feedback publishes the
stable task-machine state, attempts, selected identities, simulation elapsed time, and latest
typed status. Results distinguish task, verification, recovery, and external-consistency failures
and include the relevant world revisions and baseline metrics.

`SurveyTray` is the coordinated read-only tray survey action: overview stations, then one
confirmation viewpoint for one selected candidate. Overview candidates in its result are
hypotheses read from `/perception/tray_candidates`; they carry no world-state identity and never
authorize a grasp. Only `OUTCOME_CONFIRMED` carries the replacing observation the confirm duty
published on `/perception/object_observations`, and `OUTCOME_REFUTED` with a typed `refutation`
is the explicit refusal path. A goal may carry `refuted_positions`: candidates a previous
confirmation refused this cycle, within the survey's merge radius, are never re-selected, so a
refutation leads to a reselection rather than the same doomed proposal, and an exhausted set
reports `OUTCOME_NO_CANDIDATE`. Cancellation abandons unvisited stations, keeps the candidates
already collected, and returns no motion goal outstanding. A goal may also carry
`absence_probe_positions` (places where the caller believes a tray product stands) and
`absence_occluder_positions` (other tracked tray bodies); when every station was visited the
result answers one `absence_probe_verdicts` entry per probe (`PROBE_SEEN`, `PROBE_VIEWED_EMPTY`,
`PROBE_OCCLUDED`, `PROBE_NOT_COVERED`, `PROBE_NO_EVIDENCE`), and an incomplete overview answers
none. Only `PROBE_VIEWED_EMPTY` is evidence of absence.

`AutonomousRestockCampaignStatus` is the autonomous mode loop's status stream. Alongside the
progress `phase` it reports the `mode` the publisher stood in -- `IDLE`, `SURVEY_SHELF`,
`SURVEY_TRAY`, `CONFIRM`, or `TRANSFER` -- and the run's cumulative `survey_arm_time_sec` and
`transfer_arm_time_sec`, so a run can say how much arm time went to looking against how much went
to moving product. Stock, deficit, and transfer counters come from the same section 4 deficit the
coordinator's selection orders pairs with.
