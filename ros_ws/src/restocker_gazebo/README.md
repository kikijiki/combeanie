# restocker_gazebo

Owns Gazebo worlds, parameterized product models, spawn utilities, deterministic scenarios, and
the simulator ground-truth adapter. The adapter is the only package component that consumes
Gazebo `Pose_V`; it converts configured top-level model names into backend-neutral
`ObjectObservation` and `LaneObservation` messages with simulation time and provenance.

The conversion library is deliberately internal so Gazebo protobuf and YAML implementation types
cannot become dependencies of application packages. Shelf geometry itself remains reusable in
`restocker_description`.

Lane evidence is computed by a separate pure geometry library from the same Gazebo sample. It
loads the description-owned workcell volumes and product collision catalog, transforms each exact
cylindrical envelope into every lane frame, and distinguishes full containment from partial
overlap. Full containment reports sorted occupancy evidence and remaining insertion depth; partial
overlap marks the lane obstructed. The library does not mutate inventory or depend on ROS node
execution, so transform, horizontal-container, and boundary cases remain unit-testable.

The simulator attachment boundary is split into pure and simulator-specific layers. The journal,
protobuf codec, transport mailbox, physical precondition validator, and configuration projection
are deterministic C++ libraries with unit tests. A separate post-physics verifier requires
consecutive later-tick evidence, bounds mutation deadlines, and applies the same transform-drift
rules to the continuous held-object watchdog. `config/attachment_boundary.yaml` owns only names,
timeouts, capacities, and tolerances; jaw dimensions and clearance policy remain in
`restocker_description/config/gripper_geometry.yaml`, while product envelopes and the exact model
allowlist come from the description catalog and selected scenario. The loader rejects disagreement
before transport services are advertised. Gazebo update callbacks will be the only layer allowed to
resolve entities or create and remove detachable-joint components.

## Product labels, and two products that share a shape

A scenario product may name a `label_texture`. It is a bare filename resolved against
`materials/textures/` by the launch file, and it becomes the albedo map on the product's lateral
surface: Gazebo unwraps a cylinder with u once around the circumference and v along the axis, so
an image is all a label needs and no mesh is involved. Products without the field keep the flat
`rgba` they have always had. `rgba` still applies underneath the map and multiplies it, which is
why a textured product declares white; a coloured base tints the label and takes its contrast with
it.

The visual is named `product_body_visual` on purpose. sdformat merges a `<gazebo reference>`
material extension only into a visual whose generated SDF name contains `<link>_visual`, and under
any other name the extension is dropped with no diagnostic and the product renders untextured.
The same rule is why the `mu1`, `mu2`, `kp` and `kd` entries next to it have never reached Gazebo:
the collision is named `product_collision`, the parser looks for `product_body_collision`, and the
friction has been silently discarded since the file was written. It is left that way deliberately (every grasp, tolerance and benchmark here was measured against the contact behaviour Gazebo
actually used), and `urdf/product.urdf.xacro` carries the note and the one-word fix.

That is a friction defect and nothing else. Products also had no product-to-product *contact*, and
the two are unrelated: a cylinder pair carrying the whole `<surface>` block this rename would
produce still passed clean through its neighbour. The contact defect was that dartsim's default
narrowphase is ODE's, which has no cylinder/cylinder collider; `worlds/restocking.sdf` now names
DART's Bullet collision detector and `test/test_product_contact_runtime.py` measures the
consequence.

`config/same_shape_products.yaml` is what labels exist for. It stocks two cans of identical
dimensions, `SIM-CAN-STD` and `SIM-CAN-CITRUS`, whose labels are the same picture in the same
colours with a different word on them. Every other scenario stocks one product per class and every
other catalogue entry has one SKU per shape, so a product's identity has always been recoverable
from its diameter and height alone; here it is recoverable only from the word. Pair it with
`restocker_world_state/config/same_shape_lanes.yaml`, whose only difference from the baseline
policy is that `lane_04` expects the citrus can, and telling the two apart becomes an observable
outcome: read them right and both lanes fill, read them wrong and selection reports no eligible
pair.

```bash
just launch-sim \
  scenario_config:=$(ros2 pkg prefix --share restocker_gazebo)/config/same_shape_products.yaml \
  lane_semantics:=$(ros2 pkg prefix --share restocker_world_state)/config/same_shape_lanes.yaml \
  lane_policy_state:=/tmp/same_shape_lane_policy.yaml
```

Pair `lane_policy_state` with the `lane_semantics` it belongs to: the state document overlays the
lane policy at startup, so reusing the baseline's saved document here would silently put `lane_04`
back on the standard can and make the recognition experiment unmeasurable. Any path you can
write works; an absent file just starts from the named `lane_semantics`.

The label images are generated by `tools/generate_product_labels.py`, which needs nothing but the
standard library and rewrites them in place. They are original flat-colour artwork with a
hand-coded block font, so they carry no third-party licence and depict no real branding.

## Repeatable randomized scenarios

`config/baseline_products.yaml` carries both the surveyed fixed scenario and, under
`randomization:`, the bounds a seeded draw is allowed to use. The `scenario_seed` launch argument
selects between them: its default `fixed` spawns `products:` verbatim, so randomization is opt-in
and every recorded baseline keeps the poses it was measured against. Any unsigned 64-bit integer
instead makes `restocker_gazebo.scenario_random` replace `products:` with a draw and repoint the
launch: the spawns, the ground-truth adapter, and the in-simulator attachment plugin all read the
generated file, which is written to `$RESTOCKER_GENERATED_SCENARIO_DIR` (a per-user temporary
directory by default) and named for its seed.

```
ros2 launch restocker_gazebo simulation.launch.py scenario_seed:=20260908
```

The seed is the whole replay instruction, so the generated document is a pure function of it:

- The generator is SplitMix64 (Steele, Lea and Flood 2014) implemented over Python's
  arbitrary-precision integers masked to 64 bits. The published constants pin the sequence to this
  file rather than to a toolchain, which `std::default_random_engine`, `std::random_device`, and
  the seed-to-output mapping of Python's own `random` module all fail to do.
- Uniform reals come from the top 53 bits scaled by a power of two, so the conversion is exact on
  any IEEE-754 machine; integer draws use rejection so no variant is over-weighted.
- Nothing reads the clock, the environment, `os.urandom`, `hash()`, or an unordered container,
  and every emitted length is quantized to a micrometre before it is validated and written, so
  two runs of the pinned serializer produce byte-identical files rather than merely equal ones.
- `generated_from.bounds_digest_sha256` fingerprints the bounds, the catalog shapes, the surveyed
  region, the jaw approach clearance, and `generator_version`. A seed only replays against the
  bounds it was drawn from, and the digest makes a changed configuration visible instead of
  mysterious. Adding, removing or reordering a draw changes what every seed means without changing
  the bounds, and so does widening the test that rejects a candidate, so either must be announced
  by bumping `generator_version`, which is why the digest covers it.

A seeded run varies which products exist and where they stand, and not where they end up. The
coordinator chooses a destination at runtime from the world state and never opens a scenario file,
so a variant's `destination_lanes` is a constraint on the bounds (it must be exactly the lanes
`restocker_world_state/config/baseline_lanes.yaml` gives the same class and SKU) rather than an
instruction to the run. Generator version 2 removed a per-product `destination_lane` draw that
nothing read; version 3 widened the pairwise separation rule below. Seeds recorded against
version 1 or 2 do not replay under version 3.

Placement is validated, not merely sampled. A product must stand on the stock tray surface (the
floor of `stock_tray.usable_volume` in `restocker_description/config/workcell_geometry.yaml`), fit
inside that envelope with its catalog radius and height plus the configured wall margin, and keep
its distance from every other product. That distance answers to two bounds, and which one binds
depends on the pair. The configured `surface_separation_m` keeps the cylinders from touching, so
physics does not push them off the pose the seed chose. Separately, grasping a product means
fitting the opened jaws around it, and the finger outer faces stand 0.071 m from the tool axis at
`open_target_m` (derived from `restocker_description/config/gripper_geometry.yaml`, not restated)
plus the 1.5 mm MoveIt inflates every robot link by before it collision-checks, for a corridor
of 0.0725 m. The padded figure is the one that governs, because the padded finger is the one the
planner checks; the generator carries it as `scenario_config.PLANNER_ROBOT_PADDING_M` and a test
fails if it drifts from the value `move_group.launch.py` sets. A neighbour must therefore be
further away than 0.0725 m plus its own radius or the approach collides from every arm
configuration the planner could start from. The jaw bound is per-pair and asymmetric in the
neighbour's radius, which is why it is not folded into the scalar gap, and it is the bound that
usually governs: two cans need 0.1055 m between centres where the surface gap asks only for
0.086 m. It is also the one failure a run cannot recover from, so a scenario that broke it read as
a planner bug rather than as a scenario defect. Height and orientation are not drawn: a
product spawned above the surface falls to a pose the seed never chose, and every catalog shape is
a cylinder, so yaw would change nothing physical. Draws that violate an invariant are rejected and
redrawn, and exhausting the attempt budget fails the launch rather than emitting a scenario that
cannot be seated.
