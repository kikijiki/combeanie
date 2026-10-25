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

The same file names the **hold mode**, which decides how a verified grasp is carried. `weld`
creates the `restocker_attachment_joint` DetachableJoint, so a rigid link holds the product and
the fingers never touch it; `physical` creates nothing and holds the product with finger contact
and friction (Milestone 10 spec 11.3). The key is required and restricted to those two spellings
because everything downstream — whether an entity is created, what a receipt means, and which
evidence a run may be cited for — follows from it. Whichever is configured, the process prints a
`hold_mode=...` line at startup and carries the value on every attachment reply and on
`/simulation_attachment/state`, so a log can never be mistaken for evidence about the other mode.
The shipped default is **`weld`**. `physical` is complete (contact predicates, interference close,
per-pad slip watchdog) but passes the end-to-end acceptance run only about 40 % of the time, so it
is opt-in until the release/insert events below are understood.

What the physical hold stands on, each measured rather than assumed:

- **Pad rows are flat boxes, one row high and one row deep, in front of an inset finger body.**
  Rounded rows (spheres, rods, twin ribs) look kinder to a cylinder but are not: Bullet's
  sphere-to-cylinder-side test reports no contact until the overlap is about 2 mm and then throws the
  product out, so a can was touched at its rim only and the fingers closed on nothing; a single
  rounded rib also pushes an off-centre product further off. Flat boxes give a contact on every row
  with the normal along the pad normal (`|n.x| > 0.9995`) and carry 40 N a side. The 13-second
  fixture `test_physical_grasp_runtime` reproduces either geometry.
- **The watchdog's slip is the net excursion of a pad's contact point in the pad plane.** The pad
  normal is excluded (a compliant row compresses 0.36 mm at 40 N), a turn about the product's own
  axis is excluded (an upright cylinder, no torsional friction), and the path length of wobble is
  kept as a receipt, not a bound.
- **The free axis is the jaw axis** (the pads' common normal), not the line through the two contact
  centroids: the centroids sit wherever the load falls among the rows and read 0.07 rad off, which
  made a 0.4 rad pendulum swing look like 0.03 rad of tilt. Slip, swing and tilt are bounded by
  `held_slip_allowance_m`, `held_swing_allowance_m` and `held_tilt_allowance_rad` in
  `attachment_boundary.yaml`; a weld keeps `post_attach_translation_drift_m`.
- **Release disarms the watchdog only when the pads let go** (aperture grown past the contact
  tolerance *and* pad force under a quarter of the hold's), so a loaded pair of pads that merely
  parts by a fraction of a millimetre stays watched.

## Product labels, and two products that share a shape

A scenario product may name a `label_texture`. It is a bare filename resolved against
`materials/textures/` by the launch file, and it becomes the albedo map on the product's lateral
surface: Gazebo unwraps a cylinder with u once around the circumference and v along the axis, so
an image is all a label needs and no mesh is involved. Products without the field keep the flat
`rgba` they have always had. `rgba` still applies underneath the map and multiplies it, which is
why a textured product declares white; a coloured base tints the label and takes its contrast with
it.

### Generated catalogue and the contoured bottle

`tools/label_factory.py` generates the fictional beverage catalogue (Card 110): 16 SKUs, two
look-alike families per shape (can, small bottle, large bottle, contour bottle), every label a
full unwrapped strip with a front panel (brand, flavour, a logo of random shapes) and a back panel
(ingredients, nutrition table, disclaimers, a valid EAN-13 whose `2000000...` prefix is GS1's
restricted-circulation range, so no real product is ever matched). Output is deterministic per
seed: `materials/textures/gen-NNN.png`, `config/generated_product_labels.yaml`, and a marked block
in `restocker_description/config/product_collision_catalog.yaml` (each SKU gets its own
`geometry_key`, as `can.citrus` does). `--check` fails when the checked-in output is stale;
`test_label_factory` runs it. The names and brands are invented.

**Strips are upright on disk, the cylinder half-turn is config.** Every `gen-NNN.png` is readable as
stored. Gazebo unwraps a cylinder primitive with the strip rotated by half a turn (upside down,
back panel first), while the contour mesh's own UVs show it upright, so each manifest row carries
`texture_rotation_deg` (180 for `can`, `small_bottle` and `large_bottle`, 0 for `contour_bottle`).
The launch file installs the texture through `restocker_gazebo/label_textures.py`, the only place
the turn is applied: a rotation of 180 loads a pixel-exact rotated copy cached under the system
temp directory (`restocker_label_cache/`), 0 or a texture the manifest does not list (the
hand-made `can_*_label.png`) is loaded unchanged. The registry's reference-view renderer
(`restocker_perception/registry/render.py`) maps u/v the way the strip is stored and applies no
rotation. Until 2026-10-20 the cylinder strips were stored pre-rotated instead; the rendered
result in Gazebo is the same.

The `contour_bottle` class has a waisted, ribbed visual (`meshes/contour_bottle.obj`, generated by
`tools/generate_bottle_meshes.py`, u around the barrel, v along the axis) while its **collision
stays the plain cylinder**: grasping, friction and the lane geometry are those of a cylinder of
radius 0.036 m, so perception sees a real bottle shape and physics is unchanged. A scenario names
the mesh with `visual_mesh`, a bare filename under `meshes/` (the same rule as `label_texture`).
`config/generated_catalogue_products.yaml` lays all 16 on a tray, which is the scene for looking
at labels, not a restocking scenario.

**Box products (CMB-119).** The catalogue also holds four fictional boxes, `GEN-017`..`GEN-020`,
`product_class: box`, `shape: {type: box, size_xyz_m: [x, y, z]}` (x width, y depth, z height; the
front face looks along -y): a look-alike pair of 200 mL cartons (65 x 40 x 105 mm), a 1 L carton
(95 x 60 x 165 mm) and a slim 330 mL box (55 x 25 x 150 mm) whose back panel carries **no barcode**
(`gtin` is absent from its manifest row and `barcode: false`). A box has no class fallback. Its
texture is a **face atlas**, the unfolded net at 8000 px/m (front, right, back, left in one row,
top and bottom below); the manifest row carries `faces` (pixel rectangle, outward normal, and the
product-frame directions of the image's right and up edges, always right-handed so no face is
mirrored), `box_variant`, `visual_mesh`, `size_xyz_m`, `mass_kg`, `atlas_px`. Atlases are stored
upright, `texture_rotation_deg` is 0. `tools/generate_box_meshes.py` writes one OBJ per variant
(`meshes/box_<variant>.obj`, 24 vertices, hard edges, per-face UVs into the atlas, from the same
layout function the atlas uses), and the product xacro takes `box_size` for a box collision and
box inertia. The scenario validator and the catalogue loaders accept `shape.type: box`; the lane
builder (`lane_columns.BOX_CATALOGUE`) packs a box column by its depth, a box lies on its back
(front face up) with roll -90 degrees. Measured: Gazebo shows every panel upright and unmirrored
(front, back and sides; see the benchmark note in the project notes).

Where a box is not supported yet, it is refused, not treated as a cylinder: the gripper pad table
has a stated-proxy `cardboard` material but the attachment system registers no attachable product
for a box (an attach request is refused as an unknown source id), the executor refuses to grasp or
place one (`ProductClass::Box` is neither graspable nor placeable) and the lane placement profiles
skip it. A box *is* observed (ground truth, lane evidence, object manifest, world state) and enters
the planning scene as a `BOX` primitive.

The visual is named `product_body_visual` on purpose. sdformat merges a `<gazebo reference>`
material extension only into a visual whose generated SDF name contains `<link>_visual`, and under
any other name the extension is dropped with no diagnostic and the product renders untextured. The
same rule governs friction: the collision must be named `product_body_collision` or the `mu1`,
`mu2`, `kp` and `kd` entries next to the material are discarded with the same silence. It was
named `product_collision` from the day the file was written until 2026-09-29, so **the catalogued
friction never reached the physics engine** — every product contact ran on the engine's own
default of 1.0. The collision now carries the merging name and
`test/test_simulation_description.py` asserts the *converted* SDF (via `gz sdf -p`), not the URDF
the extension lives in, because the URDF was always the one that looked right.

Only `mu1`/`mu2` do any work. DART's contact solver combines the two bodies' coefficients by
taking the **smaller** of the two, so a product held by fingers at 1.0 grips at the catalogued
value (0.65 for the can, 0.72 and 0.76 for the bottles), product-to-shelf runs at the smaller of
the shelf's 0.8 and the product's, and product-to-product at the smaller of the two products'.
`kp` and `kd` are emitted into `surface/contact/ode` and then **ignored**: the DART backend does
not read them (its contact ERP/CFM are compiled in), so they are not a stiffness tuning surface
and nothing here should be tuned through them.

Because the combination is a minimum, a coefficient on the finger face alone would do nothing, so
the attachment system sets the **pair** instead: for a contact between a finger pad and a product
it overrides both friction directions with the pad's own figure — 0.8 for the aluminium can, 0.3
for the PET bottles (a stated proxy: silicone on acrylic at the load our pads carry) — through
gz-sim's contact-surface customization, which only the dartsim backend implements. Every other
pair keeps the rule above untouched: product-to-shelf, product-to-tray and product-to-product are
still the minimum of their two catalogued values. The coefficient each product class grips at is
printed once at startup as `AttachmentSystem pad-to-product friction ...`, so a run's log states
which physics it measured.

The fingers that produce that grip are **effort-driven**: `gripper_controller` claims the
`effort` command interface of both finger joints and runs a force-mode position controller whose
output is clamped at `gripper_geometry.yaml`'s `effort_n` (40 N). Free motion moves the jaw; a jaw
blocked by the product pushes with exactly the clamp, which is what a force-limited parallel
gripper does. The arm and the rail keep their velocity interface and their gains unchanged; the
only other thing that moves with it is the controller-manager rate, which is now 1 kHz (the
physics step) because a force command is held until the next controller update and a 10 ms hold
on a 0.18 kg jaw is unstable for any gain that reaches 40 N inside the 1 mm interference. Each pad
is also modelled as four small links on spring-damper joints along the jaw normal in the simulated
description only — MoveIt plans against the plain tiled geometry it has always seen.

Delivering the friction invalidated the evidence that was gathered without it. Every grasp,
attachment, gravity-feed, contact and transfer tolerance measured before 2026-09-29 was measured
at engine-default friction and is **historical**: it may not be cited as evidence for the current
contact behaviour, and Card 024's remeasurement replaces it.

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

## The showcase scene

`config/showcase_products.yaml` is the scene `just demo` opens: the generated catalogue's
straight-cylinder SKUs (labelled, textured) as a stocked shelf and a packed back room. Six lanes
each hold one SKU's column (five SKUs across the three shapes; the large bottle holds two lanes),
pinned at the computed lane poses, some at the lane target and some short. The stock tray holds one
two-column block per SKU, rows 12 mm apart and columns on the gripper's approach-corridor pitch,
with reserve stock for every SKU. Every shelf label faces the shelf front (+Y); each tray product
has its own seeded random yaw (`showcase.tray_yaw_rad`), as stock set down in a back room is. The scene is paired with
`restocker_world_state/config/showcase_lanes.yaml` and the dressed world
`worlds/restocking_showcase.sdf` (same physics and plugins as `restocking.sdf`, a cold-room store
back with a spot key light and a fill, and a Gazebo window that opens on the `wide` view of
`website/scripts/showcase_views.py`).

The launch argument `bundle_scenario:=true` writes the workcell and every scenario product into the
world file (`restocker_gazebo/world_bundle.py`, one `gz sdf -p` conversion per distinct product
description) so they stand there from the first frame instead of being spawned by one `create`
process each; `scene_materials:=<yaml>` restyles the bundled workcell's visuals by name
(`config/showcase_materials.yaml`), leaving the description every test shares untouched. The floor
texture is generated by `tools/generate_showcase_floor.py`.

The scenario is generated, not edited: `restocker_gazebo/showcase.py` holds the arithmetic and the
lists that decide what stands where, `tools/generate_showcase_scenario.py` writes both documents
(`--check` reports a stale pair), and `test/test_showcase.py` pins determinism, non-overlap, the
corridor pitch, the tray bounds and the agreement between scenario, label manifest, collision
catalogue and lane policy. The contoured bottles and boxes are not placed: the grasp and
placement stack handles straight cylinders only.

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
