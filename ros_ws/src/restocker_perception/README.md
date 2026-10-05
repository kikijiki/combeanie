# restocker_perception

This package reads the RGB-D streams and does two jobs with them.

- **Product perception** answers where a catalogued product is. It provides backend-neutral
  detection and 6-DoF pose-estimation interfaces, a colour-and-depth backend that implements them,
  the nodes that run it, and a deterministic fake backend so downstream code can be tested without
  a camera or GPU.
- **Obstacle reconstruction** answers what is in the way that no model describes. It turns the
  overhead depth stream into boxes for the planning scene.

The two jobs share the depth stream and nothing else: they do not link each other and publish on
separate topics.

On the default launch the overhead camera is used for obstacles only
(`obstacle_perception` defaults to on). Product poses come from the wrist camera through the tray
overview and tray confirm duties (`tray_overview_perception` and `tray_confirm_perception`, both on
by default in `baseline.launch.py`). Overhead product perception exists but is off by default
(`perception:=false`) because swapping it in deadlocks the retreat after a release.

## Product perception

### Interfaces

`DetectionPort` turns one RGB-D acquisition into a `PerceptionFrame`: the detections and their
instance masks. A mask is delivered inside its detection, and the frame owns the stamp and
coordinate frame for the whole acquisition.

`PoseEstimationPort` turns a `PerceptionFrame` plus depth into `ObjectObservation` messages, which
already carry frame, stamp, confidence, 6x6 covariance and backend name, and are what the world
state ingests. `estimate` takes the camera-to-planning transform explicitly, so an implementation
cannot emit camera-frame poses labelled as world poses.

The types are built so that a value from one frame or instant cannot be read in another:

- `RgbdFrame` can only be built from colour and depth images that agree on frame and stamp.
- `FramedPose` bundles pose, covariance and frame. Measured poses are never passed as a bare
  `Eigen::Isometry3d`.
- `FramedTransform::apply` converts pose and covariance together and stamps the result with the
  target frame.
- `validate_estimation_inputs` is the first check every `PoseEstimationPort` runs. It rejects
  detections that did not come from the supplied acquisition and transforms from the wrong frame.
- `compare_framed_poses` refuses to compare poses across frames.

### Colour-and-depth backend

`ColourDepthBackend` finds the three catalogued products by chromaticity, then fits each blob's
depth returns to the cylinder its category is catalogued as. It is not a neural network. The scene
is three brightly coloured cylinders of known size on a surveyed tray.

Colour proposes and geometry decides. `detect` is image and depth work only and is permissive (the
robot's own blue links classify as small-bottle blue). `estimate` knows the camera transform and
rejects anything outside the workspace or whose top face is not a horizontal disc of the catalogued
radius at the catalogued height. A rejected detection produces no observation rather than a
low-confidence one. The robot links that pass the colour stage fail the disc fit by a factor of four.

The pose comes from the cylinder's top face, not its silhouette, because from above the top is a
full disc whose centre is the cylinder axis. The axis is an algebraic circle fit to the convex hull
of the top-face band. A plain band mean is pulled 2 to 7 mm toward the camera at the shipped
51-degree tray elevation. The covariance comes from the fit, and yaw carries the variance of a
uniform distribution over a full turn because it cannot be observed on a surface of revolution.

The same backend runs on the wrist camera for the tray duties, configured in
`config/wrist_tray_perception.yaml`. The overhead configuration is
`config/overhead_rgbd_perception.yaml`.

**Measured accuracy** (seeded trays, against simulator ground truth):

- Confirmation-range translation error is 0.19 to 0.36 mm (12 admitted), with world-Y residual
  at most 0.27 mm.
- Overview-range median translation error is 0.69 mm.
- Against the covariance-derived attachment budget, 12 of 12 confirm cells are within budget and
  none fail closed.
- The confirm duty's `translation_sigma_floor_m` is 0.002 m. The fidelity budget is
  3 x 2 + 1 = 7 mm before the per-product jaw ceiling (17, 16 and 5 mm for can, small and large).

**Limits:**

- Touching same-class pairs produce no separated overview observation.
- Close-neighbour and non-upright cases are refused rather than admitted with a wrong pose.
- Cans must keep the catalogue albedo. A white base with label texture is invisible to chromaticity
  detection against `can.reference_rgb`.
- The backend cannot tell two SKUs of one class apart. The signature catalogue is one entry per
  product class (`can.sku`, `can.reference_rgb`, `can.nominal_radius_m`), and the detection message
  carries no SKU. `SIM-CAN-STD` and `SIM-CAN-CITRUS` share a class and shape, so both are reported
  under whichever SKU the parameters list first. The pose is right and the SKU may be wrong.
  Supporting the pair would need a variant-keyed signature namespace and a SKU on the detection
  message.

### Tray duties and topics

The overview duty publishes candidates to `/perception/tray_candidates`. Nothing in the world
state subscribes to it, so an overview hypothesis can never become world evidence. The confirm duty
publishes the replacing observation to `/perception/object_observations`, the topic the world state
ingests, and must be its only producer. Ground truth is published on
`/perception/ground_truth/object_observations` instead, and the world state's single-publisher check
refuses any second producer on the admitted topic. `test/test_wrist_tray_topics.py` checks this
split.

`confirmation_viewpoint()` in `survey_stations.cpp` computes one camera pose per candidate. It scales
the tray stations' 51-degree approach direction into the confirm duty's 0.25 to 0.35 m standoff
band, applies the elevation floor, and checks that the largest catalogued product fits in frame.
Reachability and line of sight are left to the plan that moves the camera there.

### Same-colour columns

Colour segments by class, so two same-class products one pitch apart can merge into one component in
the 51-degree wrist view. When `top_face_split_gap_m` is positive (0.008 m for the wrist duties, 0
for overhead), the component's top band (points within `top_face_band_m` of its top) is grouped on
a world-XY grid of that cell size. If two or more groups have at least `minimum_top_face_points`,
each is fitted as its own disc and judged by the same gates, all or nothing:

- One refused disc refuses the whole component, unless occlusion explains it (below).
- Two admitted centres closer than two catalogued radii also refuse it (counted as `split_overlap`).

A failing disc is set aside, not refused, when all of these hold:

- at least one disc passes;
- every failing disc is farther from the camera than every passing one, since only something nearer
  can hide part of a disc;
- every point of a failing disc is more than 1.5 catalogued radii from every passing disc's centre.

A set-aside disc is not published. This lets a confirm view along a dense column publish its fully
visible front product even when the product behind it is partly hidden. Tops that touch never
separate and stay refused. On a single product at any tilt the split admits nothing the unsplit band
refuses (`test_non_upright_container`).

### Identity

Perception measures where products are. Names come from the inventory the cell is stocked from, read
by `load_object_manifest`: each product's category, source identity and declared `spawn_pose`
translation. `ObservationIdentityAssigner` names detections and never invents an identity.

- **Category stocked once:** any well-formed detection of that category is that product. A second
  blob of the category in one acquisition is dropped as surplus.
- **Category stocked more than once:** the name comes from pose. Each stocked instance has a
  reference position: its declared position until first bound, then the last measured position
  published under its name. A detection takes an instance's name only if the reference is within
  `identity_association_radius_m` (0.035 m), every other instance is at least
  `identity_ambiguity_margin_m` (0.020 m) farther away, and no other detection in the same
  acquisition chose the same instance. Otherwise it is dropped. The published pose is always the
  measured one. A manifest with a repeated category and no finite `spawn_pose` for each product is
  refused at start-up.

References are never retired, so if a product leaves unobserved, a same-class neighbour displaced
near its last reference can be given its name. In simulation the attach predicate catches this,
because it measures the named model against the gripper fingers and any other body is at least
about 0.085 m away. This safety net depends on the simulator's name-to-model mapping and is not a
hardware guarantee.

### Nodes

- `perception_node` owns the two ports, synchronizes colour and depth on exact stamp match, reads
  the camera-to-planning transform at the acquisition stamp, and publishes `ObjectObservation` on
  `/perception/object_observations`. Simulator ground truth owns that topic by default, so running
  this node requires moving ground truth to `/perception/ground_truth/object_observations`.
- `pose_error_evaluator_node` compares observations with ground truth and publishes
  `PoseErrorSample`. It is evaluation output only and nothing on the execution path reads it. An
  optional `candidate_topic` feeds it the overview duty's output. It does not pair stale samples:
  Gazebo freezes a carried product's pose while the attachment joint holds it, so the evaluator
  watches `/simulation_attachment/state` and reports `STATUS_STALE_PAIRING`.

### Refusal diagnostics

Refusals are silent by default. With `log_refusal_diagnostics` true, a duty logs one INFO line per
acquisition that dropped anything. The line gives each proposal's class, pixel box, point counts,
planning-frame centroid and extent, and each disc's centre, radial ratios and verdict. It also lists
same-class components dropped for being under `minimum_component_pixels` and a census of the central
40 x 40 pixels, where a confirm view aims its product. It changes no observation or drop count.

### Fake backend

`FakePerceptionBackend` is a pure function of its script and the acquisition: no clocks, randomness
or I/O. Its objects are scripted in the camera frame so it exercises the same camera-to-planning
conversion a real backend must perform. It is installed, not test-only, because other packages use
it to test their perception consumers.

## Obstacle reconstruction

`depth_obstacle_extraction` recovers obstacles that a depth image shows and the workcell model does
not describe. It unprojects a 32FC1 depth buffer with the camera intrinsics, crops to a configured
volume, discards returns inside padded robot-link spheres, downsamples to voxels, groups the
survivors into 26-connected clusters and bounds each with an axis-aligned box. It is a pure function
of a depth buffer and a sensor pose, so it is tested on synthetic renders.

`depth_obstacle_node` publishes one `restocker_interfaces/msg/ObstacleObservation` per image on
`/perception/obstacle_observations`. The sensor pose and self-filter volumes are resolved from TF at
the image's own stamp, and the node never accumulates across images. Boxes are ordered
deterministically, so an unchanged scene gives an unchanged message. That matters because
`restocker_task_executor`'s planning-scene projector treats any change in the obstacle set as changed
collision content, which invalidates a plan in flight. `robot_static` reports whether the
self-filtered links moved during the probe window ending at the image stamp, so a consumer can
refuse to adopt geometry from a moving robot.

## Tests

```
just test-package restocker_perception
```

This builds the package and its dependencies and runs the C++ and Python tests under `test/`.

## Out of scope

Observation admission policy lives in `restocker_world_state`. Simulator protobufs, model weights and
image encodings do not belong in this package.
