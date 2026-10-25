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

## Product registry

`restocker_perception.registry` is a pure-Python registry of the products the identity channel
(CMB-112) matches against. It has no ROS runtime and no machine-learning dependency: embedding
backends are loaded lazily and are deliberately not in the Nix flake.

### Record and format

One directory per registry, one sub-directory per product:

```text
<registry>/registry.json                  schema, sorted product ids, content hash, notes
<registry>/products/<id>/product.json     the record (product.yaml is also read)
<registry>/products/<id>/views/az000.png ... az315.png
<registry>/.cache/                        derived data, never part of the registry
```

A record holds the **product id** (the SKU id and the key of the record, its directory and every
derived-cache entry), an optional **GTIN** (EAN-13, check digit verified; the generated catalogue
prints the same number in the label barcode), a display name, a product class, a geometry reference
(`geometry_key` plus `radius_m` and `height_m`, matching `product_collision_catalog.yaml`), eight
exemplar views at 45 degree steps around the barrel, the printed OCR strings, optional aliases (the
simulator's `GEN-001`) and a source note. The view azimuth is nominal: it orders the views and says
which one matched, it is not a pose. The registry refuses a duplicate id, GTIN or alias, a bad
check digit, anything but eight views, and a directory whose view hashes or content hash no longer match.
`Registry.content_hash()` covers every record field and every view hash and nothing else.

Embeddings and OCR reads are **derived** and cached under
`.cache/<kind>/<backend name>/<backend version>/<product id>`. An entry is read only if it was made by that
backend name and version and from the same views as now, so replacing the encoder or editing a
product re-derives exactly what changed.

### Product ids and products without a barcode

Loose produce, bakery and refilled bottles have no GTIN, so the GTIN is an attribute, not the key
(CMB-118). The id rules:

- A product **with a GTIN** has `product_id == gtin`. Nothing changes for it: same directory, same
  cache path, same SKU in `/identify_product`, and `product.json` does not even carry a
  `product_id` field, so its content hash is unchanged.
- A product **without a GTIN** gets `prod-<12 hex>`: the first 12 hex digits of the SHA-256 of its
  display name and view hashes, computed once at registration and then stored in `product.json`
  (write-once: re-rendering a view later does not rename the product, and a rebuilt registry of
  the same inputs gets the same ids). `--product-id` / `register(product_id=...)` assigns one by
  hand (letters, digits, `_`, `-`, at most 64; an all-digit id must be the GTIN).
- `gtin` is `null` when absent, never `""`: an empty or malformed GTIN is refused. GTINs are unique.
  `Registry.gtins` lists only the products that have one, `Registry.product_ids` all of them,
  `Registry.by_gtin` and `gtin_map()` look up, and `resolve()` takes an id, a GTIN or an alias.

**Migration.** Nothing to run. A schema 1 registry (manifest key `gtins`, GTIN-keyed) loads as it
is: each record's id is its GTIN, the content hash is the same one (so calibration files stay
valid), and the derived caches are reused. The manifest is rewritten as schema 2 (`product_ids`)
the next time the registry is changed; schema 2 registries are not readable by older code.

### Registering products

```
product_registry add-images --registry R --images DIR --gtin 2000000000015 --name "Real Cola" \
    --product-class can --geometry-key can.real --radius 0.033 --height 0.122 --ocr COLA
product_registry add-images --registry R --images DIR --name "Loose pear" \
    --product-class pear --geometry-key pear.loose --radius 0.04 --height 0.09 --ocr PEAR  # no --gtin
product_registry add-sim --registry R --use-mesh --exclude-held-out \
    ros_ws/src/restocker_benchmarks/config/perception_splits.json
product_registry verify --registry R --splits .../perception_splits.json
product_registry embed|query --registry R --backend fake|siglip2 [IMAGE...]
```

`add-images` takes eight photographs (`az000.png` .. `az315.png`, or eight files in turntable order).
**Boxes.** `add-sim` registers the box SKUs too (CMB-119): `render_box_views` ray-casts the
oriented box and samples every face from its own atlas rectangle (the manifest's `faces` table),
eight yaws at 45 degrees (0 faces the front squarely, 90 the left, 180 the back, 270 the right;
the odd multiples show two faces) at a 20 degree look-down, the same eight-view contract as a
cylinder. The record gains `size_xyz_m` (absent for bodies of revolution, so no older record
changes its hash) and its `radius_m` is the footprint's bounding radius. `render_box_view` takes
any yaw and elevation. A box without a barcode registers with a derived `prod-` id like any other
GTIN-less product.

`add-sim` renders the eight views offline from each product's unwrapped texture (CPU, numpy only, a
CMB-101 style cylinder ray caster; byte-identical on every run, so the registry is reproducible from
the texture), and with `--use-mesh` follows the contoured bottle's radius profile. Pillow is used to
read non-PNG input when installed and is otherwise not needed. **Benchmark registries must be built
with `--exclude-held-out`**: held-out products are left out and the split file's hash is recorded in
the registry notes, and `verify --splits` fails if one is present.

### Retrieval

`RegistryIndex.from_registry(registry, backend, cache)` keeps all eight exemplars per SKU.
`match` / `identify` give the ranked cosine scores (per-SKU score is the best exemplar by default,
`mean` and `top2` are available), the top-1/top-2 `margin`, the winner of every query view, the
`vote_agreement`, and the provenance of the numbers (backend name and version, registry hash,
whitening). Several query views are voted by averaging scores. There is **no abstention policy**
here: accepting or rejecting a match is CMB-112's job, which calibrates it from these outputs.
CMB-101 recommends SigLIP 2 base with PCA whitening (`Whitener`) and 3-5 view voting, with OCR and
barcode as the acceptance gate. `SigLip2Backend` is that adapter (needs `torch` and `transformers`,
pin `revision`); `FakeEmbeddingBackend` is the deterministic stand-in the tests use.

## Identity channel

`restocker_perception.identity` names the product in one object's crops. The core is pure Python
(numpy; torch, transformers, RapidOCR and zxing-cpp are loaded lazily and are not in the flake) and
imports no ROS; only `identity/node.py` does.

```text
crops of one object -> embedding (registry backend, whitened) -> prototype match, view voting
                    -> OCR lines  -> match against the registry's OCR strings
barcode images      -> EAN-13 decode, check digit, exact GTIN lookup
                    -> fusion with explicit abstention -> IdentityResult
```

`IdentityResult` holds `sku` (the registry product id, which is the GTIN for a product that has
one, or `None`), `status` (`identified`, `abstain`, `unknown`), the evidence list (kind
`embedding`, `ocr` or `barcode`, the product it points at, scores), `channels` (the channels that
ran, whether or not they named anything) and provenance (backend name and version, registry hash,
whitening fingerprint, OCR and barcode backends, `barcode_enabled`).

**Products without a barcode.** The barcode channel can only name a product that has a GTIN
(`IdentityEngine.gtin_to_product`, from `Registry.gtin_map()`); a barcode-less product is named by
the embedding and OCR alone, under the same rules and thresholds. The channel does not run when
the registry holds no GTIN at all. A valid GTIN that no product carries still gives `unknown`.
`barcode_enabled=False` (engine, `IdentifyService.load`, node parameter) leaves the barcode
channel out of every decision at query time, whatever images are passed: the barcode-off arm of a
benchmark, and the behaviour a registry of barcode-less products has anyway. It defaults to true.

**How a product is named.** Rules, in order: a barcode that decodes to a valid GTIN that is not
registered gives `unknown`; a registered GTIN names it; an OCR answer names its product; in both
cases the embedding must rank that product no lower than `supported_max_rank` (default 2), and
contradicting channels abstain. Otherwise the embedding alone names its top-1 only if all three
`alone_*` thresholds (top-1 score, top-1/top-2 margin, multi-view vote agreement) and that
product's own floor are met; else the result is `abstain`, or `unknown` when the best match is
below `unknown_below_score`. Embedding margin alone does not reject an unknown product (CMB-101),
which is why the thresholds are fitted with unknown products in the data and why each product has
a floor: a product that ever won wrongly on calibration data (a hub, or the registered twin of an
unregistered look-alike) must score above its worst wrong win plus a margin, and the calibration
reports every product's share of naming decisions and flags a hub. The OCR channel is deliberately
strict: a product is named only if every word that no other registered product has was read (fuzzy,
difflib ratio 0.8) and at least two words were read, so a look-alike that shares brand and kind
with a registered product is not named from those. It tries the configured rotations
(`ocr_rotations`, default 0 and 45 degrees) on at most `ocr_max_crops` crops; the first rotation
that names a product wins.

**Options are fixed, thresholds are calibrated.** SigLIP 2 base, PCA whitening (64 components,
fit on unlabeled registry exemplars plus calibration crops), up to five voted views and `max`
scoring are fixed in `IdentityConfig` beforehand (CMB-101's recommendation); calibration fits only
numbers. `identity_benchmark calibrate` fits them on the benchmark's train and validation seeds
only, with unknown products made by leaving products out of the index in three folds that put
look-alike twins in different folds, and reports the in-sample metrics (which per-product floors
make look perfect) next to the out-of-fold ones (5 folds by scene seed), which are the honest
ones. It writes a small JSON file with its own SHA-256 (the thresholds, what they were fitted on,
the registry, splits, whitening and bounds hashes) and refuses any other seed split. The
acceptance bounds are declared in `config/identity_acceptance_bounds_r2.json` before any fresh
held-out data is read, with the sample sizes a bound needs; `identity_benchmark evaluate` checks
every hash against the calibration file, reads only held-out seeds (the decision is made on the
fresh ones; the burned ones are reported separately), applies the pass rule (the upper Wilson limit
under the bound and the sample size met) and refuses to overwrite an earlier result. No model is
trained or fine-tuned.

**Barcode.** `restocker_benchmarks.perception_barcode_views` makes aimed close-up viewpoints that
orbit to the azimuth where a cylinder product's barcode faces out (0.22 m; closer is rejected by
the planner); `tools/record_perception_dataset.py --barcode-reach-deg N` requests them. The reader
tries the unrolled barrel (`unwrap_cylinder`) and the raw crop, accepts a decode only with a valid
GTIN-13 check digit and looks it up exactly.

```
identity_benchmark calibrate --registry R --dataset D... --out cal.json --whitener-out w.npz
identity_benchmark evaluate  --registry R --dataset H... --calibration cal.json --whitener w.npz --out result.json
identity_benchmark diagnose  # python -m restocker_perception.identity.diagnose: per-factor top-1 on calibration data
```

**Service.** `identify_product_node` serves `/identify_product` (`restocker_interfaces/srv/IdentifyProduct`):
RGB8 or BGR8 crops (and optional barcode images) of one object in, status, SKU and evidence out.
Parameters: `registry_path`, `calibration_path`, `whitener_path`, `backend` (`siglip2` or `fake`),
`backend_revision`, `device`, `ocr`, `barcode` (`none`, the default, or `zxing`: the decoder for
the barcode images; `none` ignores them) and `barcode_enabled` (default true; false disables the
barcode channel). If any of the three paths is empty, a file is missing or
edited, the registry or backend differs from the calibration, or the model fails, the node stays up
and answers `unknown` with `sidecar_available: false` and the reason. The service only reports;
SKU write-once semantics stay in world state, and the world-state identity path does not call this
service yet.

The reply is additive over what it was: `sku` is the registry product id (the GTIN for a product
that has one, `prod-<hex>` for one that has none, with `has_sku` as before); new are `has_gtin` and
`gtin` (empty when `has_gtin` is false: an empty string is never a GTIN), `evidence_product_ids`
(the product each piece of evidence points at, what `sku` is compared with), `channels_used` and
`barcode_enabled`. `evidence_gtins` stays, now holding each evidence's GTIN, empty where the
product has none; for products that have a GTIN it is what it was. The lane-stock node maps
`sku` and `evidence_product_ids` (parameter `gtin_map_path`, a `{product id: sku}` JSON).

## Identity ladder

`restocker_perception.identity` also holds the round-3 identity design (CMB-112): cheap evidence
first, then ask to look again, and never a guess. The registry holds every handled product (its
eight views, its label text, its GTIN when it has one); the ladder only ever compares what it saw
with that. Nothing is trained or fine-tuned, and nothing in it is specific to a product.

```text
look k:  crops (+ barcode close-ups) -> embedding (shortlist) -> label text (object-band OCR)
         accumulated over looks 0..k  -> 1. barcode     a valid GTIN of a registered product: exact
                                         2. label text  one product with a clear margin: named
                                         3. not yet     facing estimate + suggested turn / new view
                                         4. needs human the look budget is spent
```

- **Label text** (`labeltext.py`, `textreader.py`). The reader cuts each crop to the object's own
  columns (`LookCrop.half_width_px`, the detector's footprint) before it reads, because a crop holds
  its neighbours; it reads the cut as it is and, for a body of revolution, unrolled; lines of more
  than three words (ingredients, storage, address) are prose, not names, and are dropped. The tokens
  are scored against **every** registered product's label text: the weighted share of the product's
  words that were read (an inverse document frequency over the registry, so a flavour counts more than
  a brand), tolerant of joined words and dropped letters. A product is named only if it clears a minimum
  score **and** a clear margin over the runner-up (`TextPolicy`: 0.5, 0.25, at least two words) and no
  read word contradicts it (a word another registered product prints and this one does not). Two
  look-alikes that share brand and kind score the same on the shared words; neither is named until the
  word that tells them apart is read. That is the tie-breaker the embedding does not have.
- **The embedding is a shortlist.** A product named by text or barcode must be in the embedding's top
  three (`shortlist_rank`), and the embedding never names a product by itself: it cannot tell a
  product from an unregistered look-alike.
- **Guidance** (`facing.py`). The registry's eight views are a turntable. Two registry-only
  measurements place a crop on it: the crop's embedding similarity to each exemplar, and (round
  bodies) a colour signature of the unrolled visible barrel correlated with the panorama rebuilt from the
  registered views. They are pooled over the look's crops, each shifted by the difference of the camera
  azimuths. The peak is the *facing*; the target is the side whose registered view carries the most
  label text that no earlier look showed (the registry's own OCR reads); the suggested turn is the
  difference, counter-clockwise seen from above. A box has four faces: its facing is the best of the
  views at 0, 90, 180 and 270 degrees (`front`, `left`, `back`, `right`) and its turn a multiple of 90.
  The estimate is in the registry's azimuth frame; only differences are used, so no convention between
  the registry's and the world's azimuth is needed.
- **State** is the looks, not the images (`LadderEngine.observe` returns a `LookRecord`); the
  caller keeps an `ObjectState` and calls `LadderEngine.decide_escalating` (or `decide`) after each
  look. `max_relooks` (default 3) turns the last "not yet" into "needs human".
- **Escalation rung** (CMB-121, `LadderConfig.escalation`, **off by default**: with it off the
  ladder is round 3's, unchanged). When the cheap reader named nothing at a look, a heavier
  pretrained reader (Florence-2 base `<OCR_WITH_REGION>`, MIT, no fine-tuning, CPU, seconds per crop)
  reads that look's two largest crops once; the label text is scored again over that look's heavy
  tokens, which **replace** the cheap reader's tokens of the look (the cheap tokens of an object it
  could not name are mostly garbage that the contradiction rule then reads), plus the earlier looks'
  tokens. The margin rule, the contradiction rule, the embedding top-three guard and the twin policy
  are the ones the cheap rung uses; a name the heavy reader gives has rung `text_escalated`. It has
  its own latency budget (`latency_ms_per_escalated_read` in the report), apart from the per-look one.
  Switch it on with `"escalation": {"enabled": true}` in the ladder config
  (`config/identity_ladder_config_r4.json`); the backend is built lazily and needs `torch` and
  `transformers` (>= 5) outside the flake, like SigLIP 2 and RapidOCR.

`/identify_object` (`restocker_interfaces/srv/IdentifyObject`, node `identify_object_node`) serves
it: crops with `half_width_px` and `camera_azimuth_deg`, an `object_key` the node keeps the looks
under, and the reply carries the status (`IDENTIFIED`, `NOT_YET`, `NEEDS_HUMAN`, `UNKNOWN`), the SKU
and GTIN, the rung, the guidance (`guidance_kind` `turn` or `new_view`, `estimated_facing_deg`,
`visible_face`, `suggested_turn_deg`, `target_azimuth_deg`, `target_face`, confidence) and the text
evidence. Parameters: `registry_path`, `whitener_path`, `config_path`, `backend`, `ocr_version`,
`ocr_threads`, `barcode` (`none` or `zxing`) and `barcode_enabled`. Sidecar absent means `UNKNOWN`
with `sidecar_available: false`. The reply's `channels_used` lists `text_escalated` when the heavy reader
was part of the decision.

**Measuring it.** No manipulation skill exists yet, so the simulator is the **rotation oracle**
(`ladder_benchmark`): `step` takes the datasets of one round and decides in three arms: `off` (the
headline: no barcode decode and **no aimed barcode close-up view**, which a product without a barcode
never gets), `off_views` (round 3's off arm: no decode, every view) and `on` (decode, every view). It
writes the cumulative turns of the objects still open;
`tools/record_perception_dataset.py --yaw-offsets FILE --relook K` re-records those scenes with the
products turned in place and views drawn for look K (`perception_views.relook_views`). `report` gives
every number (coverage by look, re-looks needed, errors, unknown products named, latency, facing
error against the simulator's truth) and applies a bounds file. The splits are version 6 (v5's
roles, seeds appended, round 3's decision seeds burned; `perception_splits_v6.json`): every
catalogue product but the listed ones is registered (held-out products too), the listed unregistered
look-alikes appear in held-out scenes only, and every held-out seed an earlier identity round read is
`burned`.

```
ladder_benchmark whitener --registry R --dataset D... --out w.npz
ladder_benchmark step   --registry R --whitener w.npz --state S.pkl --round 0 --dataset D... --offsets-out o1.json
tools/record_perception_dataset.py --yaw-offsets o1.json --relook 1 --out ... --seed ...
ladder_benchmark step   --registry R --whitener w.npz --state S.pkl --round 1 --dataset D1... --offsets-out o2.json
ladder_benchmark report --registry R --state S.pkl --budget 3 --out report.json [--bounds B.json]
```

## Lane stock estimator

`restocker_perception.stock` states, per lane, what is in it and how many: a count, a composition
against the lane policy, free depth, coverage, and an explicit cannot-tell. It is new beside the
lane survey and changes nothing in it: `LaneObservation` and every existing consumer are untouched,
and nothing consumes the new `restocker_interfaces/msg/LaneStock` yet (`ShelfLane.contents` stays
the ledger; a count here is an estimate). The core is pure Python (numpy for the depth profile) and
imports no ROS; only `stock/node.py` does.

```text
wrist survey frame (depth, RGB, lane <- optical at the frame's stamp)
  -> lane depth histogram (numpy port of lane_depth_measurement.cpp, same window and thresholds)
  -> free depth, coverage, obstruction  (or: the LaneObservation the survey already published)
  -> top-height staircase: the rear product's class, runs of equal products
  -> count = occupied depth over the contact pitch, cross-checked against the top surface
  -> crop of the rear product -> /identify_product -> naming policy
  -> LaneStock
```

**Count.** The column is packed against the front retainer, so with `n` upright products of one
class the nearest surface is at `bed_datum - r - extent - (n-1) * pitch` (the arithmetic of
`restocker_gazebo.lane_columns`, restated in `config/lane_stock.yaml` and pinned by
`test_lane_stock.py`). The estimator inverts it: `n = 1 + (bed_datum - r - extent - nearest) /
pitch`, takes the nearest whole number, and reports `count_min == count_max` when the column
length is within 0.2 pitch of it, a one-product range when it is within 0.35, and a refusal
(`REASON_INCONSISTENT_GEOMETRY`) beyond that. The pitch is the policy's product unless the rear
product's measured top height says otherwise.

**Products and orientations (CMB-120).** A product is a pair that the formulas above need: half
its extent along the lane (`r`) and its height. A cylinder standing is `(radius, height)`; a box
is `size_xyz_m = (across, along, up)` in `config/lane_stock.yaml`, with `r = along / 2`, and a box
resting on another face is another pair (`ProductGeometry.orientations()`: its six faces). The
lane policy names a class and a SKU; class 4 (box) covers every box shape, so the SKU's registered
shape (`sku_products`) is what the lane expects, and the rear top height is matched to the
catalogue (every product standing, every orientation of the expected one except its quarter
turns) within `max(10 mm, 3 bed sigmas)`. A quarter turn (the registered box turned about the
vertical: same top height, another side along the lane) is not a candidate: it cannot be told from
the registered orientation by height, and under a few millimetres of noise a 55 mm pitch fits a
25 mm column by chance (7 slim boxes read as 3; at 1 % noise 6, 5 and 2 read as 3, 2 and 1). Lanes
are packed with the depth side along them, so a box turned in a lane is outside what the estimator
can count, and it is not detected either. When several candidates still fit by height, the lane's
own product is the answer (the caller then refuses if it does not count the column in whole
products); another product is named only when the lane's own is not among them, by the column's
length measured in metres against the noise (`max(4 mm, 2.5 bed sigmas)`, not in pitches, which
lets a long pitch fit anything), and two that fit and count differently are refused as ambiguous. A column of another shape than
the expected one is `class_mismatch` (misplaced) even when both are boxes. The surface must cover
at least 2 depth bins and 0.7 of the matched product's pitch. The cross-check (`visible_faces`) counts the bins of
the top surface in units of the pitch; if it disagrees with the depth count by more than one
product, `count_consistent` is false and the range spans both. A column whose top height steps
(a bottle in a can lane) is counted run by run (`LaneStock` carries the totals; the estimate's
`segments` the runs) and flagged as misplaced. A surface that tops out lower than every
catalogued product is refused, not counted: through range noise the bed's own speckle reads as a
product at the lane mouth, and counting it would turn an empty lane into a full one.

**Cannot-tell.** `STATE_CANNOT_TELL` with a reason, never a low-confidence number: the producer
refused (`REASON_NO_EVIDENCE`), coverage below the survey's 0.90 floor (`REASON_LOW_COVERAGE`),
something protruding from the mouth, no policy class and no geometry to use instead, or geometry
no class explains. A lane seen to be empty along its whole length is an answer (`COMPOSITION_EMPTY`,
count 0); a lane that was not seen is not. All count and composition fields of a refusal are zero
and must be ignored.

**Identity and composition.** The identity channel is consumed as CMB-112 round 2 left it, with
its failure in mind: unregistered look-alike twins of registered singles are named as the twin
43.6 % of the time by the embedding alone. The estimator therefore applies a **safe naming
policy** (`stock/identity_policy.py`): a name is accepted only when OCR or the barcode confirms
the same GTIN; anything the embedding alone says is an abstention (`--identity-mode embedding`
exists only to measure what that costs). Identity is a possibly-abstaining input: with the sidecar
absent, `/identify_product` unreachable or the face too small to ask about, the count is still
reported and `identity_channel_reachable` says what happened. Composition is not a boolean:
`COMPOSITION_CONFORMING` needs every product named and expected, `PARTIAL` some, `MISPLACED` a
named wrong SKU or a class the policy does not hold (`misplaced` is set only from evidence, never
from an abstention), `UNKNOWN` none; `unverified_count` is the products nobody named. **Limit:
a survey frame shows the rear product (nearest the camera) and hides the rest, so at most one
product per lane can be named, and an unflagged lane is not a verified-clean lane.**

**Node.** `lane_stock_node` (parameters `lane_observation_topic`, `identify_service`,
`world_state_service`, `identity_mode`, `gtin_map_path`, ...) subscribes to the survey's
`/perception/lane_observations`, the wrist depth/RGB frames and `camera_info`, resolves the lane
transform at the observation's stamp, reads the lane policy from `/world_state/get_snapshot`,
calls `/identify_product` on the rear product's crop (off the executor thread) and publishes
`/perception/lane_stock`. Free depth and coverage come from the observation; the top profile is
recomputed from the frame. `ros2 launch restocker_bringup baseline.launch.py lane_stock:=true`
runs it beside the lane survey (default off). The identity channel names registry GTINs and a
lane policy names SKUs; `gtin_map_path` (JSON `{gtin: sku}`) bridges them, otherwise they are
compared as written.

**Benchmark.** `stock_benchmark` (`stock/cli.py`) runs the estimator over recorded lane survey
frames (`tools/record_perception_dataset.py --lane-views`) and writes prediction rows;
`benchmark_perception stock-score|stock-evaluate|stock-seeds` (restocker_benchmarks) score them
against the lane ground truth and apply `config/lane_stock_acceptance_bounds.json`, declared before
any held-out lane frame was recorded. Arms: `abstain` (no identity), `channel` (the real
`/identify_product` logic, safe policy), `oracle` (an ablation: a perfect channel for whatever face
is framed); `--degrade` blinds a seeded band of the depth image to test refusal, `--noise` and
`--dropout` add seeded range noise and pixel dropout (a synthetic perturbation: the simulator's
depth camera is ideal).

**Measured (simulator, one run, bounds declared beforehand).** On 450 held-out lanes (75 seeds, six
lighting conditions) the count was within one product on 450 / 450 sufficient-coverage lanes (exact
on all), every one of 227 blinded low-coverage lanes was refused, and no clean lane was flagged
misplaced (0 / 318). The identity channel as built named no lane product under the safe policy: at
lane range (1.0-1.4 m) every answer was embedding-alone, OCR and the barcode never confirmed one,
and the embedding alone was wrong two times in three, so misplaced recall for the system is 0 / 132;
a perfect channel would reach the lanes whose wrong product is the visible one (53 / 132). Known
limits: a survey frame shows only the rear product.

**Noise and blinding (round 2).** The estimator and the lane depth measurement follow the noise of
the frame: the floor margin is `max(10 mm, |bed offset| + 4 robust sigmas)` of the bed's own height
residuals, the front inset twice that noise term, and the nearest surface moves back by 4 sigmas
(`floor_noise_sigma_factor`, `edge_bias_sigma_factor`; `lane_survey.yaml`, `lane_stock.yaml`; 0
restores the fixed margin, and an ideal frame, sigma about 1 mm, is unchanged). It refuses, rather
than widening, a top surface that disagrees with the depth count, a surface under three bins wide
(not a product) and a step in the top height shorter than a product; above 8 mm of bed sigma it
reads no class and makes no class claim. Measured on 450 fresh held-out lanes per arm against bounds
declared beforehand (seeded Gaussian range noise of 0.5 %, 1 % and 2 % of range with 2 % dropout, a
synthetic model of an ideal render, not a camera): 0 wrong answers in 449, 448 and 413 answered
lanes, empty lanes read empty 82 / 82, 82 / 82 and 80 / 82, no clean lane flagged, and under
partial blinding 0 wrong of 238 answered. At 2 % noise 8 % of lanes are cannot-tells. A real camera's
noise structure is not tested.

## Product proposals and unknown products

`restocker_perception.proposals` (numpy, plus cv2 when a proposal is made; neither model nor colour)
answers "where does something stand that could be a product", and `identity_state` says what the
rest of the system may be told about it. Offline Python for now: no node publishes proposals, and
the default C++ path is unchanged.

**Geometry proposer** (`proposals/geometry.py`). One z-depth image and the camera pose in, a
`ProposalSet` out (a per-pixel label image and one `Proposal` each). Depth is back-projected to world
points and kept above a known support surface inside its region (the stock tray, or one lane's 4
degree roller bed, whose front retainer rail is masked as known structure). The points go into a 4 mm
top-down height map; each connected component is split the way the colour-and-depth backend splits
tops: take the tallest flat plateau (4 mm band) and ask which shape it is. A plateau that a circle
explains is a `disc`, a catalogued cylinder's top; a plateau that fills its oriented rectangle
(second-moment fit, 0.86 overlap), has no deep concavity and whose outline is a cliff, not a slope,
is a `rect` (CMB-120: a carton standing, a box lying on a face; `size_m`, `yaw_rad`), and the
rectangle wins when it beats the circle by 0.06. A lying cylinder's ridge is a slope (it falls a few
millimetres per cell) and stays a blob. Two flat tops of equal height that touch have a waist: the
plateau is cut there and each piece judged again, kept only if a piece is a rectangle (touching
discs keep their distance-transform peeling). Each shape gets its footprint, is removed, and the
loop repeats. What nothing explains (a lying cylinder, a contoured bottle, a foreign object) is a
`blob` that reaches one product radius around its plateau, except where a flat surface lower than
the blob lies, which is another product's top. A `disc` carries a `catalogue_class` when its radius
and height match a catalogued cylinder; a `rect` never does. It never reads a colour, a
class or a SKU, so it works on label-textured products, which the colour stage cannot (a white-based
can has no chromaticity). All tolerances are in `ProposalConfig`, fixed from the catalogue and
frozen in `config/proposal_config_frozen.json` (CMB-113) and `config/proposal_config_cmb120.json`
(CMB-120, which added the rectangle, neck and flat-top parameters).

**Unknown product** (`proposals/identity_state.py`). `resolve_identity(proposal, identity_result)`
gives `IDENTIFIED` only when the identity channel named a registered SKU and the proposal's fitted disc
does not contradict that product's registered radius and height. Everything else is
`UNKNOWN_PRODUCT`: the channel said unknown, abstained, was unavailable or absent, or its name was
contradicted by the geometry. It never carries a SKU; `reason` says which case, and the evidence
lists each channel's score plus a `geometry` entry (1.0 when an upright catalogued cylinder was
fitted, 0.0 for a blob). `identity_fields` returns the additive fields of
`restocker_interfaces/msg/ObjectObservation` (`identity_state`, `identity_evidence_kinds`,
`identity_evidence_scores`, `identity_reason`; `IDENTITY_NOT_ASSESSED` is what every earlier producer
leaves). World state refuses an observation that pairs `UNKNOWN_PRODUCT` with a SKU or `IDENTIFIED`
without one. The identity channel is consumed as calibrated: an unknown product is not a claim that
the product is foreign, only that nothing may name it.

Measured numbers and limits: `docs/measurements/2026-10-19-proposal-benchmark.md` in the project
notes; the benchmark is described in `restocker_benchmarks/README.md`.

## Inventory

`restocker_perception.inventory` aggregates the lane stock estimates and the tray into one queryable
inventory: per-SKU totals with their uncertainty spelled out, per-lane contents, the age of every
reading against a declared horizon, and a comparison with the authoritative world state. It is a reader:
`LaneStock`, `ShelfLane.contents` and every existing consumer are unchanged, and nothing consumes the
inventory yet. The core (`model.py`, `aggregate.py`, `query.py`, `world.py`) is pure Python and imports no
ROS; `messages.py`, `node.py` and `cli.py` are the only ROS-facing modules.

```text
LaneStock per lane (/perception/lane_stock)  --+
tray: perception reading (ObjectObservation    +--> aggregate --> Inventory  (/perception/inventory)
      with identity state) else the world's    |               \-> QueryInventory (/query_inventory)
world state snapshot (/world_state/get_snapshot)+
```

**What a total means.** A SKU's total is the sum of three kinds of product, kept apart in the message:
*confirmed* (the identity channel named the product), *assumed* (counted by the sensor in a lane whose
policy lists the SKU and not individually named; a range, the estimator's count range) and *reported*
(listed by the world state only, the tray's fallback). Only *confirmed* is evidence of the SKU. A survey
frame shows only the rear product of a lane, so an unflagged lane is not a verified-clean lane: the
assumed share can contain a hidden wrong product, and the total's range covers the count, not that. A
lane flagged misplaced is contested: its unnamed products are no longer assumed to be the policy SKU (that
share's lower end is zero, and a class mismatch with no name puts the wrong products in an unknown-SKU
bucket). A product the identity channel positively said is unregistered goes to that bucket too.

**What is left out, and said so.** A lane that was refused (`cannot tell`), never read, or last read more
than the horizon ago contributes nothing to any total and makes the SKUs whose policy lane it is
`complete = false` with the lane named (`unread_lanes`, `stale_lanes`). A SKU is only complete when
every lane that could hold it was read and fresh and the tray is covered. The horizon is a deployment
parameter (`stale_after_s`, default 900 s; the per-query `max_age_s` overrides it), not a measured constant.

**Tray.** From the perception path when it has a fresh reading (`tray_observation_topic`, off by default;
each `ObjectObservation` is `IDENTIFIED` and so confirmed, `UNKNOWN_PRODUCT`, or not assessed and so
assumed from the manifest), otherwise from the world state (tracked, free objects that no lane's ledger or
fresh occupancy accounts for), where it is reported, not sensed. In simulation the world state is fed
ground truth by default, so a world-sourced tray is the simulator's belief.

**Discrepancy.** Each counted lane is compared with its world ledger (`ShelfLane.contents`): the observed
count range lies above or below the ledger's count (`count_delta` says by how much), or a name the identity
channel gave contradicts the SKU the ledger records for the rearmost product (`sku_discrepancy`, never raised
from an abstention). A lane that could not be compared is `NOT_COMPARABLE`, which is not agreement. Per SKU,
the world's figure and the difference are given when the SKU is complete.

**Query.** `QueryInventory` takes a SKU, a lane or neither, an evidence floor (`min_confidence`: any,
expected = sensor-counted only, confirmed = identity-named only) and a `max_age_s`. A SKU query returns the
count, its range, how much of it is confirmed, how many products the floor excluded, whether the answer is
complete and a sentence saying why. `inventory_tool` is the operator front end:

```text
inventory_tool query --sku GEN-001              # how many in stock, and how sure
inventory_tool query --sku GEN-001 --min-confidence confirmed
inventory_tool query --lane lane_03 --max-age 120
inventory_tool show                              # every lane and every SKU
inventory_tool replay --datasets <pbench-dir>... --predictions <rows.jsonl> --seed 12 --sku GEN-001
```

`ros2 launch restocker_bringup baseline.launch.py lane_stock:=true inventory:=true` runs it beside the lane
stock estimator (default off). Parameters of `inventory_node`: `lane_stock_topic`, `inventory_topic`,
`query_service`, `world_state_service`, `tray_observation_topic`, `stale_after_s`, `publish_period_s`,
`world_refresh_period_s`, `world_max_age_s`, `tray_window_s`.

**Replay evaluation.** `inventory/replay.py` and `benchmark_perception inventory-score|inventory-evaluate`
replay recorded lane stock predictions (`stock_benchmark predict`) as inventories per benchmark seed and score
the aggregation against the labelled lane and tray contents, against bounds declared beforehand
(`config/inventory_acceptance_bounds.json`). The world state in the replay is the labelled truth, optionally
perturbed by a declared seeded schedule (ledger counts off by one or two, rearmost SKU swapped); ages follow a
seeded schedule. The benchmark has no tray reading, so its tray is world-sourced.

**Measured (simulator, one run on 75 held-out seeds, 450 lanes, bounds declared beforehand; all 33 bound
rows pass).** Complete SKU totals were within one product of the labels in 98 % of the cases without an
identity channel (449 / 459) and 99.6 % with a perfect one; only 60 % (79 % with a perfect channel) were
exact, the rest being the wrong products the survey cannot see, which is why the range is not claimed to
contain the true SKU total. With half the frames partly blinded, no SKU with a refused policy lane was
reported complete (0 / 162). A perturbed ledger was flagged on 100 % of the answered lanes (149 / 149, 97 %
at 2 % range noise) with no false alarm on 301 truthful ones; stale readings were all flagged and left out of
the totals; a swapped ledger SKU is detected only when the identity channel names the rear product (89 / 89
with a perfect channel, and not at all without one, since nothing is named at lane range yet). The tray in
this benchmark is the world state's, so tray perception is not measured. The rendering is Gazebo and the
depth noise a synthetic model, not a camera.

## Tests

```
just test-package restocker_perception
```

This builds the package and its dependencies and runs the C++ and Python tests under `test/`.
The registry tests alone need no build: `python3 -m pytest ros_ws/src/restocker_perception/test/test_product_registry.py`.

## Out of scope

Observation admission policy lives in `restocker_world_state`. Simulator protobufs, model weights and
image encodings do not belong in this package.
