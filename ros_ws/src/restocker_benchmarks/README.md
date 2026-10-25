# restocker_benchmarks

Owns seeded scenarios, metrics, result metadata, serialization, and plots. It consumes public
runtime interfaces rather than implementation internals.

## What is here

| Module | Purpose |
| --- | --- |
| `scenarios.py` | The scenario table: a stock file and a goal sequence, and nothing else |
| `runner.py` | One headless run: launch, drive the goals, record every outcome, tear down |
| `taxonomy.py` | Grouping a terminal outcome by cause and by how far it got |
| `report.py` | A campaign directory to `summary.json`, `report.md` and plots |
| `plots.py` | SVG bar charts, standard library only |
| `perception_dataset.py` | Seeded scenes, ground-truth labels, camera projection, content-hashed dataset directory |
| `perception_splits.py` | Frozen train/validation/held-out splits over products and seeds, and the run guard |
| `perception_scorer.py` | Predictions JSONL scorer with confidence intervals, and the baseline predictors |
| `perception_cli.py` | `benchmark_perception`: splits, check, scenes, score, baseline, verify, stock-score, stock-evaluate, stock-seeds |
| `perception_lane_views.py` | The wrist survey's six lane stations as recorder viewpoints (`--lane-views`) |
| `perception_stock_scorer.py` | Lane stock estimator scorer (CMB-114): count, refusal and misplaced metrics, declared bounds |
| `tray_perception_campaign.py` | Predeclared wrist-tray matrix, covariance-derived attachment budget, offline summary |

Executables: `ros2 run restocker_benchmarks benchmark_run <scenario>` and
`ros2 run restocker_benchmarks benchmark_report [campaign]`, wrapped by `just benchmark`,
`just benchmark-all` and `just benchmark-report`.

## Behavior

This is a measurement tool, not a gate. `benchmark_run` exits 0 when it records a failure. The
measured system fails intermittently, and a pass count alone would hide the causes.

A failed goal does not stop its run. Failures are grouped by distinct cause with counts rather than
totalled, and the report lists the brief's metrics it cannot measure instead of filling them with
defaults.

## Dependency shape

The runner launches `restocker_bringup`'s public composition as a subprocess, by name, exactly as
an operator would. It does not declare a manifest dependency on it, because runtime packages must
not depend on the composition root. It does depend on `restocker_gazebo` and
`restocker_moveit_config` for their share directories, so a benchmark reads the same stock and
planner configuration the launch loads rather than a copy.

`taxonomy.py`, `scenarios.py`, `report.py`, `plots.py` and `tray_perception_campaign.py` import
nothing from ROS. A stored campaign can be re-aggregated and re-plotted from a plain Python
interpreter after the workspace that produced it is gone; only `runner.py` needs `rclpy`.

## Perception benchmark (Card 109)

Labelled wrist and overhead frames from fixed-seed scenes, frozen splits, and a scorer. The
simulator only supplies pixels, camera poses and intrinsics; every label comes from the scenario
document Gazebo is told to spawn, and the recorder refuses to write a frame unless ground truth
reports every product within 3 cm of its scenario pose.

**Record** (one simulator at a time, under the machine-wide slot and a leased ROS domain):

```
tools/record_perception_dataset.py --out DIR --split held_out        # or --seed N ...
ros2 run restocker_benchmarks benchmark_perception verify DIR/pbench-<hash>
```

**Dataset directory** `pbench-<first 16 hex of dataset_hash>/`: `manifest.json`, `labels.jsonl`
(one row per frame), `scenarios/<id>.yaml`, `frames/<frame_id>.png` (RGB) and
`frames/<frame_id>.depth.png` (16-bit millimetres). The manifest records seeds, git SHA and dirty
flag, SHA-256 of the world file, the catalogue and the split file, the hash of every scenario and
every file. `labels_hash` covers only renderer-independent label content (reproducible on any
machine for fixed seeds); `dataset_hash` also covers pixels and camera poses, which depend on the
renderer. A label row holds: `frame_id`, `scenario_id`, `seed`, `seed_split`, `sensor`
(wrist or overhead), `viewpoint` (null at the home pose, or the close-range view's name),
`camera` (optical-frame pose in world `[x y z qx qy qz qw]`, pinhole intrinsics), `lighting`
(`condition` name and `intensity_scale`, with the ambient, background and sun read back from the
world file the frame was rendered in), `objects`
(`object_id`, `sku`, `product_class`, `pose_world`, `lane_id` or null for the tray, `lane_order`
front first, `misplaced`, `pixel`, `range_m`, `in_view`) and `lanes` (`expected_sku`, `count`,
`skus_in_order`, `misplaced_count`, `in_view`).

**Splits** are `config/perception_splits.json` (embeds its own SHA-256, so an edit is detected).
Products and seeds are each split train/validation/held-out with a pinned generator. At most one
member of a look-alike pair is held out, so every held-out product has a registered twin it could
be confused with. Train and validation seeds contain only registered products; held-out seeds
contain all of them. Regenerate only deliberately: `benchmark_perception splits --catalogue ...
--out ...` (then every result against the old split is void).

**Version 2 (append-only extension).** Twenty-four seeds leave about 14 tray objects in the
held-out split, too few for any bound to be demonstrated, and a held-out run spends its split.
`config/perception_splits.json` is therefore a version-2 file that extends the frozen version-1
file (`config/perception_splits_v1.json`, kept byte-for-byte, embedded sha256 `bcbda3f8...`) by
seeds 25..184, shuffled with their own pinned stream (`split_seed ^ SPLIT_SALT` plus 2) into 40 train,
20 validation and 100 held-out seeds. Seeds 1..24 keep their exact assignments, the product split
and look-alike groups are copied unchanged, and the version-1 held-out seeds (3, 6, 7, 9, 15, 16)
are recorded as `held_out_generations.burned`: they decided the first identity-channel round, are
off-limits to calibration like any held-out seed, and may only be reported as a secondary number.
The added held-out seeds are `fresh` (150 of them). Every earlier document is embedded in the file, so
`benchmark_perception splits --catalogue ... --verify FILE` re-derives and checks the whole chain.
Extend deliberately, never edit: `benchmark_perception splits --catalogue ... --extend V1.json
--last-seed N --counts TRAIN,VALIDATION,HELD_OUT --out V2.json`. `test_perception_splits_v2`
pins the preserved assignments, the burned set and the reproduction. Guard a run before it
starts:

```
benchmark_perception check --splits config/perception_splits.json --run run.json
```

`run.json` holds `registry_skus`, `training_seeds`, `calibration_seeds`, `registry_source_seeds`
and optionally `splits_sha256`. The check exits 2, naming each overlap, if any of them touches the
held-out split, or if the run used another split file.

**Version 3 (the box SKUs, CMB-119).** Adding four box SKUs changes the catalogue the splits
freeze, so the catalogue view splits v1 and v2 pinned (`load_generated_catalogue`, no contour
bottle, no box, sha256 `b17ef8f1...`) stays the default and both files stay valid and byte-for-byte
as they were. `config/perception_splits_v3.json` (schema 3) is the same kind of append-only chain
(it embeds the v2 file): it pins the catalogue *with* boxes (`catalogue_includes_boxes`, sha256
`6e6414f9...`), assigns the four new SKUs with their own pinned stream (train GEN-017 and GEN-019,
validation GEN-018, held-out GEN-020, the barcode-less box; no look-alike pair is split across
held-out) and appends seeds 235..284 (25 train, 12 validation, 13 held-out, all `fresh`).
`box_seed_start` (235) is the first seed whose scene may draw boxes: every earlier seed draws
exactly the scene it always drew (`test_perception_splits_v3` compares the scenes), so no earlier
result changes meaning. Use `load_catalogue_for_splits(catalogue, splits)` to get the view a file
froze, `scene_includes_boxes(splits, seed)` for the scene rule. Extend with `benchmark_perception
splits --catalogue ... --extend-boxes V2.json --last-seed 284 --counts 25,12,13 --product-counts
2,1,1 --out V3.json`; `--verify` re-derives the chain. The proposal benchmark has its own box
scenes (development seeds 1101..1112, tray kind `box`; no held-out box seeds are declared until
round 3 commits them with its bounds).

**Version 4 (more box seeds, CMB-120 round 2).** `config/perception_splits_v4.json` (schema 4) extends
v3 with seeds only: 285..364, all held-out (80 `fresh`), no product added, catalogue view and
`box_seed_start` unchanged. It also moves the 13 v3 held-out seeds the first lane-stock run decided on
(242, 243, 246, 247, 257, 258, 260, 266, 267, 279, 280, 282, 283) from `fresh` to `burned`. Extend with
`benchmark_perception splits --catalogue ... --extend-box-seeds V3.json --last-seed 364 --counts 0,0,80
--burn 242,... --out V4.json`; `--verify` re-derives the chain. Which fresh seeds a measurement then
uses is recorded with that measurement.

**Scoring**: `benchmark_perception baseline --kind truth|noisy ...` writes predictions;
`benchmark_perception score --labels DIR --predictions P --splits S [--json out]`. The prediction
schema is documented at the top of `perception_scorer.py`. Per condition (sensor, lighting,
known/unknown product, lane/tray, product class, seed split, range band) it reports identity
accuracy, abstention rate, **false-identity rate** (a named SKU that is wrong; for an unregistered
product any named SKU), unknown-rejection rate, miss rate, lane count error, misplaced-flag
accuracy, and pose error (translation and symmetry-axis angle, as the pose-error evaluator), with
Wilson intervals for proportions and bootstrap intervals for means.

**Lighting** (`perception_lighting.py`). Every seed draws one of six conditions from its own
stream: `nominal` (the shipped world), `dim`, `bright`, `warm`, `cool` and `directional` (a low
side sun, long shadows), each with a seeded intensity scale of +-10 % and, for `directional`, a
seeded turn of the sun about the vertical. The recorder writes a copy of `restocking.sdf` with the
ambient colour and the `workcell_sun` light changed (the shipped world is never edited) and
launches it through the `world` argument. The condition name goes in each row's `lighting`, in the
manifest's per-scenario `lighting` (with the SHA-256 of that generated world) and in `labels_hash`.
It is a pure function of the seed and cannot change which products a scene holds.
`--no-lighting` records the nominal world.

**Close-range wrist views** (`perception_views.py`). At the home pose the wrist camera sees no
product, so each scene that has tray products also gets up to three seeded viewpoints 0.28-0.45 m
from a tray product (the aim is 5 cm up the barrel; the eye stays below world z 1.0 m, where the
arm can hold a tray view). They are camera poses sent to `/survey_viewpoint`, so the planner and
controllers move the arm, as for the survey stations; the approach direction is the confirm duty's.
A seed turns it by up to 30 degrees and tilts it by 6. The recorder then grabs a wrist frame
stamped after arrival and labels it from the TF pose (`viewpoint` = `close00`...); the test fails
if a reached view does not hold its target at 0.2-0.55 m. A view the planner cannot reach is
recorded in `part.json` and the manifest (`close_views`), not silently dropped. `--close-views 0`
disables them. Limits: only tray products are targeted (lane products are further from the rail
than the arm can bring the camera to reading range; not measured per product); the label side facing the lens depends on the product's random yaw, so a
close frame shows the front panel only on some views, and no label-facing flag is recorded.

**Label orientation** (Gazebo, measured). A cylinder primitive maps a label strip rotated by half
a turn, so `label_factory.py` rotates the strip of every cylinder SKU by 180 degrees; the contour
mesh's own UVs render the strip upright and unmirrored, so contour textures are left alone. At
yaw 0 the cylinder's front panel faces world -x and the contour bottle's faces +y.

Conservative defaults chosen without the owner: the registry for scoring is train plus
validation; a missing prediction is a *miss*, never an abstention; `in_view` means the object
centre projects inside the image, not that it is unoccluded (no instance masks yet); frames at
the home pose come from the camera without arm motion; contoured bottles are never drawn into
benchmark scenes (their class is perception-only and the executor stack refuses it).

## Lane stock benchmark (Card 114)

The lane stock estimator (`restocker_perception.stock`) is scored on frames taken where the wrist
survey takes them. `tools/record_perception_dataset.py --lane-views` sends the arm to each lane's
survey station (`perception_lane_views.py`: 0.55 m behind the shelf's rear edge, 0.55 m above the
lane floor, over the lane's centre line, aimed at mid-lane depth, the same numbers as
`survey_stations.cpp`) through `/survey_viewpoint`, and records a wrist RGB-D frame there with the
`viewpoint` set to the lane's id (`lane_01` ... `lane_06`) and the TF camera pose. Every lane gets a
view, empty ones included. From the station the camera looks down the lane from its rear entrance,
so the frame shows the rear-most product (the last one placed) and hides the rest; the label rows
are unchanged (`lanes[*].count`, `skus_in_order`, `misplaced_count`).

`perception_stock_scorer.py` is additive: `perception_scorer.py` scores every in-view lane of a
frame, and a survey frame frames one lane, so this module scores the lane the frame was taken for.
`benchmark_perception stock-score --labels DIR... --predictions P... [--json out]` reports, per
condition (lighting, seed split, product class, lane fill, identity arm, degradation): the share of
sufficient-coverage lanes (measured coverage at or above the producer's 0.90 floor) answered within
one product of the truth (a refusal there counts as a miss), refusal on low-coverage lanes and how
often an answered one is wrong, misplaced recall (all lanes, and the subset whose misplaced product
is the one the camera can see), the false-alarm rate on clean lanes, lanes declared conforming that
hold a misplaced product, the share of lanes with a named face, and Wilson intervals for each.
`stock-seeds` prints the pre-declared held-out population (the fresh held-out seeds, every second
one) and `stock-evaluate --bounds restocker_perception/config/lane_stock_acceptance_bounds.json
--report ARM=score.json ...` applies the bounds declared before any held-out lane frame was
recorded (pass needs the Wilson limit on the right side of the bound and the sample size met).

Limits: the simulator's depth camera is ideal, so the estimator is also run on seeded range noise
and pixel dropout (`stock_benchmark --noise --dropout`), which is a synthetic perturbation, not a
sensor model; misplaced products in the generated scenes are same-class look-alike twins at a
random depth, so geometry alone cannot see them and only a named rear product can raise the flag.

## Proposal benchmark (Card 113)

Scores where products are, not which they are. `perception_proposals.py` builds seeded scenes: the
identity benchmark's lane columns plus one of six pinned tray arrangements per seed (sparse, dense
with touching neighbours, tight rows, lying products, contoured bottles, tall products in front of
short ones). The recorder drives the wrist camera to the shipped tray survey stations, two lane
stations of non-empty lanes and one close view (`tools/record_proposal_dataset.py`; views carry
`"range_check": false` and no target). Contoured bottles are drawn for the camera through a simulator
stand-in scenario (the ground-truth adapter and the planning-scene projector refuse the contour
class); labels and masks use the contour's own cylinder. Seeds 1001..1024 are development (the only
ones a proposer tolerance may be examined on), 2001..2048 are held out. The overhead camera is not
scored: its benchmark frames are almost entirely black.

`perception_proposal_scorer.py` ray-casts the scene document through the recorded camera pose
(one cylinder per product, any pose), keeps a pixel where the recorded depth agrees with the model
(-20/+30 mm), and so gets an instance mask without simulator support; the median depth residual is
reported (0.7 mm on the development set). A product is recallable with at least 250 visible pixels,
30 % of its silhouette visible and 60 % of it inside the image. A proposal recalls it at mask IoU
0.5, one to one (strict; a proposal merging two products recalls at most one), and loose recall
(a proposal covers half of the visible mask) is reported next to it. Intervals resample whole
scenes. `benchmark_proposals evaluate --dataset D... [--bounds B --out R]` scores the geometry
proposer; a held-out dataset needs `--bounds` and `--out`, refuses an existing result and records
the proposer-configuration hash. The pass rule is `restocker_perception/config/proposal_acceptance_bounds.json`.
